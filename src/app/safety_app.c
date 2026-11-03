#include "app/safety_app.h"

#include "app/node_health_core.h"
#include "features/cornering/cornering.h"
#include "platform.h"
#include "platform_e2e.h"
#include "platform_mailbox.h"
#include "services/mailbox.h"
#include "vehicle_e2e.h"

#include <string.h>

_Static_assert(PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_LENGTH == PLATFORM_SAFETY_WARNING_STATUS_LENGTH,
               "slot length = frame length");
_Static_assert(PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_LENGTH == PLATFORM_HEARTBEAT_SAFETY_LENGTH,
               "slot length = frame length");
/* The all-raw-0 0x023 fallback must mean "no information" in every field (safety-reviewer
 * MINOR-3): a defs change that moved these values would turn it into "side clear". */
_Static_assert(PLATFORM_SAFETY_WARNING_STATUS_CORNERING_LEVEL_UNAVAILABLE_CHOICE == 0u, "raw 0 = UNAVAILABLE");
_Static_assert(PLATFORM_SAFETY_WARNING_STATUS_CORNERING_SOURCE_NONE_CHOICE == 0u, "raw 0 = NONE");
_Static_assert(PLATFORM_SAFETY_WARNING_STATUS_BSM_LEFT_UNAVAILABLE_CHOICE == 0u, "raw 0 = UNAVAILABLE");
_Static_assert(PLATFORM_SAFETY_WARNING_STATUS_BSM_RIGHT_UNAVAILABLE_CHOICE == 0u, "raw 0 = UNAVAILABLE");
_Static_assert(PLATFORM_SAFETY_WARNING_STATUS_IMMOBILIZER_STATE_UNKNOWN_CHOICE == 0u, "raw 0 = UNKNOWN");
_Static_assert((SAFETY_APP_PASS_MAX_MS * 4u) <= PLATFORM_EKF_LEAN_CYCLE_TIME_MS, "four reads per 0x020 cycle");

typedef struct {
    bool initialised;
    bool first_pass; /* the first pass only sets the gap baseline (D-101 item 2) */
    uint32_t last_pass_ms;
    uint32_t warn_due_ms;
    uint32_t hb_due_ms;
    uint32_t pending_events; /* event faults found after node_health_pass(), for the next pass */
    vehicle_e2e_tx_state_t warn_tx;
    vehicle_e2e_tx_state_t hb_tx;
    node_health_t health;
    safety_app_status_t status;
} safety_app_ctx_t;

static safety_app_ctx_t app;

// cppcheck-suppress misra-c2012-19.2 ; DEV-003: the D-093 region type, passed through to mailbox_init()
bool safety_app_init(volatile mailbox_region_t* region, uint32_t now_ms)
{
    (void)memset(&app, 0, sizeof app);
    cornering_init();
    if (!mailbox_init(region, now_ms)) {
        return false;
    }
    vehicle_e2e_tx_init(&app.warn_tx);
    vehicle_e2e_tx_init(&app.hb_tx);
    node_health_init(&app.health, now_ms);
    app.first_pass = true;
    app.last_pass_ms = now_ms;
    app.warn_due_ms = now_ms;
    app.hb_due_ms = now_ms;
    app.initialised = true;
    app.status.initialised = true;
    return true;
}

/* Wrap-safe "now_ms is at or after t_ms" for times less than half the 32-bit range apart. */
#define SAFETY_APP_HALF_RANGE (0x80000000u)
static bool reached(uint32_t now_ms, uint32_t t_ms)
{
    return (uint32_t)(now_ms - t_ms) < SAFETY_APP_HALF_RANGE;
}

static bool due(uint32_t* next_ms, uint32_t cycle_ms, uint32_t now_ms)
{
    bool is_due = false;
    if (reached(now_ms, *next_ms)) {
        is_due = true;
        *next_ms += cycle_ms;
        if (reached(now_ms, *next_ms)) {
            *next_ms = now_ms + cycle_ms; /* late by a whole cycle: resync, no burst */
        }
    }
    return is_due;
}

/* 0x023: cornering pair from the decision, blind spot UNAVAILABLE and immobilizer UNKNOWN
 * (raw 0, the no-information values) until those modules exist. */
static void offer_warning(void)
{
    struct platform_safety_warning_status_t msg;
    uint8_t frame[PLATFORM_SAFETY_WARNING_STATUS_LENGTH];
    cornering_output_t out = cornering_output();
    bool ok = true;

    (void)memset(&msg, 0, sizeof msg);
    if (cornering_output_consistent(out)) {
        msg.cornering_level = (uint8_t)out.level;
        msg.cornering_source = (uint8_t)out.source;
    } else {
        ok = false; /* never send an inconsistent pair; raw 0 = UNAVAILABLE/NONE */
    }
    msg.bsm_left = (uint8_t)PLATFORM_SAFETY_WARNING_STATUS_BSM_LEFT_UNAVAILABLE_CHOICE;
    msg.bsm_right = (uint8_t)PLATFORM_SAFETY_WARNING_STATUS_BSM_RIGHT_UNAVAILABLE_CHOICE;
    msg.immobilizer_state = (uint8_t)PLATFORM_SAFETY_WARNING_STATUS_IMMOBILIZER_STATE_UNKNOWN_CHOICE;

    if (platform_safety_warning_status_pack_checked(frame, &msg, sizeof frame) != (int)sizeof frame) {
        ok = false;
        (void)memset(frame, 0, sizeof frame); /* the documented fallback: all raw 0 */
    }
    if (!vehicle_e2e_is_valid(platform_safety_warning_status_e2e_protect(frame, sizeof frame, &app.warn_tx)) ||
        !mailbox_offer((uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_INDEX, frame, (uint8_t)sizeof frame)) {
        ok = false;
    } else {
        app.status.warn_sent++;
    }
    if (!ok) {
        app.pending_events |= NODE_FAULT_OUTPUT;
        app.status.output_faults++;
    }
}

static void offer_heartbeat(void)
{
    struct platform_heartbeat_safety_t msg;
    uint8_t frame[PLATFORM_HEARTBEAT_SAFETY_LENGTH];
    cornering_output_t out = cornering_output();
    bool ok = false;

    (void)memset(&msg, 0, sizeof msg);
    msg.node_mode = node_health_mode(&app.health, out.source == CORNERING_SOURCE_EKF);
    msg.error_count = node_health_error_count(&app.health);
    msg.uptime = node_health_uptime(&app.health);
    /* A failed pack sends nothing: the M7 then sees the 0x082 timeout. */
    if ((platform_heartbeat_safety_pack_checked(frame, &msg, sizeof frame) == (int)sizeof frame) &&
        vehicle_e2e_is_valid(platform_heartbeat_safety_e2e_protect(frame, sizeof frame, &app.hb_tx)) &&
        mailbox_offer((uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_INDEX, frame, (uint8_t)sizeof frame)) {
        ok = true;
        node_health_frame_sent(&app.health);
        app.status.hb_sent++;
        app.status.last_node_mode = msg.node_mode;
    }
    if (!ok) {
        app.pending_events |= NODE_FAULT_OUTPUT;
        app.status.output_faults++;
    }
}

void safety_app_step(uint32_t now_ms)
{
    mailbox_health_t mh;
    uint32_t level = 0u;
    uint32_t events;

    if (!app.initialised) {
        return;
    }
    /* 1-2: reads, then the cornering decision (first WCET group). */
    mailbox_step(now_ms);
    cornering_step(now_ms);

    /* 5 (before 4, so the frames carry this pass's health): own faults. */
    events = app.pending_events;
    app.pending_events = 0u;
    if (!app.first_pass && ((uint32_t)(now_ms - app.last_pass_ms) > SAFETY_APP_PASS_MAX_MS)) {
        events |= NODE_FAULT_PASS_GAP;
        app.status.pass_gap_faults++;
    }
    app.first_pass = false;
    app.last_pass_ms = now_ms;
    mh = mailbox_health(now_ms);
    if ((mh.peer == MAILBOX_PEER_MISMATCH) ||
        ((mh.peer == MAILBOX_PEER_NEVER_MATCHED) && (mh.peer_wait_ms > SAFETY_APP_PEER_BOOT_BUDGET_MS))) {
        level |= NODE_FAULT_PEER; /* "mailbox peer missing or wrong image", never rt-core */
    }
    node_health_pass(&app.health, level, events, now_ms);

    /* 4: outputs on their cycle grids. */
    if (due(&app.warn_due_ms, PLATFORM_SAFETY_WARNING_STATUS_CYCLE_TIME_MS, now_ms)) {
        offer_warning();
    }
    if (due(&app.hb_due_ms, PLATFORM_HEARTBEAT_SAFETY_CYCLE_TIME_MS, now_ms)) {
        offer_heartbeat();
    }

    node_health_pass_done(&app.health);
    app.status.level_faults = level;
    app.status.passes++;
    app.status.error_count = node_health_error_count(&app.health);
    app.status.rt_core_inputs_lost = cornering_status().rt_core_inputs_lost;
}

safety_app_status_t safety_app_status(void)
{
    return app.status;
}
