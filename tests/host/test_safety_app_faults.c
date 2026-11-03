/*
 * Fault injection into the output paths of app/safety_app (ISSUES E-29 (1), safety-reviewer
 * MINOR-5 of safety-node#10): the checked pack, the E2E protect and mailbox_offer() of
 * 0x023 and 0x082 cannot fail through the public API, so this test builds its own copy of
 * src/app/safety_app.c with those five calls renamed to the fi_* seams below (compile
 * definitions in tests/host/CMakeLists.txt). Each seam calls the real function unless a
 * failure is armed. A link-time seam (-Wl,--wrap) was the first plan; the renamed copy
 * works with every linker, so the test also runs on macOS.
 *
 * Covered (D-101 items 1-2, safety_app.c offer_warning/offer_heartbeat):
 *   - 0x023 pack failure: the documented all-raw-0 frame (UNAVAILABLE/NONE) still goes out
 *     with a valid E2E; output fault counted;
 *   - 0x023 protect or offer failure: no frame that cycle; output fault counted;
 *   - 0x082 pack or protect failure: no 0x082 that cycle and no counter gap (the next one is
 *     OK); offer failure: signed but not offered, so the M7 rejects the next 0x082 as
 *     WRONG_SEQUENCE; output fault counted;
 *   - every output fault is an event fault: ERROR_COUNT + 1 and exactly one DEGRADED 0x082
 *     (the next one sent), then NORMAL again.
 * The sink records the E2E status of every frame and asserts it wherever the content is
 * checked. E2E note (generated receiver): WRONG_SEQUENCE does not refresh the receive time,
 * so after one signed-but-lost 0x082 the M7 never accepts the DEGRADED frame, and the next
 * OK lands exactly on the 300 ms timeout (zero margin; with read jitter it is INITIAL, also
 * not valid). Fail-safe; recorded in ISSUES E-29.
 */
#include "app/safety_app.h"

#include "m7_model.h"
#include "services/mailbox.h"
#include "platform_e2e.h"

#include <unity.h>

/* ------------------------------------------------------------------ the seams */

int fi_warn_pack(uint8_t* dst_p, const struct platform_safety_warning_status_t* src_p, size_t size);
int fi_hbs_pack(uint8_t* dst_p, const struct platform_heartbeat_safety_t* src_p, size_t size);
vehicle_e2e_status_t fi_warn_protect(uint8_t* data, size_t size, vehicle_e2e_tx_state_t* state);
vehicle_e2e_status_t fi_hbs_protect(uint8_t* data, size_t size, vehicle_e2e_tx_state_t* state);
bool fi_mailbox_offer(uint8_t slot, const uint8_t* data, uint8_t len);

/* The seams must keep the real signatures (the renamed copy of safety_app.c calls them
 * through its own prototypes, in another translation unit): one typed pointer each. */
static int (*const k_chk_warn_pack[2])(uint8_t*, const struct platform_safety_warning_status_t*, size_t) = {
    platform_safety_warning_status_pack_checked, fi_warn_pack};
static int (*const k_chk_hbs_pack[2])(uint8_t*, const struct platform_heartbeat_safety_t*, size_t) = {
    platform_heartbeat_safety_pack_checked, fi_hbs_pack};
static vehicle_e2e_status_t (*const k_chk_warn_protect[2])(uint8_t*, size_t, vehicle_e2e_tx_state_t*) = {
    platform_safety_warning_status_e2e_protect, fi_warn_protect};
static vehicle_e2e_status_t (*const k_chk_hbs_protect[2])(uint8_t*, size_t, vehicle_e2e_tx_state_t*) = {
    platform_heartbeat_safety_e2e_protect, fi_hbs_protect};
static bool (*const k_chk_offer[2])(uint8_t, const uint8_t*, uint8_t) = {mailbox_offer, fi_mailbox_offer};

typedef enum { FI_WARN_PACK, FI_HBS_PACK, FI_WARN_PROTECT, FI_HBS_PROTECT, FI_WARN_OFFER, FI_HBS_OFFER, FI_COUNT } fi_point_t;

static uint32_t fi_armed[FI_COUNT]; /* calls still to fail */
static uint32_t fi_hits[FI_COUNT];  /* injected failures */

static bool fi_fail(fi_point_t p)
{
    if (fi_armed[p] > 0u) {
        fi_armed[p]--;
        fi_hits[p]++;
        return true;
    }
    return false;
}

int fi_warn_pack(uint8_t* dst_p, const struct platform_safety_warning_status_t* src_p, size_t size)
{
    if (fi_fail(FI_WARN_PACK)) {
        (void)memset(dst_p, 0xA5, size); /* garbage: the caller must not send it */
        return -1;
    }
    return platform_safety_warning_status_pack_checked(dst_p, src_p, size);
}

int fi_hbs_pack(uint8_t* dst_p, const struct platform_heartbeat_safety_t* src_p, size_t size)
{
    if (fi_fail(FI_HBS_PACK)) {
        return -1;
    }
    return platform_heartbeat_safety_pack_checked(dst_p, src_p, size);
}

vehicle_e2e_status_t fi_warn_protect(uint8_t* data, size_t size, vehicle_e2e_tx_state_t* state)
{
    if (fi_fail(FI_WARN_PROTECT)) {
        return VEHICLE_E2E_BAD_ARGUMENT; /* nothing signed, counter not advanced */
    }
    return platform_safety_warning_status_e2e_protect(data, size, state);
}

vehicle_e2e_status_t fi_hbs_protect(uint8_t* data, size_t size, vehicle_e2e_tx_state_t* state)
{
    if (fi_fail(FI_HBS_PROTECT)) {
        return VEHICLE_E2E_BAD_ARGUMENT;
    }
    return platform_heartbeat_safety_e2e_protect(data, size, state);
}

bool fi_mailbox_offer(uint8_t slot, const uint8_t* data, uint8_t len)
{
    bool fail = false;
    if (slot == (uint8_t)PLATFORM_MAILBOX_SLOT_SAFETY_WARNING_STATUS_INDEX) {
        fail = fi_fail(FI_WARN_OFFER);
    } else if (slot == (uint8_t)PLATFORM_MAILBOX_SLOT_HEARTBEAT_SAFETY_INDEX) {
        fail = fi_fail(FI_HBS_OFFER);
    } else {
        /* another slot: never injected */
    }
    if (fail) {
        return false;
    }
    return mailbox_offer(slot, data, len);
}

/* ------------------------------------------------------------- the M7 side */

typedef struct {
    mailbox_reader_t reader;
    vehicle_e2e_rx_state_t rx;
    uint32_t frames;
    uint32_t ok;
    uint32_t initial; /* boot only in these tests */
    uint32_t wrong_sequence;
    uint32_t other_bad;
    vehicle_e2e_status_t last_st;
    uint8_t last[FB_LEN];
} m7_sink_t;

static m7_rt_core_t rt;
static m7_sink_t warn;
static m7_sink_t hbs;
static uint32_t now;
static uint32_t next_pass;
static uint32_t steady_hb_frames; /* 0x082 frames read by STEADY_MS (the last one: 400) */

static void sink_init(m7_sink_t* s, uint8_t len)
{
    (void)memset(s, 0, sizeof *s);
    mailbox_reader_init(&s->reader, len);
    vehicle_e2e_rx_init(&s->rx);
}

static void sink_poll(m7_sink_t* s, uint32_t slot, uint16_t id, uint8_t max_delta, uint32_t timeout)
{
    uint8_t d[MAILBOX_PAYLOAD_BYTES];
    if (m7_read(slot, &s->reader, now, d)) {
        const vehicle_e2e_status_t st = vehicle_e2e_check(id, max_delta, timeout, d, FB_LEN, &s->rx, now);
        s->frames++;
        if (st == VEHICLE_E2E_OK) {
            s->ok++;
        } else if (st == VEHICLE_E2E_INITIAL) {
            s->initial++;
        } else if (st == VEHICLE_E2E_WRONG_SEQUENCE) {
            s->wrong_sequence++;
        } else {
            s->other_bad++;
        }
        s->last_st = st;
        (void)memcpy(s->last, d, FB_LEN);
    }
}

/* The last frame was accepted (OK) and carries this NODE_MODE / ERROR_COUNT. */
static void expect_hbs_ok(uint32_t mode, uint32_t errors);

static uint32_t hbs_mode(void)
{
    return fb_tx_get(hbs.last, fb_hbs_mode_pack, 8u);
}

static uint32_t hbs_errors(void)
{
    return fb_tx_get(hbs.last, fb_hbs_errors_pack, 8u);
}

static void expect_hbs_ok(uint32_t mode, uint32_t errors)
{
    TEST_ASSERT_EQUAL(VEHICLE_E2E_OK, hbs.last_st);
    TEST_ASSERT_EQUAL_UINT32(mode, hbs_mode());
    TEST_ASSERT_EQUAL_UINT32(errors, hbs_errors());
}

static void run_to(uint32_t t_end)
{
    while ((uint32_t)(t_end - now) - 1u < 0x80000000u) { /* now before t_end, wrap-safe */
        now++;
        m7_rt_core_tick(&rt, now);
        if ((int32_t)(now - next_pass) >= 0) {
            safety_app_step(now);
            next_pass = now + 2u; /* the D-101 item 2 nominal period */
        }
        sink_poll(&warn, M7_WARN, PLATFORM_SAFETY_WARNING_STATUS_E2E_DATA_ID,
                  (uint8_t)PLATFORM_SAFETY_WARNING_STATUS_E2E_MAX_DELTA_COUNTER,
                  PLATFORM_SAFETY_WARNING_STATUS_E2E_TIMEOUT_MS);
        sink_poll(&hbs, M7_HBS, PLATFORM_HEARTBEAT_SAFETY_E2E_DATA_ID,
                  (uint8_t)PLATFORM_HEARTBEAT_SAFETY_E2E_MAX_DELTA_COUNTER,
                  PLATFORM_HEARTBEAT_SAFETY_E2E_TIMEOUT_MS);
    }
}

/* Booted, on EKF and NORMAL (EKF from 380 ms, D-101 item 4; the 0x082 built at 400). */
#define STEADY_MS 450u

static void start_steady(void)
{
    TEST_ASSERT_TRUE((k_chk_warn_pack[1] != NULL) && (k_chk_hbs_pack[1] != NULL) && (k_chk_warn_protect[1] != NULL) &&
                     (k_chk_hbs_protect[1] != NULL) && (k_chk_offer[1] != NULL));
    (void)memset(fi_armed, 0, sizeof fi_armed);
    (void)memset(fi_hits, 0, sizeof fi_hits);
    m7_reset(true);
    m7_rt_core_init(&rt);
    rt.lean_raw = m7_lean_raw(5.0f);
    sink_init(&warn, (uint8_t)PLATFORM_SAFETY_WARNING_STATUS_LENGTH);
    sink_init(&hbs, (uint8_t)PLATFORM_HEARTBEAT_SAFETY_LENGTH);
    now = 0u;
    next_pass = 2u;
    TEST_ASSERT_TRUE(safety_app_init(&m7_region, 0u));
    run_to(STEADY_MS);
    expect_hbs_ok(PLATFORM_HEARTBEAT_SAFETY_NODE_MODE_NORMAL_CHOICE, 0u);
    steady_hb_frames = hbs.frames;
    TEST_ASSERT_EQUAL_UINT32(0u, safety_app_status().output_faults);
    TEST_ASSERT_EQUAL_UINT32(0u, warn.wrong_sequence + warn.other_bad + hbs.wrong_sequence + hbs.other_bad);
}

typedef enum {
    HB_SENT,     /* the fault was on 0x023: the 0x082 of 500 goes out */
    HB_UNSIGNED, /* the 0x082 of 500 failed before signing: nothing, no counter gap */
    HB_SIGNED    /* the 0x082 of 500 was signed but never offered: a counter gap */
} hb_fate_t;

/* After an output fault in [STEADY_MS, 500]: ERROR_COUNT + 1 and one DEGRADED 0x082 (the
 * next one sent: 500, or 600 when the 0x082 of 500 is the failed one), then NORMAL. */
static void expect_one_degraded_heartbeat(hb_fate_t fate)
{
    const uint32_t hb_frames_before = steady_hb_frames;
    run_to(550u); /* the 0x082 of 500 */
    TEST_ASSERT_EQUAL_UINT32(1u, safety_app_status().output_faults);
    TEST_ASSERT_EQUAL_UINT8(1u, safety_app_status().error_count);
    if (fate == HB_SENT) {
        TEST_ASSERT_EQUAL_UINT32(hb_frames_before + 1u, hbs.frames);
        expect_hbs_ok(PLATFORM_HEARTBEAT_SAFETY_NODE_MODE_DEGRADED_CHOICE, 1u);
        run_to(650u); /* one DEGRADED frame only (D-101 item 1) */
        expect_hbs_ok(PLATFORM_HEARTBEAT_SAFETY_NODE_MODE_NORMAL_CHOICE, 1u);
    } else {
        TEST_ASSERT_EQUAL_UINT32(hb_frames_before, hbs.frames); /* nothing at 500 */
        run_to(650u); /* the 0x082 of 600 carries the held DEGRADED */
        TEST_ASSERT_EQUAL_UINT32(PLATFORM_HEARTBEAT_SAFETY_NODE_MODE_DEGRADED_CHOICE, hbs_mode());
        TEST_ASSERT_EQUAL_UINT32(1u, hbs_errors());
        if (fate == HB_UNSIGNED) {
            TEST_ASSERT_EQUAL(VEHICLE_E2E_OK, hbs.last_st); /* accepted */
            TEST_ASSERT_EQUAL_UINT32(0u, hbs.wrong_sequence);
        } else {
            /* rejected: the M7 never accepts this DEGRADED frame (E2E note above) */
            TEST_ASSERT_EQUAL(VEHICLE_E2E_WRONG_SEQUENCE, hbs.last_st);
            TEST_ASSERT_EQUAL_UINT32(1u, hbs.wrong_sequence);
        }
    }
    run_to(750u); /* the 0x082 of 700; after HB_SIGNED it is OK exactly at the timeout */
    expect_hbs_ok(PLATFORM_HEARTBEAT_SAFETY_NODE_MODE_NORMAL_CHOICE, 1u);
    TEST_ASSERT_EQUAL_UINT32(0u, warn.other_bad + hbs.other_bad);
    TEST_ASSERT_EQUAL_UINT32(1u, warn.initial); /* the boot frame only: no resync */
    TEST_ASSERT_EQUAL_UINT32(1u, hbs.initial);
}

void setUp(void) {}
void tearDown(void) {}

static void test_warning_pack_failure_sends_the_raw_zero_frame(void)
{
    start_steady();
    const uint32_t sent = safety_app_status().warn_sent;
    const uint32_t frames = warn.frames;
    fi_armed[FI_WARN_PACK] = 1u;
    run_to(470u); /* the 0x023 of 460 */
    TEST_ASSERT_EQUAL_UINT32(1u, fi_hits[FI_WARN_PACK]);
    TEST_ASSERT_EQUAL_UINT32(frames + 1u, warn.frames);
    TEST_ASSERT_EQUAL(VEHICLE_E2E_OK, warn.last_st); /* signed and in sequence: accepted */
    for (uint32_t i = 0u; i < FB_LEN; i++) {
        if ((i != (uint32_t)VEHICLE_E2E_CRC_OFFSET) && (i != (uint32_t)VEHICLE_E2E_COUNTER_OFFSET)) {
            TEST_ASSERT_EQUAL_HEX8(0u, warn.last[i]); /* all raw 0 = UNAVAILABLE/NONE/UNKNOWN */
        }
    }
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_SAFETY_WARNING_STATUS_CORNERING_LEVEL_UNAVAILABLE_CHOICE,
                             fb_tx_get(warn.last, fb_warn_level_pack, 8u));
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_SAFETY_WARNING_STATUS_CORNERING_SOURCE_NONE_CHOICE,
                             fb_tx_get(warn.last, fb_warn_source_pack, 8u));
    TEST_ASSERT_EQUAL_UINT32(sent + 1u, safety_app_status().warn_sent); /* the fallback frame went */
    expect_one_degraded_heartbeat(HB_SENT);
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_SAFETY_WARNING_STATUS_CORNERING_SOURCE_EKF_CHOICE,
                             fb_tx_get(warn.last, fb_warn_source_pack, 8u)); /* the next ones are normal */
}

static void test_warning_protect_failure_sends_nothing_that_cycle(void)
{
    start_steady();
    const uint32_t sent = safety_app_status().warn_sent;
    const uint32_t frames = warn.frames;
    fi_armed[FI_WARN_PROTECT] = 1u;
    run_to(470u);
    TEST_ASSERT_EQUAL_UINT32(1u, fi_hits[FI_WARN_PROTECT]);
    TEST_ASSERT_EQUAL_UINT32(frames, warn.frames);
    TEST_ASSERT_EQUAL_UINT32(sent, safety_app_status().warn_sent);
    run_to(490u); /* the 0x023 of 480: the counter never advanced, so no gap */
    TEST_ASSERT_EQUAL_UINT32(frames + 1u, warn.frames);
    TEST_ASSERT_EQUAL(VEHICLE_E2E_OK, warn.last_st);
    expect_one_degraded_heartbeat(HB_SENT);
}

static void test_warning_offer_failure_is_one_counted_gap(void)
{
    start_steady();
    const uint32_t frames = warn.frames;
    fi_armed[FI_WARN_OFFER] = 1u;
    run_to(470u);
    TEST_ASSERT_EQUAL_UINT32(1u, fi_hits[FI_WARN_OFFER]);
    TEST_ASSERT_EQUAL_UINT32(frames, warn.frames);
    run_to(490u); /* signed but never offered: the 0x023 of 480 is WRONG_SEQUENCE */
    TEST_ASSERT_EQUAL_UINT32(frames + 1u, warn.frames);
    TEST_ASSERT_EQUAL_UINT32(1u, warn.wrong_sequence);
    TEST_ASSERT_EQUAL(VEHICLE_E2E_WRONG_SEQUENCE, warn.last_st);
    run_to(510u); /* the 0x023 of 500: OK at exactly the 60 ms timeout */
    TEST_ASSERT_EQUAL(VEHICLE_E2E_OK, warn.last_st);
    expect_one_degraded_heartbeat(HB_SENT);
}

static void test_heartbeat_pack_failure_sends_no_heartbeat(void)
{
    start_steady();
    fi_armed[FI_HBS_PACK] = 1u;
    expect_one_degraded_heartbeat(HB_UNSIGNED);
    TEST_ASSERT_EQUAL_UINT32(1u, fi_hits[FI_HBS_PACK]);
    TEST_ASSERT_EQUAL_UINT32(0u, hbs.wrong_sequence); /* never signed: no gap */
}

static void test_heartbeat_protect_failure_sends_no_heartbeat(void)
{
    start_steady();
    fi_armed[FI_HBS_PROTECT] = 1u;
    expect_one_degraded_heartbeat(HB_UNSIGNED);
    TEST_ASSERT_EQUAL_UINT32(1u, fi_hits[FI_HBS_PROTECT]);
    TEST_ASSERT_EQUAL_UINT32(0u, hbs.wrong_sequence);
}

static void test_heartbeat_offer_failure_is_one_counted_gap(void)
{
    start_steady();
    fi_armed[FI_HBS_OFFER] = 1u;
    expect_one_degraded_heartbeat(HB_SIGNED);
    TEST_ASSERT_EQUAL_UINT32(1u, fi_hits[FI_HBS_OFFER]);
    TEST_ASSERT_EQUAL_UINT32(1u, hbs.wrong_sequence); /* signed, never offered: the 0x082 of 600 */
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_warning_pack_failure_sends_the_raw_zero_frame);
    RUN_TEST(test_warning_protect_failure_sends_nothing_that_cycle);
    RUN_TEST(test_warning_offer_failure_is_one_counted_gap);
    RUN_TEST(test_heartbeat_pack_failure_sends_no_heartbeat);
    RUN_TEST(test_heartbeat_protect_failure_sends_no_heartbeat);
    RUN_TEST(test_heartbeat_offer_failure_is_one_counted_gap);
    return UNITY_END();
}
