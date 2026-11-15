/*
 * End-to-end host test of one super-loop pass (app/safety_app) on the platform-bus model
 * (D-118): an rt-core model sends 0x081, 0x022 and 0x020 into the CAN host HAL; the node
 * pass (every 2 ms, the D-101 item 2 nominal period) decides and transmits 0x023 / 0x082;
 * a sink on the bus reads them back and checks them with the generated E2E core. Covers
 * D-101 items 1-4 on the wire: INIT first, then DEGRADED while UNAVAILABLE, NORMAL on EKF;
 * own-fault onsets (pass gap, refused TX) in ERROR_COUNT; the loss of rt-core's inputs
 * never counted; a node that boots before rt-core; a masquerading second sender of 0x020;
 * 0x120 never influencing the outputs.
 */
#include "app/safety_app.h"

#include "bus_model.h"
#include "features/cornering/cornering.h"
#include "platform_e2e.h"
#include "services/platform_link.h"

#include <unity.h>

static rt_core_t rt;
static bus_sink_t warn;
static bus_sink_t hbs;
static uint32_t now;
static uint32_t next_pass;
static uint32_t pass_ms;
static uint8_t first_mode;
static bool first_mode_seen;
/* Extra senders on the bus, run_to() drives them: a second sender of 0x020 (masquerade, its own
 * counter, the correct Data ID: the worst case) and a 0x120 SafetyRequest stream. */
static bool masq_on;
static vehicle_e2e_tx_state_t masq_tx;
static uint32_t masq_pass_ekf; /* passes after the masquerade started that still used EKF */
static uint32_t masq_pass_n;
static bool masq_started; /* the first fake frame was injected */
typedef enum { REQ_OFF, REQ_VALID, REQ_CORRUPT, REQ_WRONG_DLC } req_mode_t;
static req_mode_t req_mode;
static vehicle_e2e_tx_state_t req_tx;

static uint32_t hbs_mode(void)
{
    return fb_tx_get(hbs.last, fb_hbs_mode_pack, 8u);
}
static uint32_t hbs_errors(void)
{
    return fb_tx_get(hbs.last, fb_hbs_errors_pack, 8u);
}
static uint32_t warn_level(void)
{
    return fb_tx_get(warn.last, fb_warn_level_pack, 8u);
}
static uint32_t warn_source(void)
{
    return fb_tx_get(warn.last, fb_warn_source_pack, 8u);
}

static void run_to(uint32_t t_end)
{
    while ((uint32_t)(t_end - now) - 1u < 0x80000000u) { /* now before t_end, wrap-safe */
        now++;
        rt_core_tick(&rt, now);
        if (masq_on && ((now % PLATFORM_EKF_LEAN_CYCLE_TIME_MS) == 0u)) {
            uint8_t f[FB_LEN];
            fb_lean(f, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_ESTIMATED_CHOICE, bus_lean_raw(60.0f), &masq_tx);
            bus_send(BUS_LEAN, f);
            masq_started = true;
        }
        if ((req_mode != REQ_OFF) && ((now % PLATFORM_SAFETY_REQUEST_CYCLE_TIME_MS) == 0u)) {
            uint8_t f[FB_LEN] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu}; /* every request bit set */
            (void)vehicle_e2e_protect(PLATFORM_SAFETY_REQUEST_E2E_DATA_ID, f, FB_LEN, &req_tx);
            if (req_mode == REQ_CORRUPT) {
                f[0] ^= 0x55u;
            }
            bus_send_dlc(BUS_REQ, f, (req_mode == REQ_WRONG_DLC) ? 5u : (uint8_t)FB_LEN);
        }
        if ((int32_t)(now - next_pass) >= 0) {
            safety_app_step(now);
            next_pass = now + pass_ms;
            if (masq_on && masq_started) {
                masq_pass_n++;
                if (cornering_output().source == CORNERING_SOURCE_EKF) {
                    masq_pass_ekf++;
                }
            }
        }
        bus_drain_tx(&warn, &hbs, now);
        if (!first_mode_seen && (hbs.frames > 0u)) {
            first_mode_seen = true;
            first_mode = (uint8_t)hbs_mode();
        }
    }
}

static void start_at(uint32_t t0, uint32_t first_pass_after)
{
    bus_reset();
    rt_core_init(&rt);
    rt.lean_raw = bus_lean_raw(5.0f);
    bus_sink_init(&warn);
    bus_sink_init(&hbs);
    now = t0;
    pass_ms = 2u;
    next_pass = t0 + first_pass_after;
    first_mode_seen = false;
    masq_on = false;
    masq_pass_ekf = 0u;
    masq_pass_n = 0u;
    masq_started = false;
    req_mode = REQ_OFF;
    vehicle_e2e_tx_init(&masq_tx);
    masq_tx.counter = 7u;
    vehicle_e2e_tx_init(&req_tx);
    TEST_ASSERT_TRUE(safety_app_init(t0));
}

static void start(void)
{
    start_at(0u, 2u);
}

void setUp(void) {}
void tearDown(void) {}

static void test_init_hal_failure_is_inert(void)
{
    bus_reset();
    can_host_set_init_fails(true);
    TEST_ASSERT_FALSE(safety_app_init(0u));
    safety_app_step(2u);
    TEST_ASSERT_FALSE(safety_app_status().initialised);
    TEST_ASSERT_EQUAL_UINT32(0u, safety_app_status().passes);
    TEST_ASSERT_EQUAL_UINT32(0u, can_host_tx_accepted());
}

static void test_boot_init_degraded_then_normal(void)
{
    start();
    run_to(50u);
    TEST_ASSERT_TRUE(first_mode_seen);
    TEST_ASSERT_EQUAL_UINT8(PLATFORM_HEARTBEAT_SAFETY_NODE_MODE_INIT_CHOICE, first_mode); /* architecture-guard MINOR-4 */
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_SAFETY_WARNING_STATUS_CORNERING_LEVEL_UNAVAILABLE_CHOICE, warn_level());
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_SAFETY_WARNING_STATUS_CORNERING_SOURCE_NONE_CHOICE, warn_source());
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_SAFETY_WARNING_STATUS_BSM_LEFT_UNAVAILABLE_CHOICE,
                             fb_tx_get(warn.last, fb_warn_bsm_left_pack, 8u));
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_SAFETY_WARNING_STATUS_BSM_RIGHT_UNAVAILABLE_CHOICE,
                             fb_tx_get(warn.last, fb_warn_bsm_right_pack, 8u));
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_SAFETY_WARNING_STATUS_IMMOBILIZER_STATE_UNKNOWN_CHOICE,
                             fb_tx_get(warn.last, fb_warn_immobilizer_pack, 8u));
    run_to(150u);
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_HEARTBEAT_SAFETY_NODE_MODE_DEGRADED_CHOICE, hbs_mode());
    run_to(450u); /* EKF from 380 (D-101 item 4), the 0x082 built at 400 */
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_SAFETY_WARNING_STATUS_CORNERING_SOURCE_EKF_CHOICE, warn_source());
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_SAFETY_WARNING_STATUS_CORNERING_LEVEL_GREEN_CHOICE, warn_level());
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_HEARTBEAT_SAFETY_NODE_MODE_NORMAL_CHOICE, hbs_mode());
    TEST_ASSERT_EQUAL_UINT32(0u, hbs_errors());
}

static void test_cadence_and_e2e_on_the_bus(void)
{
    start();
    run_to(2003u);
    /* 0x023 every 20 ms, 0x082 every 100 ms, from the first pass at 2 ms. */
    TEST_ASSERT_EQUAL_UINT32(101u, safety_app_status().warn_sent);
    TEST_ASSERT_EQUAL_UINT32(21u, safety_app_status().hb_sent);
    TEST_ASSERT_EQUAL_UINT32(101u, warn.frames);
    TEST_ASSERT_EQUAL_UINT32(21u, hbs.frames);
    TEST_ASSERT_EQUAL_UINT32(0u, warn.bad);
    TEST_ASSERT_EQUAL_UINT32(0u, hbs.bad);
    TEST_ASSERT_EQUAL_UINT32(1u, warn.initial);
    TEST_ASSERT_EQUAL_UINT32(1u, hbs.initial);
    TEST_ASSERT_EQUAL_UINT32(2u, fb_tx_get(hbs.last, fb_hbs_uptime_pack, 16u));
    TEST_ASSERT_EQUAL_UINT32(0u, safety_app_status().output_faults);
}

static void test_pass_gap_is_an_own_fault(void)
{
    start();
    run_to(500u);
    next_pass = now + 5u; /* a 5 ms gap is within the bound */
    run_to(520u);
    TEST_ASSERT_EQUAL_UINT32(0u, safety_app_status().pass_gap_faults);
    next_pass = now + 6u; /* one gap of 6 ms */
    run_to(530u);
    TEST_ASSERT_EQUAL_UINT32(1u, safety_app_status().pass_gap_faults);
    TEST_ASSERT_EQUAL_UINT8(1u, safety_app_status().error_count);
    run_to(650u); /* the 0x082 built at 600 carries it */
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_HEARTBEAT_SAFETY_NODE_MODE_DEGRADED_CHOICE, hbs_mode());
    TEST_ASSERT_EQUAL_UINT32(1u, hbs_errors());
    run_to(750u); /* one frame only (D-101 item 1) */
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_HEARTBEAT_SAFETY_NODE_MODE_NORMAL_CHOICE, hbs_mode());
    TEST_ASSERT_EQUAL_UINT32(1u, hbs_errors());
}

static void test_rt_core_inputs_lost_is_not_an_own_fault(void)
{
    start();
    run_to(500u);
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_HEARTBEAT_SAFETY_NODE_MODE_NORMAL_CHOICE, hbs_mode());
    TEST_ASSERT_FALSE(safety_app_status().rt_core_inputs_lost);
    rt.lean_on = false; /* a platform-bus fault looks the same (D-118) */
    rt.mu_on = false;
    rt.hb_on = false;
    run_to(1050u);
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_SAFETY_WARNING_STATUS_CORNERING_LEVEL_UNAVAILABLE_CHOICE, warn_level());
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_SAFETY_WARNING_STATUS_CORNERING_SOURCE_NONE_CHOICE, warn_source());
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_HEARTBEAT_SAFETY_NODE_MODE_DEGRADED_CHOICE, hbs_mode());
    TEST_ASSERT_EQUAL_UINT32(0u, hbs_errors());
    TEST_ASSERT_EQUAL_UINT8(0u, safety_app_status().error_count);
    TEST_ASSERT_TRUE(safety_app_status().rt_core_inputs_lost);
    TEST_ASSERT_EQUAL_UINT32(0u, safety_app_status().level_faults);
}

static void test_first_pass_only_sets_the_baseline(void)
{
    /* safety-reviewer MINOR-2: board init between safety_app_init() and the first pass. */
    start_at(0u, 50u);
    run_to(300u);
    TEST_ASSERT_EQUAL_UINT32(0u, safety_app_status().pass_gap_faults);
    TEST_ASSERT_EQUAL_UINT8(0u, safety_app_status().error_count);
}

static void test_runs_across_the_ms_wrap(void)
{
    /* 32-bit ms wrap about 0.5 s after start: due(), the pass gap, the E2E timeouts on both
     * cores and the uptime all use wrap-safe deltas. */
    const uint32_t t0 = UINT32_MAX - 499u;
    start_at(t0, 2u);
    run_to(t0 + 1500u);
    TEST_ASSERT_EQUAL_UINT32(0u, safety_app_status().pass_gap_faults);
    TEST_ASSERT_EQUAL_UINT32(0u, warn.bad);
    TEST_ASSERT_EQUAL_UINT32(0u, hbs.bad);
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_SAFETY_WARNING_STATUS_CORNERING_SOURCE_EKF_CHOICE, warn_source());
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_HEARTBEAT_SAFETY_NODE_MODE_NORMAL_CHOICE, hbs_mode());
    TEST_ASSERT_EQUAL_UINT32(1u, fb_tx_get(hbs.last, fb_hbs_uptime_pack, 16u));
}

static void test_late_pass_resyncs_the_grid(void)
{
    start();
    run_to(100u);
    next_pass = now + 50u; /* a 50 ms stall: one gap fault, no burst of 0x023 */
    run_to(151u);
    TEST_ASSERT_EQUAL_UINT32(1u, safety_app_status().pass_gap_faults);
    run_to(400u);
    TEST_ASSERT_EQUAL_UINT32(0u, warn.bad);
}

/* E-29 (7), safety-reviewer test 2 of safety-node#11, on the bus: the node boots first and
 * rt-core appears later (D-118: no peer handshake, the node never waits for rt-core). The
 * node transmits from its first cycle (INITIAL, then OK at the receiver), the cornering
 * source stays NONE and 0x082 DEGRADED until rt-core's frames are usable; no own fault. */
static void test_boot_node_first_then_rt_core_appears(void)
{
    start();
    rt.lean_on = false;
    rt.mu_on = false;
    rt.hb_on = false;
    run_to(500u);
    TEST_ASSERT_TRUE(warn.frames > 20u);
    TEST_ASSERT_TRUE(hbs.frames > 3u);
    TEST_ASSERT_EQUAL_UINT32(0u, warn.bad);
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_HEARTBEAT_SAFETY_NODE_MODE_DEGRADED_CHOICE, hbs_mode());
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_SAFETY_WARNING_STATUS_CORNERING_LEVEL_UNAVAILABLE_CHOICE, warn_level());
    rt.lean_on = true;
    rt.mu_on = true;
    rt.hb_on = true;
    run_to(1000u);
    TEST_ASSERT_EQUAL_UINT32(1u, warn.initial);
    TEST_ASSERT_EQUAL_UINT32(1u, hbs.initial);
    TEST_ASSERT_EQUAL_UINT32(0u, warn.bad);
    TEST_ASSERT_EQUAL_UINT32(0u, hbs.bad);
    TEST_ASSERT_EQUAL_UINT32(0u, safety_app_status().output_faults);
    TEST_ASSERT_EQUAL_UINT8(0u, safety_app_status().error_count);
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_SAFETY_WARNING_STATUS_CORNERING_SOURCE_EKF_CHOICE, warn_source());
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_HEARTBEAT_SAFETY_NODE_MODE_NORMAL_CHOICE, hbs_mode());
}

/* A refused TX (full FIFO, bus-off) is an own output fault, counted once per refusal, never
 * retried: the 0x023 of 600 is refused, the next one shows the counter gap at the receiver
 * (WRONG_SEQUENCE, never valid), the one after is OK again. */
static void test_refused_send_is_counted_never_retried(void)
{
    start();
    run_to(594u);
    const uint32_t sent = safety_app_status().warn_sent;
    const uint32_t accepted = can_host_tx_accepted();
    can_host_refuse_next_sends(1u);
    run_to(600u); /* the 0x023 of 600 is the first send of this pass */
    TEST_ASSERT_EQUAL_UINT32(1u, can_host_tx_refused());
    TEST_ASSERT_EQUAL_UINT32(1u, safety_app_status().output_faults);
    TEST_ASSERT_EQUAL_UINT32(sent, safety_app_status().warn_sent);
    TEST_ASSERT_EQUAL_UINT32(1u, platform_link_health().tx_refused[PLATFORM_LINK_TX_SAFETY_WARNING_STATUS]);
    run_to(610u); /* no retry in the passes between the cycles */
    TEST_ASSERT_EQUAL_UINT32(1u, can_host_tx_refused());
    TEST_ASSERT_EQUAL_UINT32(accepted + 1u, can_host_tx_accepted()); /* the 0x082 of 600 only */
    run_to(630u);
    TEST_ASSERT_EQUAL(VEHICLE_E2E_WRONG_SEQUENCE, warn.last_st); /* the 0x023 of 620: the counter gap is visible */
    run_to(650u);
    TEST_ASSERT_EQUAL(VEHICLE_E2E_OK, warn.last_st); /* the 0x023 of 640: OK at exactly the 60 ms timeout */
}

/* safety-reviewer MINOR-2 of safety-node#11 on the bus: a pass gap ends in the pass at 600 and
 * a refused 0x082 of that cycle (it carries DEGRADED) is not sent; the next one is
 * WRONG_SEQUENCE at the receiver (counter gap), never silently valid. */
static void test_refused_heartbeat_is_a_visible_gap(void)
{
    start();
    run_to(594u);
    next_pass = 600u; /* a 6 ms gap: the own fault onset at 600 */
    run_to(599u);
    can_host_refuse_next_sends(2u); /* the 0x023 and the 0x082 of 600 */
    run_to(601u);
    TEST_ASSERT_EQUAL_UINT32(1u, safety_app_status().pass_gap_faults);
    TEST_ASSERT_EQUAL_UINT32(2u, can_host_tx_refused());
    TEST_ASSERT_EQUAL_UINT32(2u, safety_app_status().output_faults);
    const uint32_t hb_frames = hbs.frames;
    run_to(650u);
    TEST_ASSERT_EQUAL_UINT32(hb_frames, hbs.frames); /* nothing sent in the cycle */
    TEST_ASSERT_TRUE(warn.bad >= 1u);                /* the 0x023 gap */
    run_to(750u);
    TEST_ASSERT_EQUAL_UINT32(hb_frames + 1u, hbs.frames);
    TEST_ASSERT_EQUAL(VEHICLE_E2E_WRONG_SEQUENCE, hbs.last_st);
    run_to(850u);
    TEST_ASSERT_EQUAL(VEHICLE_E2E_OK, hbs.last_st);
}

/* D-118 masquerade: a second sender of 0x020 (here a RED-level 60 degrees with a valid CRC
 * and its own counter) interleaves with rt-core's. The link checks every frame, so the
 * counter sequence breaks (WRONG_SEQUENCE) in the very pass: from the first fake frame on the
 * EKF source is never used, the ring shows UNAVAILABLE / NONE (v1 has no own-IMU estimator,
 * D-100 item 2; with one it is the OWN_IMU source) and 0x082 reports DEGRADED. Never a
 * silent use of either sender's lean, never an own fault (it is an input problem). */
static void test_masquerade_on_0x020_is_never_used_and_degrades(void)
{
    start();
    run_to(500u);
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_HEARTBEAT_SAFETY_NODE_MODE_NORMAL_CHOICE, hbs_mode());
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_SAFETY_WARNING_STATUS_CORNERING_SOURCE_EKF_CHOICE, warn_source());
    masq_on = true;
    run_to(1200u);
    TEST_ASSERT_TRUE(masq_pass_n > 100u);
    TEST_ASSERT_EQUAL_UINT32(0u, masq_pass_ekf); /* not even in the pass of the first fake frame */
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_SAFETY_WARNING_STATUS_CORNERING_SOURCE_NONE_CHOICE, warn_source());
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_SAFETY_WARNING_STATUS_CORNERING_LEVEL_UNAVAILABLE_CHOICE, warn_level());
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_HEARTBEAT_SAFETY_NODE_MODE_DEGRADED_CHOICE, hbs_mode());
    TEST_ASSERT_EQUAL_UINT32(0u, hbs_errors());
    TEST_ASSERT_TRUE(safety_app_status().rt_core_inputs_lost);
    TEST_ASSERT_TRUE(platform_link_health().rx[PLATFORM_LINK_RX_EKF_LEAN].e2e_rejected > 10u);
    /* The masquerader stops: rt-core's stream alone resyncs and is used again after the
     * D-101 item 4 return rule (ten usable frames), never earlier. */
    masq_on = false;
    run_to(1260u);
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_SAFETY_WARNING_STATUS_CORNERING_SOURCE_NONE_CHOICE, warn_source());
    run_to(1700u);
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_SAFETY_WARNING_STATUS_CORNERING_SOURCE_EKF_CHOICE, warn_source());
}

/* D-072 item 7 (unchanged by D-118): 0x120 SafetyRequest (QM, immobilizer only) never influences
 * the cornering decision, the LEDs' source or 0x082, whatever it carries: valid with every
 * request bit set, corrupted, or with a wrong DLC. */
static void test_safety_request_never_influences_the_decision(void)
{
    static const req_mode_t modes[3] = {REQ_VALID, REQ_CORRUPT, REQ_WRONG_DLC};
    for (uint32_t m = 0u; m < 3u; m++) {
        start();
        req_mode = modes[m];
        run_to(450u);
        TEST_ASSERT_EQUAL_UINT32(PLATFORM_SAFETY_WARNING_STATUS_CORNERING_SOURCE_EKF_CHOICE, warn_source());
        TEST_ASSERT_EQUAL_UINT32(PLATFORM_SAFETY_WARNING_STATUS_CORNERING_LEVEL_GREEN_CHOICE, warn_level());
        TEST_ASSERT_EQUAL_UINT32(PLATFORM_HEARTBEAT_SAFETY_NODE_MODE_NORMAL_CHOICE, hbs_mode());
        TEST_ASSERT_EQUAL_UINT32(0u, hbs_errors());
        TEST_ASSERT_EQUAL_UINT32(0u, safety_app_status().output_faults);
        TEST_ASSERT_EQUAL_UINT32(PLATFORM_SAFETY_WARNING_STATUS_IMMOBILIZER_STATE_UNKNOWN_CHOICE,
                                 fb_tx_get(warn.last, fb_warn_immobilizer_pack, 8u)); /* no immobilizer module yet */
        TEST_ASSERT_TRUE(platform_link_health().rx[PLATFORM_LINK_RX_SAFETY_REQUEST].seen);
    }
    TEST_ASSERT_TRUE(platform_link_health().rx[PLATFORM_LINK_RX_SAFETY_REQUEST].bad_dlc > 0u);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_hal_failure_is_inert);
    RUN_TEST(test_boot_init_degraded_then_normal);
    RUN_TEST(test_cadence_and_e2e_on_the_bus);
    RUN_TEST(test_pass_gap_is_an_own_fault);
    RUN_TEST(test_rt_core_inputs_lost_is_not_an_own_fault);
    RUN_TEST(test_late_pass_resyncs_the_grid);
    RUN_TEST(test_first_pass_only_sets_the_baseline);
    RUN_TEST(test_runs_across_the_ms_wrap);
    RUN_TEST(test_boot_node_first_then_rt_core_appears);
    RUN_TEST(test_refused_send_is_counted_never_retried);
    RUN_TEST(test_refused_heartbeat_is_a_visible_gap);
    RUN_TEST(test_masquerade_on_0x020_is_never_used_and_degrades);
    RUN_TEST(test_safety_request_never_influences_the_decision);
    return UNITY_END();
}
