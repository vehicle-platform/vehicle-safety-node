/*
 * End-to-end host test of one super-loop pass (app/safety_app): an M7 model writes 0x081,
 * 0x022 and 0x020; the M4 pass (every 2 ms, the D-101 item 2 nominal period) decides and
 * offers 0x023 / 0x082; the M7 model reads them back through its D-093 reader and checks
 * them with the generated E2E core. Covers D-101 items 1-4 on the wire: INIT first, then
 * DEGRADED while UNAVAILABLE, NORMAL on EKF; own-fault onsets (pass gap, peer missing or
 * wrong image) in ERROR_COUNT; the loss of rt-core's inputs never counted.
 */
#include "app/safety_app.h"

#include "m7_model.h"
#include "platform_e2e.h"

#include <unity.h>

typedef struct {
    mailbox_reader_t reader;
    vehicle_e2e_rx_state_t rx;
    uint32_t frames;
    uint32_t bad; /* E2E results other than OK / INITIAL */
    uint32_t initial;
    uint8_t last[FB_LEN];
} m7_sink_t;

static m7_rt_core_t rt;
static m7_sink_t warn;
static m7_sink_t hbs;
static uint32_t now;
static uint32_t next_pass;
static uint32_t pass_ms;
static uint8_t first_mode;
static bool first_mode_seen;

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
        vehicle_e2e_status_t st = vehicle_e2e_check(id, max_delta, timeout, d, FB_LEN, &s->rx, now);
        s->frames++;
        if (st == VEHICLE_E2E_INITIAL) {
            s->initial++;
        } else if (st != VEHICLE_E2E_OK) {
            s->bad++;
        }
        (void)memcpy(s->last, d, FB_LEN);
    }
}

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
        m7_rt_core_tick(&rt, now);
        if ((int32_t)(now - next_pass) >= 0) {
            safety_app_step(now);
            next_pass = now + pass_ms;
        }
        sink_poll(&warn, M7_WARN, PLATFORM_SAFETY_WARNING_STATUS_E2E_DATA_ID,
                  (uint8_t)PLATFORM_SAFETY_WARNING_STATUS_E2E_MAX_DELTA_COUNTER,
                  PLATFORM_SAFETY_WARNING_STATUS_E2E_TIMEOUT_MS);
        sink_poll(&hbs, M7_HBS, PLATFORM_HEARTBEAT_SAFETY_E2E_DATA_ID,
                  (uint8_t)PLATFORM_HEARTBEAT_SAFETY_E2E_MAX_DELTA_COUNTER,
                  PLATFORM_HEARTBEAT_SAFETY_E2E_TIMEOUT_MS);
        if (!first_mode_seen && (hbs.frames > 0u)) {
            first_mode_seen = true;
            first_mode = (uint8_t)hbs_mode();
        }
    }
}

static void start_at(bool layout_ok, uint32_t t0, uint32_t first_pass_after)
{
    m7_reset(layout_ok);
    m7_rt_core_init(&rt);
    rt.lean_raw = m7_lean_raw(5.0f);
    sink_init(&warn, (uint8_t)PLATFORM_SAFETY_WARNING_STATUS_LENGTH);
    sink_init(&hbs, (uint8_t)PLATFORM_HEARTBEAT_SAFETY_LENGTH);
    now = t0;
    pass_ms = 2u;
    next_pass = t0 + first_pass_after;
    first_mode_seen = false;
    TEST_ASSERT_TRUE(safety_app_init(&m7_region, t0));
}

static void start(bool layout_ok)
{
    m7_reset(layout_ok);
    m7_rt_core_init(&rt);
    rt.lean_raw = m7_lean_raw(5.0f);
    sink_init(&warn, (uint8_t)PLATFORM_SAFETY_WARNING_STATUS_LENGTH);
    sink_init(&hbs, (uint8_t)PLATFORM_HEARTBEAT_SAFETY_LENGTH);
    now = 0u;
    pass_ms = 2u;
    next_pass = 2u;
    first_mode_seen = false;
    TEST_ASSERT_TRUE(safety_app_init(&m7_region, 0u));
}

void setUp(void) {}
void tearDown(void) {}

static void test_init_null_is_inert(void)
{
    m7_reset(true);
    TEST_ASSERT_FALSE(safety_app_init(NULL, 0u));
    safety_app_step(2u);
    TEST_ASSERT_FALSE(safety_app_status().initialised);
    TEST_ASSERT_EQUAL_UINT32(0u, safety_app_status().passes);
}

static void test_boot_init_degraded_then_normal(void)
{
    start(true);
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

static void test_cadence_and_e2e_on_the_m7_side(void)
{
    start(true);
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
    start(true);
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

static void test_peer_missing_counts_once_and_clears(void)
{
    start(false); /* no M7 layout word: peer NEVER_MATCHED */
    run_to(SAFETY_APP_PEER_BOOT_BUDGET_MS);
    TEST_ASSERT_EQUAL_UINT32(0u, safety_app_status().level_faults);
    TEST_ASSERT_EQUAL_UINT8(0u, safety_app_status().error_count);
    TEST_ASSERT_EQUAL_UINT32(0u, warn.frames); /* never written to a non-matching peer */
    run_to(SAFETY_APP_PEER_BOOT_BUDGET_MS + 2u);
    TEST_ASSERT_NOT_EQUAL_UINT32(0u, safety_app_status().level_faults);
    TEST_ASSERT_EQUAL_UINT8(1u, safety_app_status().error_count);
    run_to(3000u);
    TEST_ASSERT_EQUAL_UINT8(1u, safety_app_status().error_count);
    m7_set_layout(PLATFORM_MAILBOX_LAYOUT_ID);
    run_to(3010u);
    TEST_ASSERT_EQUAL_UINT32(0u, safety_app_status().level_faults);
    TEST_ASSERT_NOT_EQUAL_UINT32(0u, warn.frames);
    run_to(3150u);
    TEST_ASSERT_EQUAL_UINT32(1u, hbs_errors());
}

static void test_rt_core_inputs_lost_is_not_an_own_fault(void)
{
    start(true);
    run_to(500u);
    TEST_ASSERT_EQUAL_UINT32(PLATFORM_HEARTBEAT_SAFETY_NODE_MODE_NORMAL_CHOICE, hbs_mode());
    TEST_ASSERT_FALSE(safety_app_status().rt_core_inputs_lost);
    rt.lean_on = false; /* FDCAN2 bus-off on the M7 looks the same (E-28 (1)) */
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
    start_at(true, 0u, 50u);
    run_to(300u);
    TEST_ASSERT_EQUAL_UINT32(0u, safety_app_status().pass_gap_faults);
    TEST_ASSERT_EQUAL_UINT8(0u, safety_app_status().error_count);
}

static void test_runs_across_the_ms_wrap(void)
{
    /* 32-bit ms wrap about 0.5 s after start: due(), the pass gap, the E2E timeouts on both
     * cores and the uptime all use wrap-safe deltas. */
    const uint32_t t0 = UINT32_MAX - 499u;
    start_at(true, t0, 2u);
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
    start(true);
    run_to(100u);
    next_pass = now + 50u; /* a 50 ms stall: one gap fault, no burst of 0x023 */
    run_to(151u);
    TEST_ASSERT_EQUAL_UINT32(1u, safety_app_status().pass_gap_faults);
    run_to(400u);
    TEST_ASSERT_EQUAL_UINT32(0u, warn.bad);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_null_is_inert);
    RUN_TEST(test_boot_init_degraded_then_normal);
    RUN_TEST(test_cadence_and_e2e_on_the_m7_side);
    RUN_TEST(test_pass_gap_is_an_own_fault);
    RUN_TEST(test_peer_missing_counts_once_and_clears);
    RUN_TEST(test_rt_core_inputs_lost_is_not_an_own_fault);
    RUN_TEST(test_late_pass_resyncs_the_grid);
    RUN_TEST(test_first_pass_only_sets_the_baseline);
    RUN_TEST(test_runs_across_the_ms_wrap);
    return UNITY_END();
}
