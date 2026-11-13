/*
 * L0 tests of the turn-signal input layer (D-075 item 4, D-082; D-111, provisional, defs#165): debounce, flash trains, the
 * 1.5 s hold, hazard, stuck-active fault, open wire, 32-bit millisecond wrap, polarity.
 * Pins are active LOW (indicator_config.h): a lit lamp pulls the GPIO to false.
 */
#include "features/indicator/indicator_config.h"
#include "features/indicator/indicator_core.h"

#include <unity.h>

#define STEP_MS (5u)

static indicator_core_t g_c;
static indicator_output_t g_o;

void setUp(void)
{
    indicator_core_init(&g_c);
    g_o.state = INDICATOR_STATE_NONE;
    g_o.fault_flags = 0u;
}
void tearDown(void) {}

/* Samples every STEP_MS in [t0, t1) with the given lamp states (true = lit = pin low). */
static uint32_t run(uint32_t t0, uint32_t t1, bool l_lit, bool r_lit)
{
    uint32_t t = t0;
    while ((uint32_t)(t - t0) < (uint32_t)(t1 - t0)) {
        g_o = indicator_core_step(&g_c, t, !l_lit, !r_lit);
        t += STEP_MS;
    }
    return t;
}

/* A flash train: n flashes of on_ms lit and off_ms dark, starting at t0. Returns the end time. */
static uint32_t flashes(uint32_t t0, unsigned n, uint32_t on_ms, uint32_t off_ms, bool l, bool r)
{
    uint32_t t = t0;
    unsigned i;
    for (i = 0u; i < n; i++) {
        t = run(t, t + on_ms, l, r);
        t = run(t, t + off_ms, false, false);
    }
    return t;
}

static void test_boot_is_none(void)
{
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, indicator_core_step(&g_c, 0u, true, true).state);
    indicator_core_step(NULL, 0u, true, true); /* NULL is ignored */
    indicator_core_init(NULL);
}

static void test_left_flash_train_signals_left_throughout(void)
{
    uint32_t t = 1000u;
    unsigned i;
    /* 90/min: 667 ms period, 330 on / 337 off; sample the result mid-way of each dark phase. */
    for (i = 0u; i < 6u; i++) {
        t = run(t, t + 330u, true, false);
        t = run(t, t + 337u, false, false);
        TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_LEFT, g_o.state);
        TEST_ASSERT_EQUAL_UINT8(0u, g_o.fault_flags);
    }
}

static void test_right_flash_train_signals_right(void)
{
    flashes(1000u, 5u, 400u, 400u, false, true);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_RIGHT, g_o.state);
}

static void test_slowest_flasher_60_per_min_never_drops_out(void)
{
    uint32_t t = 2000u;
    unsigned i;
    for (i = 0u; i < 8u; i++) {
        t = run(t, t + 500u, true, false);
        TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_LEFT, g_o.state);
        t = run(t, t + 495u, false, false); /* just before the next lit edge */
        TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_LEFT, g_o.state);
        t = run(t, t + 5u, false, false);
    }
}

static void test_signalling_starts_at_the_first_debounced_edge(void)
{
    uint32_t t = run(1000u, 1100u, false, false);
    /* lit from t: not yet signalling before the debounce time, signalling after it. */
    run(t, t + 15u, true, false);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, g_o.state);
    run(t + 15u, t + 30u, true, false);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_LEFT, g_o.state);
}

static void test_hazard_when_both_sides_flash(void)
{
    flashes(1000u, 4u, 400u, 400u, true, true);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_HAZARD, g_o.state);
}

static void test_hazard_falls_back_to_one_side_after_the_other_expires(void)
{
    uint32_t t = flashes(1000u, 3u, 400u, 400u, true, true);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_HAZARD, g_o.state);
    t = flashes(t, 3u, 400u, 400u, true, false);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_LEFT, g_o.state);
}

static void test_hold_expires_exactly_1500_ms_after_the_last_edge(void)
{
    /* one flash: lit 400 ms, last (falling) debounced edge at t_off + debounce. */
    uint32_t t = run(1000u, 1400u, true, false);
    uint32_t off = t; /* dark from here; the falling edge is debounced at off + 20 */
    uint32_t edge = off + INDICATOR_DEBOUNCE_MS;
    run(off, edge + INDICATOR_HOLD_MS - STEP_MS, false, false);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_LEFT, g_o.state);
    run(edge + INDICATOR_HOLD_MS - STEP_MS, edge + INDICATOR_HOLD_MS, false, false);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_LEFT, g_o.state); /* last step at HOLD - 5 */
    run(edge + INDICATOR_HOLD_MS, edge + INDICATOR_HOLD_MS + STEP_MS, false, false);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, g_o.state);
    run(edge + INDICATOR_HOLD_MS + STEP_MS, edge + 4000u, false, false);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, g_o.state);
}

static void test_bounce_shorter_than_debounce_is_rejected(void)
{
    uint32_t t = 1000u;
    unsigned i;
    t = run(t, t + 100u, false, false);
    for (i = 0u; i < 20u; i++) { /* 10 ms lit / 10 ms dark chatter */
        t = run(t, t + 10u, true, true);
        TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, g_o.state);
        t = run(t, t + 10u, false, false);
        TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, g_o.state);
    }
}

static void test_single_sample_glitch_is_rejected(void)
{
    uint32_t t = run(1000u, 1100u, false, false);
    g_o = indicator_core_step(&g_c, t, false, true); /* one low sample on the left pin */
    t = run(t + STEP_MS, t + 200u, false, false);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, g_o.state);
}

static void test_dark_bounce_inside_a_lit_phase_is_rejected(void)
{
    uint32_t t = run(1000u, 1100u, true, false); /* signalling */
    t = run(t, t + 10u, false, false);           /* 10 ms drop-out of the lamp feed */
    t = run(t, t + 300u, true, false);
    t = run(t, t + 400u, false, false);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_LEFT, g_o.state);
}

static void test_stuck_active_is_a_fault_and_not_signalling(void)
{
    uint32_t t = run(1000u, 1000u + INDICATOR_STUCK_MS - 100u, true, false);
    TEST_ASSERT_EQUAL_UINT8(0u, g_o.fault_flags);
    t = run(t, t + 300u, true, false);
    TEST_ASSERT_EQUAL_UINT8(INDICATOR_FAULT_LEFT_STUCK, g_o.fault_flags);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, g_o.state);
    t = run(t, t + 20000u, true, false);
    TEST_ASSERT_EQUAL_UINT8(INDICATOR_FAULT_LEFT_STUCK, g_o.fault_flags);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, g_o.state);
}

static void test_stuck_active_from_boot_is_a_fault(void)
{
    run(0u, INDICATOR_STUCK_MS + 100u, false, true);
    TEST_ASSERT_EQUAL_UINT8(INDICATOR_FAULT_RIGHT_STUCK, g_o.fault_flags);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, g_o.state);
}

static void test_stuck_release_clears_the_fault_without_signalling(void)
{
    uint32_t t = run(1000u, 6000u, true, false);
    TEST_ASSERT_EQUAL_UINT8(INDICATOR_FAULT_LEFT_STUCK, g_o.fault_flags);
    t = run(t, t + 100u, false, false);
    TEST_ASSERT_EQUAL_UINT8(0u, g_o.fault_flags);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, g_o.state);
    /* a real flash afterwards works again */
    flashes(t, 3u, 400u, 400u, true, false);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_LEFT, g_o.state);
}

/* Left lamp input stays active for good (stuck), the right one flashes. */
static uint32_t left_stuck_right_flashes(uint32_t t, unsigned n)
{
    unsigned i;
    for (i = 0u; i < n; i++) {
        t = run(t, t + 400u, true, true);
        t = run(t, t + 400u, true, false);
    }
    return t;
}

static void test_stuck_side_does_not_make_a_hazard_and_the_other_side_reports(void)
{
    uint32_t t = run(1000u, 5000u, true, false); /* left stuck */
    TEST_ASSERT_EQUAL_UINT8(INDICATOR_FAULT_LEFT_STUCK, g_o.fault_flags);
    left_stuck_right_flashes(t, 4u);
    TEST_ASSERT_EQUAL_UINT8(INDICATOR_FAULT_LEFT_STUCK, g_o.fault_flags);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_RIGHT, g_o.state);
}

static void test_open_wire_or_dead_opto_reads_not_signalling(void)
{
    /* pull-up: both pins stay HIGH forever */
    run(0u, 60000u, false, false);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, g_o.state);
    TEST_ASSERT_EQUAL_UINT8(0u, g_o.fault_flags);
}

static void test_wire_breaks_during_signalling_expires_after_the_hold(void)
{
    uint32_t t = flashes(1000u, 4u, 400u, 400u, true, false);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_LEFT, g_o.state);
    run(t, t + INDICATOR_HOLD_MS + 100u, false, false); /* feed lost, pin stays high */
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, g_o.state);
}

static void test_timestamp_wrap_in_the_middle_of_a_flash_train(void)
{
    uint32_t t = 0xFFFFFFFFu - 1500u;
    flashes(t, 6u, 400u, 400u, true, false);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_LEFT, g_o.state);
}

static void test_timestamp_wrap_does_not_extend_or_cut_the_hold(void)
{
    uint32_t t = 0xFFFFFFFFu - 600u;
    t = run(t, t + 400u, true, false);
    t = run(t, t + 300u, false, false); /* wraps here */
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_LEFT, g_o.state);
    t = run(t, t + 1300u, false, false);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, g_o.state);
}

static void test_wrap_during_a_stuck_input(void)
{
    uint32_t t = 0xFFFFFFFFu - 1000u;
    run(t, t + 4000u, false, true);
    TEST_ASSERT_EQUAL_UINT8(INDICATOR_FAULT_RIGHT_STUCK, g_o.fault_flags);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, g_o.state);
}

static void test_a_stale_hold_never_revives_after_a_long_quiet_time(void)
{
    uint32_t t = flashes(1000u, 3u, 400u, 400u, true, false);
    t = run(t, t + 3000u, false, false);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, g_o.state);
    /* now jump ahead by about 2^32 - 1000 ms in one step: elapsed since the old edge would
     * read < HOLD again if the hold had not been dropped when it expired. */
    g_o = indicator_core_step(&g_c, t + 0xFFFFFC00u, true, true);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, g_o.state);
}

static void test_timestamp_that_stops_cannot_start_signalling(void)
{
    uint32_t i;
    for (i = 0u; i < 1000u; i++) {
        g_o = indicator_core_step(&g_c, 5000u, false, true); /* same ms, active pin */
    }
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, g_o.state);
}

static void test_the_polarity_is_active_low(void)
{
    TEST_ASSERT_EQUAL_INT(1, INDICATOR_ACTIVE_LEVEL_LOW);
    /* pin HIGH for a long time never signals; pin LOW does */
    run(0u, 500u, false, false);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, g_o.state);
    run(500u, 600u, true, false);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_LEFT, g_o.state);
}

/* ---- sampling-rate and jitter sweeps ------------------------------------------------ */

/* A flash train of the given period/on time sampled every step_ms with +/- step_ms/4
 * pseudo-random jitter. Counts the samples after the first period that do not read LEFT. */
static unsigned train_misses(uint32_t period, uint32_t on, uint32_t step, uint32_t t0, unsigned cycles)
{
    uint32_t t = t0;
    uint32_t seed = 12345u;
    unsigned misses = 0u;
    while ((uint32_t)(t - t0) < (period * cycles)) {
        const bool lit = (((uint32_t)(t - t0)) % period) < on;
        g_o = indicator_core_step(&g_c, t, !lit, true);
        if (((uint32_t)(t - t0) > period) && (g_o.state != INDICATOR_STATE_LEFT)) {
            misses++;
        }
        seed = (seed * 1103515245u) + 12345u;
        t += (step - (step / 4u)) + ((seed >> 16) % ((step / 2u) + 1u));
    }
    return misses;
}

static void sweep(uint32_t period, uint32_t on)
{
    static const uint32_t steps[] = {1u, 25u, 50u};
    unsigned i;
    for (i = 0u; i < 3u; i++) {
        setUp();
        TEST_ASSERT_EQUAL_UINT(0u, train_misses(period, on, steps[i], 1000u, 12u));
        TEST_ASSERT_EQUAL_UINT8(0u, g_o.fault_flags);
    }
}

static void test_sweep_60_per_min_short_on_time(void) { sweep(1000u, 300u); }
static void test_sweep_90_per_min(void) { sweep(667u, 200u); }
static void test_sweep_120_per_min_short_on_time(void) { sweep(500u, 150u); }
static void test_hyperflash_180_per_min_reads_left(void) { sweep(333u, 166u); }

/* ---- sample-count and time boundaries ----------------------------------------------- */

static void test_debounce_two_samples_over_20_ms_do_not_switch(void)
{
    indicator_core_step(&g_c, 0u, true, true);
    indicator_core_step(&g_c, 100u, false, true);
    g_o = indicator_core_step(&g_c, 125u, false, true);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, g_o.state);
}

static void test_debounce_three_samples_in_19_ms_do_not_switch(void)
{
    indicator_core_step(&g_c, 0u, true, true);
    indicator_core_step(&g_c, 100u, false, true);
    indicator_core_step(&g_c, 110u, false, true);
    g_o = indicator_core_step(&g_c, 119u, false, true);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, g_o.state);
}

static void test_debounce_three_samples_in_20_ms_switch(void)
{
    indicator_core_step(&g_c, 0u, true, true);
    indicator_core_step(&g_c, 100u, false, true);
    indicator_core_step(&g_c, 110u, false, true);
    g_o = indicator_core_step(&g_c, 120u, false, true);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_LEFT, g_o.state);
}

static void test_stuck_flag_at_exactly_3000_ms_not_2999(void)
{
    indicator_core_step(&g_c, 0u, true, true);
    indicator_core_step(&g_c, 100u, false, true);
    indicator_core_step(&g_c, 110u, false, true);
    indicator_core_step(&g_c, 120u, false, true); /* debounced edge at 120 */
    g_o = indicator_core_step(&g_c, 120u + INDICATOR_STUCK_MS - 1u, false, true);
    TEST_ASSERT_EQUAL_UINT8(0u, g_o.fault_flags);
    g_o = indicator_core_step(&g_c, 120u + INDICATOR_STUCK_MS, false, true);
    TEST_ASSERT_EQUAL_UINT8(INDICATOR_FAULT_LEFT_STUCK, g_o.fault_flags);
}

static void test_both_sides_stuck_read_none_with_both_flags(void)
{
    run(1000u, 6000u, true, true);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, g_o.state);
    TEST_ASSERT_EQUAL_UINT8(0x03u, g_o.fault_flags);
}

/* ---- undetected faults: they CAN read as signalling (escalate-only contract) ---------- */

static void test_frozen_clock_after_signalling_keeps_signalling(void)
{
    uint32_t i;
    flashes(1000u, 3u, 400u, 400u, true, false);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_LEFT, g_o.state);
    for (i = 0u; i < 1000u; i++) {
        g_o = indicator_core_step(&g_c, 1000u + 2400u, true, true);
    }
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_LEFT, g_o.state);
}

static void test_lit_line_with_20_ms_dropouts_reads_signalling(void)
{
    uint32_t t = 1000u;
    unsigned i;
    for (i = 0u; i < 20u; i++) {
        t = run(t, t + 100u, true, false);
        t = run(t, t + 25u, false, false);
    }
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_LEFT, g_o.state);
    TEST_ASSERT_EQUAL_UINT8(0u, g_o.fault_flags);
}

static void test_lamp_lit_at_boot_or_wrong_polarity_signals_for_the_hold(void)
{
    uint32_t t = run(0u, 100u, true, false); /* an idle-low pin reads the same */
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_LEFT, g_o.state);
    t = run(t, INDICATOR_HOLD_MS - 10u, true, false);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_LEFT, g_o.state);
    t = run(t, INDICATOR_HOLD_MS + 50u, true, false);
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_NONE, g_o.state);
    run(t, INDICATOR_STUCK_MS + 200u, true, false);
    TEST_ASSERT_EQUAL_UINT8(INDICATOR_FAULT_LEFT_STUCK, g_o.fault_flags);
}

static void test_floating_pin_chatter_reads_signalling(void)
{
    uint32_t t = 1000u;
    unsigned i;
    for (i = 0u; i < 40u; i++) {
        t = run(t, t + 30u, true, true);
        t = run(t, t + 30u, false, false);
    }
    TEST_ASSERT_EQUAL_INT(INDICATOR_STATE_HAZARD, g_o.state);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_boot_is_none);
    RUN_TEST(test_left_flash_train_signals_left_throughout);
    RUN_TEST(test_right_flash_train_signals_right);
    RUN_TEST(test_slowest_flasher_60_per_min_never_drops_out);
    RUN_TEST(test_signalling_starts_at_the_first_debounced_edge);
    RUN_TEST(test_hazard_when_both_sides_flash);
    RUN_TEST(test_hazard_falls_back_to_one_side_after_the_other_expires);
    RUN_TEST(test_hold_expires_exactly_1500_ms_after_the_last_edge);
    RUN_TEST(test_bounce_shorter_than_debounce_is_rejected);
    RUN_TEST(test_single_sample_glitch_is_rejected);
    RUN_TEST(test_dark_bounce_inside_a_lit_phase_is_rejected);
    RUN_TEST(test_stuck_active_is_a_fault_and_not_signalling);
    RUN_TEST(test_stuck_active_from_boot_is_a_fault);
    RUN_TEST(test_stuck_release_clears_the_fault_without_signalling);
    RUN_TEST(test_stuck_side_does_not_make_a_hazard_and_the_other_side_reports);
    RUN_TEST(test_open_wire_or_dead_opto_reads_not_signalling);
    RUN_TEST(test_wire_breaks_during_signalling_expires_after_the_hold);
    RUN_TEST(test_timestamp_wrap_in_the_middle_of_a_flash_train);
    RUN_TEST(test_timestamp_wrap_does_not_extend_or_cut_the_hold);
    RUN_TEST(test_wrap_during_a_stuck_input);
    RUN_TEST(test_a_stale_hold_never_revives_after_a_long_quiet_time);
    RUN_TEST(test_timestamp_that_stops_cannot_start_signalling);
    RUN_TEST(test_the_polarity_is_active_low);
    RUN_TEST(test_sweep_60_per_min_short_on_time);
    RUN_TEST(test_sweep_90_per_min);
    RUN_TEST(test_sweep_120_per_min_short_on_time);
    RUN_TEST(test_hyperflash_180_per_min_reads_left);
    RUN_TEST(test_debounce_two_samples_over_20_ms_do_not_switch);
    RUN_TEST(test_debounce_three_samples_in_19_ms_do_not_switch);
    RUN_TEST(test_debounce_three_samples_in_20_ms_switch);
    RUN_TEST(test_stuck_flag_at_exactly_3000_ms_not_2999);
    RUN_TEST(test_both_sides_stuck_read_none_with_both_flags);
    RUN_TEST(test_frozen_clock_after_signalling_keeps_signalling);
    RUN_TEST(test_lit_line_with_20_ms_dropouts_reads_signalling);
    RUN_TEST(test_lamp_lit_at_boot_or_wrong_polarity_signals_for_the_hold);
    RUN_TEST(test_floating_pin_chatter_reads_signalling);
    return UNITY_END();
}
