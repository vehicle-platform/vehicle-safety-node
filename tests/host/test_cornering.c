/*
 * Glue tests of features/cornering with the real platform-link service, the CAN host model
 * and real generated E2E frames from an rt-core model on the bus (0x081 100 ms, 0x022
 * 100 ms, 0x020 20 ms; the node pass every 2 ms, D-101 item 2). Covers D-042 item 3 source selection on E2E faults
 * (CRC, repeated counter, timeout, INITIAL), the heartbeat, Q-029, the D-029 rule on 0x022,
 * the D-101 item 4 return rule at boot and after a loss, and the D-100 hysteresis on fresh
 * frames.
 */
#include "features/cornering/cornering.h"

#include "bus_model.h"
#include "platform_limits.h"
#include "services/platform_link.h"

#include <math.h>
#include <unity.h>

#define PASS_MS 2u

static rt_core_t rt;
static uint32_t now;

static void m4_pass(uint32_t t)
{
    platform_link_step(t);
    cornering_step(t);
}

/* Advance the simulation to t_end inclusive: rt-core first, then the node pass. */
static void run_to(uint32_t t_end)
{
    while (now < t_end) {
        now++;
        rt_core_tick(&rt, now);
        if ((now % PASS_MS) == 0u) {
            m4_pass(now);
        }
    }
}

void setUp(void)
{
    bus_reset();
    rt_core_init(&rt);
    rt.lean_raw = bus_lean_raw(5.0f);
    now = 0u;
    TEST_ASSERT_TRUE(platform_link_init(0u));
    cornering_init();
    m4_pass(0u); /* before the first frame of rt-core */
}
void tearDown(void) {}

static cornering_output_t out(void)
{
    return cornering_output();
}

/* First EKF pass at boot: hb INITIAL at 100, OK at 200; lean usable from 200 on, the tenth
 * usable frame at 380 (D-101 item 4). */
#define BOOT_EKF_MS 380u

static void to_ekf(void)
{
    run_to(BOOT_EKF_MS);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_EKF, out().source);
}

static void test_boot_is_unavailable_until_ten_usable_frames(void)
{
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_UNAVAILABLE, out().level);
    TEST_ASSERT_TRUE(cornering_status().rt_core_inputs_lost);
    run_to(BOOT_EKF_MS - PASS_MS);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, out().source);
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_UNAVAILABLE, out().level);
    TEST_ASSERT_EQUAL_UINT8(CORNERING_RETURN_FRAMES - 1u, cornering_status().good_frames);
    run_to(BOOT_EKF_MS);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_EKF, out().source);
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_GREEN, out().level);
    TEST_ASSERT_FALSE(cornering_status().rt_core_inputs_lost);
    TEST_ASSERT_TRUE(cornering_output_consistent(out()));
}

static void test_lean_timeout_is_unavailable_and_named_inputs_lost(void)
{
    to_ekf();
    run_to(400u);
    rt.lean_on = false; /* e.g. rt-core stopped or a bus fault: no new frames */
    run_to(400u + PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_EKF, out().source);
    run_to(400u + PLATFORM_EKF_LEAN_E2E_TIMEOUT_MS + PASS_MS);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, out().source);
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_UNAVAILABLE, out().level);
    TEST_ASSERT_TRUE(cornering_status().rt_core_inputs_lost);
    TEST_ASSERT_EQUAL_UINT32(1u, cornering_status().inputs_lost_onsets);
    TEST_ASSERT_EQUAL_UINT32(1u, cornering_status().lean_timeouts);
    /* Back: the first frame after the timeout resyncs as INITIAL (not usable), then ten. */
    rt.lean_on = true; /* next frame at 480 (INITIAL), usable from 500 on */
    run_to(480u + (CORNERING_RETURN_FRAMES * PLATFORM_EKF_LEAN_CYCLE_TIME_MS) - PASS_MS);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, out().source);
    run_to(480u + (CORNERING_RETURN_FRAMES * PLATFORM_EKF_LEAN_CYCLE_TIME_MS));
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_EKF, out().source);
}

static void test_bad_crc_drops_at_once(void)
{
    uint8_t f[FB_LEN];
    to_ekf();
    rt.lean_on = false;
    fb_lean(f, rt.lean_state, rt.lean_raw, &rt.lean_tx);
    f[VEHICLE_E2E_CRC_OFFSET] ^= 0x01u;
    bus_send(BUS_LEAN, f);
    run_to(BOOT_EKF_MS + PASS_MS);
    TEST_ASSERT_EQUAL(VEHICLE_E2E_WRONG_CRC, cornering_status().lean_e2e);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, out().source);
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_UNAVAILABLE, out().level);
}

/* defs v0.30.0 (E-5): a wrong DLC reaches cornering as WRONG_LENGTH, never usable. A stream of
 * wrong-DLC 0x020 frames in place of the real ones drops the source at once and the level never
 * reads GREEN again (UNAVAILABLE: the D-042 fallback, the own-IMU path is outside this glue). */
static void test_wrong_dlc_0x020_drops_at_once_and_is_never_green(void)
{
    const uint8_t dlcs[2] = {7u, 0u};
    uint8_t f[FB_LEN];
    to_ekf();
    rt.lean_on = false;
    for (uint32_t i = 0u; i < 20u; i++) {
        fb_lean(f, rt.lean_state, rt.lean_raw, &rt.lean_tx);
        bus_send_dlc(BUS_LEAN, f, dlcs[i % 2u]);
        run_to(now + PASS_MS);
        TEST_ASSERT_EQUAL(VEHICLE_E2E_WRONG_LENGTH, cornering_status().lean_e2e);
        TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, out().source);
        TEST_ASSERT_EQUAL(CORNERING_LEVEL_UNAVAILABLE, out().level);
        TEST_ASSERT_NOT_EQUAL(CORNERING_LEVEL_GREEN, out().level);
        run_to(now + 18u);
        TEST_ASSERT_NOT_EQUAL(CORNERING_LEVEL_GREEN, out().level);
    }
    TEST_ASSERT_TRUE(cornering_status().rt_core_inputs_lost);
}

/* The same for 0x081: wrong-DLC heartbeats are WRONG_LENGTH, hb_ok clears, the source drops. */
static void test_wrong_dlc_0x081_drops_the_source_and_is_never_green(void)
{
    uint8_t f[FB_LEN];
    to_ekf();
    rt.hb_on = false;
    for (uint32_t i = 0u; i < 12u; i++) {
        fb_hb(f, PLATFORM_HEARTBEAT_RT_CORE_RT_CORE_NODE_MODE_NORMAL_CHOICE, &rt.hb_tx);
        bus_send_dlc(BUS_HB, f, (i % 2u == 0u) ? 7u : 0u);
        run_to(now + PASS_MS);
        TEST_ASSERT_EQUAL(VEHICLE_E2E_WRONG_LENGTH, cornering_status().hb_e2e);
        TEST_ASSERT_FALSE(cornering_status().hb_ok);
        TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, out().source);
        TEST_ASSERT_NOT_EQUAL(CORNERING_LEVEL_GREEN, out().level);
        run_to(now + 98u);
        TEST_ASSERT_NOT_EQUAL(CORNERING_LEVEL_GREEN, out().level);
    }
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_UNAVAILABLE, out().level);
}

static void test_repeated_counter_drops_at_once(void)
{
    uint8_t f[FB_LEN];
    vehicle_e2e_tx_state_t again;
    to_ekf();
    rt.lean_on = false;
    again = rt.lean_tx;
    again.counter = (uint8_t)(again.counter - 1u); /* the counter of the frame already taken */
    fb_lean(f, rt.lean_state, rt.lean_raw, &again);
    bus_send(BUS_LEAN, f);
    run_to(BOOT_EKF_MS + PASS_MS);
    TEST_ASSERT_EQUAL(VEHICLE_E2E_REPEATED, cornering_status().lean_e2e);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, out().source);
}

static void test_heartbeat_not_normal_drops(void)
{
    to_ekf();
    rt.hb_mode = PLATFORM_HEARTBEAT_RT_CORE_RT_CORE_NODE_MODE_DEGRADED_CHOICE;
    run_to(398u);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_EKF, out().source);
    run_to(400u);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, out().source);
    TEST_ASSERT_FALSE(cornering_status().hb_ok);
    rt.hb_mode = PLATFORM_HEARTBEAT_RT_CORE_RT_CORE_NODE_MODE_INIT_CHOICE;
    run_to(800u);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, out().source);
    rt.hb_mode = PLATFORM_HEARTBEAT_RT_CORE_RT_CORE_NODE_MODE_NORMAL_CHOICE;
    run_to(900u + ((CORNERING_RETURN_FRAMES - 1u) * PLATFORM_EKF_LEAN_CYCLE_TIME_MS));
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_EKF, out().source);
}

static void test_heartbeat_timeout_drops(void)
{
    to_ekf();
    run_to(400u);
    rt.hb_on = false; /* last valid 0x081 at 400 */
    run_to(400u + PLATFORM_HEARTBEAT_RT_CORE_E2E_TIMEOUT_MS);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_EKF, out().source);
    run_to(400u + PLATFORM_HEARTBEAT_RT_CORE_E2E_TIMEOUT_MS + PASS_MS);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, out().source);
    TEST_ASSERT_EQUAL_UINT32(1u, cornering_status().hb_timeouts);
}

static void test_q029_estimated_above_clamp_drops(void)
{
    uint32_t before;
    to_ekf();
    before = cornering_status().lean_rejected; /* the boot INITIAL frame counts too */
    rt.lean_raw = bus_lean_raw(PLATFORM_LIMIT_LEAN_ANGLE_CLAMP_MAX_DEG + 1.0f);
    run_to(400u);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, out().source);
    TEST_ASSERT_EQUAL_UINT32(before + 1u, cornering_status().lean_rejected);
    /* CLAMPED at the clamp is usable (RED at every mu, D-065 item 3). */
    rt.lean_state = PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_CLAMPED_CHOICE;
    rt.lean_raw = bus_lean_raw(PLATFORM_LIMIT_LEAN_ANGLE_CLAMP_MAX_DEG);
    run_to(400u + (CORNERING_RETURN_FRAMES * PLATFORM_EKF_LEAN_CYCLE_TIME_MS));
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_EKF, out().source);
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_RED, out().level);
}

static void test_friction_rule_and_loss(void)
{
    rt.lean_raw = bus_lean_raw(12.0f);
    rt.mu_raw = bus_mu_raw(0.3f); /* DEFAULT 0.3: min(0.3, 0.5) -> u = tan 12 / 0.3 = 0.71 */
    to_ekf();
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.3f, cornering_status().mu_eff);
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_YELLOW, out().level);
    /* 0x022 stops: after its timeout mu_eff = min(last valid 0.3, 0.5) = 0.3 (D-101 item 5),
     * so the warning stays. */
    rt.mu_on = false;
    run_to(300u + PLATFORM_EKF_FRICTION_MASS_E2E_TIMEOUT_MS + PASS_MS);
    TEST_ASSERT_FALSE(cornering_status().mu_live);
    TEST_ASSERT_EQUAL_UINT32(1u, cornering_status().mu_timeouts);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.3f, cornering_status().mu_eff);
    run_to(1000u);
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_YELLOW, out().level);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_EKF, out().source); /* 0x022 is not a source criterion */
}

/* D-119: a valid ESTIMATED 0x022 above the default (here 0.9 and the clamp maximum 1.2) can
 * never raise mu_eff: it stays the default, so the lean-20 warning is YELLOW (u = 0.73 at 0.5;
 * it would be GREEN, u = 0.40, at 0.9) before and after the 0x022 goes away. */
static void test_d119_valid_high_mu_stays_at_the_default(void)
{
    const float mus[2] = {0.9f, PLATFORM_LIMIT_FRICTION_COEFF_CLAMP_MAX};
    for (uint32_t i = 0u; i < 2u; i++) {
        setUp();
        rt.lean_raw = bus_lean_raw(20.0f);
        rt.mu_state = PLATFORM_EKF_FRICTION_MASS_FRICTION_COEFF_STATE_ESTIMATED_CHOICE;
        rt.mu_raw = bus_mu_raw(mus[i]);
        to_ekf();
        TEST_ASSERT_TRUE(cornering_status().mu_live); /* valid, only capped */
        TEST_ASSERT_EQUAL_FLOAT(PLATFORM_LIMIT_FRICTION_COEFF_DEFAULT, cornering_status().mu_eff);
        TEST_ASSERT_EQUAL(CORNERING_LEVEL_YELLOW, out().level);
        rt.mu_on = false;
        run_to(300u + PLATFORM_EKF_FRICTION_MASS_E2E_TIMEOUT_MS + PASS_MS);
        TEST_ASSERT_EQUAL_FLOAT(PLATFORM_LIMIT_FRICTION_COEFF_DEFAULT, cornering_status().mu_eff);
        TEST_ASSERT_EQUAL(CORNERING_LEVEL_YELLOW, out().level);
    }
}

static void test_d119_valid_lower_mu_is_used(void)
{
    rt.lean_raw = bus_lean_raw(12.0f); /* u = 0.43 at 0.5 (GREEN), 0.71 at 0.3 (YELLOW) */
    rt.mu_state = PLATFORM_EKF_FRICTION_MASS_FRICTION_COEFF_STATE_ESTIMATED_CHOICE;
    rt.mu_raw = bus_mu_raw(0.3f);
    to_ekf();
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.3f, cornering_status().mu_eff);
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_YELLOW, out().level);
}

/* D-118 addendum (a): rt-core sends no 0x022 yet. Never received: mu_eff is the default, the
 * source is EKF and the decision works (GREEN at 12 degrees, u = 0.43). */
static void test_d119_no_0x022_ever_uses_the_default(void)
{
    rt.mu_on = false;
    rt.lean_raw = bus_lean_raw(12.0f);
    to_ekf();
    TEST_ASSERT_FALSE(cornering_status().mu_live);
    TEST_ASSERT_EQUAL_FLOAT(PLATFORM_LIMIT_FRICTION_COEFF_DEFAULT, cornering_status().mu_eff);
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_GREEN, out().level);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_EKF, out().source);
}

static void test_friction_loss_keeps_red_on_low_mu(void)
{
    /* safety-reviewer MAJOR-1: RED at 15 deg with mu 0.3 must not release when 0x022 goes. */
    rt.lean_raw = bus_lean_raw(15.0f);
    rt.mu_state = PLATFORM_EKF_FRICTION_MASS_FRICTION_COEFF_STATE_ESTIMATED_CHOICE;
    rt.mu_raw = bus_mu_raw(0.3f);
    to_ekf();
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_RED, out().level);
    rt.mu_on = false;
    run_to(1200u);
    TEST_ASSERT_FALSE(cornering_status().mu_live);
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_RED, out().level);
}

static void test_friction_invalid_value_keeps_red_then_recovers(void)
{
    /* safety-reviewer round-2 MAJOR-2 (D-101 item 5): an E2E-valid 0x022 with state INVALID
     * after ESTIMATED 0.3 keeps mu_eff 0.3; a later valid 0.9 applies D-029 again. */
    rt.lean_raw = bus_lean_raw(15.0f); /* u = 0.89 at 0.3 (RED), 0.30 at 0.9 (GREEN) */
    rt.mu_state = PLATFORM_EKF_FRICTION_MASS_FRICTION_COEFF_STATE_ESTIMATED_CHOICE;
    rt.mu_raw = bus_mu_raw(0.3f);
    to_ekf();
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_RED, out().level);
    const uint32_t value_rejects = cornering_status().mu_value_rejected;
    const uint32_t e2e_rejects = cornering_status().mu_e2e_rejected;
    rt.mu_state = PLATFORM_EKF_FRICTION_MASS_FRICTION_COEFF_STATE_INVALID_CHOICE;
    rt.mu_raw = bus_mu_raw(0.9f);
    run_to(800u);
    TEST_ASSERT_FALSE(cornering_status().mu_live);
    TEST_ASSERT_TRUE(cornering_status().mu_value_rejected > value_rejects); /* a value reject, */
    TEST_ASSERT_EQUAL_UINT32(e2e_rejects, cornering_status().mu_e2e_rejected); /* not an E2E one */
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.3f, cornering_status().mu_eff);
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_RED, out().level);
    rt.mu_state = PLATFORM_EKF_FRICTION_MASS_FRICTION_COEFF_STATE_ESTIMATED_CHOICE;
    run_to(900u); /* the valid 0x022 at 900 */
    TEST_ASSERT_TRUE(cornering_status().mu_live);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, PLATFORM_LIMIT_FRICTION_COEFF_DEFAULT, cornering_status().mu_eff); /* 0.9 capped, D-119 */
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_RED, out().level); /* the hold starts at 900 */
    run_to(998u);
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_RED, out().level);
    run_to(1000u);
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_GREEN, out().level);
}

static void test_friction_returns_after_a_timeout(void)
{
    rt.lean_raw = bus_lean_raw(15.0f);
    rt.mu_state = PLATFORM_EKF_FRICTION_MASS_FRICTION_COEFF_STATE_ESTIMATED_CHOICE;
    rt.mu_raw = bus_mu_raw(0.3f);
    to_ekf();
    rt.mu_on = false;
    run_to(700u);
    TEST_ASSERT_FALSE(cornering_status().mu_live);
    rt.mu_on = true;
    rt.mu_raw = bus_mu_raw(0.9f);
    run_to(800u); /* INITIAL after the timeout: still not usable */
    TEST_ASSERT_FALSE(cornering_status().mu_live);
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_RED, out().level);
    run_to(900u);
    TEST_ASSERT_TRUE(cornering_status().mu_live);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, PLATFORM_LIMIT_FRICTION_COEFF_DEFAULT, cornering_status().mu_eff); /* 0.9 capped, D-119 */
    run_to(1000u);
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_GREEN, out().level);
}

/* MINOR-3: one fake 0x020 frame with the NEXT counter and a valid CRC, sent in another pass
 * than rt-core's frames (no interleaving inside a pass), passes the E2E check once: it is used
 * once and can only raise the level (here RED from a 50 degree lean). rt-core's own frame for
 * that counter then arrives as REPEATED, so the source drops to NONE at once. */
static void test_fake_next_counter_in_another_pass_is_used_once_then_caught(void)
{
    vehicle_e2e_tx_state_t fake;
    uint8_t f[FB_LEN];
    to_ekf();
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_GREEN, out().level);
    fake = rt.lean_tx; /* exactly the counter rt-core uses next */
    fb_lean(f, PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_ESTIMATED_CHOICE, bus_lean_raw(50.0f), &fake);
    run_to(390u); /* rt-core's last frame was 380, the next is 400 */
    bus_send(BUS_LEAN, f);
    run_to(392u);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_EKF, out().source);
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_RED, out().level); /* used once, only ever more severe */
    run_to(402u); /* rt-core's frame of 400 carries the same counter */
    TEST_ASSERT_EQUAL(VEHICLE_E2E_REPEATED, cornering_status().lean_e2e);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, out().source);
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_UNAVAILABLE, out().level);
}

/* MINOR-R2-2 (D-026) at the decision level: four safety frames in one pass at FIFO depth 3
 * (the fourth is lost). The three that arrive are E2E-valid, but the seen loss invalidates the
 * cycle: no EKF use in that pass. Then the existing return rules apply (ten usable frames). */
static void test_a_safety_fifo_loss_is_never_used_as_ekf(void)
{
    uint8_t f[FB_LEN];
    to_ekf();
    run_to(390u);
    fb_lean(f, rt.lean_state, rt.lean_raw, &rt.lean_tx);
    bus_send(BUS_LEAN, f);
    fb_mu(f, rt.mu_state, rt.mu_raw, &rt.mu_tx);
    bus_send(BUS_MU, f);
    fb_hb(f, rt.hb_mode, &rt.hb_tx);
    bus_send(BUS_HB, f);
    fb_lean(f, rt.lean_state, rt.lean_raw, &rt.lean_tx);
    TEST_ASSERT_FALSE(bus_try_send(BUS_LEAN, f)); /* lost: FIFO depth 3 */
    run_to(392u);
    TEST_ASSERT_EQUAL_UINT32(1u, platform_link_health().rx_overflow[0]);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, out().source);
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_UNAVAILABLE, out().level);
    TEST_ASSERT_TRUE(cornering_status().rt_core_inputs_lost);
    run_to(400u + ((CORNERING_RETURN_FRAMES + 3u) * PLATFORM_EKF_LEAN_CYCLE_TIME_MS));
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_EKF, out().source); /* back per the existing rules */
}

static void test_skipped_lean_frame_drops_at_once(void)
{
    uint8_t f[FB_LEN];
    to_ekf();
    rt.lean_on = false;
    rt.lean_tx.counter = (uint8_t)(rt.lean_tx.counter + 1u); /* one frame lost on the bus */
    fb_lean(f, rt.lean_state, rt.lean_raw, &rt.lean_tx);
    bus_send(BUS_LEAN, f);
    run_to(BOOT_EKF_MS + PASS_MS);
    TEST_ASSERT_EQUAL(VEHICLE_E2E_WRONG_SEQUENCE, cornering_status().lean_e2e);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, out().source);
    TEST_ASSERT_EQUAL_UINT8(0u, cornering_status().good_frames);
}

static void test_friction_bad_crc_keeps_the_last_low_mu(void)
{
    uint32_t before;
    uint8_t f[FB_LEN];
    rt.mu_raw = bus_mu_raw(0.2f);
    to_ekf();
    before = cornering_status().mu_e2e_rejected; /* the boot INITIAL frame counts too */
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.2f, cornering_status().mu_eff);
    rt.mu_on = false;
    fb_mu(f, rt.mu_state, rt.mu_raw, &rt.mu_tx);
    f[VEHICLE_E2E_CRC_OFFSET] ^= 0x80u;
    bus_send(BUS_MU, f);
    run_to(BOOT_EKF_MS + PASS_MS);
    TEST_ASSERT_EQUAL(VEHICLE_E2E_WRONG_CRC, cornering_status().mu_e2e);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.2f, cornering_status().mu_eff); /* min(last valid, default) */
    TEST_ASSERT_EQUAL_UINT32(before + 1u, cornering_status().mu_e2e_rejected);
}

/* E-29 (3): an E2E-valid ESTIMATED 0x022 whose raw value is outside the DBC range is an
 * unusable value (counted apart from E2E failures): mu_eff stays min(last valid, default). */
static void friction_out_of_range_after(float last_valid_mu, float want_mu_eff)
{
    uint8_t f[FB_LEN];
    rt.mu_state = PLATFORM_EKF_FRICTION_MASS_FRICTION_COEFF_STATE_ESTIMATED_CHOICE;
    rt.mu_raw = bus_mu_raw(last_valid_mu);
    to_ekf();
    const cornering_status_t st0 = cornering_status();
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, fminf(last_valid_mu, PLATFORM_LIMIT_FRICTION_COEFF_DEFAULT), st0.mu_eff); /* D-119 cap */
    uint16_t bad = 0u;
    while ((bad < UINT16_MAX) && platform_ekf_friction_mass_friction_coeff_is_in_range(bad)) {
        bad++; /* the first raw value gen/ calls out of range */
    }
    TEST_ASSERT_FALSE(platform_ekf_friction_mass_friction_coeff_is_in_range(bad));
    rt.mu_on = false;
    fb_mu(f, (uint8_t)PLATFORM_EKF_FRICTION_MASS_FRICTION_COEFF_STATE_ESTIMATED_CHOICE, bad, &rt.mu_tx);
    TEST_ASSERT_EQUAL_UINT32(bad, fb_mu_get(f)); /* the raw value fits the signal */
    bus_send(BUS_MU, f);
    run_to(BOOT_EKF_MS + PASS_MS);
    const cornering_status_t st = cornering_status();
    TEST_ASSERT_TRUE(vehicle_e2e_is_valid(st.mu_e2e));
    TEST_ASSERT_FALSE(st.mu_live);
    TEST_ASSERT_EQUAL_UINT32(st0.mu_value_rejected + 1u, st.mu_value_rejected);
    TEST_ASSERT_EQUAL_UINT32(st0.mu_e2e_rejected, st.mu_e2e_rejected);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, want_mu_eff, st.mu_eff); /* min(last valid, default) */
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_EKF, out().source); /* 0x022 is no source criterion */
}

static void test_friction_out_of_range_value_keeps_the_last_low_mu(void)
{
    friction_out_of_range_after(0.2f, 0.2f);
}

/* The escalation path on a value reject: a high last-valid µ falls back to the default. */
static void test_friction_out_of_range_value_after_a_high_mu_gives_the_default(void)
{
    friction_out_of_range_after(0.9f, PLATFORM_LIMIT_FRICTION_COEFF_DEFAULT);
}

static void test_escalates_at_once_and_holds_on_fresh_frames(void)
{
    to_ekf();
    rt.lean_raw = bus_lean_raw(25.0f); /* u = 0.93 at mu 0.5 */
    run_to(400u);
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_RED, out().level);
    rt.lean_raw = bus_lean_raw(5.0f); /* u = 0.17: relaxed GREEN from the frame at 420 */
    run_to(518u);
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_RED, out().level);
    run_to(520u); /* the fresh frame 100 ms after 420 */
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_GREEN, out().level);
}

static void test_status_and_inert_take(void)
{
    cornering_status_t s = cornering_status();
    TEST_ASSERT_EQUAL(VEHICLE_E2E_NO_NEW_DATA, s.lean_e2e);
    TEST_ASSERT_FALSE(s.lean_live);
    TEST_ASSERT_FALSE(s.hb_ok);
    TEST_ASSERT_EQUAL_FLOAT(PLATFORM_LIMIT_FRICTION_COEFF_DEFAULT, s.mu_eff);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_boot_is_unavailable_until_ten_usable_frames);
    RUN_TEST(test_lean_timeout_is_unavailable_and_named_inputs_lost);
    RUN_TEST(test_bad_crc_drops_at_once);
    RUN_TEST(test_wrong_dlc_0x020_drops_at_once_and_is_never_green);
    RUN_TEST(test_wrong_dlc_0x081_drops_the_source_and_is_never_green);
    RUN_TEST(test_repeated_counter_drops_at_once);
    RUN_TEST(test_heartbeat_not_normal_drops);
    RUN_TEST(test_heartbeat_timeout_drops);
    RUN_TEST(test_q029_estimated_above_clamp_drops);
    RUN_TEST(test_friction_rule_and_loss);
    RUN_TEST(test_friction_bad_crc_keeps_the_last_low_mu);
    RUN_TEST(test_friction_out_of_range_value_keeps_the_last_low_mu);
    RUN_TEST(test_friction_out_of_range_value_after_a_high_mu_gives_the_default);
    RUN_TEST(test_d119_valid_high_mu_stays_at_the_default);
    RUN_TEST(test_d119_valid_lower_mu_is_used);
    RUN_TEST(test_d119_no_0x022_ever_uses_the_default);
    RUN_TEST(test_friction_loss_keeps_red_on_low_mu);
    RUN_TEST(test_fake_next_counter_in_another_pass_is_used_once_then_caught);
    RUN_TEST(test_a_safety_fifo_loss_is_never_used_as_ekf);
    RUN_TEST(test_skipped_lean_frame_drops_at_once);
    RUN_TEST(test_friction_invalid_value_keeps_red_then_recovers);
    RUN_TEST(test_friction_returns_after_a_timeout);
    RUN_TEST(test_escalates_at_once_and_holds_on_fresh_frames);
    RUN_TEST(test_status_and_inert_take);
    return UNITY_END();
}
