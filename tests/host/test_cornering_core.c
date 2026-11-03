/*
 * L0 tests of the pure cornering core: D-041 item 2 inputs and decision, D-075 item 5
 * (Q-029 clamp check), the D-029 friction rule, the D-042 item 3 / D-075 item 6 (4) /
 * D-101 item 4 source selection and the D-100 item 3 hysteresis.
 */
#include "features/cornering/cornering_core.h"

#include "platform.h"
#include "platform_limits.h"

#include <math.h>
#include <unity.h>

#define EST PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_ESTIMATED_CHOICE
#define CLA PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_CLAMPED_CHOICE
#define RES PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_RESERVED_CHOICE
#define INV PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_INVALID_CHOICE
#define MU_EST PLATFORM_EKF_FRICTION_MASS_FRICTION_COEFF_STATE_ESTIMATED_CHOICE
#define MU_CLA PLATFORM_EKF_FRICTION_MASS_FRICTION_COEFF_STATE_CLAMPED_CHOICE
#define MU_DEF PLATFORM_EKF_FRICTION_MASS_FRICTION_COEFF_STATE_DEFAULT_CHOICE
#define MU_INV PLATFORM_EKF_FRICTION_MASS_FRICTION_COEFF_STATE_INVALID_CHOICE

/* Raw values in the DBC scales (0.01 deg, 0.001), derived from the gen decoders below. */
static int16_t lean_raw(float deg)
{
    return (int16_t)lrintf(deg / platform_ekf_lean_lean_angle_decode(1));
}
static uint16_t mu_raw(float mu)
{
    return (uint16_t)lrintf(mu / platform_ekf_friction_mass_friction_coeff_decode(1u));
}

void setUp(void) {}
void tearDown(void) {}

static void test_float_threshold_relations(void)
{
    /* architecture-guard MINOR-3: not integer constant expressions, so checked here. */
    TEST_ASSERT_TRUE((PLATFORM_LIMIT_K_YELLOW - CORNERING_HYST_U_MARGIN) > 0.0f);
    TEST_ASSERT_TRUE((PLATFORM_LIMIT_K_RED - CORNERING_HYST_U_MARGIN) > PLATFORM_LIMIT_K_YELLOW);
    TEST_ASSERT_TRUE(PLATFORM_LIMIT_FRICTION_COEFF_CLAMP_MIN > 0.0f);
}

static void test_lean_states(void)
{
    float d = -1.0f;
    TEST_ASSERT_TRUE(cornering_lean_usable(EST, lean_raw(10.0f), &d));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, d);
    TEST_ASSERT_TRUE(cornering_lean_usable(EST, lean_raw(-10.0f), &d));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -10.0f, d);
    TEST_ASSERT_FALSE(cornering_lean_usable(RES, lean_raw(10.0f), &d));
    TEST_ASSERT_FALSE(cornering_lean_usable(INV, lean_raw(10.0f), &d));
    TEST_ASSERT_FALSE(cornering_lean_usable(7u, lean_raw(10.0f), &d));
    TEST_ASSERT_FALSE(cornering_lean_usable(EST, lean_raw(10.0f), NULL));
}

static void test_lean_dbc_range(void)
{
    float d;
    TEST_ASSERT_FALSE(cornering_lean_usable(CLA, (int16_t)(lean_raw(90.0f) + 1), &d));
    TEST_ASSERT_FALSE(cornering_lean_usable(CLA, (int16_t)(lean_raw(-90.0f) - 1), &d));
    TEST_ASSERT_FALSE(cornering_lean_usable(EST, INT16_MIN, &d));
}

static void test_q029_clamp_check(void)
{
    float d;
    const int16_t c = lean_raw(PLATFORM_LIMIT_LEAN_ANGLE_CLAMP_MAX_DEG);
    /* The one-step tolerance of D-075 item 5 is the 0.01 deg DBC step. */
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.01f, platform_ekf_lean_lean_angle_decode(1));
    /* ESTIMATED up to the clamp, not above (both signs). */
    TEST_ASSERT_TRUE(cornering_lean_usable(EST, c, &d));
    TEST_ASSERT_TRUE(cornering_lean_usable(EST, (int16_t)-c, &d));
    TEST_ASSERT_FALSE(cornering_lean_usable(EST, (int16_t)(c + 1), &d));
    TEST_ASSERT_FALSE(cornering_lean_usable(EST, (int16_t)(-c - 1), &d));
    /* CLAMPED at the clamp or one step below, never further below (safety-reviewer MINOR-1). */
    TEST_ASSERT_TRUE(cornering_lean_usable(CLA, c, &d));
    TEST_ASSERT_TRUE(cornering_lean_usable(CLA, (int16_t)-c, &d));
    TEST_ASSERT_TRUE(cornering_lean_usable(CLA, (int16_t)(c - 1), &d));
    TEST_ASSERT_TRUE(cornering_lean_usable(CLA, (int16_t)(-c + 1), &d));
    TEST_ASSERT_FALSE(cornering_lean_usable(CLA, (int16_t)(c - 2), &d));
    TEST_ASSERT_FALSE(cornering_lean_usable(CLA, (int16_t)(-c + 2), &d));
    /* CLAMPED above the clamp (within the DBC range) is still usable: it reads RED. */
    TEST_ASSERT_TRUE(cornering_lean_usable(CLA, (int16_t)(c + 1), &d));
    TEST_ASSERT_FALSE(cornering_lean_usable(CLA, lean_raw(30.0f), &d));
    TEST_ASSERT_FALSE(cornering_lean_usable(CLA, 0, &d));
}

static void test_mu_lost_never_raises(void)
{
    const float def = PLATFORM_LIMIT_FRICTION_COEFF_DEFAULT;
    TEST_ASSERT_EQUAL_FLOAT(def, cornering_mu_eff_lost(false, 0.3f));
    TEST_ASSERT_EQUAL_FLOAT(0.3f, cornering_mu_eff_lost(true, 0.3f));
    TEST_ASSERT_EQUAL_FLOAT(def, cornering_mu_eff_lost(true, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(PLATFORM_LIMIT_FRICTION_COEFF_CLAMP_MIN, cornering_mu_eff_lost(true, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(def, cornering_mu_eff_lost(true, NAN));
}

static float muv(uint8_t state, uint16_t raw, bool* ok)
{
    float v = -1.0f;
    *ok = cornering_mu_value(state, raw, &v);
    return v;
}

static void test_mu_rule(void)
{
    const float def = PLATFORM_LIMIT_FRICTION_COEFF_DEFAULT;
    const float lo = PLATFORM_LIMIT_FRICTION_COEFF_CLAMP_MIN;
    const float hi = PLATFORM_LIMIT_FRICTION_COEFF_CLAMP_MAX;
    bool ok;
    float v;
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.9f, muv(MU_EST, mu_raw(0.9f), &ok));
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.9f, muv(MU_CLA, mu_raw(0.9f), &ok));
    TEST_ASSERT_EQUAL_FLOAT(hi, muv(MU_EST, mu_raw(1.5f), &ok));
    TEST_ASSERT_EQUAL_FLOAT(lo, muv(MU_EST, mu_raw(0.05f), &ok));
    TEST_ASSERT_EQUAL_FLOAT(lo, muv(MU_EST, 0u, &ok));
    TEST_ASSERT_TRUE(ok);
    /* DEFAULT: max(clamp_min, min(received, default)). */
    TEST_ASSERT_EQUAL_FLOAT(def, muv(MU_DEF, mu_raw(0.9f), &ok));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.3f, muv(MU_DEF, mu_raw(0.3f), &ok));
    TEST_ASSERT_EQUAL_FLOAT(lo, muv(MU_DEF, mu_raw(0.05f), &ok));
    TEST_ASSERT_TRUE(ok);
    /* INVALID, unknown, out of the DBC range: not usable, output untouched (D-101 item 5). */
    v = muv(MU_INV, mu_raw(0.3f), &ok);
    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, v);
    (void)muv(9u, mu_raw(0.3f), &ok);
    TEST_ASSERT_FALSE(ok);
    (void)muv(MU_EST, (uint16_t)(mu_raw(2.0f) + 1u), &ok);
    TEST_ASSERT_FALSE(ok);
    (void)muv(MU_EST, UINT16_MAX, &ok);
    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_FALSE(cornering_mu_value(MU_EST, mu_raw(0.9f), NULL));
}

static cornering_level_t lvl(float deg, float mu)
{
    return cornering_level_for(deg, mu, PLATFORM_LIMIT_K_YELLOW, PLATFORM_LIMIT_K_RED);
}

#define RAD_TO_DEG (57.29578f)
/* The lean at which u = k for a given mu: atan(k * mu). */
static float onset(float k, float mu)
{
    return atanf(k * mu) * RAD_TO_DEG;
}

static void test_decision_levels(void)
{
    const float ky = PLATFORM_LIMIT_K_YELLOW;
    const float kr = PLATFORM_LIMIT_K_RED;
    const float mus[] = {0.5f, 1.0f, PLATFORM_LIMIT_FRICTION_COEFF_CLAMP_MIN, PLATFORM_LIMIT_FRICTION_COEFF_CLAMP_MAX};
    for (uint32_t i = 0u; i < (sizeof mus / sizeof mus[0]); i++) {
        const float mu = mus[i];
        TEST_ASSERT_EQUAL(CORNERING_LEVEL_GREEN, lvl(0.0f, mu));
        TEST_ASSERT_EQUAL(CORNERING_LEVEL_GREEN, lvl(onset(ky, mu) - 0.1f, mu));
        TEST_ASSERT_EQUAL(CORNERING_LEVEL_YELLOW, lvl(onset(ky, mu) + 0.1f, mu));
        TEST_ASSERT_EQUAL(CORNERING_LEVEL_YELLOW, lvl(-(onset(ky, mu) + 0.1f), mu));
        TEST_ASSERT_EQUAL(CORNERING_LEVEL_YELLOW, lvl(onset(kr, mu) - 0.1f, mu));
        TEST_ASSERT_EQUAL(CORNERING_LEVEL_RED, lvl(onset(kr, mu) + 0.1f, mu));
        TEST_ASSERT_EQUAL(CORNERING_LEVEL_RED, lvl(-(onset(kr, mu) + 0.1f), mu));
        TEST_ASSERT_EQUAL(CORNERING_LEVEL_RED, lvl(90.0f, mu));
    }
    /* D-048 reference points at mu 0.5: 16.70 / 21.80 deg. */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 16.70f, onset(ky, 0.5f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 21.80f, onset(kr, 0.5f));
}

static void test_decision_non_finite_is_red(void)
{
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_RED, lvl(NAN, 0.5f));
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_RED, lvl(INFINITY, 0.5f));
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_RED, lvl(5.0f, NAN));
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_RED, lvl(5.0f, INFINITY));
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_RED, cornering_level_for(5.0f, 0.5f, NAN, 0.8f));
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_RED, cornering_level_for(5.0f, 0.5f, 0.6f, NAN));
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_RED, cornering_level_for(5.0f, 3e38f, 0.6f, 3e38f));
}

static cornering_source_t sel_frames(cornering_select_t* s, uint32_t n, bool hb)
{
    cornering_source_t src = CORNERING_SOURCE_NONE;
    for (uint32_t i = 0u; i < n; i++) {
        src = cornering_select_step(s, true, true, true, hb);
    }
    return src;
}

static void test_select_return_needs_ten_frames(void)
{
    cornering_select_t s;
    cornering_select_init(&s);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, s.source);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, sel_frames(&s, CORNERING_RETURN_FRAMES - 1u, true));
    /* Passes without a new frame neither count nor switch. */
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, cornering_select_step(&s, false, false, true, true));
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, cornering_select_step(&s, false, false, true, true));
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_EKF, sel_frames(&s, 1u, true));
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_EKF, sel_frames(&s, 30u, true));
    TEST_ASSERT_EQUAL_UINT8(CORNERING_RETURN_FRAMES, s.good_frames);
}

static void test_select_drops_at_once(void)
{
    cornering_select_t s;
    cornering_select_init(&s);
    (void)sel_frames(&s, CORNERING_RETURN_FRAMES, true);
    /* First not-usable frame. */
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, cornering_select_step(&s, true, false, false, true));
    TEST_ASSERT_EQUAL_UINT8(0u, s.good_frames);
    (void)sel_frames(&s, CORNERING_RETURN_FRAMES, true);
    /* Timeout between frames. */
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, cornering_select_step(&s, false, false, false, true));
    (void)sel_frames(&s, CORNERING_RETURN_FRAMES, true);
    /* Heartbeat lost between frames. */
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, cornering_select_step(&s, false, false, true, false));
    TEST_ASSERT_EQUAL_UINT8(0u, s.good_frames);
}

static void test_select_breaks_reset_the_run(void)
{
    cornering_select_t s;
    cornering_select_init(&s);
    (void)sel_frames(&s, CORNERING_RETURN_FRAMES - 1u, true);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, cornering_select_step(&s, true, false, false, true));
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, sel_frames(&s, CORNERING_RETURN_FRAMES - 1u, true));
    /* Frames while the heartbeat is not OK do not count. */
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, sel_frames(&s, 5u, false));
    TEST_ASSERT_EQUAL_UINT8(0u, s.good_frames);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, sel_frames(&s, CORNERING_RETURN_FRAMES - 1u, true));
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_EKF, sel_frames(&s, 1u, true));
}

static void test_select_null(void)
{
    cornering_select_init(NULL);
    TEST_ASSERT_EQUAL(CORNERING_SOURCE_NONE, cornering_select_step(NULL, true, true, true, true));
}

#define G CORNERING_LEVEL_GREEN
#define Y CORNERING_LEVEL_YELLOW
#define R CORNERING_LEVEL_RED

static void test_hyst_first_and_escalation(void)
{
    cornering_hyst_t h;
    cornering_hyst_reset(&h);
    TEST_ASSERT_EQUAL(CORNERING_LEVEL_UNAVAILABLE, h.level);
    TEST_ASSERT_EQUAL(Y, cornering_hyst_step(&h, Y, Y, 0u));
    TEST_ASSERT_EQUAL(R, cornering_hyst_step(&h, R, R, 1u));
    cornering_hyst_reset(&h);
    TEST_ASSERT_EQUAL(G, cornering_hyst_step(&h, G, G, 2u));
    TEST_ASSERT_EQUAL(R, cornering_hyst_step(&h, R, R, 3u)); /* GREEN -> RED in one step */
}

static void test_hyst_red_to_yellow_after_hold(void)
{
    cornering_hyst_t h;
    cornering_hyst_reset(&h);
    (void)cornering_hyst_step(&h, R, R, 1000u);
    /* raw YELLOW but relaxed still RED (u between k_red - margin and k_red): stays RED. */
    TEST_ASSERT_EQUAL(R, cornering_hyst_step(&h, Y, R, 1020u));
    TEST_ASSERT_EQUAL(R, cornering_hyst_step(&h, Y, R, 1500u));
    /* relaxed below RED from 1520 on. */
    TEST_ASSERT_EQUAL(R, cornering_hyst_step(&h, Y, Y, 1520u));
    TEST_ASSERT_EQUAL(R, cornering_hyst_step(&h, Y, Y, 1520u + CORNERING_HYST_HOLD_MS - 1u));
    TEST_ASSERT_EQUAL(Y, cornering_hyst_step(&h, Y, Y, 1520u + CORNERING_HYST_HOLD_MS));
}

static void test_hyst_break_restarts_the_hold(void)
{
    cornering_hyst_t h;
    cornering_hyst_reset(&h);
    (void)cornering_hyst_step(&h, R, R, 0u);
    TEST_ASSERT_EQUAL(R, cornering_hyst_step(&h, Y, Y, 10u));
    TEST_ASSERT_EQUAL(R, cornering_hyst_step(&h, Y, R, 60u)); /* break */
    TEST_ASSERT_EQUAL(R, cornering_hyst_step(&h, Y, Y, 80u));
    TEST_ASSERT_EQUAL(R, cornering_hyst_step(&h, Y, Y, 179u));
    TEST_ASSERT_EQUAL(Y, cornering_hyst_step(&h, Y, Y, 180u));
}

static void test_hyst_yellow_to_green_and_red_to_green(void)
{
    cornering_hyst_t h;
    cornering_hyst_reset(&h);
    (void)cornering_hyst_step(&h, Y, Y, 0u);
    TEST_ASSERT_EQUAL(Y, cornering_hyst_step(&h, G, Y, 20u)); /* relaxed not GREEN */
    TEST_ASSERT_EQUAL(Y, cornering_hyst_step(&h, G, G, 40u));
    TEST_ASSERT_EQUAL(Y, cornering_hyst_step(&h, G, G, 139u));
    TEST_ASSERT_EQUAL(G, cornering_hyst_step(&h, G, G, 140u));

    cornering_hyst_reset(&h);
    (void)cornering_hyst_step(&h, R, R, 0u);
    TEST_ASSERT_EQUAL(R, cornering_hyst_step(&h, G, G, 20u));
    TEST_ASSERT_EQUAL(G, cornering_hyst_step(&h, G, G, 120u)); /* both holds done at once */
}

static void test_hyst_timer_wraps(void)
{
    cornering_hyst_t h;
    cornering_hyst_reset(&h);
    (void)cornering_hyst_step(&h, R, R, UINT32_MAX - 50u);
    TEST_ASSERT_EQUAL(R, cornering_hyst_step(&h, Y, Y, UINT32_MAX - 40u));
    TEST_ASSERT_EQUAL(Y, cornering_hyst_step(&h, Y, Y, 60u)); /* 101 ms across the wrap */
}

static void test_hyst_invalid_inputs(void)
{
    cornering_hyst_t h;
    cornering_hyst_reset(&h);
    TEST_ASSERT_EQUAL(R, cornering_hyst_step(&h, CORNERING_LEVEL_UNAVAILABLE, G, 0u));
    cornering_hyst_reset(&h);
    TEST_ASSERT_EQUAL(R, cornering_hyst_step(&h, G, (cornering_level_t)7, 0u));
    cornering_hyst_reset(&h);
    /* relaxed less severe than raw is lifted to raw. */
    TEST_ASSERT_EQUAL(Y, cornering_hyst_step(&h, Y, G, 0u));
    TEST_ASSERT_EQUAL(Y, cornering_hyst_step(&h, Y, G, 500u));
    TEST_ASSERT_EQUAL(R, cornering_hyst_step(NULL, G, G, 0u));
    cornering_hyst_reset(NULL);
}

static void test_decide_uses_shifted_thresholds(void)
{
    cornering_hyst_t h;
    /* mu 0.5: u = 0.77 lies between k_red - 0.05 = 0.75 and k_red = 0.8. */
    const float m = CORNERING_HYST_U_MARGIN;
    const float u077 = onset(PLATFORM_LIMIT_K_RED - (0.6f * m), 0.5f); /* between k_red - m and k_red */
    const float u070 = onset(PLATFORM_LIMIT_K_RED - (2.0f * m), 0.5f); /* below k_red - m */
    const float u057 = onset(PLATFORM_LIMIT_K_YELLOW - (0.6f * m), 0.5f); /* GREEN, not below k_yellow - m */
    const float u052 = onset(PLATFORM_LIMIT_K_YELLOW - (1.6f * m), 0.5f); /* below k_yellow - m */
    cornering_hyst_reset(&h);
    TEST_ASSERT_EQUAL(R, cornering_decide(&h, 25.0f, 0.5f, 0u));
    TEST_ASSERT_EQUAL(R, cornering_decide(&h, u077, 0.5f, 20u));
    TEST_ASSERT_EQUAL(R, cornering_decide(&h, u077, 0.5f, 200u)); /* never below 0.75 */
    TEST_ASSERT_EQUAL(R, cornering_decide(&h, u070, 0.5f, 220u));
    TEST_ASSERT_EQUAL(Y, cornering_decide(&h, u070, 0.5f, 320u));
    /* u 0.57 is GREEN raw but not below 0.55: stays YELLOW. */
    TEST_ASSERT_EQUAL(Y, cornering_decide(&h, u057, 0.5f, 340u));
    TEST_ASSERT_EQUAL(Y, cornering_decide(&h, u057, 0.5f, 600u));
    TEST_ASSERT_EQUAL(Y, cornering_decide(&h, u052, 0.5f, 620u));
    TEST_ASSERT_EQUAL(G, cornering_decide(&h, u052, 0.5f, 720u));
}

static void test_output_pairs(void)
{
    cornering_output_t o;
    o.source = CORNERING_SOURCE_NONE;
    o.level = CORNERING_LEVEL_UNAVAILABLE;
    TEST_ASSERT_TRUE(cornering_output_consistent(o));
    o.level = G;
    TEST_ASSERT_FALSE(cornering_output_consistent(o));
    o.source = CORNERING_SOURCE_EKF;
    TEST_ASSERT_TRUE(cornering_output_consistent(o));
    o.level = R;
    TEST_ASSERT_TRUE(cornering_output_consistent(o));
    o.level = CORNERING_LEVEL_UNAVAILABLE;
    TEST_ASSERT_FALSE(cornering_output_consistent(o));
    o.source = (cornering_source_t)2; /* OWN_IMU is not produced in v1 */
    o.level = G;
    TEST_ASSERT_FALSE(cornering_output_consistent(o));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_float_threshold_relations);
    RUN_TEST(test_lean_states);
    RUN_TEST(test_lean_dbc_range);
    RUN_TEST(test_q029_clamp_check);
    RUN_TEST(test_mu_rule);
    RUN_TEST(test_mu_lost_never_raises);
    RUN_TEST(test_decision_levels);
    RUN_TEST(test_decision_non_finite_is_red);
    RUN_TEST(test_select_return_needs_ten_frames);
    RUN_TEST(test_select_drops_at_once);
    RUN_TEST(test_select_breaks_reset_the_run);
    RUN_TEST(test_select_null);
    RUN_TEST(test_hyst_first_and_escalation);
    RUN_TEST(test_hyst_red_to_yellow_after_hold);
    RUN_TEST(test_hyst_break_restarts_the_hold);
    RUN_TEST(test_hyst_yellow_to_green_and_red_to_green);
    RUN_TEST(test_hyst_timer_wraps);
    RUN_TEST(test_hyst_invalid_inputs);
    RUN_TEST(test_decide_uses_shifted_thresholds);
    RUN_TEST(test_output_pairs);
    return UNITY_END();
}
