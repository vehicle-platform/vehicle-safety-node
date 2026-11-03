#include "features/cornering/cornering_core.h"

#include "platform.h"
#include "platform_limits.h"

#include <math.h>
#include <stddef.h>

_Static_assert((uint32_t)CORNERING_LEVEL_UNAVAILABLE == PLATFORM_SAFETY_WARNING_STATUS_CORNERING_LEVEL_UNAVAILABLE_CHOICE,
               "level values follow the generated 0x023 choices");
_Static_assert((uint32_t)CORNERING_LEVEL_GREEN == PLATFORM_SAFETY_WARNING_STATUS_CORNERING_LEVEL_GREEN_CHOICE,
               "level values follow the generated 0x023 choices");
_Static_assert((uint32_t)CORNERING_LEVEL_YELLOW == PLATFORM_SAFETY_WARNING_STATUS_CORNERING_LEVEL_YELLOW_CHOICE,
               "level values follow the generated 0x023 choices");
_Static_assert((uint32_t)CORNERING_LEVEL_RED == PLATFORM_SAFETY_WARNING_STATUS_CORNERING_LEVEL_RED_CHOICE,
               "level values follow the generated 0x023 choices");
_Static_assert((uint32_t)CORNERING_SOURCE_NONE == PLATFORM_SAFETY_WARNING_STATUS_CORNERING_SOURCE_NONE_CHOICE,
               "source values follow the generated 0x023 choices");
_Static_assert((uint32_t)CORNERING_SOURCE_EKF == PLATFORM_SAFETY_WARNING_STATUS_CORNERING_SOURCE_EKF_CHOICE,
               "source values follow the generated 0x023 choices");

/* The float relations (k_yellow - margin > 0, k_red - margin > k_yellow, clamp_min > 0) are
 * not integer constant expressions in C11; test_cornering_core checks them
 * (architecture-guard MINOR-3). */
_Static_assert(CORNERING_RETURN_FRAMES < 255u, "good_frames is a capped uint8_t");

_Static_assert(CORNERING_HYST_HOLD_MS == (5u * PLATFORM_EKF_LEAN_CYCLE_TIME_MS), "D-100 item 3: 5 frames of 0x020");

/* D-075 item 5 (Q-029): a CLAMPED lean more than one LEAN_ANGLE step (0.01 deg, the DBC
 * scale; test_cornering_core checks it) below the clamp is not usable. Compared on raw
 * integers, so the one-step tolerance is exact (safety-reviewer MINOR-1). */
#define CORNERING_CLAMP_TOLERANCE_STEPS (1)
/* D-041 item 2: |theta| <= 90 deg. The DBC range of LEAN_ANGLE already implies it; kept as
 * a second check on the decoded value. */
#define CORNERING_LEAN_BOUND_DEG (90.0f)
#define CORNERING_DEG_TO_RAD (0.017453292519943295f)

/* isfinite() yields an int: compare it, so the condition is essentially Boolean (14.4). */
static bool finite_f(float x)
{
    return isfinite(x) != 0;
}

/* The lean clamp in raw LEAN_ANGLE steps (5500 for 55 deg at 0.01 deg). */
static int32_t clamp_raw(void)
{
    return (int32_t)lrintf(PLATFORM_LIMIT_LEAN_ANGLE_CLAMP_MAX_DEG / platform_ekf_lean_lean_angle_decode(1));
}

bool cornering_lean_usable(uint8_t lean_state, int16_t lean_raw, float* lean_deg)
{
    float deg;
    int32_t mag_raw;
    if (lean_deg == NULL) {
        return false;
    }
    if (!platform_estimate_state_usable(lean_state) || !platform_ekf_lean_lean_angle_is_in_range(lean_raw)) {
        return false;
    }
    deg = platform_ekf_lean_lean_angle_decode(lean_raw);
    if (!finite_f(deg)) {
        return false;
    }
    if (fabsf(deg) > CORNERING_LEAN_BOUND_DEG) {
        return false;
    }
    mag_raw = (lean_raw < 0) ? -(int32_t)lean_raw : (int32_t)lean_raw;
    if ((lean_state == PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_ESTIMATED_CHOICE) && (mag_raw > clamp_raw())) {
        return false;
    }
    if ((lean_state == PLATFORM_EKF_LEAN_LEAN_ANGLE_STATE_CLAMPED_CHOICE) &&
        (mag_raw < (clamp_raw() - CORNERING_CLAMP_TOLERANCE_STEPS))) {
        return false;
    }
    *lean_deg = deg;
    return true;
}

bool cornering_mu_value(uint8_t mu_state, uint16_t mu_raw, float* mu_eff)
{
    bool usable = false;
    if ((mu_eff != NULL) && platform_ekf_friction_mass_friction_coeff_is_in_range(mu_raw)) {
        const float mu = platform_ekf_friction_mass_friction_coeff_decode(mu_raw);
        if (finite_f(mu)) {
            if (platform_estimate_state_usable(mu_state)) {
                *mu_eff = fminf(fmaxf(mu, PLATFORM_LIMIT_FRICTION_COEFF_CLAMP_MIN), PLATFORM_LIMIT_FRICTION_COEFF_CLAMP_MAX);
                usable = true;
            } else if (mu_state == PLATFORM_EKF_FRICTION_MASS_FRICTION_COEFF_STATE_DEFAULT_CHOICE) {
                *mu_eff = fmaxf(PLATFORM_LIMIT_FRICTION_COEFF_CLAMP_MIN, fminf(mu, PLATFORM_LIMIT_FRICTION_COEFF_DEFAULT));
                usable = true;
            } else {
                usable = false; /* INVALID, RESERVED, unknown: D-101 item 5 */
            }
        }
    }
    return usable;
}

float cornering_mu_eff_lost(bool have_last, float last_mu_eff)
{
    float result = PLATFORM_LIMIT_FRICTION_COEFF_DEFAULT;
    if (have_last && finite_f(last_mu_eff)) {
        result = fmaxf(PLATFORM_LIMIT_FRICTION_COEFF_CLAMP_MIN, fminf(last_mu_eff, PLATFORM_LIMIT_FRICTION_COEFF_DEFAULT));
    }
    return result;
}

cornering_level_t cornering_level_for(float lean_deg, float mu_eff, float k_lo, float k_hi)
{
    cornering_level_t level = CORNERING_LEVEL_RED;
    if (finite_f(lean_deg) && finite_f(mu_eff) && finite_f(k_lo) && finite_f(k_hi)) {
        const float a = fabsf(lean_deg) * CORNERING_DEG_TO_RAD;
        const float s = sinf(a);
        const float c = cosf(a);
        const float lo = k_lo * mu_eff * c;
        const float hi = k_hi * mu_eff * c;
        if (finite_f(s) && finite_f(lo) && finite_f(hi)) {
            if (s < lo) {
                level = CORNERING_LEVEL_GREEN;
            } else if (s < hi) {
                level = CORNERING_LEVEL_YELLOW;
            } else {
                level = CORNERING_LEVEL_RED;
            }
        }
    }
    return level;
}

void cornering_select_init(cornering_select_t* s)
{
    if (s != NULL) {
        s->source = CORNERING_SOURCE_NONE;
        s->good_frames = 0u;
    }
}

cornering_source_t cornering_select_step(cornering_select_t* s, bool new_lean_frame, bool lean_frame_usable,
                                         bool lean_live, bool hb_ok)
{
    bool usable_now;
    if (s == NULL) {
        return CORNERING_SOURCE_NONE;
    }
    usable_now = lean_live && hb_ok;
    if (new_lean_frame) {
        if (lean_frame_usable && hb_ok) {
            if (s->good_frames < CORNERING_RETURN_FRAMES) {
                s->good_frames++;
            }
        } else {
            s->good_frames = 0u;
        }
    }
    if (!usable_now) {
        s->good_frames = 0u; /* a heartbeat loss or a timeout between frames breaks the run */
        s->source = CORNERING_SOURCE_NONE;
    } else if ((s->source == CORNERING_SOURCE_NONE) && (s->good_frames >= CORNERING_RETURN_FRAMES)) {
        s->source = CORNERING_SOURCE_EKF;
    } else {
        /* keep the source */
    }
    return s->source;
}

void cornering_hyst_reset(cornering_hyst_t* h)
{
    if (h != NULL) {
        h->level = CORNERING_LEVEL_UNAVAILABLE;
        h->below_red = false;
        h->below_red_ms = 0u;
        h->below_yellow = false;
        h->below_yellow_ms = 0u;
    }
}

static bool level_valid(cornering_level_t l)
{
    return (l == CORNERING_LEVEL_GREEN) || (l == CORNERING_LEVEL_YELLOW) || (l == CORNERING_LEVEL_RED);
}

static bool held_for(uint32_t since_ms, uint32_t now_ms)
{
    return (uint32_t)(now_ms - since_ms) >= CORNERING_HYST_HOLD_MS;
}

cornering_level_t cornering_hyst_step(cornering_hyst_t* h, cornering_level_t raw, cornering_level_t relaxed,
                                      uint32_t now_ms)
{
    cornering_level_t r = raw;
    cornering_level_t x = relaxed;
    cornering_level_t next;
    if (h == NULL) {
        return CORNERING_LEVEL_RED;
    }
    if (!level_valid(r) || !level_valid(x)) {
        r = CORNERING_LEVEL_RED;
        x = CORNERING_LEVEL_RED;
    }
    if ((int32_t)x < (int32_t)r) {
        x = r; /* relaxed is never less severe than raw */
    }

    /* The "below" runs: broken by any evaluation that is not below. */
    if (x != CORNERING_LEVEL_RED) {
        if (!h->below_red) {
            h->below_red = true;
            h->below_red_ms = now_ms;
        }
    } else {
        h->below_red = false;
    }
    if (x == CORNERING_LEVEL_GREEN) {
        if (!h->below_yellow) {
            h->below_yellow = true;
            h->below_yellow_ms = now_ms;
        }
    } else {
        h->below_yellow = false;
    }

    if (!level_valid(h->level)) {
        next = r; /* first evaluation after a reset */
    } else if ((int32_t)r > (int32_t)h->level) {
        next = r; /* escalation at once */
    } else {
        next = h->level;
        if ((next == CORNERING_LEVEL_RED) && h->below_red && held_for(h->below_red_ms, now_ms)) {
            next = CORNERING_LEVEL_YELLOW;
        }
        if ((next == CORNERING_LEVEL_YELLOW) && h->below_yellow && held_for(h->below_yellow_ms, now_ms)) {
            next = CORNERING_LEVEL_GREEN;
        }
    }
    h->level = next;
    return next;
}

cornering_level_t cornering_decide(cornering_hyst_t* h, float lean_deg, float mu_eff, uint32_t now_ms)
{
    cornering_level_t raw = cornering_level_for(lean_deg, mu_eff, PLATFORM_LIMIT_K_YELLOW, PLATFORM_LIMIT_K_RED);
    cornering_level_t relaxed = cornering_level_for(lean_deg, mu_eff, PLATFORM_LIMIT_K_YELLOW - CORNERING_HYST_U_MARGIN,
                                                    PLATFORM_LIMIT_K_RED - CORNERING_HYST_U_MARGIN);
    return cornering_hyst_step(h, raw, relaxed, now_ms);
}

bool cornering_output_consistent(cornering_output_t out)
{
    bool ok = false;
    if (out.source == CORNERING_SOURCE_NONE) {
        ok = (out.level == CORNERING_LEVEL_UNAVAILABLE);
    } else if (out.source == CORNERING_SOURCE_EKF) {
        ok = level_valid(out.level);
    } else {
        ok = false;
    }
    return ok;
}
