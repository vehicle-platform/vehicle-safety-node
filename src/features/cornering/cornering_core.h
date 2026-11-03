#ifndef FEATURES_CORNERING_CORNERING_CORE_H
#define FEATURES_CORNERING_CORNERING_CORE_H

/*
 * Pure Layer 1 cornering logic (no hal/, no services/): input usability, the D-029
 * friction rule, the D-042 item 3 source selection with the D-075 item 6 (4) / D-101
 * item 4 return rule, the division-free D-041 decision and the D-100 item 3 level
 * hysteresis. Host-tested; the glue (cornering.c) feeds it from the mailbox.
 *
 * v1 scope (D-100): lateral only (Q-022 open), no own-IMU estimator (Q-023 (5)/(6) open).
 * The own-IMU source is "not usable" by construction, so whenever rt-core's lean is not
 * usable the only outcome is UNAVAILABLE (D-042 item 4), never GREEN. No OWN_IMU branch
 * exists until the estimator lands with the IMU hardware.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Values equal the generated 0x023 CORNERING_LEVEL choices (checked in cornering_core.c).
 * The order GREEN < YELLOW < RED is the severity order; UNAVAILABLE is never compared by
 * order (0x023 rule: test it by value). */
typedef enum {
    CORNERING_LEVEL_UNAVAILABLE = 0,
    CORNERING_LEVEL_GREEN = 1,
    CORNERING_LEVEL_YELLOW = 2,
    CORNERING_LEVEL_RED = 3,
} cornering_level_t;

/* Values equal the generated 0x023 CORNERING_SOURCE choices. OWN_IMU (2) is not produced
 * in v1 (D-100 item 2). */
typedef enum {
    CORNERING_SOURCE_NONE = 0,
    CORNERING_SOURCE_EKF = 1,
} cornering_source_t;

/* The decision output. Invariant: source NONE <=> level UNAVAILABLE (0x023 pairing). */
typedef struct {
    cornering_level_t level;
    cornering_source_t source;
} cornering_output_t;

/* D-100 item 3 (provisional, tuned on HIL): a less severe level is taken only after u has
 * stayed below the lower level's threshold minus this margin (absolute u) ... */
#define CORNERING_HYST_U_MARGIN (0.05f)
/* ... for this long without a break, on the M4's own timebase (5 frames of 0x020). */
#define CORNERING_HYST_HOLD_MS (100u)
/* D-075 item 6 (4), D-101 item 4 (provisional): consecutive usable 0x020 frames, each
 * received while the 0x081 heartbeat is OK and NORMAL, before the source becomes EKF;
 * boot included. */
#define CORNERING_RETURN_FRAMES (10u)

/* A 0x020 frame whose E2E check passed: true if its content is usable (D-041 item 2:
 * LEAN_ANGLE_STATE ESTIMATED or CLAMPED, raw value within the DBC range so |theta| <= 90
 * deg and finite; D-075 item 5 / Q-029: ESTIMATED with |theta| > the clamp, or CLAMPED with
 * |theta| < clamp - 0.01 deg (one raw step, compared on raw integers), is not usable). *lean_deg gets the decoded lean when true.
 * NULL lean_deg gives false. */
bool cornering_lean_usable(uint8_t lean_state, int16_t lean_raw, float* lean_deg);

/* D-101 item 5: while 0x022 (the frame or its value) is not usable, mu_eff = min(the last D-029 result of a valid
 * 0x022, friction_coeff_default), or the default with no valid 0x022 since boot
 * (have_last false or a non-finite last value). Never above the default, never below
 * clamp_min: a lost low mu never raises mu_eff (safety-reviewer MAJOR-1 on C2 PR2). */
float cornering_mu_eff_lost(bool have_last, float last_mu_eff);

/* The D-029 rule (limits/platform_limits.yaml friction_coeff_default) for the value of an
 * E2E-valid 0x022: true and *mu_eff set when the value is usable:
 *   ESTIMATED/CLAMPED -> the received value clamped to [clamp_min, clamp_max];
 *   DEFAULT -> max(clamp_min, min(received, friction_coeff_default)).
 * False (*mu_eff unchanged) for INVALID (the 2-bit FRICTION_COEFF_STATE has no RESERVED value), a raw value outside
 * the DBC range, a non-finite value or NULL: the caller then uses cornering_mu_eff_lost()
 * (D-101 item 5, which narrows D-029's "INVALID -> default" on this side). */
bool cornering_mu_value(uint8_t mu_state, uint16_t mu_raw, float* mu_eff);

/* The D-041 level for the thresholds k_lo < k_hi, division-free: GREEN only if
 * sin|theta| < k_lo * mu * cos|theta|, YELLOW only if sin|theta| < k_hi * mu * cos|theta|,
 * otherwise RED. Any non-finite input or intermediate gives RED. */
cornering_level_t cornering_level_for(float lean_deg, float mu_eff, float k_lo, float k_hi);

/* D-042 item 3 source selection (NONE/EKF), D-075 item 6 (4) asymmetric switching. */
typedef struct {
    cornering_source_t source;
    uint8_t good_frames; /* consecutive usable 0x020 frames with the heartbeat OK, capped */
} cornering_select_t;

void cornering_select_init(cornering_select_t* s);

/* Once per pass. new_lean_frame: a 0x020 frame was checked in this pass, lean_frame_usable
 * its result (E2E valid and content usable). lean_live: the latest 0x020 frame was usable
 * and its E2E timeout has not expired. hb_ok: the latest 0x081 frame was E2E-valid with
 * NODE_MODE NORMAL and its E2E timeout has not expired. EKF -> NONE at once when
 * lean_live && hb_ok fails; NONE -> EKF only after CORNERING_RETURN_FRAMES usable frames in
 * a row with hb_ok (any break resets the count). Returns the source. */
cornering_source_t cornering_select_step(cornering_select_t* s, bool new_lean_frame, bool lean_frame_usable,
                                         bool lean_live, bool hb_ok);

/* D-100 item 3 level hysteresis. */
typedef struct {
    cornering_level_t level; /* UNAVAILABLE after reset */
    bool below_red;          /* u < k_red - margin since below_red_ms, without a break */
    uint32_t below_red_ms;
    bool below_yellow; /* u < k_yellow - margin since below_yellow_ms, without a break */
    uint32_t below_yellow_ms;
} cornering_hyst_t;

void cornering_hyst_reset(cornering_hyst_t* h);

/* One evaluation with a usable source. raw = cornering_level_for(k_yellow, k_red),
 * relaxed = cornering_level_for(k_yellow - margin, k_red - margin) (never less severe than
 * raw). A more severe raw level is taken at once; RED -> YELLOW after relaxed has been
 * below RED for CORNERING_HYST_HOLD_MS, -> GREEN after relaxed has been GREEN that long.
 * After a reset the level starts at raw. UNAVAILABLE raw/relaxed values are not valid
 * inputs and give RED. Returns the level (never UNAVAILABLE). */
cornering_level_t cornering_hyst_step(cornering_hyst_t* h, cornering_level_t raw, cornering_level_t relaxed,
                                      uint32_t now_ms);

/* The whole decision for one pass with a usable source: raw and relaxed levels at lean/mu
 * with the generated k_yellow/k_red, then the hysteresis. */
cornering_level_t cornering_decide(cornering_hyst_t* h, float lean_deg, float mu_eff, uint32_t now_ms);

/* True if the pair is one 0x023 allows: NONE with UNAVAILABLE, or EKF with GREEN, YELLOW
 * or RED. */
bool cornering_output_consistent(cornering_output_t out);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_CORNERING_CORNERING_CORE_H */
