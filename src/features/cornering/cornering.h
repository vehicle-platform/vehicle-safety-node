#ifndef FEATURES_CORNERING_CORNERING_H
#define FEATURES_CORNERING_CORNERING_H

/*
 * Layer 1 cornering glue (D-041, D-042, D-100, D-101): takes the M7->M4 frames 0x081
 * HeartbeatRtCore, 0x022 EkfFrictionMass and 0x020 EkfLean from services/mailbox, runs the
 * generated E2E check on each (compile-time length, INITIAL not usable, D-026) and the
 * generated timeout check every pass, unpacks with gen/ and feeds cornering_core.
 *
 * Called from the super loop only (app/safety_app.c), after mailbox_step() of the same
 * pass. The decision is evaluated only on fresh input (a new 0x020 or 0x022 frame, or the
 * pass that switches the source to EKF), so the D-100 hysteresis timers advance on fresh
 * frames only; between them the level is held, and a lost input still goes to UNAVAILABLE
 * at once (architecture-guard MINOR-2).
 *
 * When the M7->M4 frames stop or are not usable the M4 cannot tell an FDCAN2 bus-off on
 * the M7 from a dead rt-core (ISSUES E-28 (1)): the condition is named "rt-core inputs
 * lost", gives UNAVAILABLE (D-100 item 2) and is never counted as a fault of rt-core.
 */

#include "features/cornering/cornering_core.h"
#include "vehicle_e2e.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    vehicle_e2e_status_t lean_e2e; /* the E2E result of the last taken frame */
    vehicle_e2e_status_t mu_e2e;
    vehicle_e2e_status_t hb_e2e;
    bool lean_live;            /* latest 0x020 usable and within its E2E timeout */
    bool mu_live;              /* latest 0x022 E2E-valid with a usable value, within its timeout */
    bool hb_ok;                /* latest 0x081 E2E-valid, NORMAL and within its E2E timeout */
    bool rt_core_inputs_lost;  /* source NONE (boot included) */
    uint32_t inputs_lost_onsets; /* EKF -> NONE switches (diagnostic, never ERROR_COUNT) */
    uint32_t lean_rejected;    /* taken 0x020 frames not usable (E2E or content) */
    uint32_t mu_rejected;      /* taken 0x022 frames not E2E-valid or with an unusable value */
    uint32_t hb_rejected;      /* taken 0x081 frames not E2E-valid or not NORMAL */
    uint32_t lean_timeouts;    /* usable 0x020 stream ended by its E2E timeout */
    uint32_t mu_timeouts;
    uint32_t hb_timeouts;
    uint8_t good_frames;       /* towards CORNERING_RETURN_FRAMES */
    float mu_eff;              /* the value of the last evaluation */
} cornering_status_t;

/* Boot, before the super loop: UNAVAILABLE / NONE, all receivers unsynced. */
void cornering_init(void);

/* Once per super-loop pass, after mailbox_step(now_ms). */
void cornering_step(uint32_t now_ms);

/* The current decision; the pair is always 0x023-consistent. */
cornering_output_t cornering_output(void);

cornering_status_t cornering_status(void);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_CORNERING_CORNERING_H */
