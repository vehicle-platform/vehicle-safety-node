#ifndef FEATURES_INDICATOR_INDICATOR_CORE_H
#define FEATURES_INDICATOR_INDICATOR_CORE_H

/*
 * Pure turn-signal input layer (D-075 item 4, D-082; D-111, provisional, defs#165; no
 * hal/, no services/). Fed with raw GPIO levels of IND_L and IND_R and a monotonic 32-bit
 * millisecond timestamp, it debounces each input and reports whether the rider is
 * signalling LEFT, RIGHT or HAZARD (both sides at once). Constants and the pin polarity:
 * indicator_config.h.
 *
 * SAFETY CONTRACT (invariants 3 and 8; read before using the output):
 *   The result may only ESCALATE a blind-spot warning (for example let an APPROACHING
 *   vehicle on the signalled side become a stronger warning). It must NEVER suppress or
 *   downgrade a PRESENT or APPROACHING warning, and it is NOT a health or plausibility
 *   signal. With INDICATOR_STATE_NONE the blind-spot decision is exactly the one it would
 *   be without this module.
 *   Detected faults (open wire, dead opto, stuck-lit > INDICATOR_STUCK_MS) read
 *   INDICATOR_STATE_NONE. Undetected faults can read as signalling: a frozen clock after
 *   signalling, a lit line with >= INDICATOR_DEBOUNCE_MS drop-outs, a lamp lit at boot
 *   (signals up to INDICATOR_HOLD_MS), wrong polarity, a floating pin. Hence signalling may
 *   only ever be used to escalate.
 *   The blind-spot decision does not exist yet (radar hardware pending); this module does
 *   not implement or assume it.
 *
 * Behaviour per side:
 *   - debounce: a level change counts after INDICATOR_DEBOUNCE_SAMPLES consecutive equal
 *     samples lasting >= INDICATOR_DEBOUNCE_MS;
 *   - signalling from the first debounced edge until INDICATOR_HOLD_MS after the last
 *     debounced edge (either direction), so the dark half of a flash is covered;
 *   - an input active for INDICATOR_STUCK_MS without an edge latches a stuck fault: not
 *     signalling, flag set until the input goes inactive (that release is not treated as
 *     a signalling edge).
 * HAZARD = both sides signalling (hazard lights flash both lamps together); a side with a
 * stuck fault does not count, so stuck-left plus a real right flash reports RIGHT.
 *
 * Timing uses unsigned 32-bit subtraction, so a wrap of the millisecond counter (49.7
 * days) is harmless; a hold is dropped as soon as it expires, so a quiet input never
 * reads as signalling again after a wrap. This holds as long as no gap between two step()
 * calls reaches 2^32 - INDICATOR_HOLD_MS ms; the real timing limit is the super-loop
 * period of INDICATOR_MAX_LOOP_MS (indicator_config.h).
 * No dynamic memory; all state is in the caller-owned indicator_core_t.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    INDICATOR_STATE_NONE = 0,
    INDICATOR_STATE_LEFT = 1,
    INDICATOR_STATE_RIGHT = 2,
    INDICATOR_STATE_HAZARD = 3,
} indicator_state_t;

/* indicator_output_t.fault_flags bits (diagnostic only; the state already reads NONE
 * for that side). */
#define INDICATOR_FAULT_LEFT_STUCK (0x01u)
#define INDICATOR_FAULT_RIGHT_STUCK (0x02u)

typedef struct {
    indicator_state_t state;
    uint8_t fault_flags;
} indicator_output_t;

/* One input. Fields are internal; zero-initialise through indicator_core_init(). */
typedef struct {
    uint32_t cand_since_ms;   /* when the current raw level started */
    uint32_t last_edge_ms;    /* last debounced edge (valid while has_edge) */
    uint32_t active_since_ms; /* last debounced rising edge (valid while debounced) */
    uint8_t cand_count;       /* consecutive samples at the current raw level, saturating */
    bool primed;              /* first sample seen */
    bool raw_active;
    bool debounced;
    bool has_edge;            /* inside the hold */
    bool stuck;
} indicator_side_t;

typedef struct {
    indicator_side_t left;
    indicator_side_t right;
} indicator_core_t;

/* Boot: not signalling, no fault, no sample. */
void indicator_core_init(indicator_core_t *c);

/* One sample of both pins (true = pin level HIGH) at now_ms. NULL c is ignored. Returns
 * the result after this sample. */
indicator_output_t indicator_core_step(indicator_core_t *c, uint32_t now_ms,
                                       bool left_pin_high, bool right_pin_high);

#ifdef __cplusplus
}
#endif

#endif
