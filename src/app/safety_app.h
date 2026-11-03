#ifndef APP_SAFETY_APP_H
#define APP_SAFETY_APP_H

/*
 * One super-loop pass of the safety partition (D-012, D-072 item 3) and the composition
 * root of its outputs: app/ is the only place that combines features (features never
 * include each other, services never include features; architecture-guard on C2 PR2).
 *
 * Pass order (the first WCET group first, D-072 item 3; mailbox.h obligations):
 *   1. mailbox_step(now)     writes the frames offered in the previous pass, reads M7->M4
 *   2. cornering_step(now)   takes 0x081/0x022/0x020, E2E, source selection, decision
 *   3. (blind spot, later)
 *   4. outputs               0x023 every 20 ms and 0x082 every 100 ms on a cycle grid of the
 *                            M4's own timebase: checked pack, E2E protect, mailbox_offer()
 *                            (written by the next pass's mailbox_step, so <= one pass late)
 *   5. housekeeping          mailbox_health(), own faults, node health (D-101)
 *   (LED ring driver, immobilizer: later, with the board; the ring driver must honour
 *    D-072 item 5: GREEN never as a static frame.)
 *
 * The caller (the board main(), red zone, not in this repo yet) passes now_ms from the M4's
 * own millisecond timebase and calls safety_app_step() with a nominal period of at most
 * 2 ms (D-101 item 2); a measured gap above SAFETY_APP_PASS_MAX_MS is an own fault.
 * safety_app_init() returning false is a fault for the caller, never a silent continue.
 * Super-loop context only, never from an ISR. Static storage, no heap.
 */

#include "services/mailbox_core.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* D-093 item 5 / D-101 item 2: four reads per 20 ms writer cycle. */
#define SAFETY_APP_PASS_MAX_MS (5u)
/* D-101 item 3 (provisional): longest NEVER_MATCHED time before "mailbox peer missing or
 * wrong image" (E-28 (6) board tests revisit it). */
#define SAFETY_APP_PEER_BOOT_BUDGET_MS (2000u)

typedef struct {
    bool initialised;
    uint32_t level_faults; /* NODE_FAULT_* active in the last pass */
    uint32_t passes;
    uint32_t pass_gap_faults;
    uint32_t output_faults;
    uint32_t warn_sent;     /* 0x023 frames offered */
    uint32_t hb_sent;       /* 0x082 frames offered */
    uint8_t last_node_mode; /* of the last 0x082 offered */
    uint8_t error_count;
    bool rt_core_inputs_lost; /* cornering source NONE: diagnosis only, never an own fault */
} safety_app_status_t;

/* Boot: cornering_init() and mailbox_init(region, now_ms). False (NULL region) leaves the
 * application inert: no step does anything. */
// cppcheck-suppress misra-c2012-19.2 ; DEV-003: the D-093 region type, passed through to mailbox_init()
bool safety_app_init(volatile mailbox_region_t* region, uint32_t now_ms);

/* One super-loop pass. */
void safety_app_step(uint32_t now_ms);

safety_app_status_t safety_app_status(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SAFETY_APP_H */
