#ifndef APP_SAFETY_APP_H
#define APP_SAFETY_APP_H

/*
 * One super-loop pass of the safety partition (D-012, D-072 item 3) and the composition
 * root of its outputs: app/ is the only place that combines features (features never
 * include each other, services never include features; architecture-guard on C2 PR2).
 *
 * Pass order (the first WCET group first, D-072 item 3):
 *   1. platform_link_step(now) reads the platform-bus RX FIFO (bounded), DLC + E2E per frame
 *   2. cornering_step(now)   takes 0x081/0x022/0x020, source selection, decision
 *   3. (blind spot, later)
 *   4. outputs               0x023 every 20 ms and 0x082 every 100 ms on a cycle grid of the
 *                            node's own timebase: checked pack, E2E protect, platform_link_send()
 *                            (into the CAN TX buffer at once; never retried; a refusal for a
 *                            bus reason is counted, only a local one is an own output fault)
 *   5. housekeeping          own faults, node health (D-101)
 *   (LED ring driver, immobilizer: later, with the board; the ring driver must honour
 *    D-072 item 5: GREEN never as a static frame.)
 *
 * The caller (the board main(), red zone, not in this repo yet) passes now_ms from the node's
 * own millisecond timebase and calls safety_app_step() with a nominal period of at most
 * 2 ms (D-101 item 2); a measured gap above SAFETY_APP_PASS_MAX_MS is an own fault.
 * safety_app_init() returning false is a fault for the caller, never a silent continue.
 * Super-loop context only, never from an ISR. Static storage, no heap.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* D-101 item 2: the longest pass gap; four passes fit one 0x020 cycle, so the RX FIFO of the
 * four accepted IDs is drained several times per cycle (D-118). */
#define SAFETY_APP_PASS_MAX_MS (5u)

typedef struct {
    bool initialised;
    uint32_t level_faults; /* NODE_FAULT_* active in the last pass */
    uint32_t passes;
    uint32_t pass_gap_faults;
    uint32_t output_faults;
    uint32_t link_tx_refused;  /* frames the CAN HAL refused (0x023 + 0x082), diagnosis */
    uint32_t link_rx_rejected; /* frames of the four RX IDs that failed DLC or E2E (INITIAL included) */
    uint32_t link_rx_unknown;  /* frames of any other ID, ignored */
    uint32_t warn_bus_refused; /* 0x023 refused for a bus reason: never an own fault (MINOR-1) */
    uint32_t hb_bus_refused;   /* the same for 0x082 */
    uint32_t warn_sent;     /* 0x023 frames sent */
    uint32_t hb_sent;       /* 0x082 frames sent */
    uint8_t last_node_mode; /* of the last 0x082 sent */
    uint8_t error_count;
    bool rt_core_inputs_lost; /* cornering source NONE: diagnosis only, never an own fault */
} safety_app_status_t;

/* Boot: cornering_init() and platform_link_init(now_ms). False (the CAN HAL did not come up)
 * leaves the application inert: no step does anything. */
bool safety_app_init(uint32_t now_ms);

/* One super-loop pass. */
void safety_app_step(uint32_t now_ms);

safety_app_status_t safety_app_status(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SAFETY_APP_H */
