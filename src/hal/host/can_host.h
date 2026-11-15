#ifndef HAL_HOST_CAN_HOST_H
#define HAL_HOST_CAN_HOST_H

/*
 * Host model of the platform bus as one node sees it (hal/can.h), for the tests. No heap.
 *   RX: one static FIFO per class (safety, QM); depth is a parameter, default 3 per class
 *       like the FDCAN RX FIFOs of the G474 (RM0440). A full FIFO drops the new frame and
 *       counts it as lost (hal_can_rx_lost). The test injects what the other nodes send.
 *   TX: a static buffer set (depth parameter, default 3 like the FDCAN TX buffers). Sending
 *       a frame cancels a still-pending frame of the same ID (counted, not an error); going
 *       bus-off cancels everything pending. The test reads what this node transmitted.
 * Injectable faults: refused sends (FIFO full), bus-off, init failure.
 */

#include "hal/can.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_HOST_FIFO_DEPTH_MAX (64u)
#define CAN_HOST_FIFO_DEPTH_DEFAULT (3u)

/* Back to power-on: empty FIFOs, depths 3, not initialised, no injected faults, counters 0. */
void can_host_reset(void);

/* Depth per RX class / of the TX buffer set, 1..CAN_HOST_FIFO_DEPTH_MAX (clamped). */
void can_host_set_rx_depth(uint32_t depth);
void can_host_set_tx_depth(uint32_t depth);

/* Test side of the bus: a frame arrives in the FIFO of the class (what the board's acceptance
 * filters would route). False (and the class's lost count + 1) when full. */
bool can_host_inject(hal_can_rx_class_t rx_class, const hal_can_frame_t* frame);

/* Test side of the bus: the oldest frame this node transmitted, if any. */
bool can_host_tx_pop(hal_can_frame_t* frame);

/* The next n hal_can_send() calls are refused with HAL_CAN_SEND_FIFO_FULL. */
void can_host_refuse_next_sends(uint32_t n);

/* While set, hal_can_send() is refused with HAL_CAN_SEND_BUS_OFF. Setting it cancels every
 * pending TX frame (the board does the same on bus-off). */
void can_host_set_bus_off(bool bus_off);

/* hal_can_init() fails while set. */
void can_host_set_init_fails(bool fails);

uint32_t can_host_rx_pending(hal_can_rx_class_t rx_class);
uint32_t can_host_tx_pending(void);
uint32_t can_host_tx_accepted(void);
uint32_t can_host_tx_refused(void);
uint32_t can_host_tx_cancelled(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_HOST_CAN_HOST_H */
