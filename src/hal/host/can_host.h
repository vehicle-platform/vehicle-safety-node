#ifndef HAL_HOST_CAN_HOST_H
#define HAL_HOST_CAN_HOST_H

/*
 * Host model of the platform bus as one node sees it (hal/can.h), for the tests: a fixed-size
 * static RX FIFO the test injects into (what the other nodes put on the bus) and a fixed-size
 * static TX FIFO the test reads (what this node transmitted). No heap. Faults can be injected:
 * refused sends and an init failure. A full FIFO drops the new frame and counts it.
 */

#include "hal/can.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_HOST_FIFO_DEPTH (64u)

/* Back to power-on: empty FIFOs, not initialised, no injected faults, counters 0. */
void can_host_reset(void);

/* Test side of the bus: a frame arrives at this node. False (and rx_overflows++) when full. */
bool can_host_inject(const hal_can_frame_t* frame);

/* Test side of the bus: the oldest frame this node transmitted, if any. */
bool can_host_tx_pop(hal_can_frame_t* frame);

/* The next n hal_can_send() calls are refused (a full TX FIFO or a bus-off node). */
void can_host_refuse_next_sends(uint32_t n);

/* hal_can_init() fails while set. */
void can_host_set_init_fails(bool fails);

uint32_t can_host_rx_pending(void);
uint32_t can_host_tx_pending(void);
uint32_t can_host_rx_overflows(void);
uint32_t can_host_tx_accepted(void);
uint32_t can_host_tx_refused(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_HOST_CAN_HOST_H */
