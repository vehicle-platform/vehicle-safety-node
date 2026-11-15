#ifndef HAL_CAN_H
#define HAL_CAN_H

/*
 * Minimal non-blocking interface to the platform-bus FDCAN of the STM32G474 (D-118): classic
 * CAN, 11-bit IDs, at most 8 data bytes. No call here ever waits: send and poll answer at
 * once, so the super loop keeps its WCET budget (D-072 item 3, CLAUDE.md rule 1).
 *
 * The board implementation (red zone, with the CubeMX project) must:
 *   - accept only the four RX IDs of services/platform_link.h in hardware (standard-ID
 *     filters, everything else rejected), so a busy bus cannot flood the RX FIFO;
 *   - never transmit anything but what hal_can_send() is given, and never queue more than
 *     the TX FIFO holds (a full FIFO or a bus-off node refuses, it never blocks or retries);
 *   - leave FDCAN error handling (bus-off recovery) to its own code: this interface only
 *     reports "sent into the TX FIFO" or "refused".
 * The host model is hal/host/can_host.h.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_CAN_MAX_DLC (8u)

typedef struct {
    uint32_t id;       /* 11-bit identifier, or 29-bit when extended */
    bool extended;     /* the platform bus uses standard IDs only; extended frames are ignored */
    uint8_t dlc;       /* data bytes, 0..HAL_CAN_MAX_DLC */
    uint8_t data[HAL_CAN_MAX_DLC];
} hal_can_frame_t;

/* Boot: false when the peripheral could not be brought up (a fault for the caller, never a
 * silent continue). Calling it again restarts from a clean state. */
bool hal_can_init(void);

/* True when the frame went into the TX FIFO; false when refused (FIFO full, bus-off, not
 * initialised, NULL or dlc > HAL_CAN_MAX_DLC). Never blocks. */
bool hal_can_send(const hal_can_frame_t* frame);

/* True and the oldest received frame when one is waiting; false otherwise. Never blocks. */
bool hal_can_poll(hal_can_frame_t* frame);

#ifdef __cplusplus
}
#endif

#endif /* HAL_CAN_H */
