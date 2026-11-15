#ifndef HAL_CAN_H
#define HAL_CAN_H

/*
 * Minimal non-blocking interface to the platform-bus FDCAN of the STM32G474 (D-118, D-118
 * addendum, safety-reviewer MAJOR-1 of safety-node#16): classic CAN, 11-bit IDs, at most 8
 * data bytes. No call here ever waits: send and poll answer at once, so the super loop keeps
 * its WCET budget (D-072 item 3, CLAUDE.md rule 1).
 *
 * Hardware facts the contract is built on (RM0440, FDCAN message RAM of 212 words per
 * instance; the numbers are those of the safety review and are checked again on the board):
 * two RX FIFOs of 3 elements each and 3 TX buffers. There is no deep hardware queue, so a
 * busy bus loses frames unless the classes below are kept apart and drained in time.
 *
 * What the board implementation (red zone, with the CubeMX project) must provide:
 *
 *  1. RX classes (MAJOR-1 a). Two classes, each its own hardware FIFO via the standard-ID
 *     filters: HAL_CAN_RX_SAFETY holds 0x020, 0x022 and 0x081 (FIFO 0), HAL_CAN_RX_QM holds
 *     0x120 (FIFO 1). Every other ID is rejected in hardware. A flood of QM frames can only
 *     overflow the QM FIFO, never the safety one. Both FIFOs run in blocking mode (RXGFC
 *     F0OM/F1OM = 0): a full FIFO drops the NEWEST frame, which E2E sees as a counter gap
 *     (never an overwritten, silently newer frame), and reports "message lost" (RFxL);
 *     hal_can_rx_lost() returns that count (cumulative since
 *     hal_can_init(), saturating). Which frame is lost is not relied on: the missing frame
 *     shows up as an E2E counter gap or a timeout.
 *  2. Frame format (MAJOR-1 b). Classic CAN only. Remote-request frames and FD frames must
 *     be rejected by the peripheral (global filter: reject remote frames, FD operation off);
 *     where it cannot, it reports them with the rtr/fd flags and services/platform_link
 *     rejects them. They are never data.
 *  3. Stale TX (MAJOR-1 c). Before queuing a frame of an ID, hal_can_send() cancels any
 *     still-pending frame of the same ID from an earlier cycle (TXBCR of that buffer, wait
 *     for the cancel flag in a later call, never spin), and when the node goes bus-off it
 *     cancels everything pending. A late stale 0x023/0x082 never goes out after bus-off
 *     recovery. A cancel is not an error of this node.
 *  4. Software rings (MAJOR-1 d). If an RX interrupt feeds a software ring in front of
 *     hal_can_poll(), it is single-producer/single-consumer (ISR writes, super loop reads),
 *     fixed size, no heap, no lock, and an overflow is counted into hal_can_rx_lost().
 *  5. Send results (MINOR-1). hal_can_send() says why it refused: the TX buffers are full,
 *     the node is bus-off, the HAL is not initialised, or the arguments are wrong. Full and
 *     bus-off are bus conditions the node cannot locate (a missing ACK looks the same),
 *     the other two are local faults.
 *  6. Bus-off (MINOR-R2-3). Recovery is automatic and never waits on rt-core or on any other
 *     node: the peripheral restarts itself after a bounded backoff (the cap is Q-038, 1 s up
 *     to 30 s provisional) and never stays off for good. After recovery RX resumes through
 *     the E2E INITIAL state and the D-101 item 4 return rule in the consumers, nothing here
 *     replays or hides the gap. hal_can_bus_off_count() counts bus-off episodes (diagnosis
 *     only, never a decision input).
 * The host model is hal/host/can_host.h.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_CAN_MAX_DLC (8u)

typedef struct {
    uint32_t id;   /* 11-bit identifier, or 29-bit when extended */
    bool extended; /* the platform bus uses standard IDs only; extended frames are ignored */
    bool rtr;      /* remote-request frame: never data */
    bool fd;       /* CAN FD format frame: never data on this bus */
    uint8_t dlc;   /* data bytes, 0..HAL_CAN_MAX_DLC */
    uint8_t data[HAL_CAN_MAX_DLC];
} hal_can_frame_t;

typedef enum {
    HAL_CAN_RX_SAFETY = 0, /* 0x020, 0x022, 0x081 */
    HAL_CAN_RX_QM = 1,     /* 0x120 */
    HAL_CAN_RX_CLASS_COUNT = 2
} hal_can_rx_class_t;

typedef enum {
    HAL_CAN_SEND_OK = 0,
    HAL_CAN_SEND_FIFO_FULL,       /* all TX buffers busy: bus condition */
    HAL_CAN_SEND_BUS_OFF,         /* bus-off, cancelled and refused: bus condition */
    HAL_CAN_SEND_NOT_INITIALISED, /* local */
    HAL_CAN_SEND_BAD_ARGUMENT     /* local: NULL or dlc > HAL_CAN_MAX_DLC */
} hal_can_send_t;

/* Boot: false when the peripheral could not be brought up (a fault for the caller, never a
 * silent continue). Calling it again restarts from a clean state (FIFOs, lost counters). */
bool hal_can_init(void);

/* HAL_CAN_SEND_OK when the frame went into a TX buffer, otherwise the reason. Never blocks. */
hal_can_send_t hal_can_send(const hal_can_frame_t* frame);

/* True and the oldest received frame of the class when one is waiting. Never blocks. */
bool hal_can_poll(hal_can_rx_class_t rx_class, hal_can_frame_t* frame);

/* Messages of the class lost to a full FIFO (or ring) since hal_can_init(). */
uint32_t hal_can_rx_lost(hal_can_rx_class_t rx_class);

/* Bus-off episodes since hal_can_init() (diagnosis only). */
uint32_t hal_can_bus_off_count(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_CAN_H */
