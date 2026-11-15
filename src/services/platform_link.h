#ifndef SERVICES_PLATFORM_LINK_H
#define SERVICES_PLATFORM_LINK_H

/*
 * The safety MCU's link to the platform CAN bus (D-118; replaces the H755 inter-core
 * mailbox of D-072 item 4c / D-093). The node is a plain participant of the bus:
 *
 *   RX  0x020 EkfLean, 0x022 EkfFrictionMass, 0x081 HeartbeatRtCore   safety inputs, E2E
 *       0x120 SafetyRequest                                            QM, immobilizer only
 *   TX  0x023 SafetyWarningStatus (20 ms), 0x082 HeartbeatSafety (100 ms)   E2E-protected
 *
 * IDs, lengths and E2E parameters come only from the generated code (gen/c/safety). Frames
 * of any other ID and extended frames are ignored and counted.
 *
 * What the link does with a received frame (the E2E profile of D-005, e2e-profile.md, as it
 * ran on the mailbox before):
 *   - DLC must equal the generated length. A frame with a known ID and another DLC is not
 *     passed to the E2E check; it reaches the consumer as VEHICLE_E2E_BAD_ARGUMENT, so it is
 *     never usable (fail safe: a foreign or broken sender on a safety ID).
 *   - the generated per-message E2E check (CRC with the Data ID, counter, timeout) runs on
 *     EVERY frame, not only the newest one per pass. Two senders of one ID therefore break
 *     the counter sequence (WRONG_SEQUENCE / REPEATED / WRONG_CRC) even when both land
 *     inside one pass; a masquerade is never silently used.
 *   - the newest payload and the E2E verdict are kept per message until platform_link_take().
 *     If any frame since the last take failed, take reports that failure (a later good
 *     frame cannot hide it), so the consumer falls back (D-042) for that pass.
 *
 * Timeouts are the generated check_timeout of the same E2E state (platform_link_check_timeout):
 * a message that stops is NO_NEW_DATA after its E2E timeout, as it was on the mailbox. The
 * link cannot tell a dead rt-core from a bus fault and does not try to (D-118): the consumers
 * name it "rt-core inputs lost" and fall back.
 *
 * TX: platform_link_send() hands one finished (packed with pack_checked, E2E-protected)
 * frame to the HAL at once. A refusal is counted and reported to the caller; the link never
 * retries and never loops. The caller's E2E counter has advanced, so the receiver sees a
 * counter gap, which is the intended evidence of a lost frame.
 *
 * Super-loop context only, never from an ISR (the HAL FIFOs are the only cross-context
 * state). One pass: platform_link_step() first (reads at most PLATFORM_LINK_RX_BURST_MAX
 * frames, bounded), then the consumers take. Static storage, no heap.
 */

#include "vehicle_e2e.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Frames read from the HAL per pass: with the four IDs the bus carries at most about two
 * frames per 5 ms pass (0x020 every 20 ms, the others every 100 ms), so 8 is headroom, and
 * the loop stays bounded. Frames beyond it wait in the HAL FIFO for the next pass. */
#define PLATFORM_LINK_RX_BURST_MAX (8u)
#define PLATFORM_LINK_PAYLOAD_BYTES (8u)

typedef enum {
    PLATFORM_LINK_RX_EKF_LEAN = 0,           /* 0x020, class safety */
    PLATFORM_LINK_RX_EKF_FRICTION_MASS = 1,  /* 0x022, class safety */
    PLATFORM_LINK_RX_HEARTBEAT_RT_CORE = 2,  /* 0x081, class safety */
    PLATFORM_LINK_RX_SAFETY_REQUEST = 3,     /* 0x120, class QM: never gates cornering, blind spot
                                                or D-042 (D-072 item 7, unchanged by D-118) */
    PLATFORM_LINK_RX_COUNT = 4
} platform_link_rx_t;

typedef enum {
    PLATFORM_LINK_TX_SAFETY_WARNING_STATUS = 0, /* 0x023 */
    PLATFORM_LINK_TX_HEARTBEAT_SAFETY = 1,      /* 0x082 */
    PLATFORM_LINK_TX_COUNT = 2
} platform_link_tx_t;

typedef struct {
    bool seen;                  /* a frame of this ID was received since init */
    uint32_t last_rx_ms;        /* receive time of the last frame (valid when seen) */
    vehicle_e2e_status_t last_status; /* E2E verdict of the last frame */
    uint32_t frames;            /* frames of this ID with the right DLC */
    uint32_t e2e_rejected;      /* ... of which not E2E-valid (INITIAL included) */
    uint32_t bad_dlc;           /* frames of this ID with a wrong DLC */
    uint32_t take_overruns;     /* frames replaced before a take (several per pass) */
} platform_link_rx_health_t;

typedef struct {
    bool initialised;
    platform_link_rx_health_t rx[PLATFORM_LINK_RX_COUNT];
    uint32_t rx_unknown;        /* frames of any other ID, or extended: ignored */
    uint32_t tx_sent[PLATFORM_LINK_TX_COUNT];
    uint32_t tx_refused[PLATFORM_LINK_TX_COUNT]; /* the HAL refused the frame */
    uint32_t tx_bad_argument;   /* wrong length or NULL: never sent */
    uint32_t skipped_steps;     /* platform_link_step() calls without a successful init */
} platform_link_health_t;

/* Boot, before the super loop: resets all state and brings up the HAL. False when the HAL
 * fails: the link stays inert (take/send/step do nothing) and the caller treats it as a
 * fault, never a silent continue. */
bool platform_link_init(uint32_t now_ms);

/* Once per super-loop pass: drains the HAL RX FIFO (bounded), filters, checks DLC and E2E. */
void platform_link_step(uint32_t now_ms);

/* True once per received frame (several frames since the last take count once): copies the
 * newest payload into out (always PLATFORM_LINK_PAYLOAD_BYTES, bytes past the generated
 * length are 0) and sets *status to the E2E verdict: the first failure since the last take
 * if there was one, otherwise the verdict of the newest frame. The caller uses the payload
 * only when vehicle_e2e_is_valid(*status), and unpacks it with the generated length. */
bool platform_link_take(platform_link_rx_t rx, uint8_t out[PLATFORM_LINK_PAYLOAD_BYTES],
                        vehicle_e2e_status_t* status);

/* The generated E2E timeout check of the message at now_ms: NO_NEW_DATA once its timeout
 * expired (or it never arrived), OK otherwise. Resyncs the receiver, as on CAN (INITIAL
 * is the next verdict). */
vehicle_e2e_status_t platform_link_check_timeout(platform_link_rx_t rx, uint32_t now_ms);

/* Sends one frame: len must equal the generated length of the message. False (counted) when
 * the link is not initialised, the arguments are wrong or the HAL refuses. */
bool platform_link_send(platform_link_tx_t tx, const uint8_t* data, uint8_t len);

platform_link_health_t platform_link_health(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_PLATFORM_LINK_H */
