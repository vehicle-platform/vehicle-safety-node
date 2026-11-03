#ifndef APP_NODE_HEALTH_CORE_H
#define APP_NODE_HEALTH_CORE_H

/*
 * Pure 0x082 HeartbeatSafety semantics (D-101, the safety-node counterpart of D-064):
 *
 *   NODE_MODE   INIT for a frame built before the first super-loop pass completed;
 *               DEGRADED while the cornering source is not EKF (UNAVAILABLE included, so
 *               also when rt-core's inputs are lost) or an own fault was active in any pass
 *               since the previous 0x082 frame (an event fault shows in at least one frame,
 *               no further latch); NORMAL otherwise.
 *   ERROR_COUNT onsets of the M4's OWN faults since boot, detected every pass (a level
 *               fault on inactive -> active, an event fault once per event), saturating at
 *               the DBC maximum. The loss of rt-core's inputs is never counted (E-28 (1)).
 *   UPTIME      whole seconds since init from wrap-safe ms deltas, saturating.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Own fault bits (D-101 item 2). */
#define NODE_FAULT_PEER (1u << 0)     /* level: mailbox peer missing or wrong image */
#define NODE_FAULT_OUTPUT (1u << 1)   /* event: a 0x023/0x082 pack_checked or offer failure */
#define NODE_FAULT_PASS_GAP (1u << 2) /* event: a super-loop pass gap above 5 ms */
#define NODE_FAULT_ALL (NODE_FAULT_PEER | NODE_FAULT_OUTPUT | NODE_FAULT_PASS_GAP)

/* Raw NODE_MODE values; equal the generated 0x082 choices (checked in the .c). */
#define NODE_MODE_INIT (0u)
#define NODE_MODE_NORMAL (1u)
#define NODE_MODE_DEGRADED (2u)

typedef struct {
    bool pass_completed;
    uint32_t level_prev; /* level faults active in the previous pass */
    uint32_t seen;       /* any fault bit active since the previous 0x082 frame */
    uint8_t error_count;
    uint32_t uptime_s;
    uint32_t rem_ms; /* ms not yet counted into uptime_s */
    uint32_t last_ms;
} node_health_t;

void node_health_init(node_health_t* h, uint32_t now_ms);

/* Once per pass, before building the frames: level_faults are the level-fault bits active
 * now, event_faults the event-fault bits that fired since the previous call. Bits outside
 * NODE_FAULT_ALL are ignored. */
void node_health_pass(node_health_t* h, uint32_t level_faults, uint32_t event_faults, uint32_t now_ms);

/* Once per pass, after the frames were built. */
void node_health_pass_done(node_health_t* h);

/* NODE_MODE for a frame built now. */
uint8_t node_health_mode(const node_health_t* h, bool source_ekf);

/* After a 0x082 frame was offered: starts the next "seen since" window. */
void node_health_frame_sent(node_health_t* h);

uint8_t node_health_error_count(const node_health_t* h);

/* UPTIME, saturated at the 16-bit DBC maximum. */
uint16_t node_health_uptime(const node_health_t* h);

#ifdef __cplusplus
}
#endif

#endif /* APP_NODE_HEALTH_CORE_H */
