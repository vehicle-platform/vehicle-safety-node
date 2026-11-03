#ifndef SERVICES_MAILBOX_H
#define SERVICES_MAILBOX_H

/*
 * The M4 side of the M7<->M4 mailbox (D-072 item 4c, D-093 items 3-6 and the 2026-11-03
 * amendment). Ownership comes from the generated slot table (gen/ platform_mailbox.h):
 * the M4 writes its layout word and the DIR_M4_TO_M7 slots (0x023 SafetyWarningStatus,
 * 0x082 HeartbeatSafety), and reads the DIR_M7_TO_M4 slots (0x020, 0x022, 0x081 class
 * safety; 0x120 class QM).
 *
 * One accessing context on the M4: the super loop. mailbox_init(), mailbox_offer(),
 * mailbox_step(), mailbox_take() and mailbox_health() are called from it only, never from
 * an ISR (a one-step HSEM take succeeds again for the same core, D-093 item 5), so the
 * state here needs no lock. One pass:
 *
 *   mailbox_step(now)       rewrites the M4 layout word; writes each pending M4->M7 frame
 *                           under its HSEM or defers it to the next pass (no wait, no
 *                           retry loop, the M4 never spin-waits, D-072 item 4c); reads each
 *                           M7->M4 slot under its HSEM ("no new frame" when refused).
 *   mailbox_take(slot, out) the newest frame read from an M7->M4 slot since the last take,
 *                           at most once. The caller runs the E2E check on it with the
 *                           compile-time length (never with the slot's len).
 *   mailbox_offer(slot, ..) an E2E-protected M4->M7 frame becomes the slot's pending frame;
 *                           a frame still pending is replaced and counted as an overrun
 *                           (bounded, counted loss, D-093 amendment (a)). Written at the
 *                           next mailbox_step().
 *
 * What a reader delivers (D-093 item 5): nothing at the first read of a slot (its
 * write_seq becomes the baseline, so a frame from before an M4 reset is never used);
 * afterwards a frame only when write_seq changed, the M7's layout word equals
 * PLATFORM_MAILBOX_LAYOUT_ID and len equals the gen/ length. Timeouts, counters and CRCs
 * are NOT checked here: the caller's E2E check does that exactly as on CAN, and a missing
 * frame is its timeout (D-042 fallback).
 *
 * An FDCAN2 bus-off on the M7 also stops new M7->M4 frames (D-093 amendment (b), ISSUES
 * E-28 (1)), and the M4 cannot tell it from a dead rt-core (0x081 stops too). Consumers
 * name it "rt-core inputs lost", take the D-042 fallback and never attribute it to rt-core
 * (0x082 NODE_MODE/ERROR_COUNT, a DTC).
 *
 * Writes need a matching peer: an M4->M7 frame is committed only in a pass that sees the
 * M7's layout word equal to the own ID; otherwise it stays pending (and is replaced by the
 * next one, counted). An M7 image with another layout may place its own data at this
 * image's slot offsets, so the M4 never writes there (safety-reviewer MINOR-5). The M7's
 * reader rejects such frames anyway.
 *
 * Obligations of the super loop (later PR): a pass <= 5 ms (readers poll four times per
 * 20 ms writer cycle, D-093 item 5); mailbox_step() reads before the cornering step and the
 * offers after it, in the first WCET group (D-072 item 3); the IRQ-masked locked sections
 * count in the timer ISR latency budget; mailbox_init() returning false is a fault, never a
 * silent continue; NEVER_MATCHED for longer than the M7's boot budget (peer_wait_ms) is the
 * wrong-image-pair fault, since a system reset restarts both cores and MISMATCH needs a
 * match first (D-072 item 4e).
 *
 * services/mailbox_core.[ch] (the pure D-093 item 5 logic, direction-agnostic) is a
 * byte-identical fork of vehicle-rt-core commit c7a8243 (the rt-core#62 merge, after
 * v0.12.0; reviewed there by safety-reviewer), and hal/hsem.h has the same code. safety-node
 * owns its copies and never includes rt-core sources (D-072 item 6). CI fails when they
 * differ from that pinned commit: the contract changes in both repos together, with
 * safety-reviewer on each, and the pinned commit is moved deliberately.
 *
 * Inert until mailbox_init(): before it, offer/take return false and step counts the
 * skipped passes. The image setup (red zone, with the linker section and the MPU region)
 * passes the SRAM4 region at PLATFORM_MAILBOX_REGION_ADDR (verified: false). Static
 * storage, no heap.
 */

#include "services/mailbox_core.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The M7's layout word as seen by the M4. */
typedef enum {
    MAILBOX_PEER_NEVER_MATCHED = 0, /* not seen equal to the own ID since init (M7 not up yet,
                                       or an M7 image with another layout) */
    MAILBOX_PEER_OK = 1,            /* equal now, or differing for at most the grace time */
    MAILBOX_PEER_MISMATCH = 2,      /* was equal, now differs for longer than the grace time */
} mailbox_peer_t;

/* Health only: the D-042 source criterion is the receiver's E2E state (INITIAL is not
 * usable, D-026), never these flags. A slot reads "not stale" for one timeout after its
 * baseline even if it never delivered a frame. */
typedef struct {
    bool initialised;
    mailbox_peer_t peer;
    uint32_t peer_wait_ms;       /* ms since init (NEVER_MATCHED) or since the last match */
    uint32_t safety_stale_slots; /* bit i: class-safety M7->M4 slot i (0x020/0x022/0x081)
                                    without a new frame within its E2E timeout */
    uint32_t qm_stale_slots;     /* the same for the class-QM slot 0x120; it never gates
                                    cornering, blind spot or D-042 (D-072 item 7) */
    uint32_t write_done;    /* M4->M7 frames committed */
    uint32_t write_deferred; /* passes a pending M4->M7 frame waited for its HSEM */
    uint32_t write_overruns; /* M4->M7 frames replaced before they were written */
    uint32_t read_busy;     /* M7->M4 reads with the HSEM not taken */
    uint32_t read_rejected; /* M7->M4 frames dropped: layout mismatch or bad len */
    uint32_t read_frames;   /* M7->M4 frames delivered to the slot buffers */
    uint32_t take_overruns; /* M7->M4 frames replaced in the buffer before a take */
    uint32_t skipped_steps; /* mailbox_step() calls before mailbox_init() */
} mailbox_health_t;

/* Boot only, before the super loop: frees this core's mailbox HSEMs (one per slot, never
 * ID 0, never the M7's), writes the M4 layout word and touches no other word of the
 * region (never clears it, D-093 item 5). Every call starts over (pending frames, slot
 * buffers, baselines, counters). NULL makes the module inert and returns false. */
bool mailbox_init(volatile mailbox_region_t* region, uint32_t now_ms);

/* True if slot is an M4->M7 slot, data is not NULL, len is exactly the slot's gen/ length
 * and the module is initialised: the frame is now the slot's pending frame. */
bool mailbox_offer(uint8_t slot, const uint8_t* data, uint8_t len);

/* Once per super-loop pass. */
void mailbox_step(uint32_t now_ms);

/* True once per new frame of an M7->M4 slot: copies its MAILBOX_PAYLOAD_BYTES (bytes past
 * the gen/ length are 0) into out. False for an M4->M7 slot, no new frame or NULL. */
bool mailbox_take(uint8_t slot, uint8_t out[MAILBOX_PAYLOAD_BYTES]);

/* The flags and counters at now_ms. */
mailbox_health_t mailbox_health(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_MAILBOX_H */
