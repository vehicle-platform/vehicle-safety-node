#ifndef HAL_HSEM_H
#define HAL_HSEM_H

/*
 * STM32H7 hardware semaphores (HSEM) for the M7<->M4 mailbox (D-072 item 4c, D-093 item
 * 5). HAL-free interface; only services/mailbox.c calls it (CI grep), and only services/
 * and hal/ include it. Every call is one bounded operation: no wait, no retry loop.
 *
 *   if (hal_hsem_take(id, &key)) {      IRQs masked, DMB done
 *       ... mailbox_core locked-section helpers only ...
 *       hal_hsem_release(id, &key);     DMB + DSB, release, IRQs restored
 *   }                                    else: not taken, IRQs as before, try next pass
 *
 * take: one-step lock (HSEM_RLRx read) with interrupts masked first (the previous mask is
 *   saved in key). On success the port issues a DMB and returns true WITH INTERRUPTS
 *   STILL MASKED; on failure it restores the mask and returns false. On the H7 a one-step
 *   take succeeds again for the core that already holds the semaphore (no PROCID), so
 *   each semaphore has ONE accessing context per core (D-093 item 5): on the M4 the super
 *   loop (never an ISR as well).
 * release: DMB + DSB (the slot stores are visible before the lock is free), write the
 *   release (COREID, PROCID 0), then restore the interrupt mask saved by take.
 * barrier: DMB, between a writer's payload copy and its write_seq increment.
 * release_own: init only, before the super loop starts: frees id if this core holds it (a lock
 *   left from before a core reset); never touches another core's lock. Per ID, never
 *   HSEM_CR (which would also free ID 0, kept for vendor boot code).
 *
 * Nothing but the mailbox_core locked-section helpers (a few aligned 32-bit loads and
 * stores) may run between a successful take and its release: the interrupt mask covers
 * that window, and its measured maximum is a D-093 item 6 board item.
 *
 * The host port (hal/host/hsem_host.c) models two cores, the same-core re-take and
 * injected contention. The H7 port comes with the board (red zone): HSEM clock enable,
 * PRIMASK handling, COREID of the CM4, no HAL_HSEM_FastTake loop.
 *
 * Forked from vehicle-rt-core commit c7a8243 (the rt-core#62 merge, after v0.12.0; code identical at 333f203, the current fork ref) src/hal/hsem.h (the same interface on both cores);
 * safety-node owns this copy and never includes rt-core sources (D-072 item 6).
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Mailbox HSEM IDs are 1..31 (D-093 item 5); 0 stays free for vendor boot code. */
#define HAL_HSEM_ID_MIN 1u
#define HAL_HSEM_ID_MAX 31u

/* The interrupt mask saved by a successful take; opaque to the caller. */
typedef struct {
    uint32_t irq_state;
} hal_hsem_key_t;

/* One-step try-lock; true = this core holds id (interrupts masked). False for an ID
 * outside HAL_HSEM_ID_MIN..MAX or a NULL key. */
bool hal_hsem_take(uint8_t id, hal_hsem_key_t* key);

/* Releases id taken by hal_hsem_take() and restores the interrupt mask in key. */
void hal_hsem_release(uint8_t id, const hal_hsem_key_t* key);

/* Data memory barrier (DMB). */
void hal_hsem_barrier(void);

/* Init only: frees id if this core holds it. */
void hal_hsem_release_own(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* HAL_HSEM_H */
