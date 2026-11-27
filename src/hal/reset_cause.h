#ifndef HAL_RESET_CAUSE_H
#define HAL_RESET_CAUSE_H

/*
 * Reset-cause decoding of the STM32G4 RCC_CSR flags (RM0440) and the IWDG reset counter, pure
 * and HAL-free, header-only static inline (no external linkage). The register read, the RMVF
 * clear and the .noinit storage live in cubemx/User/boot_checks.c; this header only turns the
 * captured value into one cause and keeps the counter arithmetic testable on the host.
 *
 * Several flags are set at once on most resets (every reset also drives the NRST pin flag, a
 * power-on reset also sets the BOR flag; the G4 has no separate POR flag), so the decode is a
 * fixed priority, most specific first: IWDG, WWDG, low power, software, BOR (power-on or
 * brown-out), option-byte load, pin. BOR is decoded before OBL until target measurements show
 * which flags a power-on really sets (an option-byte reload that also set BOR reads as BOR,
 * which only clears the counter, the safe direction for a power cycle). A value with none of the known flags (including 0) is
 * RESET_CAUSE_UNKNOWN. The bit positions are the RM0440 positions; the cubemx build asserts
 * them against the CMSIS RCC_CSR_*_Msk macros.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* RCC_CSR flag bits (RM0440). */
#define RESET_CAUSE_CSR_RMVF 0x00800000u    /* write 1: clear all flags */
#define RESET_CAUSE_CSR_OBLRSTF 0x02000000u /* option byte loader reset */
#define RESET_CAUSE_CSR_PINRSTF 0x04000000u /* NRST pin reset */
#define RESET_CAUSE_CSR_BORRSTF 0x08000000u /* BOR reset (also set by a power-on reset) */
#define RESET_CAUSE_CSR_SFTRSTF 0x10000000u /* software reset (SYSRESETREQ) */
#define RESET_CAUSE_CSR_IWDGRSTF 0x20000000u
#define RESET_CAUSE_CSR_WWDGRSTF 0x40000000u
#define RESET_CAUSE_CSR_LPWRRSTF 0x80000000u /* illegal Stop/Standby/Shutdown entry */

typedef enum {
    RESET_CAUSE_UNKNOWN = 0,
    RESET_CAUSE_IWDG,
    RESET_CAUSE_WWDG,
    RESET_CAUSE_LOW_POWER,
    RESET_CAUSE_SOFTWARE,
    RESET_CAUSE_OPTION_BYTE_LOAD,
    RESET_CAUSE_POWER_ON_OR_BROWN_OUT,
    RESET_CAUSE_PIN
} reset_cause_t;

/* Decode an RCC_CSR value (the RMVF bit itself and the non-flag bits are ignored). */
static inline reset_cause_t reset_cause_decode(uint32_t csr)
{
    reset_cause_t cause = RESET_CAUSE_UNKNOWN;

    if ((csr & RESET_CAUSE_CSR_IWDGRSTF) != 0u) {
        cause = RESET_CAUSE_IWDG;
    } else if ((csr & RESET_CAUSE_CSR_WWDGRSTF) != 0u) {
        cause = RESET_CAUSE_WWDG;
    } else if ((csr & RESET_CAUSE_CSR_LPWRRSTF) != 0u) {
        cause = RESET_CAUSE_LOW_POWER;
    } else if ((csr & RESET_CAUSE_CSR_SFTRSTF) != 0u) {
        cause = RESET_CAUSE_SOFTWARE;
    } else if ((csr & RESET_CAUSE_CSR_BORRSTF) != 0u) {
        cause = RESET_CAUSE_POWER_ON_OR_BROWN_OUT;
    } else if ((csr & RESET_CAUSE_CSR_OBLRSTF) != 0u) {
        cause = RESET_CAUSE_OPTION_BYTE_LOAD;
    } else if ((csr & RESET_CAUSE_CSR_PINRSTF) != 0u) {
        cause = RESET_CAUSE_PIN;
    } else {
        /* no known flag: stays UNKNOWN */
    }
    return cause;
}

/*
 * Abnormal-reset counter kept in .noinit RAM across resets. A cold boot leaves the RAM as garbage,
 * so the record carries a magic word and the bitwise inverse of the count; anything that does
 * not match is treated as "no history" and restarts at 0. The record is plain data (no
 * pointers), 12 bytes.
 */
#define RESET_COUNTER_MAGIC 0x1DE6C0DEu
#define RESET_COUNTER_MAX 0xFFFFu /* saturates here, never wraps to 0 */

typedef struct {
    uint32_t magic;
    uint32_t count;
    uint32_t count_inv;
} reset_counter_t;

static inline bool reset_counter_valid(const reset_counter_t *rc)
{
    return (rc->magic == RESET_COUNTER_MAGIC) && (rc->count_inv == ~rc->count) &&
           (rc->count <= RESET_COUNTER_MAX);
}

static inline void reset_counter_set(reset_counter_t *rc, uint32_t count)
{
    rc->magic = RESET_COUNTER_MAGIC;
    rc->count = count;
    rc->count_inv = ~count;
}

/* True for the resets that count toward the safe-halt limit (consecutive abnormal resets):
 * IWDG, WWDG, illegal low-power entry, and a software reset that left a valid fault record
 * (the fault/Error_Handler path, which ends in SYSRESETREQ). A software reset without a
 * record is a deliberate one (not abnormal, count unchanged). */
static inline bool reset_cause_is_abnormal(reset_cause_t cause, bool has_fault_record)
{
    return (cause == RESET_CAUSE_IWDG) || (cause == RESET_CAUSE_WWDG) ||
           (cause == RESET_CAUSE_LOW_POWER) || ((cause == RESET_CAUSE_SOFTWARE) && has_fault_record);
}

/*
 * Boot-time update with the decoded cause. An invalid record (cold boot, corruption) restarts
 * at 0 first. Then: every abnormal reset (reset_cause_is_abnormal) increments (saturating), so
 * an HSE-timeout, watchdog or fault reset loop is counted; power-on/brown-out and the
 * NRST-pin-only reset clear (a deliberate or electrical restart ends a loop); every other cause
 * (deliberate software reset, option-byte load, unknown) leaves the count unchanged.
 * Returns the count after the update.
 */
static inline uint32_t reset_counter_update(reset_counter_t *rc, reset_cause_t cause,
                                            bool has_fault_record)
{
    if (!reset_counter_valid(rc)) {
        reset_counter_set(rc, 0u);
    }
    if (reset_cause_is_abnormal(cause, has_fault_record)) {
        if (rc->count < RESET_COUNTER_MAX) {
            reset_counter_set(rc, rc->count + 1u);
        }
    } else if ((cause == RESET_CAUSE_POWER_ON_OR_BROWN_OUT) || (cause == RESET_CAUSE_PIN)) {
        reset_counter_set(rc, 0u);
    } else {
        /* unchanged */
    }
    return rc->count;
}

/*
 * Safe-halt policy input: true when `count` consecutive abnormal resets reached `limit`. A limit of
 * 0 disables the policy (false), so an unconfigured limit can never halt the node. The halt
 * itself (outputs off, no relay pulse, D-097) is wired in T13.
 */
static inline bool boot_should_safe_halt(uint32_t count, uint32_t limit)
{
    return (limit != 0u) && (count >= limit);
}

#ifdef __cplusplus
}
#endif

#endif /* HAL_RESET_CAUSE_H */
