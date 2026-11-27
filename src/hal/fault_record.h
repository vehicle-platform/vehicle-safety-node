#ifndef HAL_FAULT_RECORD_H
#define HAL_FAULT_RECORD_H

/*
 * Fault record kept in .noinit RAM across the reset that ends a fault (T77): which fault path
 * ran (tag), the Cortex-M fault status registers CFSR and HFSR and the RCC clock interrupt
 * flags CIFR (CSSF = HSE clock security system failure). Pure and HAL-free,
 * header-only static inline. The record carries a magic word and a check word, so cold-boot
 * garbage or a torn write reads as "no record". The store is always_inline: it runs in the
 * fault path before the reset, possibly on a freshly reloaded stack, and must not need a call.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FAULT_RECORD_MAGIC 0xFA17ED00u

/* Tags: which path stored the record. */
/* The plain numbers (_N) exist because the naked fault entries in cubemx/User/boot_checks.h
 * use basic asm and need a literal; C code uses the unsigned FAULT_TAG_* forms. */
// cppcheck-suppress misra-c2012-20.10 ; DEV-005: # and ## build the asm literal of the naked fault entry
#define FAULT_TAG_U2_(n) n##u
#define FAULT_TAG_U_(n) FAULT_TAG_U2_(n)
#define FAULT_TAG_NONE_N 0
#define FAULT_TAG_NMI_N 1
#define FAULT_TAG_HARDFAULT_N 2
#define FAULT_TAG_MEMMANAGE_N 3
#define FAULT_TAG_BUSFAULT_N 4
#define FAULT_TAG_USAGEFAULT_N 5
#define FAULT_TAG_ERROR_HANDLER_N 6
#define FAULT_TAG_NONE FAULT_TAG_U_(FAULT_TAG_NONE_N)
#define FAULT_TAG_NMI FAULT_TAG_U_(FAULT_TAG_NMI_N)
#define FAULT_TAG_HARDFAULT FAULT_TAG_U_(FAULT_TAG_HARDFAULT_N)
#define FAULT_TAG_MEMMANAGE FAULT_TAG_U_(FAULT_TAG_MEMMANAGE_N)
#define FAULT_TAG_BUSFAULT FAULT_TAG_U_(FAULT_TAG_BUSFAULT_N)
#define FAULT_TAG_USAGEFAULT FAULT_TAG_U_(FAULT_TAG_USAGEFAULT_N)
#define FAULT_TAG_ERROR_HANDLER FAULT_TAG_U_(FAULT_TAG_ERROR_HANDLER_N)

typedef struct {
    uint32_t magic;
    uint32_t tag;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t cifr;
    uint32_t check; /* ~(magic ^ tag ^ cfsr ^ hfsr ^ cifr) */
} fault_record_t;

static inline uint32_t fault_record_check(const fault_record_t *r)
{
    return ~(r->magic ^ r->tag ^ r->cfsr ^ r->hfsr ^ r->cifr);
}

__attribute__((always_inline)) static inline void fault_record_store(fault_record_t *r, uint32_t tag,
                                                                    uint32_t cfsr, uint32_t hfsr,
                                                                    uint32_t cifr)
{
    r->magic = FAULT_RECORD_MAGIC;
    r->tag = tag;
    r->cfsr = cfsr;
    r->hfsr = hfsr;
    r->cifr = cifr;
    r->check = ~(FAULT_RECORD_MAGIC ^ tag ^ cfsr ^ hfsr ^ cifr);
}

static inline bool fault_record_valid(const fault_record_t *r)
{
    return (r->magic == FAULT_RECORD_MAGIC) && (r->check == fault_record_check(r)) &&
           (r->tag >= FAULT_TAG_NMI) && (r->tag <= FAULT_TAG_ERROR_HANDLER);
}

#ifdef __cplusplus
}
#endif

#endif /* HAL_FAULT_RECORD_H */
