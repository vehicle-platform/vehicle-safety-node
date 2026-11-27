#ifndef HAL_FAULT_RECORD_H
#define HAL_FAULT_RECORD_H

/*
 * Fault record kept in .noinit RAM across the reset that ends a fault (T77): which fault path
 * ran (tag) and the Cortex-M fault status registers CFSR and HFSR. Pure and HAL-free,
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
#define FAULT_TAG_NONE 0u
#define FAULT_TAG_NMI 1u
#define FAULT_TAG_HARDFAULT 2u
#define FAULT_TAG_MEMMANAGE 3u
#define FAULT_TAG_BUSFAULT 4u
#define FAULT_TAG_USAGEFAULT 5u
#define FAULT_TAG_ERROR_HANDLER 6u

typedef struct {
    uint32_t magic;
    uint32_t tag;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t check; /* ~(magic ^ tag ^ cfsr ^ hfsr) */
} fault_record_t;

static inline uint32_t fault_record_check(const fault_record_t *r)
{
    return ~(r->magic ^ r->tag ^ r->cfsr ^ r->hfsr);
}

__attribute__((always_inline)) static inline void fault_record_store(fault_record_t *r, uint32_t tag,
                                                                    uint32_t cfsr, uint32_t hfsr)
{
    r->magic = FAULT_RECORD_MAGIC;
    r->tag = tag;
    r->cfsr = cfsr;
    r->hfsr = hfsr;
    r->check = ~(FAULT_RECORD_MAGIC ^ tag ^ cfsr ^ hfsr);
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
