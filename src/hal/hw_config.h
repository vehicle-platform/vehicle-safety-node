#ifndef HAL_HW_CONFIG_H
#define HAL_HW_CONFIG_H

/*
 * Boot check of the hardware configuration against the values written in cubemx/README.md
 * ("First flash: option bytes" and "IWDG"), pure and HAL-free, header-only static inline.
 * The register reads (FLASH->OPTR, IWDG->PR/RLR/WINR) are in cubemx/User/boot_checks.c; this
 * header compares the values and returns a bit mask of what differs (0 = as configured).
 *
 * FLASH_OPTR bit positions are the RM0440 ones; the cubemx build asserts them against the
 * CMSIS FLASH_OPTR_*_Msk macros.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* FLASH_OPTR fields compared (RM0440). */
#define HW_CONFIG_OPTR_BOR_LEV_POS 8u
#define HW_CONFIG_OPTR_BOR_LEV_MASK 0x7u
#define HW_CONFIG_OPTR_IWDG_SW_MASK 0x00010000u   /* 0 = hardware watchdog */
#define HW_CONFIG_OPTR_NSWBOOT0_MASK 0x04000000u  /* 0 = boot source from the nBOOT0 bit */
#define HW_CONFIG_OPTR_NBOOT0_MASK 0x08000000u    /* 1 = boot from main flash */

/* Expected values (cubemx/README.md: nSWBOOT0=0 nBOOT0=1 BOR_LEV=4 IWDG_SW=0). */
#define HW_CONFIG_EXPECT_BOR_LEV 4u
#define HW_CONFIG_EXPECT_IWDG_PR 3u      /* prescaler /32 (IWDG_PR code 3) */
#define HW_CONFIG_EXPECT_IWDG_RLR 99u    /* about 100 ms at 32 kHz LSI */
#define HW_CONFIG_EXPECT_IWDG_WINR 4095u /* window disabled */

/* Mismatch bits. */
#define HW_CONFIG_BAD_BOR_LEV 0x01u
#define HW_CONFIG_BAD_IWDG_SW 0x02u    /* software-started IWDG option: not hardware watchdog */
#define HW_CONFIG_BAD_BOOT_SRC 0x04u   /* nSWBOOT0 != 0 or nBOOT0 != 1 */
#define HW_CONFIG_BAD_IWDG_TIMING 0x08u /* IWDG PR / RLR / WINR differ from the configuration */

/* Mask of the option-byte mismatches in a FLASH_OPTR value; 0 = as configured. */
static inline uint32_t hw_config_check_optr(uint32_t optr)
{
    uint32_t bad = 0u;

    if (((optr >> HW_CONFIG_OPTR_BOR_LEV_POS) & HW_CONFIG_OPTR_BOR_LEV_MASK) !=
        HW_CONFIG_EXPECT_BOR_LEV) {
        bad |= HW_CONFIG_BAD_BOR_LEV;
    }
    if ((optr & HW_CONFIG_OPTR_IWDG_SW_MASK) != 0u) {
        bad |= HW_CONFIG_BAD_IWDG_SW;
    }
    if (((optr & HW_CONFIG_OPTR_NSWBOOT0_MASK) != 0u) || ((optr & HW_CONFIG_OPTR_NBOOT0_MASK) == 0u)) {
        bad |= HW_CONFIG_BAD_BOOT_SRC;
    }
    return bad;
}

/* HW_CONFIG_BAD_IWDG_TIMING when the IWDG registers differ from the expected values, else 0. */
static inline uint32_t hw_config_check_iwdg(uint32_t pr, uint32_t rlr, uint32_t winr)
{
    uint32_t bad = 0u;

    if ((pr != HW_CONFIG_EXPECT_IWDG_PR) || (rlr != HW_CONFIG_EXPECT_IWDG_RLR) ||
        (winr != HW_CONFIG_EXPECT_IWDG_WINR)) {
        bad = HW_CONFIG_BAD_IWDG_TIMING;
    }
    return bad;
}

#ifdef __cplusplus
}
#endif

#endif /* HAL_HW_CONFIG_H */
