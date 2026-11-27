/* Boot-time checks and the fault safe state, see boot_checks.h (T77). */
#include "boot_checks.h"

#include "hal/can_timing.h"

/* The pure headers carry their own bit positions; they must match the CMSIS ones. */
_Static_assert(RESET_CAUSE_CSR_RMVF == RCC_CSR_RMVF_Msk, "RMVF");
_Static_assert(RESET_CAUSE_CSR_OBLRSTF == RCC_CSR_OBLRSTF_Msk, "OBLRSTF");
_Static_assert(RESET_CAUSE_CSR_PINRSTF == RCC_CSR_PINRSTF_Msk, "PINRSTF");
_Static_assert(RESET_CAUSE_CSR_BORRSTF == RCC_CSR_BORRSTF_Msk, "BORRSTF");
_Static_assert(RESET_CAUSE_CSR_SFTRSTF == RCC_CSR_SFTRSTF_Msk, "SFTRSTF");
_Static_assert(RESET_CAUSE_CSR_IWDGRSTF == RCC_CSR_IWDGRSTF_Msk, "IWDGRSTF");
_Static_assert(RESET_CAUSE_CSR_WWDGRSTF == RCC_CSR_WWDGRSTF_Msk, "WWDGRSTF");
_Static_assert(RESET_CAUSE_CSR_LPWRRSTF == RCC_CSR_LPWRRSTF_Msk, "LPWRRSTF");
_Static_assert(HW_CONFIG_OPTR_BOR_LEV_POS == FLASH_OPTR_BOR_LEV_Pos, "BOR_LEV pos");
// cppcheck-suppress misra-c2012-12.2 ; DEV-006: constant mask << position inside _Static_assert, operands in range
_Static_assert((HW_CONFIG_OPTR_BOR_LEV_MASK << HW_CONFIG_OPTR_BOR_LEV_POS) == FLASH_OPTR_BOR_LEV_Msk, "BOR_LEV mask");
_Static_assert(HW_CONFIG_OPTR_IWDG_SW_MASK == FLASH_OPTR_IWDG_SW_Msk, "IWDG_SW");
_Static_assert(HW_CONFIG_OPTR_NSWBOOT0_MASK == FLASH_OPTR_nSWBOOT0_Msk, "nSWBOOT0");
_Static_assert(HW_CONFIG_OPTR_NBOOT0_MASK == FLASH_OPTR_nBOOT0_Msk, "nBOOT0");
_Static_assert(CAN_TIMING_NBTP_NSJW_POS == FDCAN_NBTP_NSJW_Pos, "NSJW pos");
_Static_assert(CAN_TIMING_NBTP_NBRP_POS == FDCAN_NBTP_NBRP_Pos, "NBRP pos");
_Static_assert(CAN_TIMING_NBTP_NTSEG1_POS == FDCAN_NBTP_NTSEG1_Pos, "NTSEG1 pos");
_Static_assert(CAN_TIMING_NBTP_NTSEG2_POS == FDCAN_NBTP_NTSEG2_Pos, "NTSEG2 pos");
// cppcheck-suppress-begin misra-c2012-12.2 ; DEV-006: constant mask << position inside _Static_assert, operands in range
_Static_assert((CAN_TIMING_NBTP_NSJW_MASK << CAN_TIMING_NBTP_NSJW_POS) == FDCAN_NBTP_NSJW_Msk, "NSJW mask");
_Static_assert((CAN_TIMING_NBTP_NBRP_MASK << CAN_TIMING_NBTP_NBRP_POS) == FDCAN_NBTP_NBRP_Msk, "NBRP mask");
_Static_assert((CAN_TIMING_NBTP_NTSEG1_MASK << CAN_TIMING_NBTP_NTSEG1_POS) == FDCAN_NBTP_NTSEG1_Msk, "NTSEG1 mask");
// cppcheck-suppress-end misra-c2012-12.2 ; DEV-006
_Static_assert((CAN_TIMING_NBTP_NTSEG2_MASK << CAN_TIMING_NBTP_NTSEG2_POS) == FDCAN_NBTP_NTSEG2_Msk, "NTSEG2 mask");

/*
 * Abnormal-reset counter: .noinit RAM, not cleared by the startup (which zeroes only .bss). The
 * section name is declared NOBITS ("%nobits", the trailing @ is the Arm comment character and
 * swallows the compiler's own flag string) so it takes no flash image. The generated linker
 * script has no USER CODE block, so it does not list .noinit and the linker places it as an
 * orphan after .bss. Two guards: tools/check_noinit.py (CI, on the linked ELF: NOBITS, start at
 * or above _ebss, end at or below _sstack) and boot_noinit_placement_ok() at run time, which
 * checks both .noinit objects against the same two linker symbols.
 */
__attribute__((section(".noinit,\"aw\",%nobits@"))) static reset_counter_t s_reset_counter;

/* Fault record of the last fault path; not touched at boot, see boot_fault_record_get(). */
__attribute__((section(".noinit,\"aw\",%nobits@"))) fault_record_t g_boot_fault_record;

static uint32_t s_reset_csr;
static reset_cause_t s_reset_cause = RESET_CAUSE_UNKNOWN;
static uint32_t s_hw_config_flags;

extern uint32_t _sstack; /* linker: bottom of the stack region (_estack - _Min_Stack_Size) */
extern uint32_t _ebss;   /* linker: end of .bss (the startup zeroes up to here only) */

static fault_record_t s_prev_fault;
static bool s_prev_fault_valid;
static bool s_can_fault;
static uint32_t s_noinit_bad; /* HW_CONFIG_BAD_NOINIT or 0, set by boot_noinit_placement_ok() */

void boot_fault_safe_state(uint32_t tag)
{
    boot_safe_state(tag);
}

bool boot_fault_record_get(fault_record_t *out)
{
    if (s_prev_fault_valid) {
        *out = s_prev_fault;
    }
    return s_prev_fault_valid;
}

void boot_fault_record_clear(void)
{
    s_prev_fault_valid = false;
}

/* True if both .noinit objects lie at or above _ebss (never zeroed by the startup) and end at or
 * below _sstack (cannot alias the stack). */
static bool boot_noinit_placement_ok(void)
{
    // cppcheck-suppress-begin misra-c2012-11.4 ; DEV-006: register access through the CMSIS peripheral macros / linker-symbol addresses
    const uint32_t lo = (uint32_t)(uintptr_t)&_ebss;
    const uint32_t hi = (uint32_t)(uintptr_t)&_sstack;
    const uint32_t a = (uint32_t)(uintptr_t)&s_reset_counter;
    const uint32_t b = (uint32_t)(uintptr_t)&g_boot_fault_record;
    // cppcheck-suppress-end misra-c2012-11.4 ; DEV-006

    s_noinit_bad = hw_config_check_placement(lo, hi, a, (uint32_t)sizeof(s_reset_counter)) |
                   hw_config_check_placement(lo, hi, b, (uint32_t)sizeof(g_boot_fault_record));
    return s_noinit_bad == 0u;
}

void boot_reset_cause_capture(void)
{
    // cppcheck-suppress misra-c2012-11.4 ; DEV-006: register access through the CMSIS peripheral macros / linker-symbol addresses
    s_reset_csr = RCC->CSR;
    s_reset_cause = reset_cause_decode(s_reset_csr);
    // cppcheck-suppress misra-c2012-11.4 ; DEV-006: register access through the CMSIS peripheral macros / linker-symbol addresses
    RCC->CSR |= RCC_CSR_RMVF;
    /* Write RMVF back to 0 explicitly: RM0440 does not say it self-clears, and a bit left at 1
     * would keep clearing the flags of the next warm reset before software could read them. */
    // cppcheck-suppress misra-c2012-11.4 ; DEV-006: register access through the CMSIS peripheral macros / linker-symbol addresses
    RCC->CSR &= ~RCC_CSR_RMVF;
    if (boot_noinit_placement_ok()) {
        /* Take over the previous run's fault record and clear it, so one fault is counted once. */
        s_prev_fault_valid = fault_record_valid(&g_boot_fault_record);
        if (s_prev_fault_valid) {
            s_prev_fault = g_boot_fault_record;
        }
        g_boot_fault_record.magic = 0u;
        (void)reset_counter_update(&s_reset_counter, s_reset_cause, s_prev_fault_valid);
    }
}

uint32_t boot_reset_cause_raw(void)
{
    return s_reset_csr;
}

reset_cause_t boot_reset_cause(void)
{
    return s_reset_cause;
}

uint32_t boot_abnormal_reset_count(void)
{
    return reset_counter_valid(&s_reset_counter) ? s_reset_counter.count : 0u;
}

void boot_abnormal_reset_count_clear(void)
{
    reset_counter_set(&s_reset_counter, 0u);
}

void boot_hw_config_check(void)
{
    // cppcheck-suppress-begin misra-c2012-11.4 ; DEV-006: register access through the CMSIS peripheral macros / linker-symbol addresses
    s_hw_config_flags = s_noinit_bad | hw_config_check_optr(FLASH->OPTR) |
                        hw_config_check_iwdg(IWDG->PR, IWDG->RLR, IWDG->WINR);
    // cppcheck-suppress-end misra-c2012-11.4 ; DEV-006
}

uint32_t boot_hw_config_flags(void)
{
    return s_hw_config_flags;
}

bool boot_fdcan_timing_ok(const FDCAN_HandleTypeDef *hfdcan, uint32_t expected_bps)
{
    bool ok = false;

    if ((hfdcan != NULL) && (hfdcan->Instance != NULL)) {
        /* HAL returns the clock source frequency; the FDCAN_CKDIV divider (PDIV) follows. */
        // cppcheck-suppress misra-c2012-11.4 ; DEV-006: register access through the CMSIS peripheral macros / linker-symbol addresses
        const uint32_t pdiv = FDCAN_CONFIG->CKDIV & FDCAN_CKDIV_PDIV_Msk;
        const uint32_t kernel_hz = can_timing_kernel_hz(HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN), pdiv);

        ok = can_timing_nbtp_ok(kernel_hz, hfdcan->Instance->NBTP, expected_bps);
    }
    return ok;
}

HAL_StatusTypeDef boot_fdcan_start(FDCAN_HandleTypeDef *hfdcan, uint32_t expected_bps)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    if (boot_fdcan_timing_ok(hfdcan, expected_bps)) {
        status = HAL_FDCAN_Start(hfdcan);
    }
    s_can_fault = (status != HAL_OK); /* controller stays stopped on a mismatch, no reset */
    return status;
}

bool boot_can_fault(void)
{
    return s_can_fault;
}
