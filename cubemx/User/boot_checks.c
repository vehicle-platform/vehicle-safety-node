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
_Static_assert((HW_CONFIG_OPTR_BOR_LEV_MASK << HW_CONFIG_OPTR_BOR_LEV_POS) == FLASH_OPTR_BOR_LEV_Msk, "BOR_LEV mask");
_Static_assert(HW_CONFIG_OPTR_IWDG_SW_MASK == FLASH_OPTR_IWDG_SW_Msk, "IWDG_SW");
_Static_assert(HW_CONFIG_OPTR_NSWBOOT0_MASK == FLASH_OPTR_nSWBOOT0_Msk, "nSWBOOT0");
_Static_assert(HW_CONFIG_OPTR_NBOOT0_MASK == FLASH_OPTR_nBOOT0_Msk, "nBOOT0");
_Static_assert(CAN_TIMING_NBTP_NSJW_POS == FDCAN_NBTP_NSJW_Pos, "NSJW pos");
_Static_assert(CAN_TIMING_NBTP_NBRP_POS == FDCAN_NBTP_NBRP_Pos, "NBRP pos");
_Static_assert(CAN_TIMING_NBTP_NTSEG1_POS == FDCAN_NBTP_NTSEG1_Pos, "NTSEG1 pos");
_Static_assert(CAN_TIMING_NBTP_NTSEG2_POS == FDCAN_NBTP_NTSEG2_Pos, "NTSEG2 pos");
_Static_assert((CAN_TIMING_NBTP_NSJW_MASK << CAN_TIMING_NBTP_NSJW_POS) == FDCAN_NBTP_NSJW_Msk, "NSJW mask");
_Static_assert((CAN_TIMING_NBTP_NBRP_MASK << CAN_TIMING_NBTP_NBRP_POS) == FDCAN_NBTP_NBRP_Msk, "NBRP mask");
_Static_assert((CAN_TIMING_NBTP_NTSEG1_MASK << CAN_TIMING_NBTP_NTSEG1_POS) == FDCAN_NBTP_NTSEG1_Msk, "NTSEG1 mask");
_Static_assert((CAN_TIMING_NBTP_NTSEG2_MASK << CAN_TIMING_NBTP_NTSEG2_POS) == FDCAN_NBTP_NTSEG2_Msk, "NTSEG2 mask");

/*
 * IWDG reset counter: .noinit RAM, not cleared by the startup (which zeroes only .bss). The
 * section name is declared NOBITS ("%nobits" with the trailing @ (the Arm comment character) swallowing the compiler's
 * own flag string) so it takes no flash image. The generated linker script has no USER CODE
 * block, so it does not list .noinit and the linker places it as an orphan; the CI map check
 * (README) and boot_noinit_placement_ok() guard that it lands below the stack region.
 */
__attribute__((section(".noinit,\"aw\",%nobits@"))) static reset_counter_t s_reset_counter;

/* Fault record of the last fault path; not touched at boot, see boot_fault_record_get(). */
__attribute__((section(".noinit,\"aw\",%nobits@"))) fault_record_t g_boot_fault_record;

static uint32_t s_reset_csr;
static reset_cause_t s_reset_cause = RESET_CAUSE_UNKNOWN;
static uint32_t s_hw_config_flags;

extern uint32_t _sstack; /* linker: bottom of the stack region (_estack - _Min_Stack_Size) */

void boot_fault_safe_state(uint32_t tag)
{
    boot_safe_state(tag);
}

bool boot_fault_record_get(fault_record_t *out)
{
    const bool ok = fault_record_valid(&g_boot_fault_record);

    if (ok) {
        *out = g_boot_fault_record;
    }
    return ok;
}

void boot_fault_record_clear(void)
{
    g_boot_fault_record.magic = 0u;
}

/* True if the .noinit record lies fully below the stack region, so it cannot alias the stack. */
static bool boot_noinit_placement_ok(void)
{
    const uintptr_t end = (uintptr_t)&s_reset_counter + sizeof(s_reset_counter);

    return end <= (uintptr_t)&_sstack;
}

void boot_reset_cause_capture(void)
{
    s_reset_csr = RCC->CSR;
    s_reset_cause = reset_cause_decode(s_reset_csr);
    RCC->CSR |= RCC_CSR_RMVF;
    /* Write RMVF back to 0 explicitly: RM0440 does not say it self-clears, and a bit left at 1
     * would keep clearing the flags of the next warm reset before software could read them. */
    RCC->CSR &= ~RCC_CSR_RMVF;
    if (boot_noinit_placement_ok()) {
        (void)reset_counter_update(&s_reset_counter, s_reset_cause);
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

uint32_t boot_iwdg_reset_count(void)
{
    return reset_counter_valid(&s_reset_counter) ? s_reset_counter.count : 0u;
}

void boot_iwdg_reset_count_clear(void)
{
    reset_counter_set(&s_reset_counter, 0u);
}

void boot_hw_config_check(void)
{
    s_hw_config_flags = hw_config_check_optr(FLASH->OPTR) |
                        hw_config_check_iwdg(IWDG->PR, IWDG->RLR, IWDG->WINR);
}

uint32_t boot_hw_config_flags(void)
{
    return s_hw_config_flags;
}

bool boot_fdcan_timing_ok(const FDCAN_HandleTypeDef *hfdcan, uint32_t expected_bps)
{
    bool ok = false;

    if ((hfdcan != NULL) && (hfdcan->Instance != NULL)) {
        /* HAL returns the clock source frequency; the FDCAN_CKDIV divider (PDIV, 0 = /1, n = /2n) follows. */
        const uint32_t pdiv = FDCAN_CONFIG->CKDIV & FDCAN_CKDIV_PDIV_Msk;
        const uint32_t kernel_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN) /
                                   ((pdiv == 0u) ? 1u : (2u * pdiv));

        ok = can_timing_nbtp_ok(kernel_hz, hfdcan->Instance->NBTP, expected_bps);
    }
    return ok;
}

HAL_StatusTypeDef boot_fdcan_start(FDCAN_HandleTypeDef *hfdcan, uint32_t expected_bps)
{
    if (!boot_fdcan_timing_ok(hfdcan, expected_bps)) {
        boot_safe_state(FAULT_TAG_ERROR_HANDLER);
    }
    return HAL_FDCAN_Start(hfdcan);
}
