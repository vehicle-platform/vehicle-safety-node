/*
 * Boot-time checks and the fault safe state of the CubeMX image (T77). Not generated: this file
 * and boot_checks.c are ours and survive a CubeMX regeneration; the generated files only call
 * into them from USER CODE blocks. Red-zone, invariant-8 code (reset handling, fault paths).
 * Pure logic lives in src/hal/ (reset_cause.h, hw_config.h, can_timing.h), host-tested.
 */
#ifndef BOOT_CHECKS_H
#define BOOT_CHECKS_H

#include "main.h"

#include "hal/hw_config.h"
#include "hal/reset_cause.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Actuator outputs low (immobilizer relay coils, dead-man kick, mirror LEDs, status LED), by
 * direct register writes only: the GPIO clocks are forced on and the pins are driven through
 * the atomic BSRR reset half, independent of MX_GPIO_Init. Set: PC2 DEADMAN_KICK, PC4 IMMO_SET,
 * PC5 IMMO_RESET (GPIOC), PB0 / PB1 mirror LEDs (GPIOB), PA5 LD2 (GPIOA, not safety relevant).
 * A coil left energised during a fault could complete an unintended latch.
 *
 * static inline with always_inline on purpose: a call would push a return address and a frame
 * onto a stack that may be exhausted (a stack-overflow fault), and a second fault there is a
 * lockup that keeps the outputs live until the IWDG fires (about 100 ms).
 */
static inline __attribute__((always_inline)) void boot_outputs_off(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
    (void)RCC->AHB2ENR;
    GPIOC->BSRR = ((uint32_t)(GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5)) << 16U;
    GPIOB->BSRR = ((uint32_t)(GPIO_PIN_0 | GPIO_PIN_1)) << 16U;
    GPIOA->BSRR = ((uint32_t)GPIO_PIN_5) << 16U;
    __DSB();
}

/*
 * Safe state, then reset: IRQs masked, outputs off, system reset request. Never returns; the
 * wait after the request is bounded by the reset itself (the IWDG backs it up). Same
 * always_inline reasoning as boot_outputs_off(). NOT "back to unlocked": the latching
 * immobilizer relay keeps a LOCKED latch through a reset (D-097); the boot state handling is T13.
 */
static inline __attribute__((always_inline, noreturn)) void boot_safe_state(void)
{
    __disable_irq();
    boot_outputs_off();
    SCB->AIRCR = (0x5FAUL << SCB_AIRCR_VECTKEY_Pos) | (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
                 SCB_AIRCR_SYSRESETREQ_Msk;
    __DSB();
    for (;;) {
        __NOP();
    }
}

/*
 * Entry of NMI, HardFault, MemManage, BusFault and UsageFault: a naked handler (no prologue,
 * so no push) that loads MSP with the top of RAM (_estack, the linker symbol) and branches to
 * boot_fault_safe_state(). Handlers run on MSP; if the fault was MSP exhaustion the old MSP is
 * not trusted, and the fault is never resumed, so restarting the stack at the top is safe. Use
 * only as the whole body of a function declared __attribute__((naked)).
 */
#define BOOT_FAULT_ENTRY()                  \
    __asm volatile("ldr r0, =_estack\n"     \
                   "msr msp, r0\n"          \
                   "isb\n"                  \
                   "b boot_fault_safe_state\n")

/* Target of BOOT_FAULT_ENTRY(); a function (not inline) because the asm branches to it by name. */
void boot_fault_safe_state(void) __attribute__((noreturn, used));

/*
 * Reset cause and IWDG reset counter. boot_reset_cause_capture() reads RCC->CSR, stores it,
 * clears the flags with RMVF and updates the .noinit counter; call it first in main(), before
 * any other init (nothing earlier reads or clears CSR: Reset_Handler and SystemInit leave it
 * alone). The raw value, the decoded cause and the count stay readable afterwards.
 * No CAN report yet: HeartbeatSafety (0x082) has only SAFETY_NODE_MODE, SAFETY_ERROR_COUNT and
 * SAFETY_UPTIME in vehicle-defs, no reset-cause field (defs follow-up, see cubemx/README.md).
 */
void boot_reset_cause_capture(void);
uint32_t boot_reset_cause_raw(void);
reset_cause_t boot_reset_cause(void);

/* Consecutive IWDG resets so far (this boot included). T13 clears it after a healthy run
 * with boot_iwdg_reset_count_clear() and feeds it to boot_should_safe_halt() (hal/reset_cause.h). */
uint32_t boot_iwdg_reset_count(void);
void boot_iwdg_reset_count_clear(void);

/*
 * Option-byte / IWDG / BOR boot check: FLASH->OPTR and the IWDG registers against
 * hal/hw_config.h. Call after MX_IWDG_Init. The result is a mask of HW_CONFIG_BAD_* bits (0 =
 * as configured), stored and readable with boot_hw_config_flags(). A mismatch is only
 * recorded, never a halt: the option bytes are not applied until the first flash
 * (cubemx/README.md), and the halt policy is T13.
 */
void boot_hw_config_check(void);
uint32_t boot_hw_config_flags(void);

/*
 * FDCAN nominal bit-timing check before HAL_FDCAN_Start: the kernel clock (after the FDCAN
 * clock divider) is read from the RCC at run time and the NBTP register is read back; both go
 * to can_timing_nbtp_ok() (hal/can_timing.h) with the expected bit rate. False for a wrong
 * clock tree, a wrong or unprogrammed NBTP, or an unreadable clock.
 *
 * vehicle-defs gen/c/safety has no bit-rate constant for the platform bus (defs follow-up);
 * the platform bus is 500 kbit/s (D-009, src/hal/can.h), so the value is local until defs has
 * one.
 */
#define BOOT_FDCAN1_EXPECTED_BPS 500000u

bool boot_fdcan_timing_ok(const FDCAN_HandleTypeDef *hfdcan, uint32_t expected_bps);

/*
 * The only way the image starts the FDCAN controller: on a failed timing check it goes to the
 * safe state (reset) and the controller is not started; otherwise it returns HAL_FDCAN_Start().
 */
HAL_StatusTypeDef boot_fdcan_start(FDCAN_HandleTypeDef *hfdcan, uint32_t expected_bps);

#ifdef __cplusplus
}
#endif

#endif /* BOOT_CHECKS_H */
