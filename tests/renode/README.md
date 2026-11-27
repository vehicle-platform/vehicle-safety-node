# Renode boot test of the CubeMX image (T67)

Boots the CubeMX-generated image of `cubemx/` (STM32G474RE, bare-metal super-loop) in the [Renode](https://renode.io) emulator and checks that it comes up cleanly. It runs in CI (job `renode-boot`) before any board is on the desk. It does **not** replace hardware bring-up: Renode models registers, not electrical behaviour, clock accuracy or timing margins.

Pinned versions: Renode **1.17.0** (released 2026-09-07; portable Linux build, SHA-256 `4ba7c68b59e2447f188ef4b4b112fcccd2802582c460beedceb63b84e5605a5f`), Robot Framework 6.1.

## What is covered (`boot.robot`)

| Test | Asserts |
|---|---|
| Core Reaches Main And Completes Clock Init | `Reset_Handler`, `main` and the first call after `SystemClock_Config()` are reached, so every HAL status check of the HSE + PLL bring-up passed (no `Error_Handler`); the RCC registers hold the requested configuration: `HSEON` set and `HSEBYP` clear (24 MHz crystal), PLL source HSE, M = 6, N = 85, R = 2 (170 MHz), `SW` = PLL |
| FDCAN Nominal Bit Timing Is Programmed | `FDCAN_NBTP` is written with NBRP 19, NTSEG1 13, NTSEG2 1, NSJW 1 (HAL prescaler 20, seg1 14, seg2 2, SJW 2) |
| Super Loop Runs And Refreshes The Watchdog | five consecutive 50 ms time slices each contain a `HAL_IWDG_Refresh` call from the super-loop |
| No Reset And No Fault Handler Within Two Seconds | 2 s of emulated time with no entry into `Error_Handler`, `HardFault_Handler`, `NMI_Handler`, `MemManage_Handler`, `BusFault_Handler`, `UsageFault_Handler` or `Default_Handler`, and no machine reset (IWDG expiry or system reset); the refresh is still running at the end |

The probes are Renode CPU hooks on those symbols (`safety-node.resc`); the firmware is not modified. `HAL_IWDG_Refresh` is called every loop iteration (millions of times per emulated second), so it is probed by a one-shot hook that the test re-arms per time slice instead of a permanent one.

## Real versus stubbed

**Renode 1.17.0 has no STM32G4 platform** (only G0, F0/F4/F7, L0, L5, H7 and others). `safety-node.repl` is therefore a minimal STM32G474 description built from Renode's generic STM32 models. Addresses and IRQ numbers are from RM0440 / the G474 datasheet.

Real Renode models: Cortex-M4F + NVIC + SysTick (170 MHz), `STM32_IndependentWatchdog` on IWDG (it really resets the machine when it is not refreshed; LSI 32 kHz, so the 100 ms window is meaningful), `MCAN` on FDCAN1 (the G4 FDCAN is the same Bosch M_CAN IP as on the H7), `STM32F7_USART` on USART3, UART4 and LPUART1, `STM32F7_I2C` on I2C3, `STM32SPI` on SPI2, `STM32_Timer` on TIM3, `STM32LDMA` on DMA1, GPIO ports A-G, flash and RAM.

**Stubs (plain Python register files in the .repl), and why:**

- **RCC**: the HAL polls `HSERDY`, `PLLRDY`, `LSIRDY` and `SWS` and times out into `Error_Handler` otherwise. The stub stores every write and derives the ready bits from the matching enable bits (`HSEON` gives `HSERDY`, `PLLON` gives `PLLRDY`, `LSION` gives `LSIRDY`, `SW` gives `SWS`), so the firmware still has to request each oscillator and the PLL. No clock tree is modelled. The "clock init" test therefore checks that the firmware programs the intended configuration and that the HAL flow completes, not that the clocks would lock on silicon.
- **PWR**: `SR2` reads 0 so `VOSF` (voltage scaling in progress) is never set and the boost-mode switch completes; other registers are stored.
- **FLASH_ACR**: stored, so the HAL's wait-state read-back after `__HAL_FLASH_SET_LATENCY` matches.
- **NBTP is decoded at write time** by a Renode peripheral-write hook, because the stock M_CAN model does not store the register. So the test proves the firmware programs the right value, not that a bus with that timing works. FDCAN is not started by the image, so no frames are exchanged.

Not modelled (writes are ignored with a log warning) and therefore **not** tested: DMAMUX, EXTI, SYSCFG, ADC, the CRS/HSI48, USB, real clock speeds and timing margins, HSE crystal behaviour, MPU, real FDCAN bus traffic, power modes, flash wait states, the option bytes (`nSWBOOT0` etc.). The IWDG model is a generic STM32 one; it does not model the option-byte hardware watchdog or the stop/standby freeze bits.

## Run locally

Install the pinned Renode release (macOS: the arm64 portable `.app` from the same GitHub release; Linux: the portable tarball). Then:

```sh
# 1. Build the image (arm-none-eabi-gcc and ninja on PATH)
(cd cubemx && cmake --preset Debug && cmake --build --preset Debug)

# 2. Python deps in a venv (Python 3.11-3.13), then run from a scratch directory (Robot writes its logs to the cwd)
python3 -m venv /tmp/rvenv && /tmp/rvenv/bin/pip install -r tests/renode/requirements.txt
mkdir -p /tmp/renode-out && cd /tmp/renode-out
PATH=/tmp/rvenv/bin:$PATH /path/to/renode/renode-test "$OLDPWD/tests/renode/boot.robot"
```

The super-loop spins at full speed, so 2 s of emulated time takes about 35 s of wall time; the whole suite takes about 1 minute.

Interactive: `renode tests/renode/safety-node.resc`, then `emulation RunFor "0.5"`. Set `$elf` first to load another ELF.

CI uploads `log.html`, `report.html`, `robot_output.xml` and the Renode logs as the `renode-boot-safety-node` artifact.

## Updating Renode

Bump `RENODE_VERSION` and `RENODE_SHA256` in `.github/workflows/ci.yml` (use a release that is at least two weeks old; the checksum is the `digest` GitHub shows for the tarball) and the versions above, then re-run the job. If a future Renode release ships an STM32G4 platform, replace the stubs with it. Probe messages and register decoding depend on the Renode models; re-check them on a bump.
