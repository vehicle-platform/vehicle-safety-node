# CLAUDE.md — vehicle-safety-node

@.claude/PLATFORM-RULES.md

## What this repo is

The firmware of the **safety partition**: the **Cortex-M4 core of the STM32H755** (NUCLEO-H755ZI-Q, D-072). The Cortex-M7 of the same chip runs `vehicle-rt-core`; this repo builds only the M4 image (flash bank 2, D2 SRAM). Modules, each separate under `src/features/`:
- `cornering/`: the Layer 1 decision (D-041: u = tan|θ| / µ_eff, division-free form, GREEN/YELLOW/RED with hysteresis; no speed, no mass) and the own-IMU fallback (D-042: DEGRADED, UNAVAILABLE, never GREEN without a usable source).
- `blind_spot/`: 2× 24 GHz radar (own UARTs) → approach logic and false-alarm filter → mirror LEDs.
- `immobilizer/`: NFC (PN532) primary, PIN as backup → bistable relay on the starter-relay coil.
- outputs: the LED ring and the mirror LEDs (Q-026 resolved: driven here, never by rt-core).

## What this repo is NOT

- EKF estimation is NOT here: lean and µ come from rt-core (M7) through the inter-core mailbox, E2E-protected (D-072 item 4c). Do not recompute them.
- No ML. ML can only tighten thresholds (min rule, D-041), never loosen the ceiling; without ML data the decision is unchanged.
- No CAN: the M4 has no FDCAN. Its heartbeat 0x082 and status go to the M7 through the mailbox; the M7 forwards them verbatim.
- No QM function (logging, telemetry, derived signals, power management, park mode: rt-core). A new function here needs a decision and `safety-reviewer`.
- No engine/ECU intervention. Outputs are warnings (LEDs) and the immobilizer relay only.

## Safety-critical rules

1. No dynamic memory, no recursion, no unbounded loops. Bare-metal super loop + timer interrupt (D-012) with a fixed schedule and a WCET budget per step: cornering and blind spot first, then the LED outputs, then the immobilizer and housekeeping, which can never delay the first two (D-072 item 3).
2. Inputs are clipped to their physical range. Mailbox payloads are checked like CAN frames (timeout, counter, CRC; D-005, `e2e-profile.md`); not usable → D-042 fallback.
3. rt-core's heartbeat (0x081 through the mailbox) is monitored; loss or NODE_MODE ≠ NORMAL → fallback (D-042 item 3).
4. Own resources only (D-072 item 4): own GPIO ports, SPI IMU, radar UARTs, timers, IWDG2; boots on its own from bank 2 and never waits on the M7; never spin-waits on the HSEM.
5. Immobilizer (invariant 6): starter-relay coil only, default unlocked, hidden bypass, 10 s timeout to unlocked from an M4 timer; a boot never pulses the relay; a lock request from the M7 is only a request.
6. A dead or hung M4 must never leave a frozen GREEN ring or "side clear" mirror (D-072 item 5): GREEN carries a moving alive pixel or the ring supply goes through a hardware dead-man.
7. The `safety-reviewer` agent is invoked after every change.

## Dependencies

`vehicle-defs` → `external/vehicle-defs` (tagged), only `gen/c/safety/` is included. Signals/IDs are never hand-written.

## Build

CMake + STM32CubeMX (HAL, the CM4 part of the dual-core H755 project) + arm-none-eabi-gcc (D-007). Decision logic is pure C without a HAL dependency and is tested under `tests/host/` with Unity (boundary values, E2E error conditions, fallback transitions).

## Context

`../vehicle-defs/docs/ARCHITECTURE.md` §2-4, §7 · DECISIONS D-041, D-042, D-072 · detail: `../vehicle-defs/docs/hardware-architecture.md` §5b.1-5b.3.
