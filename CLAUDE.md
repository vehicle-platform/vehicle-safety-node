# CLAUDE.md — vehicle-safety-node

@.claude/PLATFORM-RULES.md

## What this repo is

The firmware of the **safety partition**: a **separate STM32G474RE** (Cortex-M4F 170 MHz, FDCAN; prototype board NUCLEO-G474RE, fallback G431RB; D-118, which supersedes the dual-core H755 partition of D-072). It is a node on the **platform CAN bus** only: own clock, flash, RAM, watchdog and reset, it starts without rt-core. `vehicle-rt-core` runs on its own STM32H743. Modules, each separate under `src/features/`:
- `cornering/`: the Layer 1 decision (D-041: u = tan|θ| / µ_eff, division-free form, GREEN/YELLOW/RED with hysteresis; no speed, no mass) and the own-IMU fallback (D-042: DEGRADED, UNAVAILABLE, never GREEN without a usable source). v1 (D-100, C2 PR2): lateral only, no own-IMU estimator yet, so a lost rt-core lean gives UNAVAILABLE; see `src/features/cornering/README.md`.
- `blind_spot/`: 2× 24 GHz radar (own UARTs) → approach logic and false-alarm filter → mirror LEDs.
- `immobilizer/`: NFC (PN532) primary, PIN as backup → bistable relay on the starter-relay coil.
- outputs: the LED ring and the mirror LEDs (Q-026 resolved: driven here, never by rt-core).

## What this repo is NOT

- EKF estimation is NOT here: lean and µ come from rt-core over the platform CAN bus (0x020, 0x022), E2E-protected (D-118, D-005). Do not recompute them.
- No ML. ML can only tighten thresholds (min rule, D-041), never loosen the ceiling; without ML data the decision is unchanged.
- The platform bus only: it receives 0x020, 0x022, 0x081 (safety inputs, E2E) and 0x120 (QM, immobilizer only, D-072 item 7) and transmits 0x023 and 0x082 itself (D-118). It never touches the vehicle bus (invariant 1); the hardware filter accepts only those four IDs.
- No QM function (logging, telemetry, derived signals, power management, park mode: rt-core). A new function here needs a decision and `safety-reviewer`.
- No engine/ECU intervention. Outputs are warnings (LEDs) and the immobilizer relay only.

## Safety-critical rules

1. No dynamic memory, no recursion, no unbounded loops. Bare-metal super loop + timer interrupt (D-012) with a fixed schedule and a WCET budget per step: cornering and blind spot first, then the LED outputs, then the immobilizer and housekeeping, which can never delay the first two (D-072 item 3).
2. Inputs are clipped to their physical range. Every received frame is E2E-checked (DLC, CRC with the Data ID, counter, timeout; D-005, `e2e-profile.md`), every frame and not only the newest, so a second sender of an ID (masquerade) gives WRONG_SEQUENCE and the D-042 fallback, never silent use.
3. rt-core's heartbeat (0x081 on the platform bus) is monitored; loss or NODE_MODE ≠ NORMAL → fallback (D-042 item 3). rt-core liveness is only this E2E/timeout path; there is no peer handshake.
4. Own resources only: own GPIO ports, SPI IMU, radar UARTs, timers, own watchdog (IWDG); boots on its own and never waits on rt-core; no call on the CAN HAL blocks, a refused send is counted and never retried.
5. Immobilizer (invariant 6): starter-relay coil only, default unlocked, hidden bypass, 10 s timeout to unlocked from an own timer; a boot never pulses the relay; a lock request from rt-core (0x120) is only a request.
6. A dead or hung node must never leave a frozen GREEN ring or "side clear" mirror (D-072 item 5): GREEN carries a moving alive pixel or the ring supply goes through a hardware dead-man.
7. The `safety-reviewer` agent is invoked after every change.

## Dependencies

`vehicle-defs` → `external/vehicle-defs` (tagged), only `gen/c/safety/` is included. Signals/IDs are never hand-written.

## Build

CMake + arm-none-eabi-gcc (D-007); the CubeMX project (target STM32G474RE, D-118), startup, linker script, FDCAN and MPU setup come with the board (red zone). Logic is pure C without a HAL dependency, tested under `tests/host/` with Unity (boundary values, E2E error conditions, fallback transitions):

- `cmake --preset host-tests && cmake --build --preset host-tests && ctest --preset host-tests` (ASan + UBSan)
- `test_safety_app_faults` (part of host-tests): fault injection into the 0x023/0x082 output paths through a second copy of `app/safety_app.c` whose pack/protect/send calls are renamed to test seams (compile definitions in `tests/host/CMakeLists.txt`; any linker, no `-Wl,--wrap`)
- `test_platform_link` and `test_safety_app` run on the platform-bus model (`tests/host/bus_model.h` over `hal/host/can_host`): valid frames, timeout, counter jump/freeze, CRC, wrong DLC, unknown IDs, masquerade, 0x120 independence, TX cycle/E2E, refused TX
- `host-coverage` preset + gcovr (CI floors 95 % lines / 85 % branches)
- `target-g474-debug` / `target-g474-release`: Cortex-M4F cross build of the same libraries
- cppcheck 2.22 + MISRA addon (deviations: `misra/README.md`; locally add `--addon-python=/usr/bin/python3` on macOS)

Layout: `src/hal/` (HAL-free interfaces, `hal/host/` models for tests), `src/services/` (`platform_link`: the platform-bus RX/TX and E2E), `src/features/<module>/` (cornering; later blind_spot, immobilizer), `src/app/` (the super-loop pass `safety_app` and the 0x082 semantics `node_health_core`, D-101). `app/` is the only composition root: features never include another feature or `app/`, services and hal never include features or `app/`, `app/` gets `now_ms` from its caller and never includes `hal/`, `*_core` files include no `hal/` or `services/`, and `hal/can.h` is used by `services/platform_link.c` only (CI greps).

## Context

`../vehicle-defs/docs/ARCHITECTURE.md` §2-4, §7 · DECISIONS D-041, D-042, D-072, D-118 · detail: `../vehicle-defs/docs/hardware-architecture.md` §5b.1-5b.3.
