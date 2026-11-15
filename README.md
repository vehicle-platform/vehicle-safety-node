# vehicle-safety-node

Part of [vehicle-platform](https://github.com/vehicle-platform), an SDV-style diagnostics, telemetry and rider-assistance platform for motorcycles (first vehicle: Honda CL250).

Firmware of the **safety partition**: a separate **STM32G474RE** (Cortex-M4F, FDCAN; prototype board NUCLEO-G474RE, D-118), a node on the platform CAN bus; `vehicle-rt-core` runs on its own MCU. It holds the cornering safety decision (D-041) with its own fallback IMU (D-042; no estimator until the IMU hardware, D-100 item 2), the blind-spot decision, the immobilizer, and drives the lean-angle LED ring and mirror LEDs. It receives rt-core's lean/friction estimates and heartbeat over the platform bus (E2E-checked, every frame) and keeps deciding when rt-core, the Raspi or the phone crash.

**Status:** the platform-bus link (`src/services/platform_link*`, `src/hal/can.h`) and the Layer 1 cornering decision, host-tested (D-041, D-042, D-100, D-118). No board image yet: the CubeMX project, startup, linker script and FDCAN setup come with the NUCLEO-G474RE (D-118).

## Build and test

```bash
git submodule update --init            # vehicle-defs, pinned to a release tag
cmake --preset host-tests && cmake --build --preset host-tests && ctest --preset host-tests
cmake --preset target-g474-release && cmake --build --preset target-g474-release        # arm-none-eabi-gcc, Cortex-M4F
```

Static analysis (CI, blocking): cppcheck 2.22 warnings and the MISRA C:2012 addon; deviations in [`misra/README.md`](misra/README.md).

- Architecture and decisions: [vehicle-defs/docs](https://github.com/vehicle-platform/vehicle-defs/tree/main/docs) (`ARCHITECTURE.md`, `DECISIONS.md`)
- Signals, CAN IDs and DIDs come only from [vehicle-defs](https://github.com/vehicle-platform/vehicle-defs) (git submodule pinned to a tag)
- Scope rules for contributors and Claude Code: [`CLAUDE.md`](CLAUDE.md)

## License

MIT, see [LICENSE](LICENSE) (D-036).
