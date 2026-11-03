# vehicle-safety-node

Part of [vehicle-platform](https://github.com/vehicle-platform), an SDV-style diagnostics, telemetry and rider-assistance platform for motorcycles (first vehicle: Honda CL250).

Firmware of the **safety partition**: the Cortex-M4 core of the **STM32H755** (D-072; prototype board NUCLEO-H745ZI-Q, D-094); the Cortex-M7 runs `vehicle-rt-core`. It holds the cornering safety decision (D-041) with its own fallback IMU (D-042; no estimator until the IMU hardware, D-100 item 2), the blind-spot decision, the immobilizer, and drives the lean-angle LED ring and mirror LEDs. It talks to rt-core only through an E2E-checked inter-core mailbox and keeps deciding when rt-core, the Raspi or the phone crash.

**Status:** skeleton + the M4 side of the M7<->M4 mailbox (D-093, `src/services/mailbox*`), host-tested; Layer 1 inputs and the cornering decision follow (D-041, D-042, D-100). No board image yet: the CubeMX CM4 project, startup, linker script and MPU setup come with the NUCLEO-H745ZI-Q on the desk (D-094).

## Build and test

```bash
git submodule update --init            # vehicle-defs, pinned to a release tag
cmake --preset host-tests && cmake --build --preset host-tests && ctest --preset host-tests
cmake --preset host-tsan && cmake --build --preset host-tsan && ctest --preset host-tsan   # two-core mailbox model
cmake --preset target-m4-release && cmake --build --preset target-m4-release            # arm-none-eabi-gcc, Cortex-M4F
```

Static analysis (CI, blocking): cppcheck 2.22 warnings and the MISRA C:2012 addon; deviations in [`misra/README.md`](misra/README.md).

- Architecture and decisions: [vehicle-defs/docs](https://github.com/vehicle-platform/vehicle-defs/tree/main/docs) (`ARCHITECTURE.md`, `DECISIONS.md`)
- Signals, CAN IDs and DIDs come only from [vehicle-defs](https://github.com/vehicle-platform/vehicle-defs) (git submodule pinned to a tag)
- Scope rules for contributors and Claude Code: [`CLAUDE.md`](CLAUDE.md)

## License

MIT, see [LICENSE](LICENSE) (D-036).
