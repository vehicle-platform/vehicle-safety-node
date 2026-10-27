# vehicle-safety-node

Part of [vehicle-platform](https://github.com/vehicle-platform), an SDV-style diagnostics, telemetry and rider-assistance platform for motorcycles (first vehicle: Honda CL250).

Firmware of the **safety partition**: the Cortex-M4 core of the **STM32H755** (D-072); the Cortex-M7 runs `vehicle-rt-core`. It holds the cornering safety decision (D-041) with its own fallback IMU (D-042), the blind-spot decision, the immobilizer, and drives the lean-angle LED ring and mirror LEDs. It talks to rt-core only through an E2E-checked inter-core mailbox and keeps deciding when rt-core, the Raspi or the phone crash.

**Status:** skeleton, no code yet. The build system, tests and CI are added by `/repo-bootstrap vehicle-safety-node` when work on this repo starts (setup order: `vehicle-defs/docs/ARCHITECTURE.md` §9).

- Architecture and decisions: [vehicle-defs/docs](https://github.com/vehicle-platform/vehicle-defs/tree/main/docs) (`ARCHITECTURE.md`, `DECISIONS.md`)
- Signals, CAN IDs and DIDs come only from [vehicle-defs](https://github.com/vehicle-platform/vehicle-defs) (git submodule pinned to a tag)
- Scope rules for contributors and Claude Code: [`CLAUDE.md`](CLAUDE.md)

## License

MIT, see [LICENSE](LICENSE) (D-036).
