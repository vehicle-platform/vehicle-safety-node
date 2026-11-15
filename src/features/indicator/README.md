# indicator: turn-signal input layer (D-075 item 4, D-082; D-111, provisional, defs#165)

The M4 reads the rider's turn-signal state on its **own** opto-isolated GPIOs (`IND_L`, `IND_R`, PC817 on C1-IO, hardware-integration G6d; Q-033), with no dependency on rt-core or the 0x120 usage contract (D-072 item 7). Basis: D-075 item 4 and D-082; D-111 (provisional, defs#165) only records the input-layer rules. This module is the input layer only: debounce, "signalling" window, hazard, fault flags. The blind-spot decision that will use it does not exist yet (radar hardware pending) and is not written here.

## Safety contract

The output may only **escalate** a blind-spot warning (for example strengthen an APPROACHING vehicle on the signalled side). It must **never suppress or downgrade PRESENT or APPROACHING**, and it is **not a health or plausibility signal**.

- **Detected faults** (open wire, dead opto, stuck-lit > 3 s) read `INDICATOR_STATE_NONE`.
- **Undetected faults** (a frozen clock after signalling, a lit line with >= 20 ms drop-outs, a lamp lit at boot or wrong polarity, a floating pin) can read as signalling. Each has a test that locks the behaviour in.
- Therefore signalling may only ever be used to escalate. With NONE the blind-spot decision is exactly what it would be without this module.
- **Lamp lit at boot** (or an idle-low pin): signals for up to 1.5 s from the first debounced edge, then reads NONE, and the stuck flag appears after 3 s. This only escalates and is intentionally not special-cased.

## Files

| File | Role |
|---|---|
| `indicator_config.h` | All constants with sources (D-111 provisional, UN ECE R6/R48), the pin polarity and the maximum loop period |
| `indicator_core.[ch]` | Pure logic, caller-owned state, no `hal/` or `services/` |
| `indicator.[ch]` | Glue: reads `hal/indicator_gpio.h`, calls the core. Not yet called from `app/` (no consumer). A failed HAL read forces both inputs inactive (open-wire level) |
| `hal/indicator_gpio.h`, `hal/host/indicator_gpio_host.[ch]` | HAL interface (`bool` read; the board port comes with CubeMX and enables the MCU internal pull-up as well as the external one) and the host model with a failure mode |

## Behaviour

- Polarity: active **LOW** (`INDICATOR_ACTIVE_LEVEL_LOW`): the opto pulls the pulled-up GPIO low while the 12 V lamp feed is lit. Derived from "pull-up sized for the minimum CTR" in hardware-integration.
- **M13 bring-up check:** with the ignition on and no indicator lit, both pins read HIGH and `fault_flags` is 0; then each indicator on its own must read LEFT / RIGHT, and the hazard switch HAZARD. A pin low at idle means wrong polarity or wiring.
- Debounce: a level change needs 3 consecutive samples lasting at least 20 ms.
- Signalling: from the first debounced edge until 1.5 s after the last debounced edge (90 +/- 30 flashes/min, UN ECE R6/R48; the slowest 60/min has a 1 s period).
- HAZARD: both sides signalling.
- Stuck-active: an input active 3 s without an edge latches `INDICATOR_FAULT_*_STUCK` and reads as not signalling; it clears when the input goes inactive (that release does not start a hold). A faulty side never counts towards HAZARD.
- Time: monotonic `uint32_t` milliseconds, unsigned subtraction (wrap-safe); an expired hold is dropped at once so a wrap cannot revive it. Correct while no gap between two steps reaches 2^32 - 1.5 s ms; the practical limit is the super-loop period of at most 40 ms (`INDICATOR_MAX_LOOP_MS`; tests also run at 1, 25 and 50 ms with jittered timestamps).

## Consumer requirements (the blind-spot feature, not written yet)

- Final level = max(base_level, escalation(indicator)); the base level is computed **without any indicator input**.
- An exhaustive test over every indicator state x base level x fault-flag value must assert final >= base.
- HAZARD means escalate both sides or neither; it never suppresses either side.
- The stuck flags must reach a status signal or DTC through `/signal-change` in vehicle-defs, never hand-written here.
- Re-run `safety-reviewer` when the consumer lands.

## Resources

No dynamic memory. State: 2 x 20 bytes in `indicator_core_t`, plus 2 B of output on target (short enums). Flash about 380 bytes (Cortex-M4F `-Os` release).

## Tests

`tests/host/test_indicator_core.c`: flash trains, hazard, hold expiry at exactly 1.5 s, bounce, debounce and stuck boundaries (2 samples >= 20 ms, 3 samples at 19 ms, 3 at 20 ms, stuck at 3000 ms not 2999, both sides stuck), open wire, wrap, polarity, 1/25/50 ms loop periods with jitter at 60/90/120/180 flashes per minute, and the four undetected-fault behaviours. `tests/host/test_indicator.c`: glue, HAL model and a failed HAL read.

## HIL

Scenarios for `vehicle-hil-bench` (write only when the host schema exists):
1. IND_L at 90/min: LEFT within one debounce, held through the dark phases, NONE 1.5 s after the stop.
2. Same for IND_R, and both together: HAZARD.
3. 60/min with a short on-time and 120/min: no drop-out.
4. Hyperflash 180/min reads LEFT.
5. IND_L held lit for 4 s: fault flag set, NONE after 1.5 s; release clears it without signalling.
6. Open wire (pins left at pull-up) and a dead opto: NONE, no fault flag.
7. Contact bounce (10 ms chatter) on a lit edge: no switching.
8. Lamp lit at boot: signalling for at most 1.5 s, then NONE, stuck flag at 3 s.
9. Floating pin or EMI chatter: documented as possibly signalling (escalation only); the blind-spot level must never fall below its base.
