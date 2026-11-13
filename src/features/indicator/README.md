# indicator: turn-signal input layer (D-111, provisional; Q-033)

The M4 reads the rider's turn-signal state on its **own** opto-isolated GPIOs (`IND_L`, `IND_R`, PC817 on C1-IO, hardware-integration G6d), with no dependency on rt-core or the 0x120 usage contract (D-072 item 7, D-075 item 4, D-111). This module is the input layer only: debounce, "signalling" window, hazard, fault flags. The blind-spot decision that will use it does not exist yet (radar hardware pending) and is not written here.

## Safety contract

The output may only **escalate** a blind-spot warning (for example strengthen an APPROACHING vehicle on the signalled side). It must **never suppress or downgrade PRESENT or APPROACHING**. Every failure reads as `INDICATOR_STATE_NONE` ("not signalling"): a broken wire, a dead opto, a stuck-active input, no sample yet. With NONE the blind-spot decision is exactly what it would be without this module (invariant 3: nothing here can make the safety path worse).

## Files

| File | Role |
|---|---|
| `indicator_config.h` | All constants with sources (D-111, UN ECE R6/R48) and the pin polarity |
| `indicator_core.[ch]` | Pure logic, caller-owned state, no `hal/` or `services/` |
| `indicator.[ch]` | Glue: reads `hal/indicator_gpio.h`, calls the core. Not yet called from `app/` (no consumer) |
| `hal/indicator_gpio.h`, `hal/host/indicator_gpio_host.[ch]` | HAL interface (board port comes with CubeMX) and the host model |

## Behaviour

- Polarity: active **LOW** (`INDICATOR_ACTIVE_LEVEL_LOW`): the opto pulls the pulled-up GPIO low while the 12 V lamp feed is lit. Derived from "pull-up sized for the minimum CTR" in hardware-integration; confirm on the board (M13).
- Debounce: a level change needs 3 consecutive samples lasting at least 20 ms.
- Signalling: from the first debounced edge until 1.5 s after the last debounced edge (90 +/- 30 flashes/min, UN ECE R6/R48; the slowest 60/min has a 1 s period).
- HAZARD: both sides signalling.
- Stuck-active: an input active 3 s without an edge latches `INDICATOR_FAULT_*_STUCK` and reads as not signalling; it clears when the input goes inactive (that release does not start a hold). A faulty side never counts towards HAZARD.
- Time: monotonic `uint32_t` milliseconds, unsigned subtraction (wrap-safe); an expired hold is dropped at once so a wrap cannot revive it. The core must be stepped at least once per 1.5 s.

## Resources

No dynamic memory. State: 2 x 20 bytes in `indicator_core_t` (plus a 2-byte output in the glue). Flash about 380 bytes (Cortex-M4F `-Os` release).

## Tests

`tests/host/test_indicator_core.c` (flash trains, hazard, hold expiry at exactly 1.5 s, bounce, stuck, open wire, wrap, polarity) and `tests/host/test_indicator.c` (glue + HAL model). Proposed HIL scenario: drive IND_L with a 90/min square wave and check LEFT within 20 ms + one flash, then stop the wave and check NONE after 1.5 s.
