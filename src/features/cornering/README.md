# features/cornering — Layer 1 cornering decision (v1)

Responsibility: decide the cornering warning level of the safety partition from rt-core's
lean (0x020) and friction (0x022) estimates, gated by rt-core's heartbeat (0x081), all
received through the M7→M4 mailbox (D-072 item 4c, D-093). Decisions: D-041, D-042 items
3-4, D-029 (µ rule), D-075 items 5-6, D-100, D-101 (item 5: µ on 0x022 loss).

## Files

| File | Role |
|---|---|
| `cornering_core.[ch]` | Pure logic: input usability, µ_eff, source selection, decision, hysteresis. No `hal/`, no `services/` (CI grep). |
| `cornering.[ch]` | Glue: `mailbox_take()` → generated E2E check (compile-time length) → generated timeout check every pass → gen unpack → core. Called by `app/safety_app.c` only. |

## Inputs (per super-loop pass, ≤ 5 ms, after `mailbox_step()`)

| Frame | Usable when | Not usable → |
|---|---|---|
| 0x081 HeartbeatRtCore | E2E OK (INITIAL is not usable, D-026), NODE_MODE = NORMAL, within its 300 ms E2E timeout | source NONE at once |
| 0x020 EkfLean | E2E OK, LEAN_ANGLE_STATE ESTIMATED/CLAMPED, raw in the DBC range (\|θ\| ≤ 90°), finite, Q-029 on raw integers (ESTIMATED \|raw\| ≤ clamp, CLAMPED \|raw\| ≥ clamp − one 0.01° step), within its 60 ms E2E timeout | source NONE at once |
| 0x022 EkfFrictionMass | E2E OK, within its 300 ms timeout, and a usable value: the D-029 rule (ESTIMATED/CLAMPED → clamp [0.1, 1.2]; DEFAULT → max(0.1, min(µ, 0.5))); INVALID (the 2-bit state has no RESERVED value) or a raw value outside the DBC range counts as not usable (D-101 item 5; counted apart from E2E failures: `mu_e2e_rejected` / `mu_value_rejected`) | µ_eff = min(last valid D-029 result, `friction_coeff_default`), or the default with no valid 0x022 since boot (D-101 item 5: a lost low µ never raises µ_eff); 0x022 is not a source criterion (D-042 item 3), so NODE_MODE is unchanged |

## Source selection (D-042 item 3, D-075 item 6 (4), D-101 item 4)

- Sources in v1: NONE and EKF. The own-IMU source is "not usable" by construction (D-100
  item 2): there is no estimator until the IMU hardware, so no OWN_IMU branch exists and the
  D-075 item 6 (1) cross-check has no object yet. Both arrive with their own decision on
  Q-023 (5)/(6).
- EKF → NONE at once on the first not-usable 0x020, a 0x020 timeout or a heartbeat that is
  not OK/NORMAL (also between frames).
- NONE → EKF only after `CORNERING_RETURN_FRAMES` (10, provisional) consecutive usable 0x020
  frames, each received while the heartbeat is OK and NORMAL; boot included. At boot the
  heartbeat itself needs two frames (INITIAL, then OK), so the first EKF decision comes
  about 380 ms after rt-core starts sending (test_cornering), on the safe side of D-101's
  "about 200 ms after the first usable frame". Any break resets the count.
- NONE is named **"rt-core inputs lost"** (`cornering_status().rt_core_inputs_lost`,
  `inputs_lost_onsets`): the M4 cannot tell an FDCAN2 bus-off on the M7 from a dead rt-core
  (ISSUES E-28 (1)), so it is never counted as an rt-core fault, never in 0x082 ERROR_COUNT.

## Decision (D-041, D-100)

- u = tan\|θ\| / µ_eff, division-free: GREEN only if sin\|θ\| < k_yellow·µ·cos\|θ\|, YELLOW
  only if sin\|θ\| < k_red·µ·cos\|θ\|, otherwise RED; any non-finite value gives RED.
  k_yellow, k_red, the µ limits and the lean clamp come from gen `platform_limits.h`.
- Hysteresis (D-100 item 3, named constants `CORNERING_HYST_U_MARGIN` 0.05 and
  `CORNERING_HYST_HOLD_MS` 100 ms, tuned on HIL): a more severe level at once; RED → YELLOW
  after u < k_red − 0.05, → GREEN after u < k_yellow − 0.05, each continuously for 100 ms on
  the M4's timebase.
- Evaluated only on fresh input (a new 0x020 or 0x022 frame, a 0x022 timeout, or the pass
  that switches to EKF), so the hold timers advance on fresh frames only; between them the
  level is held. A lost source gives UNAVAILABLE at once and resets the hysteresis.
- v1 is lateral only (Q-022 open): no friction circle, no camber.

## Output

`cornering_output()` = {level, source} with the 0x023 pairing invariant (NONE ⇔
UNAVAILABLE; EKF with GREEN/YELLOW/RED), checked again by `app/` before packing 0x023.
The LED ring driver (later, with the board) must honour D-072 item 5: GREEN is never a static
frame (moving alive pixel or a hardware dead-man), and a dark or frozen ring means "no
warning available".

## Timing and resources

- Super loop only, never from an ISR; static storage (~100 B RAM), no heap; the first WCET
  group (D-072 item 3). M4 release: about 1.5 KB text for core + glue, plus `sinf`/`cosf`
  from libm at the final link.

## Tests

`tests/host/test_cornering_core.c` (boundaries, Q-029, µ rule, selection, hysteresis),
`tests/host/test_cornering.c` (glue with real gen E2E frames: CRC, repeated counter,
timeout, INITIAL, heartbeat, µ loss), `tests/host/test_safety_app.c` (end to end through
the mailbox, first-pass baseline, 32-bit ms wrap). Not covered on the host: the defensive
output-fault paths of `app/safety_app.c` (pack/protect/offer failure), unreachable through
the public API; a `-Wl,--wrap` fault-injection build follows before the board PR
(safety-reviewer MINOR-5).

## HIL scenario (proposal, `vehicle-hil-bench/scenarios/`)

`cornering_rt_core_inputs_lost`: restbus sends 0x020 at 10° with µ 0.5 DEFAULT and 0x081
NORMAL; after the ring reports GREEN, stop 0x020. Expected: 0x023 CORNERING_LEVEL
UNAVAILABLE / SOURCE NONE within 60 ms + one pass of the last frame, 0x082 NODE_MODE DEGRADED
in the next heartbeat, ERROR_COUNT unchanged; resume 0x020 → GREEN again only after 11
frames (one INITIAL + 10).
