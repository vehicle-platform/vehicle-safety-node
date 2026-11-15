#ifndef FEATURES_INDICATOR_INDICATOR_CONFIG_H
#define FEATURES_INDICATOR_INDICATOR_CONFIG_H

/*
 * Turn-signal input layer constants (D-075 item 4, D-082; D-111, provisional, defs#165; Q-033). Every value is a
 * compile-time constant in this one header; none comes from vehicle-defs because the
 * signals are the M4's own GPIO inputs, not CAN signals (invariant 2: no CAN ID, signal
 * name or DID is written here).
 */

/*
 * Pin polarity (hardware-integration.md section 3.4 / G6d, C1-IO): IND_L and IND_R each
 * drive a PC817 opto-isolator LED through a series resistor; the phototransistor side has
 * a pull-up to 3V3 sized for the minimum CTR. A lit indicator lamp (12 V on the feed)
 * therefore pulls the M4 GPIO LOW: the active level is LOW. An open wire, a dead opto or a
 * dead lamp feed reads HIGH = "not signalling". The hardware doc does not state the level
 * in words, so it is derived from "pull-up sized for the minimum CTR" and is CONFIRMED on
 * the board (M13). Set to 0 only if the board turns out active-high.
 */
#define INDICATOR_ACTIVE_LEVEL_LOW (1)

/*
 * Debounce: a raw level change is accepted only after it has stayed unchanged for at least
 * this many consecutive samples AND this many milliseconds (both must hold, so the rule
 * does not depend on the super-loop period). Rejects relay/contact bounce and EMI spikes
 * on the harness; a flash on-time is at least ~300 ms (below), so 20 ms costs nothing.
 */
#define INDICATOR_DEBOUNCE_MS (20u)
#define INDICATOR_DEBOUNCE_SAMPLES (3u)

/*
 * Hold: a side signals from its first debounced edge until this long after its last
 * debounced edge (D-111 item 2, provisional). UN ECE R6 / R48 give 90 +/- 30 flashes/min, so the
 * slowest flasher (60/min) has a 1 s period; 1.5 s covers one missed flash. The hold also
 * covers the dark half of the cycle, which is why a lit-only reading would flicker.
 */
#define INDICATOR_HOLD_MS (1500u)

/*
 * Stuck-active fault: an input that has read active for this long without a debounced edge
 * is a fault (shorted opto, lamp feed shorted to the feed, a lamp on continuously), not a
 * flash: even at the slowest 60/min (1 s period) a lamp is lit for under 1 s, so 3 s is well
 * beyond any flash on-time. A fault is reported as a flag and reads as "not signalling".
 */
#define INDICATOR_STUCK_MS (3000u)

/*
 * Maximum period between two step() calls (super loop): 40 ms. With 3 samples within the
 * 20 ms debounce the loop only needs to be faster than the shortest flash on-time (about
 * 150 ms at the fastest 120/min with a short duty) divided by a few samples; 40 ms keeps
 * at least 3 samples inside any real on-time. Documentation only, nothing enforces it; the
 * tests also run at 50 ms to show the margin.
 */
#define INDICATOR_MAX_LOOP_MS (40u)

#endif
