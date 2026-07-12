---
status: pending
---

# Real-robot motion calibration — systematic ~6-12% undershoot on turns and straights

## Context (bench gate, 2026-07-11, firmware v0.20260711.4 on tovez)

The library-native motion redesign (Ruckig directional velocity bands,
ride-the-tail terminal stops, replan failure discipline — see
`segment-executor-stop-decel-drain-overshoot-reverses.md`'s resolution) was
flashed and verified on the stand:

- VER handshake: `0.20260711.4` ✓
- `cmd=` vs `vel=` are now DISTINCT on the wire (max |cmd−vel| ~125 mm/s
  during ramps) — the TLM cmd= mislabel fix is live; hardware tracking
  error is finally observable ✓
- **Zero commanded reversal** at every turn end (measured floor −0.4 °/s ≈
  noise) — the encoder-wedge reversal write-train trigger is structurally
  gone on hardware ✓

But absolute accuracy on the REAL plant undershoots systematically:

| move | landed | error |
|---|---|---|
| RT +90° | +78.8° | −11.2° (−12%) |
| RT −90° | −77.9° | +12.1° (symmetric) |
| RT +180° | +157.6° | −22.4° (−12%) |
| D 345 | 324.5 mm | −20.5 mm (−6%) |

Proportional (not fixed) error, symmetric in direction — a scale/tracking
deficit, not an endgame defect.

## Why (hypothesis, informed by the sim work)

The sim reached ±0.3°/±1mm only after its plant-specific quantities were
MEASURED as a set: exact feed-forward (kff = 1/plateau), honest velocity
filter, ceiling ≤ plant capability, and the effective dead time re-measured
for those gains (`kOutputHops`). The real robot has never had that pass:

- boot `kff = 0.001` vs a measured plateau of ~600-650 mm/s (1/650 ≈
  0.00154) — the feed-forward UNDERDRIVES ~35%, and ki is slow to make up
  the difference within a segment → the plant runs below its setpoint all
  segment → the encoder stops/exhaustion accept the shortfall
  ("residual accuracy is calibration work").
- `kOutputHops = 4` (80 ms) was measured for the OLD gain regime; the sim
  showed this constant is gain-dependent (its calibration moved 2.0 → 1.5
  after the gain fix).
- The divergence-replan extend-on-deficit path should be chasing exactly
  this shortfall — whether it fires on hardware (thresholds vs real noise
  floor) is unverified.

## Suggested method (transcribe the sim's, on the bench)

1. Measure the real per-wheel duty→speed plateau (steps at several duties,
   read `vel=` — cmd= is trustworthy now).
2. Set kff = 1/plateau in the robot JSON / boot config; re-pick kp/ki
   around it; decide the honest velocity-filter alpha.
3. Re-measure effective dead time (encoder-position vs command-integral
   cross-correlation — the notebook/discriminator scripts from 2026-07-11
   do this directly over serial).
4. Re-measure the replan divergence noise floor on hardware; verify the
   thresholds sit above it and below the deficit signal.
5. Acceptance: `wheel_motion_trace.ipynb` in bench mode — turns within a
   few degrees, D within a few mm, still zero reversal; then the recorded
   tour on the playfield.

HITL: wheels on the stand for everything except the final tour.

## Regression log (2026-07-11 bench session, wheels on stand)

Method as suggested above, executed as measure -> set -> flash -> score with
`wheel_motion_trace.ipynb` as the acceptance instrument. Scores are final
heading error per turn (deg):

| config | 90 | 180 | 360 |
|---|---|---|---|
| baseline (kff=0.001, hops=4) | -11.2 | -20.4 | -27.5 |
| iter 1: kff=1/650 | -16.0 | -13.4 | -24.6 |
| iter 2: + hw kOutputHops 4->6 (120ms, re-measured onset) | -10.0 | -3.1 | -5.5 |
| iter 3: + ki 0.0018->0.006 | -18.0 | -2.8 | -32.1 | REJECTED
| **landed = iter 2** (final clean capture) | **-10.6** | **-2.2** | **-9.0** |

Measurements behind the knobs:
- Duty-saturated plateau: 620-740 mm/s per wheel, BATTERY/THERMAL-STATE
  DEPENDENT (fwd L 676-737 / R 616-702; sagged to ~408 under sustained
  load in one window). kff = 1/650 chosen mid-conservative; the integrator
  absorbs the sag band.
- Motion-onset dead time (command commit `now` vs first encoder movement
  `ts`): 112-136ms -> hw kOutputHops = 6 (120ms). GAIN-DEPENDENT: the old
  80ms fit the old sluggish (kff=0.001) plant's effective lag; after the
  FF fix the 80ms model made maybeReplanPivot() shrink-retarget every
  pivot ~15-25 deg short.
- Sub-plateau duty->speed nonlinearity: tracking sits ~7-8% below the
  plateau-fitted kff line at ~150-300 mm/s -- this is the remaining 90 deg
  residual (~-10 deg, pivots cruise ~250 mm/s); 180/360 cruise near the
  plateau where kff is exact. Raising ki to close it in-move (iter 3)
  destabilized the score instead.

## Open items

1. The ~-10 deg residual on SHORT pivots: needs either a second FF point /
   speed-dependent FF, a faster-but-stable integrator, or acceptance.
2. RUN-TO-RUN VARIANCE is now the limiting factor: battery-state sag moves
   the plateau (and scores) between consecutive runs. Calibrate/score on a
   controlled battery state (fresh charge or bench supply).
3. Heavy telemetry frame loss during fast motion (known IRQ/serial-RX
   coupling) blinds host-side instruments: the cmd= integral undercounts on
   hardware, and completion flags can vanish -- any bench harness MUST
   treat encoder movement as the ground truth for "did it run" (a lossy
   run's busy flag never arriving led a retry to double-queue a segment:
   robot turned ~684 deg on a 360 ask). The notebook's capture guards
   encode these rules now.

## Iteration 4 (2026-07-11, later): reanchor velocity seed -- trajectory quality

The stakeholder's bench runs exposed that iter-2's decent ENDPOINTS hid
garbage TRAJECTORIES: mid-pivot the commanded velocity cliffed to zero,
the robot stalled ~0.25s, then re-accelerated a second full bell (heading
plateau clearly visible). Mechanism: gross-divergence `reanchor()` seeded
velocity = 0.0f ("no reliable measured angular rate") while the wheels ran
~300 mm/s -- Ruckig, told the robot was at rest, planned from rest. Fixed:
seed with the measured rate (vR - vL)/trackwidth (plan-sampled fallback).
The translate reanchor always did this; the pivot one was left lazy.

Scores after (same instrument, humps = trajectory-quality metric, sim = 1):

| turn | endpoint err | humps |
|---|---|---|
| 90  | **+3.9** (was -10.6) | 3 |
| 180 | **+4.3** (was -2.2)  | 4 |
| 360 | -19.8 (was -9.0)     | 4 |

90/180 now within ~4 deg of target with the bench heading riding ON the sim
curves. Remaining: 360 (ceiling-speed cruise) still ragged/short -- replans
still fire there (peak measured wheel 431 mm/s, above the sim plant model's
400); next measured knob is the hardware replan noise floor at ceiling
speeds. The endpoint-only scoring mistake is corrected: `humps` (count of
distinct acceleration bells; healthy = 1) is now part of the notebook
summary.

Bench-harness rule refined after burst losses sank two runs: a resend is
safe if-and-only-if nothing provably started (no busy flag AND no encoder
movement over a 3s probe); the notebook now retries up to 4 verified-idle
sends instead of skipping the source.
