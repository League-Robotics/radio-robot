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
