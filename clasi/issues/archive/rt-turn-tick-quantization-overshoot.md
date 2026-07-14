---
status: obsolete
---

> **OBSOLETE (2026-07-14 stakeholder triage).** Superseded by the single-loop
> firmware rebuild (`clasi/issues/single-loop-firmware-de-fiber-delete-the-elite-plumbing-telemetry-only-return-path.md`;
> review: `docs/code_review/2026-07-13-devices-drive-review.md`). The RT open-loop turn path (and the entire on-robot motion stack, sim plant included) is deleted; turns become host-planned twist streams with heading feedback.

# RT open-loop turn overshoots +1.08° per 90° — tick-quantization residual in coast anticipation

## Problem

`RT` (open-loop relative turn, encoder-arc stop) deterministically over-rotates
by **+1.08° per 90° turn** in sim with all error knobs at zero and calibration
neutralized. Tour 2 has six `RT 9000` corners, so the error accumulates to
**+6.5° of heading** by the end of the tour and the loop visibly fails to
close (final heading −173.54° vs 180°; position closes to (0, −12) mm).

## Evidence

Measured from `recordings/recording_20260703_122632.jsonl` (Tour 2, robot
`tovez nocal`, zero sim errors, `SET rotSlip=0` push confirmed working —
overshoot is NOT the 0.92 slip factor, which would read ~99°):

- All six `RT 9000` → **91.08° net body rotation each** (91.38/91.07
  alternation with the following D leg absorbing ~0.3°; deterministic, both
  `pose` and `otos` agree, and otos derives from plant truth).
- Per-wheel encoder arc: **101.75 mm actual** vs **100.53 mm geometric
  target** (90° at trackwidth 128, slip = 1.0) vs **96.73 mm stop fire**
  (coast anticipation predicts 3.80 mm = 70²/(2·720) → 3.40°; actual coast
  ≈ 5.0 mm).
- Deficit = **1.2 mm/wheel ≈ 1.5 control ticks** (controlPeriod = 10 ms,
  wheel speed 78.2 mm/s at the 70°/s RT cruise rate) = +1.08° body.

## Root cause

Sprint 073 (`sim-turn-undershoot`) replaced the stale hand-tuned 8 mm coast
constant with a ramp-dynamics formula in `PlannerBegin.cpp` `beginRotation()`
— but the formula is the *continuous* integral `rate²/(2·yawAccMax)`. Two
discrete effects are unmodeled:

1. The ROTATION stop condition is only polled once per control tick, so the
   wheels travel up to a full tick (avg ~half) past the stop threshold before
   the SOFT ramp even starts.
2. The discrete trapezoid ramp-down coasts ~`v·dt/2` more than the continuous
   integral predicts.

Together ≈ 1.5 ticks of cruise arc ≈ 1.2 mm/wheel, matching the measurement.

## Proposed fix

Either (or both):

- Extend the coast anticipation by the discretization term:
  `coastArc += wheelSpeed · dt · 1.5` (computed from live
  `cfg.controlPeriod`, keeping 067's live-reference guarantee), or
- Make the ROTATION stop predictive: fire when
  `remaining arc < wheelSpeed · dt` instead of on threshold crossing.

## Notes

- `D` legs overrun 3–6 mm (~1–1.5 ticks at 200 mm/s) — same quantization
  class, minor for loop closure; a fix here could share the same predictive
  stop mechanism but is secondary.
- `TURN` (closed-loop on fused heading) does not accumulate this error; tours
  could alternatively use TURN at absolute headings, but that works around
  rather than fixes RT.
- Acceptance: Tour 2 geometry at zero sim error should close with < 1°
  accumulated heading error over six RT 9000 corners (cf.
  `tests/testgui/test_tour1_geometry.py` from 073-004).
