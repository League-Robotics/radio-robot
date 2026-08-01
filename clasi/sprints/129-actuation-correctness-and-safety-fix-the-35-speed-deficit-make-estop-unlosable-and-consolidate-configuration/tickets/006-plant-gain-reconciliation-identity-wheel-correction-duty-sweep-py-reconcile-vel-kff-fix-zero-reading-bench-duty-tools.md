---
id: '006'
title: 'Plant-gain reconciliation: identity wheel correction, duty_sweep.py, reconcile
  vel_kff; fix zero-reading bench duty tools'
status: open
use-cases: [SUC-006]
depends-on: ['001']
github-issue: ''
issue:
- 06-duty-per-speed-and-wheel-gain-disagree-with-the-plant.md
- bench-duty-readers-see-zero-after-stageduty-park.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Plant-gain reconciliation: identity wheel correction, duty_sweep.py, reconcile vel_kff; fix zero-reading bench duty tools

## Description

Depends on ticket 001 (ESTOP fix) — this ticket requires bench driving on
the stand, which should not happen before the safety fix lands. Must
land, and be bench-verified, *before* ticket 007 (the adaptive learner) —
007's calibration corrects a residual around this ticket's baseline, not
a still-wrong one.

The unmanaged `+500` button (open loop by design) measures ~52 mm/s on a
150 mm/s request (ratio 0.347) because three constants claim to encode
the same plant gain and disagree by up to 2.34x
(`duty_per_speed_left/right` = 0.00187325 implies 534 mm/s; `vel_kff` =
0.0008 implies 1250 mm/s; `vel_kff`'s own derivation note implies 650
mm/s), and `wheel_gain`/`wheel_intercept` — fitted against
`duty_per_speed`, which was fitted against it — now points the wrong way
on tovez, compounding rather than correcting the error (1.52x from the
gain inversion x 1.9x from the real plant being ~282 mm/s per duty, not
534 = 2.9x total, matching the observed 0.347 ratio).

1. Set `wheel_gain_* = 1.0`, `wheel_intercept_* = 0.0` (identity) in
   `data/robots/*.json` first — required before any recalibration, per
   the file's own `_wheel_correction_note`.
2. New `src/tests/bench/duty_sweep.py` (closest precedent:
   `velocity_step_response.py`): sweep ~0.10–0.60 duty both directions,
   dwell to steady state, fit `speed = m*duty + b` per wheel; new
   `duty_per_speed` default is `1/m`. Report `b` (deadband intercept) and
   the L/R spread as a measurement.
3. Reconcile `vel_kff` to the measured gain, or record in the file why it
   is deliberately below it (the 106-002 resonance-detuning history is
   preserved, not lost).
4. Fix the residual bench-tooling reads
   (`bench-duty-readers-see-zero-after-stageduty-park`): `hil_drive.py
   --duty` and `square_tour_sim.py`'s `plannerDuty()` read see zero after
   128-015 parked `Planner::stageDuty()` from the live tick. Call
   `stageDuty()` explicitly before reading, or print a "duty stage parked
   (128-015)" warning — implementer's choice between the issue's own
   listed options.

## Acceptance Criteria

- [ ] `duty_sweep.py` reports `m`, `b`, and the L/R spread as a
      measurement, not an inference.
- [ ] The +500 button meets the full stakeholder-agreed acceptance spec:
      both wheels rise to cruise in ≤0.3 s; flat plateau at 150 mm/s;
      ripple ≤±10 mm/s frame to frame; |vL−vR| ≤10 mm/s through cruise;
      taper over the last 60 mm to a 90 mm/s floor (neither wheel reaches
      0 while the other still moves); elapsed ~4 s; encoders land
      500±15 mm, wheels within 10 mm of each other; net heading ≤3 deg,
      cross-track ≤30 mm; camera-measured travel 500±25 mm (encoders are
      not allowed to be right on their own).
- [ ] `hil_drive.py --duty`/`square_tour_sim.py` no longer silently
      report zero.
- [ ] The "Wheel speed — commanded vs actual" chart plots the commanded
      series too (currently missing — the one comparison that would
      settle the deficit at a glance).

## Testing

- **Existing tests to run**: firmware pytest tiers, `motion_tests`,
  `uv run python -m pytest`.
- **New tests to write**: `duty_sweep.py` itself (new bench tool, not a
  pytest unit test, but should be scriptable/repeatable); a regression
  test that `hil_drive.py --duty`/`square_tour_sim.py` no longer read
  zero.
- **Bench verification (required)**: full duty sweep on the stand per
  Acceptance Criteria; full +500 acceptance-spec run, camera-measured.
- **Verification command**: `uv run pytest`.

## Implementation Plan

- **Approach**: measure first, then reconcile — do not hand-derive a
  replacement constant. The identity-correction step must land before
  the sweep runs, or the measurement inherits the mismatch it's meant to
  replace (the file's own `_wheel_correction_note` says this explicitly).
- **Files to modify**: `data/robots/{tovez,togov,tovez_nocal}.json`
  (`wheel_gain_*`/`wheel_intercept_*` → identity, `duty_per_speed_*` →
  measured value, `vel_kff` reconciled or documented), new
  `src/tests/bench/duty_sweep.py`, `src/tests/bench/hil_drive.py`,
  `src/tests/bench/square_tour_sim.py`, the TestGUI chart panel for the
  commanded-vs-actual wheel-speed plot.
- **Documentation updates**: update `_wheel_correction_note` and
  `_drive_calibration_note` in each robot JSON with the new measurement
  provenance (date, method, result) — the same "preserve provenance"
  standard sprint 128 applied to deleted comment blocks.
