---
id: '003'
title: OTOS distance-scale calibration against camera
status: open
use-cases: [SUC-003]
depends-on: ['001', '002']
github-issue: ''
issue: otos-telemetry-bring-up-and-camera-calibration.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# OTOS distance-scale calibration against camera

## Description

Measure whether `data/robots/tovez.json`'s `calibration.otos_linear_scale`
(committed 1.067, provenance unknown) is correct, against camera ground
truth, and correct it if not.

Add `--mode distance` to `otos_calibration_bench.py`:

1. Preflight (lights, camera bring-up), reusing the established helper.
2. Drive a set of straight-line runs (straight moves are always
   tether-safe) of **differing lengths** (e.g. short/medium/long — pick a
   spread that stays inside the field's ±67.15/±44.65 cm limits) in
   **both directions** (forward and backward, or east-going and
   west-going — cover both signs of travel). At least 3 repeats per
   length/direction combination, to get a spread, not just a point.
3. At each run's start and end (at rest, post-settle), capture camera
   truth (median-of-7) AND read the OTOS pose.
4. For each run, compute camera-measured distance and OTOS-reported
   distance (already scaled by the currently-committed 1.067, since the
   chip's on-chip scale register was set from that value at `begin()`).
5. Fit the linear relationship (OTOS distance vs. camera distance) across
   all runs; derive the corrected `otos_linear_scale` (current committed
   value × the fitted ratio) and the residual — mean and spread (std dev)
   of the per-run error, in both raw mm and as a percentage.
6. Print a clear report: per-run table, fitted scale, residual mean/spread,
   and an explicit PASS/FAIL on whether 1.067 remains within the
   measured uncertainty or needs correcting.
7. **If the measured scale disagrees with 1.067** (outside the measured
   uncertainty): edit `data/robots/tovez.json`'s
   `calibration.otos_linear_scale` to the corrected value, note the change
   (old value, new value, measured residual) in a `_otos_linear_scale_note`
   companion key following this file's existing `_..._note` convention
   (e.g. `_rotational_slip_note`, `_rotation_calibration_note`), and
   reflash the robot (`mbdeploy deploy --build`) — this value is
   boot-baked, a runtime `SET` cannot apply it. Re-run a subset of the
   distance runs post-reflash to confirm the corrected value tracks
   camera truth.
8. `estop()` (never `stop()`) in a `finally` block.

## Acceptance Criteria

- [ ] `otos_calibration_bench.py --mode distance` exists, reusing the
      established preflight/camera helpers.
- [ ] Multiple straight-run lengths, both directions, at least 3 repeats
      per combination, each with camera truth and OTOS reading captured
      at rest at both ends.
- [ ] Script prints the fitted linear scale and the residual (mean AND
      spread), not just a pass/fail verdict.
- [ ] If measurement disagrees with 1.067, `otos_linear_scale` is updated
      in `tovez.json` with a provenance note, the robot is reflashed, and
      a post-reflash spot-check confirms the correction; if measurement
      agrees, `tovez.json` is left untouched and that agreement is stated.
- [ ] Every connection path calls `estop()` (never `stop()`) in a
      `finally` block.
- [ ] Run against the real robot on the playfield; raw results included in
      the ticket's completion notes.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/sim -q`
  (note: if `otos_linear_scale` is edited, `test_calibration_kwargs.py`'s
  `test_calibration_commands_tovez_json_snapshot` will start failing on
  its pinned `OL` value — that is expected and is ticket 005's job to
  fix, not this ticket's; do not edit that test here).
- **New tests to write**: None — hardware/playfield measurement script.
- **Verification command**: `uv run python src/tests/bench/otos_calibration_bench.py --port <robot-port> --mode distance`
