---
id: '004'
title: OTOS heading/turn-scale calibration against camera
status: open
use-cases: [SUC-004]
depends-on: ['001', '002']
github-issue: ''
issue: otos-telemetry-bring-up-and-camera-calibration.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# OTOS heading/turn-scale calibration against camera

## Description

Measure whether `data/robots/tovez.json`'s `calibration.otos_angular_scale`
(committed 0.987, provenance unknown) is correct, against camera ground
truth, and correct it if not.

Add `--mode heading` to `otos_calibration_bench.py`:

1. Preflight (lights, camera bring-up), reusing the established helper.
2. Drive a set of in-place turns of **differing magnitudes** (e.g.
   15/45/90/135 deg — enough spread to catch a magnitude-dependent error,
   not just a single-point check) in **both directions**, **respecting
   the tether rule throughout**: every turn goes east→west through north,
   never through south, and successive turns **alternate direction** so
   net cable wrap stays zero across the whole sequence. Sequence the runs
   accordingly (do not simply repeat the same-direction turn back to
   back). At least 3 repeats per magnitude/direction combination.
3. At each turn's start and end (at rest, post-settle), capture camera
   truth (median-of-7, heading) AND read the OTOS heading.
4. For each turn, compute camera-measured heading change and
   OTOS-reported heading change (already scaled by the currently-committed
   0.987, applied on-chip at `begin()`).
5. Fit the linear relationship (OTOS heading-change vs. camera
   heading-change) across all turns; derive the corrected
   `otos_angular_scale` (current committed value × the fitted ratio) and
   the residual — mean and spread (std dev) of the per-turn error, in
   both raw degrees and as a percentage.
6. Print a clear report: per-turn table, fitted scale, residual
   mean/spread, and an explicit PASS/FAIL on whether 0.987 remains within
   the measured uncertainty or needs correcting.
7. **If the measured scale disagrees with 0.987** (outside the measured
   uncertainty): edit `data/robots/tovez.json`'s
   `calibration.otos_angular_scale` to the corrected value, note the
   change (old value, new value, measured residual) in an
   `_otos_angular_scale_note` companion key following this file's
   existing `_..._note` convention, and reflash the robot (`mbdeploy
   deploy --build`) — boot-baked, a runtime `SET` cannot apply it.
   Re-run a subset of the turns post-reflash to confirm the corrected
   value tracks camera truth.
8. `estop()` (never `stop()`) in a `finally` block.

## Acceptance Criteria

- [ ] `otos_calibration_bench.py --mode heading` exists, reusing the
      established preflight/camera helpers.
- [ ] Multiple turn magnitudes, both directions, tether rule respected
      (through-north only, alternating direction), at least 3 repeats per
      combination, each with camera truth and OTOS heading captured at
      rest at both ends.
- [ ] Script prints the fitted angular scale and the residual (mean AND
      spread), not just a pass/fail verdict.
- [ ] If measurement disagrees with 0.987, `otos_angular_scale` is updated
      in `tovez.json` with a provenance note, the robot is reflashed, and
      a post-reflash spot-check confirms the correction; if measurement
      agrees, `tovez.json` is left untouched and that agreement is stated.
- [ ] Every connection path calls `estop()` (never `stop()`) in a
      `finally` block.
- [ ] Run against the real robot on the playfield; raw results included in
      the ticket's completion notes.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/sim -q`
  (note: if `otos_angular_scale` is edited,
  `test_calibration_commands_tovez_json_snapshot`'s pinned `OA` value will
  start failing — expected, fixed in ticket 005, not here).
- **New tests to write**: None — hardware/playfield measurement script.
- **Verification command**: `uv run python src/tests/bench/otos_calibration_bench.py --port <robot-port> --mode heading`
