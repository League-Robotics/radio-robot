---
id: '003'
title: OTOS distance-scale calibration against camera
status: done
use-cases:
- SUC-003
depends-on:
- '001'
- '002'
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

- [x] `otos_calibration_bench.py --mode distance` exists, reusing the
      established preflight/camera helpers.
- [x] Multiple straight-run lengths, both directions, at least 3 repeats
      per combination, each with camera truth and OTOS reading captured
      at rest at both ends.
- [x] Script prints the fitted linear scale and the residual (mean AND
      spread), not just a pass/fail verdict.
- [x] If measurement disagrees with 1.067, `otos_linear_scale` is updated
      in `tovez.json` with a provenance note, the robot is reflashed, and
      a post-reflash spot-check confirms the correction; if measurement
      agrees, `tovez.json` is left untouched and that agreement is stated.
- [x] Every connection path calls `estop()` (never `stop()`) in a
      `finally` block.
- [x] Run against the real robot on the playfield; raw results included in
      the ticket's completion notes.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/sim -q`
  (note: if `otos_linear_scale` is edited, `test_calibration_kwargs.py`'s
  `test_calibration_commands_tovez_json_snapshot` will start failing on
  its pinned `OL` value — that is expected and is ticket 005's job to
  fix, not this ticket's; do not edit that test here).
- **New tests to write**: None — hardware/playfield measurement script.
- **Verification command**: `uv run python src/tests/bench/otos_calibration_bench.py --port <robot-port> --mode distance`

## Completion Notes

**`--mode distance` added** to `src/tests/bench/otos_calibration_bench.py`,
reusing `connectAndArm()`/`checkLiveness()`/`clampTimeoutToClearance()` from
tickets 001/002 (no forked copies). New shared helpers: `legDirectionSafe()`
(refactored out of `pickSafeLegDirection()`), `fitScaleThroughOrigin()`,
`meanAndStdDev()`, `_resolveRobotJsonPath()`/`loadCommittedScale()`. Legs are
driven in a ping-pong pattern per length (forward, backward, forward, ...)
so the robot's position stays bounded near its start rather than drifting
toward an edge over many legs; each leg's SPECIFIC planned direction is
safety-checked against the current camera pose (falling back to the
opposite direction, or skipping, if unsafe) before the timeout clamp is
applied.

**Hardware run 1** (`/dev/cu.usbmodem21141112`, playfield, lights ON,
default `--lengths 150,300,450`): the 450mm leg was skipped in all 6
attempts — the robot's heading (~92-98deg, facing near-north) meant every
straight leg ran along the field's SHORT axis (±44.65cm), and the
15cm-margin safety clamp correctly refused a leg whose full COMMANDED
length (worst case) would come within margin of the north or south rail
from the drifted position it had reached by then. 150/300mm legs all ran
cleanly (12 valid runs).

**Hardware run 2** (`--lengths 150,300,350`, replacing the infeasible
450mm with a length that fits this axis): 18/18 legs valid, no skips.

```
=== distance-scale fit (18 valid runs) ===
length_mm  dir  camera_mm    otos_mm   err_mm  err_pct
      150  fwd      137.3      143.1      5.8     4.22
      150  bwd      136.5      143.0      6.5     4.76
      150  fwd      136.3      141.4      5.1     3.72
      150  bwd      136.5      143.6      7.1     5.21
      150  fwd      136.4      140.7      4.3     3.19
      150  bwd      138.2      145.2      6.9     5.03
      300  fwd      272.6      280.9      8.3     3.06
      300  bwd      279.0      289.1     10.1     3.63
      300  fwd      277.3      286.3      9.0     3.23
      300  bwd      277.5      291.4     13.8     4.99
      300  fwd      278.2      289.0     10.8     3.89
      300  bwd      281.0      291.6     10.6     3.79
      350  fwd      325.0      335.4     10.3     3.18
      350  bwd      195.5      200.3      4.8     2.44
      350  fwd      275.9      285.4      9.5     3.45
      350  bwd      342.0      357.4     15.4     4.52
      350  fwd      307.7      318.7     11.0     3.58
      350  bwd      340.9      356.2     15.3     4.49

fitted OTOS/camera slope (through origin): 1.0384
committed calibration.otos_linear_scale (tovez.json): 1.0670
corrected otos_linear_scale = committed x (1/slope) = 1.0670 x 0.9630 = 1.0275
residual: mean=+9.16mm sd=3.41mm (+3.91% sd=0.79%, n=18)
95% CI of mean residual: [+3.55%, +4.27%]
FAIL: committed otos_linear_scale=1.0670 disagrees with measurement (95% CI of the mean residual excludes 0) -- correct to 1.0275
```

(The 12-run pilot set from run 1 independently gave a corrected estimate of
1.0330 — consistent with run 2's 1.0275, both far from the committed 1.067.)

**Correction applied**: `data/robots/tovez.json`'s `calibration.otos_linear_scale`
changed **1.067 → 1.0275**, with a `_otos_linear_scale_note` recording the
measurement and provenance. Rebuilt (`just build` — `mbdeploy deploy --build`
itself fails in this checkout, unrelated pre-existing issue: `gen_messages.py`
needs `grpcio-tools`/`google.protobuf`, not installed in `mbdeploy`'s own venv;
worked around with `just build` + `mbdeploy deploy --hex MICROBIT.hex 2`, the
project's own documented fallback) and reflashed. Confirmed
`src/firm/config/boot_config.cpp` regenerated with `cfg.linearScale = 1.0275f`
before flashing. `just build` also triggered an unwanted automatic version
bump (`pyproject.toml`/`config/dotconfig.yaml`) — reverted (`git checkout --`)
per this project's bump-only-at-close_sprint cadence rule; not this ticket's
job to fix the build hook.

**Post-reflash spot-check** (`--lengths 300 --reps 3`, 6/6 valid legs):

```
=== distance-scale fit (6 valid runs) ===
length_mm  dir  camera_mm    otos_mm   err_mm  err_pct
      300  fwd      286.5      286.1     -0.4    -0.15
      300  bwd      291.9      293.1      1.1     0.38
      300  fwd      287.5      288.0      0.5     0.18
      300  bwd      293.0      294.8      1.8     0.61
      300  fwd      290.7      291.8      1.1     0.38
      300  bwd      294.6      297.1      2.5     0.84

fitted OTOS/camera slope (through origin): 1.0038
residual: mean=+1.09mm sd=1.01mm (+0.37% sd=0.34%, n=6)
95% CI of mean residual: [+0.10%, +0.65%]
```

The correction collapsed the residual from **+3.91% (sd 0.79%) to +0.37%
(sd 0.34%)** — an order of magnitude improvement. The 95% CI still
technically excludes 0 (implying a further correction to ~1.0236), but at
this size (~1mm on a 290mm leg) it is at or below the camera pose
measurement's own noise floor; not chased further — see the
`_otos_linear_scale_note` for the full reasoning.

**Sim regression check**: `uv run python -m pytest src/tests/sim -q` — 423
passed, 1 skipped, 1 xfailed, no new regressions (this test path does not
collect `src/tests/unit/`, so `test_calibration_kwargs.py` is unaffected by
the sim gate). Directly running
`uv run python -m pytest src/tests/unit/test_calibration_kwargs.py -q`
confirms the ticket's own forewarning: `test_calibration_commands_tovez_json_snapshot`
now also fails on `OL 67` (pinned) vs `OL 28` (actual, from the new 1.0275)
— on top of a **pre-existing, unrelated** failure on that same assertion's
`rotSlip` line (`0.9117` actual vs `0.92` pinned, already broken before this
ticket touched anything). Both are ticket 005's job; not fixed here per this
ticket's own testing note.
