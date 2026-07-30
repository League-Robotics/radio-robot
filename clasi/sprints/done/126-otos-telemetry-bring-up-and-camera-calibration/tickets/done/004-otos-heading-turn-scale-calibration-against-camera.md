---
id: '004'
title: OTOS heading/turn-scale calibration against camera
status: done
use-cases:
- SUC-004
depends-on:
- '001'
- '002'
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

- [x] `otos_calibration_bench.py --mode heading` exists, reusing the
      established preflight/camera helpers.
- [x] Multiple turn magnitudes, both directions, tether rule respected
      (through-north only, alternating direction), at least 3 repeats per
      combination, each with camera truth and OTOS heading captured at
      rest at both ends.
- [x] Script prints the fitted angular scale and the residual (mean AND
      spread), not just a pass/fail verdict.
- [x] If measurement disagrees with 0.987, `otos_angular_scale` is updated
      in `tovez.json` with a provenance note, the robot is reflashed, and
      a post-reflash spot-check confirms the correction; if measurement
      agrees, `tovez.json` is left untouched and that agreement is stated.
- [x] Every connection path calls `estop()` (never `stop()`) in a
      `finally` block.
- [x] Run against the real robot on the playfield; raw results included in
      the ticket's completion notes.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/sim -q`
  (note: if `otos_angular_scale` is edited,
  `test_calibration_commands_tovez_json_snapshot`'s pinned `OA` value will
  start failing — expected, fixed in ticket 005, not here).
- **New tests to write**: None — hardware/playfield measurement script.
- **Verification command**: `uv run python src/tests/bench/otos_calibration_bench.py --port <robot-port> --mode heading`

## Completion Notes

**`--mode heading` added** to `src/tests/bench/otos_calibration_bench.py`,
reusing `connectAndArm()`/`checkLiveness()`/`clampTimeoutToClearance()` and
126-002's `sweepCrossesAngle()`/`closestApproachToSouthDeg()`. New
`isTurnSafe()` validates a SPECIFIC planned signed turn (needed here because,
unlike `pickSafeTurnDirection()`, this mode must cover BOTH directions across
the run, not just whichever direction is safe); falls back to the opposite
direction (with the fallback logged), or skips, if unsafe. Turns are
sequenced +magnitude, -magnitude, +magnitude, ... per magnitude bucket
(ping-pong, same principle as ticket 003's distance legs) so net rotation
stays bounded across the whole sequence — tracked and printed
(`net-rotation-so-far`) as a live check, not assumed.

**Hardware run** (`/dev/cu.usbmodem21141112`, playfield, lights ON, default
`--turn-angles 15,45,90,135 --reps 3`): 24/24 turns valid, no skips. Tether
safety held throughout — minimum south-clearance observed was 37.9deg (a
90deg turn), well above the 20deg floor; net rotation across the whole
24-turn sequence ended at **+11.1deg**, not a meaningful wrap.

```
=== heading-scale fit (24 valid turns) ===
 mag_deg  dir  camera_deg   otos_deg  err_deg  err_pct
      15  ccw       13.42      14.15     0.73     5.46
      15   cw      -12.60     -14.61    -2.01    15.99
      15  ccw       11.92      13.69     1.78    14.90
      15   cw      -14.99     -14.67     0.32    -2.13
      15  ccw       15.24      14.32    -0.92    -6.02
      15   cw      -11.78     -13.52    -1.74    14.79
      45  ccw       41.71      43.37     1.67     4.00
      45   cw      -41.36     -43.89    -2.53     6.11
      45  ccw       44.93      44.00    -0.92    -2.06
      45   cw      -44.44     -43.32     1.13    -2.53
      45  ccw       46.20      45.49    -0.70    -1.52
      45   cw      -43.94     -43.32     0.63    -1.43
      90  ccw       85.26      84.17    -1.09    -1.28
      90   cw      -84.41     -83.77     0.64    -0.76
      90  ccw       85.95      86.29     0.34     0.40
      90   cw      -83.09     -85.31    -2.22     2.67
      90  ccw       87.94      88.12     0.18     0.20
      90   cw      -86.94     -85.77     1.16    -1.34
     135   cw     -131.41    -131.49    -0.08     0.06
     135  ccw      131.37     128.97    -2.40    -1.83
     135   cw     -128.54    -128.92    -0.37     0.29
     135  ccw      129.52     128.92    -0.61    -0.47
     135   cw     -129.23    -127.88     1.34    -1.04
     135  ccw      130.36     131.84     1.48     1.13

fitted OTOS/camera slope (through origin): 0.9984
committed calibration.otos_angular_scale (tovez.json): 0.9870
corrected otos_angular_scale = committed x (1/slope) = 0.9870 x 1.0016 = 0.9886
residual: mean=-0.175deg sd=1.343deg (+1.82% sd=5.80%, n=24)
95% CI of mean residual: [-0.50%, +4.14%]
PASS: committed otos_angular_scale=0.9870 is within measured uncertainty (95% CI of the mean residual includes 0) -- no correction needed
```

At the 15deg magnitude the PERCENTAGE residuals look large (up to ~16%) but
this is small-denominator noise, not a magnitude-dependent scale error: the
absolute error stays small at every magnitude (max 2.53deg, at 45deg), and
the fitted slope through the origin (0.9984, dominated by the larger-angle
turns which carry the most weight in a through-origin least-squares fit) is
extremely close to 1. The 95% CI of the mean percentage residual
([-0.50%, +4.14%]) **includes 0** — the committed `otos_angular_scale=0.987`
is confirmed correct within measured uncertainty.

**No correction made**: `data/robots/tovez.json`'s `calibration.otos_angular_scale`
is **unchanged (0.987)** — measurement agrees with the committed value, per
this ticket's own instruction not to edit `tovez.json` when it does. No
reflash needed for this ticket (the firmware already carries the correct
value; ticket 003's reflash for `otos_linear_scale` already covered the one
correction this sprint needed).

**Sim regression check**: `uv run python -m pytest src/tests/sim -q` — 423
passed, 1 skipped, 1 xfailed, no new regressions (unchanged from ticket 003,
as expected since `tovez.json` was not touched by this ticket).
`DO NOT touch weight_heading_otos/weight_omega_otos` — confirmed untouched
(still 0.0 in `data/robots/tovez.json`'s `estimator` section).
