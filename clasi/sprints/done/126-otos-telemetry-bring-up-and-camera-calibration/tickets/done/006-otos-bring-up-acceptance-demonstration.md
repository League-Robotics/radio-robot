---
id: '006'
title: OTOS bring-up acceptance demonstration
status: done
use-cases:
- SUC-005
depends-on:
- '001'
- '002'
- '003'
- '004'
- '005'
github-issue: ''
issue: otos-telemetry-bring-up-and-camera-calibration.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# OTOS bring-up acceptance demonstration

## Description

Demonstrate all seven of the issue's acceptance criteria together, on the
playfield, against camera truth, with results printed and charted. This
is the sprint's closing ticket — it does not introduce new measurement
technique, it assembles and re-runs what tickets 001-004 already built,
plus a full-tour liveness check and the two checks the issue asks for that
aren't covered by any single earlier ticket (fusion weights unchanged,
sim suite at the known baseline).

Add `--mode acceptance` to `otos_calibration_bench.py`:

1. Preflight (lights, camera bring-up).
2. **Criterion 1 (presence)**: confirm `STATUS otos=1`, flag bit 0 set, on
   a `READY` robot with `connL=1`/`connR=1`.
3. **Criterion 2 (liveness)**: re-run ticket 001's units/liveness check,
   AND additionally run one longer multi-leg tour (reuse
   `square_tour.py`'s tour shape or a comparable multi-segment path) to
   confirm the OTOS pose survives a full tour without dropping out or
   diverging — capture camera fixes at every segment boundary per
   `.claude/rules/playfield-testing.md`'s mandatory per-boundary
   convention.
4. **Criterion 3 (lever arm/frame)**: re-run ticket 002's rotation check
   (or reuse its already-captured data if the sprint ran back to back)
   and restate the frame/lever-arm conclusion.
5. **Criterion 4 (distance calibration)**: restate ticket 003's fitted
   scale and residual, and the final `otos_linear_scale` value in
   `tovez.json` (post-correction if corrected).
6. **Criterion 5 (heading calibration)**: restate ticket 004's fitted
   scale and residual, and the final `otos_angular_scale` value.
7. **Criterion 6 (residuals stated)**: pull distance and heading residual
   mean/spread from steps 5-6 into one summary table — this is the
   number set a later fusion decision will weigh.
8. **Criterion 7 (sim suite + fusion untouched)**:
   - Run `uv run python -m pytest src/tests/sim -q`; confirm exactly the
     known, reduced pre-existing-failure set (10, `testgui`-only).
   - Grep `data/robots/tovez.json`'s `estimator.weight_heading_otos` /
     `weight_omega_otos` and confirm both are still `0.0`.
9. Produce one chart (matplotlib, matching the project's existing
   bench-script chart convention — see `square_tour.py`'s /
   `speed_map.py`'s PNG output pattern) showing: distance calibration fit
   (OTOS vs. camera, both directions), heading calibration fit (OTOS vs.
   camera, both directions), and the full-tour trajectory overlay
   (OTOS-reported path vs. camera-measured path).
10. Print a final consolidated PASS/FAIL report against all seven
    criteria, and save the chart alongside the script's existing PNG
    output convention (`src/tests/bench/`).
11. `estop()` (never `stop()`) in a `finally` block.

## Acceptance Criteria

- [ ] `otos_calibration_bench.py --mode acceptance` exists and runs all
      seven of the issue's acceptance criteria in one session, reusing
      tickets 001-004's helpers/modes rather than reimplementing them.
- [ ] A full multi-leg tour is run with per-segment-boundary camera fixes,
      demonstrating OTOS liveness survives beyond a single leg.
- [ ] A consolidated report (printed) and chart (saved PNG) cover all
      seven criteria, including the distance/heading residual summary
      table.
- [ ] Sim suite (`uv run python -m pytest src/tests/sim -q`) shows exactly
      the known, reduced pre-existing-failure set (10, `testgui`-only).
- [ ] `estimator.weight_heading_otos` / `weight_omega_otos` confirmed
      still `0.0` in `tovez.json`.
- [ ] Every connection path calls `estop()` (never `stop()`) in a
      `finally` block.
- [ ] Run against the real robot on the playfield; raw output and the
      chart included in the ticket's completion notes (send the chart per
      the project's own "always send the chart" convention).

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/sim -q`
  (full suite, final confirmation of the reduced known-failure baseline).
- **New tests to write**: None — hardware/playfield acceptance run.
- **Verification command**: `uv run python src/tests/bench/otos_calibration_bench.py --port <robot-port> --mode acceptance`
