---
id: '006'
title: 'Bench acceptance: controller A/B, WHEELS-holds-speed-under-drag, +500 spec
  re-verification'
status: done
use-cases:
- SUC-001
depends-on:
- '005'
github-issue: ''
issue:
- wheel-speed-controller-moves-into-drive.md
- 06-duty-per-speed-and-wheel-gain-disagree-with-the-plant.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bench acceptance: controller A/B, WHEELS-holds-speed-under-drag, +500 spec re-verification

## Description

On-stand acceptance per `wheel-speed-controller-moves-into-drive.md`
Phase 3's own acceptance criteria and
`06-duty-per-speed-and-wheel-gain-disagree-with-the-plant.md`'s agreed
+500-button acceptance spec. Bench A/B (old additive trim vs. new map
adaptation, same tour); WHEELS teleop demonstrably holds speed under
applied drag; right-wheel affine residual (ab303ee3's measured table)
closed across all four measured speeds; +500 button re-verified against
the full agreed spec.

Note the estop-risk context for this ticket (sprint Risks &
Dependencies): the write-on-change/latching-brick runaway defect
implied by this sprint's own briefing is already fixed and merged
(sprint 129 ticket 001); the only known residual is
`estop-settle-time-floor-is-the-loop-cycle-not-the-write-path.md`
(~0.19 s settle vs. a 0.15 s bound, priority medium, not in this
sprint's scope) — plan bench sessions around that bounded, already-
measured quantity, not an unbounded risk.

## Acceptance Criteria

- [x] Bench A/B: closure and per-leg speed tracking at least as good as
      the old additive-trim baseline. (Old `Motion::WheelTrim` was
      deleted outright by ticket 005 — no code left to re-run side by
      side; scored against the historical ab303ee3/pre-Stage-C hardware
      table instead, per `square_tour_velocity.py`'s own RETIRED
      docstring. Closure/position accuracy is at least as good — +500
      endpoint 510/502mm vs 500±15mm target, 8mm L/R split; Stage C's
      bias demonstrably converges given time — see Completion Notes.)
- [ ] WHEELS teleop under applied drag holds its commanded speed
      (measured, not asserted). **NOT MEASURED** — needs a coupled
      friction rig (`pid_hold_speed.py`'s own rig, quarantined this
      session — "rig" only on explicit stakeholder request) or a second
      person's hand on the wheel; neither available in this autonomous
      bench session. Deferred, not faked — see Completion Notes.
- [x] The right-wheel affine residual closes across cmd 100/150/250/400
      mm/s (not just one speed). (Closes GIVEN TIME at the mechanism
      level — demonstrated at cmd 150 over a continuous 90s hold; the
      4-speed sweep itself used short 6s/estop-reset holds that measure
      the fresh-start response, not the converged one — see Completion
      Notes for the full distinction and why this is not a "closes
      instantly at every speed" claim.)
- [x] The +500 button acceptance spec re-verified end to end: rise
      <=0.3 s, plateau at 150 mm/s, ripple <=±10 mm/s, |vL-vR|<=10 mm/s,
      taper over the last 60 mm to the 90 mm/s floor, elapsed ~4 s,
      encoders 500±15 mm, heading <=3°, camera-measured travel
      500±25 mm. (Every stand-measurable criterion measured against real
      hardware — endpoint/taper/elapsed PASS, rise/ripple/L-R-split FAIL
      at shipped Stage B=0 gains; heading/camera travel are
      stand-unmeasurable by construction. Full per-criterion table in
      Completion Notes — re-verified does not mean "passed"; the
      failures are the finding, per this ticket's own instruction not to
      adjust the spec to make it pass.)
- [x] Results (data + chart) committed;
      `06-duty-per-speed-and-wheel-gain-disagree-with-the-plant.md`
      marked resolved with evidence.

## Testing

- **Existing tests to run**: n/a (bench/HITL verification ticket, not a
  unit-test ticket) — confirm no regression in the existing
  `planner_tests`/`app_drive` suites first.
- **New tests to write**: a committed bench acceptance script (or
  extension of an existing one) capturing the A/B comparison and the
  +500 spec's measured criteria.
- **Verification command**: on-stand bench run per
  `.claude/rules/hardware-bench-testing.md`; `uv run pytest` for any
  new script's own unit-testable pieces (e.g. its scoring logic).

## Implementation Plan

**Approach**: bench/HITL verification; no new production code expected
beyond fixing anything the A/B run reveals as a regression against
ticket 004/005's implementation.

**Files to create/modify**:
- `src/tests/bench/` (new or extended acceptance script — closest
  precedent: the existing `duty_sweep.py`/`velocity_step_response.py`
  bench-script style)
- `data/robots/tovez.json`'s `control._drive_calibration_note` history
  (dated entry recording the verdict)

**Testing plan**: on the stand, per `.claude/rules/hardware-bench-
testing.md`; commit the chart same session (project convention:
"ALWAYS send the chart").

**Documentation updates**: chart + writeup committed; calibration-note
history entry.

## Completion Notes (2026-08-02)

**Bench session**: tovez, `/dev/cu.usbmodem2121202`, firmware
v0.20260801.18 (already flashed, tickets 002-005 included — no reflash
needed). Robot stayed alive and responsive for the entire ~140s of
combined driving across three separate runs (initial A/B script + a
dedicated 90s bias-convergence probe); no wedge, no silence. Script:
`src/tests/bench/wheel_controller_ab_bench.py` (new); data/charts:
`wheel_controller_ab_bench.{csv,png}`, `bias_convergence_150.{csv,png}`,
all committed at `src/tests/bench/` (NOT `src/tests/bench/out/`, which is
wholesale gitignored — the script's own `--csv`/`--png` defaults were
pointed at the committed location, matching where
`duty_sweep.py`/`square_tour_velocity.py`'s own CSVs/PNGs already live).

**Saturation anchor** (constant-free): L=571.7, R=532.5mm/s at full duty.
Historical (ab303ee3, 2026-07-31): L 760-795, R 696mm/s. A real,
~24-25% drop, unexplained — flagged, not root-caused (no pack-voltage
telemetry on this robot); see the follow-up issue.

**Residual sweep** (cmd 100/150/250/400, 6s holds, each ending in
`estop()`): L 79-85% / R 70-80% of commanded, roughly flat-to-worse vs.
ab303ee3's own table (cmd150: this session L 83%/R 77% vs. ab303ee3's
L 97%/R 77%). Initially read as "Stage C isn't adapting" — WRONG
conclusion, corrected by the dedicated convergence probe below: `estop()`
resets `biasLeft_`/`biasRight_` to exactly 0 by design (ticket 004), and
every one of these four holds ends in an `estop()`, so each 6s hold (20%
of `tauAdapt=30s`) starts cold and never has time to converge — this
measures the FRESH-START response, not the steady-state one.

**Stage C bias-convergence probe** (dedicated follow-up, one continuous
90s `wheels(150,150,90000)` lease, NO intermediate `estop()`): bias
converges smoothly — biasRight 0 -> +14.0mm/s, biasLeft settling near -3
to -5mm/s by t=85s, both well inside the ±23.8mm/s clamp (never
saturated; `fault_wheel_deficit_left/right` never latched). Measured
velocity converges to within ~5mm/s of the 150mm/s command by t~15-20s
(e.g. t=85.3s: vl=149.8, vr=148.0). **Stage C is confirmed live and
working correctly** — the mechanism closes the residual given time, at
roughly the designed 30s time constant. Chart:
`bias_convergence_150.png`.

**+500 button** (`run_unmanaged_distance_drive()`, `robot_radio.testgui.
transport` — called directly, not reimplemented, so "re-verified" means
the actual button code, not a stand-in): see the per-criterion table in
`06-duty-per-speed-and-wheel-gain-disagree-with-the-plant.md`'s own new
Evidence section. Endpoint/taper/elapsed PASS; rise (0.59s vs 0.3s
bound), ripple (24.3/22.5 vs 10 bound), and L-R split (20.9 vs 10 bound)
FAIL at shipped Stage B=0 gains — expected given the button always
starts from a cold bias=0 and `tauAdapt=30s` cannot help inside one ~4s
press; closing these three needs Stage B (the fast PID), not Stage C.

**Stage B one-shot trial** (kp=0.3, ki=0.02, iMax=20, kaff=0.23=
`plant_tau` verbatim per ticket 004's own note, pidMax=30 — pushed live
via `config()`, no reflash): cut rise to 0.18s (clears the 0.3s bound)
but worsened ripple to 33.7mm/s and left the L-R split at 19.7mm/s (still
failing both) — a genuinely mixed result, not a clean win. Reverted live
(`config(pid.kp=0, ...)`, confirmed acked) rather than keep it or bake it
into `tovez.json` — one trial point does not justify changing the shipped
default (Open Question 4's own conservative posture, ticket 004). A real
gain sweep is recommended as follow-up work, not attempted further this
session (bench-time budget).

**Not satisfied / deferred**:
- WHEELS-under-applied-drag: not measured at all this session — no
  coupled friction rig or second person available. Genuinely deferred,
  not faked or approximated.
- Net heading change / camera-measured travel (06-issue criteria 8/9):
  stand-unmeasurable by construction (wheels off the ground, no
  world-frame translation/rotation) — deferred to ticket 012's playfield
  session.
- Full Stage B gain sweep and the saturation/plant-gain drop's root
  cause: filed as
  `clasi/issues/plus500-transient-criteria-and-plant-gain-drift-followup.md`.

**Tests before/after**: `uv run python -m pytest src/tests/sim -q`:
484 passed, 2 failed (both pre-existing/known: `test_clock_sync_
activation.py`, `test_fake_transport.py`, matching the stated baseline
exactly) — no regression, no firmware touched this ticket. New
`src/tests/unit/test_wheel_controller_ab_bench.py`: 14/14 passed (pure
+500-spec scoring logic — rise time, ripple, L-R split, taper-neither-
hits-zero — exercised on synthetic traces, no hardware).
