---
id: '004'
title: 'Wheel-speed controller algorithm: conversion map + fast PID + bias trim'
status: done
use-cases:
- SUC-001
depends-on:
- '001'
- '003'
github-issue: ''
issue:
- wheel-speed-controller-moves-into-drive.md
- 04-continuous-duty-per-speed-calibration.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Wheel-speed controller algorithm: conversion map + fast PID + bias trim

## Description

Implement the three-timescale controller in `App::Drive` per
`wheel-speed-controller-moves-into-drive.md` Phase 2 (Stage A
conversion map, Stage B fast PID, Stage C bias trim), consuming ticket
001's population defaults/bounds and ticket 003's extended interface.

**This ticket supersedes `04-continuous-duty-per-speed-calibration.md`'s
gain-scaling adaptation design** (sprint Architecture Design Rationale
Decision 3) — implement Stage C's bias/intercept adaptation instead,
not issue 04's `dutyPerSpeed`-scaling approach. Fold in issue 04's two
valuable provisions: its safety intent (achieved here via `bias`
clamped to `±biasMax`, `tauAdapt`-slow update, reset on `estop()` —
a stronger bumpless-by-construction property than 04's own bleed-the-
integral formula, since the SLOPE never adapts, only the additive
intercept) and note that its observability mandate is completed in
ticket 005 (telemetry/TestGUI), not here.

Resolve, as implementation-time decisions documented in this ticket's
own notes:
- **Open Question 1**: `bias` per-wheel vs. per-wheel-per-direction.
- **Open Question 2**: speed-floor policy — round a sub-`vMin` command
  up to `vMin`, or refuse it outright.
- **Open Question 4**: whether the fast PID needs a nonzero `kp` at all
  given the profiler's every-tick re-plan-from-measured-position, or
  whether that already supplies equivalent correction at the planning
  layer.

Apply ticket 001's power-ceiling verdict: if a hard simultaneous-wheel
current-limit ceiling was confirmed (not a session/battery artifact),
`biasMax`/`pidMax` must be derated so the controller never advertises
recovery authority the power budget cannot actually deliver (sprint
Architecture Design Rationale Decision 5).

## Acceptance Criteria

- [x] Stage A: `v_corrected = gain[dir]*v_cmd + intercept[dir] + bias`,
      `dutyFF = dutyPerSpeed * v_corrected`, `gain`/`intercept` sourced
      from ticket 001's population defaults with per-robot override.
- [x] Stage B: fast PID (`p + i + ff`, clamped `±pidMax`) with the
      integrator frozen outside steady state (`|a_cmd| < aSteady`) and
      reset on `estop()`.
- [x] Stage C: `bias` adapts only when steady (`|a_cmd| < aSteady`,
      `|v_cmd| >= vMin`, fresh sample), clamped to `±biasMax` — derated
      per ticket 001's power-ceiling verdict if a hard ceiling was found.
- [x] Speed-floor and deficit-flag policy implemented and documented
      (Open Question 2 resolved with stated rationale).
- [x] Unit tests: convergence under a known plant-gain error; the
      `biasMax` clamp holds against a divergent error; duty is
      continuous (bumpless) across a `bias` update.
- [x] `04-continuous-duty-per-speed-calibration.md` explicitly noted as
      superseded by this ticket's Stage C in the ticket's own notes and
      commit message — no parallel gain-scaling implementation exists.

## Completion Notes (2026-08-01/02)

**Algorithm, per stage** (`src/firm/app/drive.{h,cpp}`):
- **Stage A**: the existing `correctedCommand()` (per-wheel/direction
  affine inversion) now takes a `bias` parameter and adds it to the
  inverted map's result INSIDE the function, after the `desired == 0.0f`
  "stop is stop" guard — so a commanded-zero wheel produces exactly
  zero corrected command regardless of the adapted bias (a nonzero bias
  added at the call site would have crept a stopped wheel).
- **Stage B**: `Drive::fastPid()` — `p + i + kaff*a_cmd`, clamped
  `±pidMax`, anti-windup (no further integration while pushing into an
  already-pinned clamp), integrator frozen (not reset) outside `steady`.
  Reset alongside Stage C on `estop()`.
- **Stage C**: `Drive::adaptBias()` — `bias += err*dt/tauAdapt` when
  steady, at/above `vMin`, and on a fresh/connected/non-frozen
  measurement (`Health::wheelFrozenLeft/Right`, per robot_state.h's own
  anticipated-consumer comment); clamped to `±biasMax`; `tauAdapt<=0`
  disables outright. Reset to 0 on `estop()`.
- **Stage D**: unchanged (crawl shaper / quiet-at-zero / stop
  re-assertion).
- **Speed floor** (Open Question 2): a nonzero sub-`vMin` command is
  rounded UP to `vMin` (`applySpeedFloor()`), matching the project's own
  established boost-to-breakaway fix (sprint 114) rather than refusing
  the command outright, which would reproduce that same bug class one
  layer up. `vMin<=0` is a no-op.
- **Deficit-flag policy**: `updateDeficit()` latches when `|err| >
  deficitThreshold`, sustained for `deficitWindow` ms, while BOTH `bias`
  and the fast PID sit pinned at their configured authority
  (`biasLeft()/Right()`, `pidLeft()/Right()`, `deficitLeft()/Right()`
  accessors — ticket 005 wires these into telemetry/TestGUI).

**WheelTrim vs. new controller**: `Motion::WheelTrim`
(`src/motion/planner/wheel_trim.h`) is left untouched and still linked
(ticket 005 deletes it once the planner sheds its own call site) —
Stage A/B/C above is new code inside `App::Drive`, not a port of
`WheelTrim`'s class, though Stage B's anti-windup shape and Stage C's
steady-gate mirror `WheelTrim::compute()`'s own pattern deliberately (a
proven, already-reviewed idiom) rather than reinventing one.

**Open Question 1 (bias granularity)**: resolved as per-wheel, NOT
per-wheel-per-direction — the physical droop this trim corrects is a
property of the wheel's instantaneous load, not which side of a ramp it
arrived from; splitting by accel/decel would fragment one physical
quantity into two separately-converging estimates for no modeled
benefit, and complicates bumpless transfer for no reason.

**Open Question 2 (speed floor)**: resolved as round-up-to-`vMin`, not
refuse — see Stage A above.

**Open Question 4 (does Stage B need a nonzero kp)**: NOT resolved by
bench measurement this ticket — `tovez` is hard-silent
(`clasi/issues/tovez-hard-silent-i2c-wedge-blocks-completing-the-population-duty-sweep.md`)
and untunable this session. Mechanism is fully implemented and unit
tested; every robot JSON ships Stage B (`wheel_pid_*`) and the
deficit-flag policy (`wheel_deficit_*`) at 0 (inert, fail-closed),
pending ticket 006's bench A/B on repaired hardware.

**Constants baked** (`data/robots/tovez.json`'s new
`control.wheel_*`/`_wheel_controller_note`): Stage C SHIPS LIVE using
ticket 001's own population-measured `wheel_v_min=99.7`/
`wheel_bias_max=23.8` (explicitly flagged LOW CONFIDENCE by ticket 001
itself — n=3, one incomplete line after a mid-session hardware fault).
`wheel_tau_adapt=30.0s`/`wheel_a_steady=30.0mm/s^2` are design-choice
placeholders (tens-of-seconds adaptation per the issue text; a
steady-gate conservatively below `planner.a_max=300`), bench-verifiable
in ticket 006. Stage B/deficit (`wheel_pid_*`/`wheel_deficit_*`) ship at
0 on every robot (see Open Question 4 above). `togov.json`/
`tovez_nocal.json` ship ALL new keys at 0 (mecanum has no
characterization; nocal is the explicit no-correction profile) — see
each file's own `_wheel_controller_note`.

**Slope-vs-intercept tension**: ticket 001's parallel-lines test came
back slope-dominated (fanned), the OPPOSITE of the intercept-only
hypothesis this Stage C design assumes — carried forward EXPLICITLY,
unresolved, in `tovez.json`'s own `_wheel_controller_note` and NOT
treated as settling the design question (n=3, one incomplete line, per
ticket 001's own explicit caveat). Intercept-only adaptation is
implemented exactly as the ticket specifies; if the fanned result holds
up with real population data, Stage C's design (not just its bounds)
needs revisiting — flagged for a future ticket/issue, not silently
switched to slope adaptation here.

**Circle-tour closure**: 22.3 mm (sim tier, `circle.tour`, PASS 4/4),
down from 58.2 mm after ticket 003 — expected and explained: Stage C's
bias trim is now live in sim too (via the shared `composeRobot()` config
path, ticket 002), actively correcting whatever per-wheel gain mismatch
`TestSim::SimPlant` models, which ticket 003's plumbing-only interface
change could not yet do anything about.

**Tests before/after**:
- `uv run python -m pytest src/tests/sim -q`: 484 passed, 2 failed
  (both pre-existing/known: `test_clock_sync_activation.py`,
  `test_fake_transport.py`, matching the stated baseline exactly), and 2
  xfailed + 1 xpassed (of 3 xfail-marked tests) — the one XPASS
  (`test_angle_stop_lands_close_to_target_with_tovez_nocal_calibration`,
  `strict=False`) is ticket 002's own known-flaky heading-hold/rotation-
  calibration boundary case, unrelated to this ticket's changes
  (confirmed: `tovez_nocal.json` ships every new `wheel_*` key at exactly
  0, so Stage A's numeric output is byte-identical to before this
  ticket for that profile).
- `ctest --test-dir src/motion/planner/build`: 10/10, unchanged.
- `just build-clean`: ARM build succeeds (FLASH 42.12%, RAM 98.33%,
  unchanged headroom).
- New `app_drive_harness.cpp` scenarios (`test_app_drive.py`): 6/6,
  including the 3 new Stage C scenarios (convergence, `biasMax` clamp
  both signs, bumpless transfer).
- `uv run python src/scripts/check_config_sync.py`: OK, no drift (new
  `control.wheel_*` pydantic fields all cleanly allowlisted).

**Not satisfied / deferred**: the actual bench A/B acceptance (Stage
B's live tuning, the deficit-flag's live telemetry wiring, and hardware
re-verification of Stage C's population bounds) all require physical
access to `tovez`, which is hard-silent this session — explicitly
ticket 006's job, not this one's, per the sprint's own ticket sequencing.

## Testing

- **Existing tests to run**: existing `app_drive` unit tests (must
  still pass with all-zero gains producing zero correction, matching
  `WheelTrim`'s original fail-closed guarantee).
- **New tests to write**: per-stage convergence/clamp/continuity unit
  tests (host build); a sim scenario with deliberately mismatched L/R
  plant gains converging to two different `bias` values and driving
  straight (the standing SIM-equals-bench case that has failed on the
  bench before).
- **Verification command**: `uv run pytest`

## Implementation Plan

**Approach**: implement Stage A/B/C as clearly separated, independently
testable methods/fields inside `Drive` — mitigating the god-component
concern flagged in the sprint's Architecture Design Rationale ("Drive's
growing size is deliberate... Stage A/B/C stay separately named,
separately unit-tested"). No new class introduced (per sprint 128's
own reasoning against a third wheel-control generation coexisting).

**Files to create/modify**:
- `src/firm/app/drive.{h,cpp}` (Stage A/B/C implementation)
- `data/robots/{tovez,togov,tovez_nocal}.json` (`biasMax`, `vMin`,
  `tauAdapt`, `aSteady`, `kp`, `ki`, `iMax`, `kaff`, `pidMax`,
  `deficitThreshold`, `deficitWindow` keys)
- `robot_config.schema.json`, `robot_config.py`, `gen_boot_config.py`
  (schema + codegen for the new keys)

**Testing plan**: per-stage unit tests (host build, no hardware
required); the mismatched-L/R-gain sim scenario above.

**Documentation updates**: `drive.h`'s header comment rewrite — deletes
"there is no controller here" and the now-false "closed-loop control
lives in Motion::Planner's own duty stage" line.
