---
id: '003'
title: Bounded linear position-feedback trim in Pilot
status: done
use-cases:
- SUC-005
- SUC-007
depends-on:
- '002'
github-issue: ''
issue: motion-control-terminal-blips-reconciled-fix-plan.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bounded linear position-feedback trim in Pilot

## Description

Issue step 5. Adds `App::Pilot`'s own bounded linear position-feedback
trim, mirroring the existing heading PD's gain/arithmetic split exactly:
`v_cmd = twist.v + distance_kp * (twist.sRef - twist.sMeas)`, clamped to a
bounded ceiling. `Motion::Executor::Twist` gains `sRef`/`sMeas` (the linear
dominant-channel's own since-activation reference position and the
measured-path accumulator, 0/0 for kPivot/kTimed — a pure straight/arc-leg
mechanism, matching the issue's own "keep the existing heading PD as the
angular tracker" instruction: this trim does NOT touch the rotational
channel). Two new `PlannerConfig` fields: `distance_kp` [1/s] (field 39)
and `distance_tol` [mm] (field 40, repurposing the role
`kDistanceSettleEpsilonMm` currently plays as a hardcoded constant — this
ticket adds the field but does NOT yet wire it into the completion decision;
that rewire is ticket 004's scope). The gain/arithmetic split (Executor
exposes references, Pilot owns the gain and the correction arithmetic)
mirrors `motion/DESIGN.md` §2c's own documented, load-bearing boundary for
the heading PD — see sprint Architecture Design Rationale Decision 3. This
ticket claims no new harness `xfail` flip; it is a regression-guard plus the
087-009 non-regression verification.

## Acceptance Criteria

- [x] `Twist` gains `float sRef` [mm] and `float sMeas` [mm]:
      `sRef = plannedPositionSinceActivation` (kArc only, else 0),
      `sMeas = measuredPathSinceActivation_` (kArc only, else 0).
- [x] `App::Pilot::tick()` computes the trim and adds it to the velocity
      handed to `Drive::setTwist()`:
      `float trim = distanceKp_ * (twist.sRef - twist.sMeas); trim =
      clamp(trim, -trimCeiling_, trimCeiling_); float v = twist.v + trim;`
      — the trim is 0 whenever `sRef`/`sMeas` are both 0 (kPivot/kTimed), a
      harmless no-op requiring no mode branching in `Pilot`.
- [x] The trim's clamp ceiling is a named constant or config value, chosen
      and documented in this ticket's own completion notes, along with the
      empirically-verified `distance_kp * distance_tol >= v_deadband`
      inequality (sprint Architecture Design Rationale Decision 6 gives the
      known `v_deadband` range, ~15-19mm/s, from `MIN_SPEED_DEFAULT`'s own
      comment in `gen_boot_config.py`, as a starting point — re-verify
      against the actual current source, don't just cite the architecture
      doc's numbers unchecked).
- [x] `msg::PlannerConfig`/`PlannerConfigPatch` gain `distance_kp` (field
      39) and `distance_tol` (field 40) in `planner.proto`; regenerated via
      `scripts/gen_messages.py`; `gen_boot_config.py` bakes their defaults
      with the inequality derivation in a comment.
- [x] `App::Pilot::applyPlannerPatch()` — document whether `distance_kp` is
      added to `PlannerConfigPatch`'s curated live-tunable set (mirroring
      `heading_kp`) or left boot-config-only for this ticket; either choice
      is acceptable, but it must be a stated decision in the ticket's
      completion notes, not an oversight.
- [x] **Guardrail (SUC-007, 087-009 non-regression)**: the trim's clamp
      ceiling is verified (by a targeted test, or by inspection cited in
      the ticket) to sit below the authority that could reproduce a
      solve-side reversal — the trim only ever perturbs the SAMPLED
      velocity reference `Drive` receives; it never triggers, seeds, or
      retargets a `JerkTrajectory` solve. Confirm no `solveToRest`/
      `solveToState`/`solveToVelocity`/`retarget`/`reanchor` call is added
      or modified by this ticket.
- [x] **Guardrail (SUC-007)**: `git diff --stat` shows no changes to
      `src/firm/app/robot_loop.cpp` or `src/firm/motion/
      jerk_trajectory.{h,cpp}`.
- [x] No new harness `xfail` flip claimed; every currently-passing check
      (including ticket 001's two flips) stays passing.
- [x] `app/DESIGN.md`'s `Pilot` PD-cascade write-up gains a parallel entry
      describing the new linear trim alongside the existing heading PD
      description.
- [x] `uv run python -m pytest` is green end to end.

## Implementation Plan

- **Approach**: additive, mirrors the heading PD's existing shape in
  `pilot.cpp` exactly (gain read from config, arithmetic computed in
  `tick()`, added to the twist before `Drive::setTwist()`).
- **Files to modify**: `src/firm/motion/executor.h`/`.cpp` (`Twist`
  fields), `src/firm/app/pilot.h`/`.cpp` (gain member + arithmetic +
  clamp), `src/protos/planner.proto`, generated `src/firm/messages/`,
  `src/scripts/gen_boot_config.py`, `src/firm/app/DESIGN.md`.
- **Documentation updates**: as listed above; this ticket's own completion
  notes must record the chosen `distance_kp`/`distance_tol`/clamp-ceiling
  values and confirm the deadband inequality holds against them.

## Testing

- **Existing tests to run**: full `test_behavior_lock.py` harness, then
  the full `uv run python -m pytest`.
- **New tests to write**: a targeted clamp-authority check (unit or
  harness-based) verifying the trim's bounded magnitude near a command's
  own tolerance — the 087-009 guardrail verification.
- **Verification command**: `uv run python -m pytest`.

## Completion Notes

**Chosen values** (`src/scripts/gen_boot_config.py` `DISTANCE_KP_DEFAULT`/
`DISTANCE_TOL_DEFAULT`, `src/firm/app/pilot.h` `App::kDistanceTrimCeiling`):

- `distance_kp = 15.0` [1/s] (`msg::PlannerConfig` field 39)
- `distance_tol = 3.0` [mm] (field 40) — matches
  `Motion::kDistanceSettleEpsilonMm`'s own current value exactly (the
  constant this field is meant to eventually repurpose, ticket 004), so
  that future rewire is a pure substitution, not a silent behavior change.
- `App::kDistanceTrimCeiling = 50.0` [mm/s] — a fixed, Pilot-local C++
  constant declared at namespace scope in `pilot.h` (mirrors
  `Motion::kDeadTime`'s "other code/tests reference the same constant by
  name" shape). Deliberately NOT a wire field — only the gain is
  per-robot tunable (sprint 112 Architecture Design Rationale Decision 3
  says nothing about the ceiling being per-robot, and there is no
  precedent/need to make it one yet).

**Deadband inequality — re-verified against the ACTUAL current source,
not the architecture doc's own cited range unchecked** (AC #3): the
doc's Decision 6 cites "~15-19mm/s" from `MIN_SPEED_DEFAULT`'s own
comment. Re-derived directly from `Devices::NezhaMotor::writeShapedDuty()`
(`nezha_motor.cpp`): a commanded duty magnitude below `outputDeadband_` is
written as an outright 0, i.e. `v_deadband = outputDeadband / vel_kff`
(the open-loop `kff * velocityTarget_` term, `nezha_motor.cpp` line ~402).
`outputDeadband` defaults to `kDefaultOutputDeadband = 0.03`
(`nezha_motor.h`) for every current robot profile — `gen_boot_config.py`'s
own `defaultMotorConfigs()` leaves `output_deadband` unset on purpose, so
every boot config gets this same firmware ship default. Computed against
the robot JSONs actually on disk:

- the CURRENTLY-ACTIVE boot config (`data/robots/active_robot.json` ->
  `tovez_nocal.json`, `control.vel_kff = 0.002`): `0.03/0.002 = 15.0mm/s`
  — matches the architecture doc's own cited lower bound exactly, and
  independently matches `src/sim/sim_harness.h`'s own pre-existing
  comment ("the smallest wheel command that moves the plant is
  ~outputDeadband/kff ~= 15mm/s").
- the historically bench-tuned `tovez.json` profile's own `vel_kff`
  (`0.0008`, post sprint-106-002's kff detune — see that file's own
  `_vel_gains_note`: "vel_kff 0.00135->0.0008"): `0.03/0.0008 = 37.5mm/s`
  — MEANINGFULLY HIGHER than the architecture doc's cited range, because
  that range predates/does not reflect the 106-002 detune. This is the
  concrete, real finding this ticket's own "re-verify against the actual
  current source" instruction turned up — the doc's number was stale, not
  wrong in kind.

`distance_kp * distance_tol = 15.0 * 3.0 = 45.0mm/s`, which clears BOTH
figures with margin: 30.0mm/s over the currently-active boot config's
floor (15.0), 7.5mm/s over the higher, currently-tuned-robot floor (37.5).
45.0mm/s also sits below `App::kDistanceTrimCeiling` (50.0mm/s) — an
in-tolerance residual error is not yet clamped, matching the heading PD's
own unclamped-near-target shape — while the ceiling itself stays far below
anything that could look like a solve-side reversal (typical cruise
speeds run 100-300mm/s; `v_body_max` defaults to 1000mm/s; the existing
40mm gross-divergence reanchor threshold would demand 600mm/s unclamped,
capped by this ceiling to a 50mm/s nudge).

**`PlannerConfigPatch` decision** (AC #4): `distance_kp` IS added to
`PlannerConfigPatch`'s curated live-tunable set (`config.proto` field 21,
`App::Pilot::applyPlannerPatch()`'s merge arm) — mirrors `heading_kp`
exactly, for the same reason: a per-robot control-law gain worth bench-
iterating live, no reflash needed. `distance_tol` is deliberately left
boot-config-only for this ticket — it has no live consumer yet (ticket
004 wires it into the completion decision; that ticket can add a
`PlannerConfigPatch` arm for it then if warranted, mirroring
`heading_dwell_tol`'s own "not yet needed live" posture).

**087-009 guardrail verification**: `grep -n "solveToRest\|solveToState\|
solveToVelocity\|retarget\|reanchor" src/firm/app/pilot.cpp src/firm/app/
pilot.h` matches only this ticket's OWN doc comments citing those names
(pilot.cpp's trim comment, pilot.h's `kDistanceTrimCeiling` comment) — no
actual call site. `pilot.cpp`'s only `Motion::Executor` calls are
`executor_.tick()` and `executor_.state()` (both pre-existing, unchanged
by this ticket) plus the pre-existing `enqueue()`/`flush()`/`plan()`/
`popEvent()` pass-throughs (pilot.h) — no `JerkTrajectory` method is ever
called from `App::`. The trim reads `Twist::sRef`/`sMeas` (populated purely from
values `Motion::Executor::tick()` already samples) and adds a clamped
correction to the LOCAL `v` handed to `Drive::setTwist()` — it never calls
back into `Motion::Executor`/`JerkTrajectory`. Verified further by
a new targeted test, `src/tests/sim/system/pilot_distance_trim_harness.cpp`
+ `test_pilot_distance_trim.py`: drives a real `App::Pilot`/
`Motion::Executor`/`App::Drive` graph (`TestSim::SimHarness`) through a
plain straight `DISTANCE` command at cruise, freezes both wheel encoders
(`SimPlant::freezePosition()`, well under the pre-existing 40mm
gross-divergence reanchor threshold so that unrelated mechanism never
fires) to force a real, growing `sRef`-`sMeas` divergence, and asserts (a)
the commanded/planned deviation never exceeds `kDistanceTrimCeiling`, (b)
the trim genuinely engages (not a vacuous pass), and (c) the PLANNED
reference itself never moves during the freeze — proving the divergence
never fed back into a re-solve. A second scenario proves `distance_kp = 0`
(the production sim-harness default every OTHER pre-existing scenario
runs against, via the new `TestSim::SimHarness::setDistanceKp()` test-only
hook) makes the trim a true no-op under the identical divergence, so this
ticket's addition perturbs no pre-existing, un-opted-in sim scenario.

`git diff --stat` confirms zero changes to `src/firm/app/robot_loop.cpp`
and `src/firm/motion/jerk_trajectory.{h,cpp}`.

**Files touched beyond the ticket's own "Files to modify" list**:
`src/protos/config.proto` (the `PlannerConfigPatch` decision above lives
here, not `planner.proto`) and `src/sim/sim_harness.h` (a narrow,
precedented test-only hook, `setDistanceKp()`, mirroring
`setYawRateMax()`/`setLeadCompensation()`'s existing shape exactly — the
new AC-required targeted test needs a way to opt a `distance_kp` value
into the harness's `PlannerConfig` without perturbing any pre-existing
scenario, and this project's own precedent for that is a narrow sim-only
hook, not a wire-level change).

**Full suite**: `uv run python -m pytest` — 1231 passed, 12 xfailed, 2
xpassed, 0 failed (baseline 1230/12/2/0 — net +1 passed, this ticket's own
new test; zero new `xfail` flips, zero regressions).
