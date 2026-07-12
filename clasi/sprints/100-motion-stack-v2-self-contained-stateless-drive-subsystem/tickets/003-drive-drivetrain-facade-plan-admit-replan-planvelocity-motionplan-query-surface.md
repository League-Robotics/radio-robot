---
id: '003'
title: 'Drive::Drivetrain facade: plan/admit/replan/planVelocity + MotionPlan query surface'
status: open
use-cases: [SUC-001, SUC-003, SUC-006]
depends-on: ['002']
github-issue: ''
issue: motion-stack-v2-a-self-contained-stateless-motion-control-subsystem.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Drive::Drivetrain facade: plan/admit/replan/planVelocity + MotionPlan query surface

## Description

Implement `Drive::Drivetrain` (`admit()`/`advance()`/`plan()`/`replan()`/
`planVelocity()`) and `Drive::MotionPlan`'s const query surface
(`duration()`/`kappa()`/`anchor()`/`goal()`/`exitSpeed()`/
`effectiveCeiling()`/`isPivot()`/`isVelocityMode()`/`referenceAt()`), per
the issue's two core header sketches (`drivetrain.h`/`motion_plan.h`),
transcribed verbatim. `MotionPlan::step()` itself is NOT implemented this
ticket — that is tickets 004/005's job (the tracker cascade and the
policy/terminal machine respectively); this ticket only produces the
immutable, closed-form-samplable plan.

Read `clasi/issues/
motion-stack-v2-a-self-contained-stateless-motion-control-subsystem.md`'s
two header sketches in full before starting — they are close to
copy-pasteable C++; deviate from them only where the `.cpp` body requires
implementation detail the header sketch does not specify.

## Acceptance Criteria

- [ ] `source/drive/drivetrain.h`/`.cpp` implements `Goal`/`PlanRequest`/
      `Verdict`/`PlanResult`/`ChainTail`/`Drivetrain` exactly per the
      issue's sketch: `Goal{arcLength, deltaHeading, exitSpeed}`; the 8
      `Verdict` enumerators (`OK`, `EXIT_UNREACHABLE`,
      `JOINT_STEP_TOO_LARGE`, `JOINT_SIGN_REVERSAL`,
      `PIVOT_NONZERO_EXIT`, `RADIUS_TOO_TIGHT`, `CEILING_INFEASIBLE`,
      `SOLVE_FAILED`).
- [ ] `admit(goal, tail)` checks exactly what the issue specifies: exit
      reachable within the arc length; joint wheel-speed step
      `v*|dKappa|*W/2` within cap; NO per-wheel sign reversal at nonzero
      joint speed; inner-wheel floor for arcs entered at speed (R >=
      ~100mm); pivot => exit must be 0.
- [ ] `plan(request)` solves ONE master jerk-limited profile (path length
      for arcs, heading for pivots) under the trim-headroom-folded
      ceiling `v_eff = min(vBodyMax, omegaMax/|k|, (vWheelMax -
      headroom)/(1+|k|W/2))`, `headroom = trimVMax + trimOmegaMax*W/2`.
      The world goal pose is composed and frozen into the plan at this
      point (never re-derived on replan — drift cannot compound).
- [ ] `replan(plan, measured, elapsed)` re-times the SAME anchored path
      from the measured state: project pose onto the arc via exact
      closed-form projection, re-solve the master profile from
      `(s_meas, v_meas)` to the SAME goal and `exitSpeed`. Never new
      geometry. Returns `verdict != OK` on solve failure (caller keeps
      the old plan — expected outcome for asks reachable only by
      reversing).
- [ ] `planVelocity(target, deadman, current)` produces a velocity-mode
      `MotionPlan` toward `(v, omega)` with a deadman duration, no pose
      goal, same `MotionPlan`/`step()` interface.
- [ ] `source/drive/motion_plan.h`/`.cpp` implements `MotionPlan`'s const
      query surface: `duration()`/`kappa()`/`anchor()`/`goal()`/
      `exitSpeed()`/`effectiveCeiling()`/`isPivot()`/`isVelocityMode()`/
      `referenceAt(elapsed)`. `referenceAt()` is pure and closed-form (no
      Ruckig re-solve per call — samples the already-solved master
      profile via arc composition).
- [ ] Property test: for any admitted plan, `max(|wheel(t)|)` over a
      dense `referenceAt(t)` sample never exceeds `vWheelMax - headroom`
      (the `v_eff` fold invariant — SUC-003's core acceptance criterion).
- [ ] Admission-verdict table test exercises every `Verdict` enumerator
      with a constructed input that produces it.
- [ ] A pivot `Goal` with nonzero `exitSpeed` is rejected
      (`PIVOT_NONZERO_EXIT`), never silently clamped to zero.
- [ ] `step()` is explicitly a stub/unimplemented placeholder this
      ticket (document the exact stub shape chosen — e.g. a `TODO(005)`
      body that asserts/returns a default `StepOutput` — in completion
      notes so ticket 005 knows exactly what it is replacing).
- [ ] `uv run python -m pytest` passes; C++ unit harnesses (admission
      verdict table, `v_eff` fold invariant) compile and run per the
      `jerk_trajectory_harness.cpp` pattern.

## Testing

- **Existing tests to run**: `uv run python -m pytest`; ticket 002's grep
  isolation test (must stay passing).
- **New tests to write**: admission-verdict-table harness; `v_eff`
  fold-invariant property test; a `plan()`/`replan()` round-trip test
  (replan to the same goal from a mid-path measured state produces a
  plan whose `referenceAt(0)` matches the measured seed within
  tolerance).
- **Verification command**: `uv run pytest`

## Implementation Plan

**Approach**: transcribe the issue's `drivetrain.h`/`motion_plan.h`
header sketches verbatim as the starting point, then implement the
`.cpp` bodies. Use `master_profile.h`'s `solveToExit` (ticket 002) as the
underlying solver for the master DOF (path length for arcs, heading for
pivots — never solve both channels independently, per the "one
trajectory per segment" rule).

**Files to create/modify**:
- `source/drive/drivetrain.h`, `source/drive/drivetrain.cpp`
- `source/drive/motion_plan.h`, `source/drive/motion_plan.cpp` (query
  surface only this ticket — `step()`'s real body lands in ticket 005)
- `tests/sim/unit/drive_admission_harness.cpp` +
  `test_drive_admission.py`
- `tests/sim/unit/drive_veff_invariant_harness.cpp` +
  `test_drive_veff_invariant.py`

**Testing plan**: unit harnesses per above; a property/fuzz test for the
`v_eff` invariant across a range of curvatures and limits.

**Documentation updates**: none.
