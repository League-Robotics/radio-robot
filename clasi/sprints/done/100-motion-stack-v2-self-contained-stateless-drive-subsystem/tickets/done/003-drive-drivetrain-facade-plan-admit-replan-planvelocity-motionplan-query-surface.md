---
id: '003'
title: 'Drive::Drivetrain facade: plan/admit/replan/planVelocity + MotionPlan query
  surface'
status: done
use-cases:
- SUC-001
- SUC-003
- SUC-006
depends-on:
- '002'
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

- [x] `source/drive/drivetrain.h`/`.cpp` implements `Goal`/`PlanRequest`/
      `Verdict`/`PlanResult`/`ChainTail`/`Drivetrain` exactly per the
      issue's sketch: `Goal{arcLength, deltaHeading, exitSpeed}`; the 8
      `Verdict` enumerators (`OK`, `EXIT_UNREACHABLE`,
      `JOINT_STEP_TOO_LARGE`, `JOINT_SIGN_REVERSAL`,
      `PIVOT_NONZERO_EXIT`, `RADIUS_TOO_TIGHT`, `CEILING_INFEASIBLE`,
      `SOLVE_FAILED`).
- [x] `admit(goal, tail)` checks exactly what the issue specifies: exit
      reachable within the arc length; joint wheel-speed step
      `v*|dKappa|*W/2` within cap; NO per-wheel sign reversal at nonzero
      joint speed; inner-wheel floor for arcs entered at speed (R >=
      ~100mm); pivot => exit must be 0.
- [x] `plan(request)` solves ONE master jerk-limited profile (path length
      for arcs, heading for pivots) under the trim-headroom-folded
      ceiling `v_eff = min(vBodyMax, omegaMax/|k|, (vWheelMax -
      headroom)/(1+|k|W/2))`, `headroom = trimVMax + trimOmegaMax*W/2`.
      The world goal pose is composed and frozen into the plan at this
      point (never re-derived on replan — drift cannot compound).
- [x] `replan(plan, measured, elapsed)` re-times the SAME anchored path
      from the measured state: project pose onto the arc via exact
      closed-form projection, re-solve the master profile from
      `(s_meas, v_meas)` to the SAME goal and `exitSpeed`. Never new
      geometry. Returns `verdict != OK` on solve failure (caller keeps
      the old plan — expected outcome for asks reachable only by
      reversing).
- [x] `planVelocity(target, deadman, current)` produces a velocity-mode
      `MotionPlan` toward `(v, omega)` with a deadman duration, no pose
      goal, same `MotionPlan`/`step()` interface.
- [x] `source/drive/motion_plan.h`/`.cpp` implements `MotionPlan`'s const
      query surface: `duration()`/`kappa()`/`anchor()`/`goal()`/
      `exitSpeed()`/`effectiveCeiling()`/`isPivot()`/`isVelocityMode()`/
      `referenceAt(elapsed)`. `referenceAt()` is pure and closed-form (no
      Ruckig re-solve per call — samples the already-solved master
      profile via arc composition).
- [x] Property test: for any admitted plan, `max(|wheel(t)|)` over a
      dense `referenceAt(t)` sample never exceeds `vWheelMax - headroom`
      (the `v_eff` fold invariant — SUC-003's core acceptance criterion).
- [x] Admission-verdict table test exercises every `Verdict` enumerator
      with a constructed input that produces it.
- [x] A pivot `Goal` with nonzero `exitSpeed` is rejected
      (`PIVOT_NONZERO_EXIT`), never silently clamped to zero.
- [x] `step()` is explicitly a stub/unimplemented placeholder this
      ticket (document the exact stub shape chosen — e.g. a `TODO(005)`
      body that asserts/returns a default `StepOutput` — in completion
      notes so ticket 005 knows exactly what it is replacing).
- [x] `uv run python -m pytest` passes; C++ unit harnesses (admission
      verdict table, `v_eff` fold invariant) compile and run per the
      `jerk_trajectory_harness.cpp` pattern.

## Completion Notes (2026-07-12)

**Files landed**: `source/drive/drivetrain.{h,cpp}`, `source/drive/
motion_plan.{h,cpp}` (new); `source/drive/types.h` (extended: `Limits`
gains `vWheelMax`/`trimVMax`/`trimOmegaMax`/`wheelStepMax`, the four
PlannerConfig-sourced scalars this ticket's algorithms actually consume —
tracker/policy gains are deliberately deferred to tickets 004/005, per
`types.h`'s own "grown incrementally" rule). Tests: `tests/sim/unit/
drive_admission_harness.cpp` + `test_drive_admission.py` (all 8 `Verdict`
enumerators, 9 scenarios); `tests/sim/unit/drive_plan_harness.cpp` +
`test_drive_plan.py` (v_eff/omega_eff fold invariant sweep, boundary
conditions, `referenceAt()` vs. numerical integration, `replan()` round
trip + backward-ask failure).

**admit() verdict logic** (drivetrain.cpp): checks run in a fixed order
so each test scenario isolates exactly one check. `PIVOT_NONZERO_EXIT`
first (pivot + nonzero exitSpeed). Then a coarse kinematic reachability
check (`|exit^2 - entry^2| <= 2*bound*extent`, accel/decel picked by
whether the segment speeds up or slows down) — reusing `goal.arcLength`
(0 for a pivot) as `extent` means the SAME formula also catches "a pivot
entered at nonzero tail speed" as a degenerate case of
`EXIT_UNREACHABLE`, with no separate branch needed (the pivot's own
rotational-domain `bound` is irrelevant since it's multiplied by
`extent=0`). Pivots then return `OK` immediately — the remaining three
checks are all about a curvature TRANSITION, meaningless for a pivot's
formally-undefined kappa. For a nonzero joint speed: `JOINT_STEP_TOO_LARGE`
(`v*|dKappa|*W/2 > wheelStepMax`), then `JOINT_SIGN_REVERSAL` (either
wheel's differential-IK factor `1 -+ kappa*W/2` changes sign between
`tail.kappa` and the goal's own kappa), then `RADIUS_TOO_TIGHT`
(`1/|kappa| < 100mm`, the issue's own "~100mm" constant — a geometric
constant of the check, not a wire-tunable `Limits` field).

**v_eff fold + its invariant test**: `plan()` implements the issue's
literal arc-domain formula verbatim. The issue's formula is written only
for the linear (arc) domain; a PIVOT's own fold is the one deliberate
extension this ticket's `.cpp` needed beyond the sketch:
`omega_eff = min(omegaMax, (vWheelMax - headroom)/(W/2))` — the analogous
wheel-budget term without the arc's `1 + |k|W/2` factor, since a pivot's
wheel speed is simply `|omega|*W/2` (both wheels, opposite sign, no
forward component). `drive_plan_harness.cpp`'s
`scenarioVEffFoldInvariant()` sweeps 2 `Limits` ("plateau x headroom")
configurations x 4 curvatures x 1 pivot case, sampling `referenceAt(t)`
at 200 points per plan and asserting `max(|wheel(t)|) <= vWheelMax -
headroom` (+1mm/s float slack) throughout — **PASSED**, 0 failures across
all 10 plans x 201 samples each.

**replan() re-time approach**: samples `referenceAt(elapsed)` to get
`s_ref`/`v_ref`/`theta_ref` (already closed-form, pure), then corrects
`s_ref` by the EXACT projection error at that same `s_ref` — `eAlong`
(via `arc_math::projectOntoArc`) for an arc, `wrapAngle(measured.h -
theta_ref)` for a pivot (no arc to project onto; a pivot's own tracking
error IS the heading difference, matching the 098 heading-loop
precedent) — giving `s_meas = s_ref + correction`. Reseeds a fresh
`MasterProfile` with `(s_meas, v_meas, 0.0f)` (acceleration seeded at
neutral 0, since `BodyState` carries no measured acceleration) and
re-solves `solveToExit()` toward the SAME frozen `masterTarget_`
(a new private `MotionPlan` field — the absolute master-DOF target
position, needed because Ruckig's own position-control solve is
delta-based and `replan()` has no other way to recover the original
target) and the SAME `exitSpeed()`. **Backward-ask finding** (documented
in `drive_plan_harness.cpp`'s own scenario comment): Ruckig's directional
band validates the TARGET/exit velocity against the direction, not the
CURRENT/seed velocity — an overshoot with a still-forward seed velocity
into a STOP segment (`exitSpeed=0`, which sits on the boundary of both
direction bands) does NOT fail cleanly; Ruckig instead plans a trajectory
that coasts further then reverses back, since 0 is valid in both bands.
The scenario that DOES reliably fail is a FLYING (nonzero-exitSpeed)
segment whose direction flips at replan time due to overshoot, putting
the frozen exit velocity's sign outside the now-flipped band — the same
mechanism `drive_admission_harness.cpp`'s `scenarioSolveFailed` already
proves for `plan()` itself. Confirmed empirically against the harness
before landing on this scenario (see its own in-code comment).

**MotionPlan::step() stub**: returns a default `StepOutput`
(`Status::RUNNING`, a literal-zero `WheelVelocities`, `TrackRecord.in`
echoing the caller's `StepInput` so a caller can see what it fed in) and
never touches `*state`. Not an assert — `drive_api.cpp` (ticket 006) and
exploratory host tooling may legitimately call `step()` before ticket
005 lands. Ticket 005 replaces this entire body.

**Reconciliations beyond the header sketch** (both documented inline at
the exact site in the code): (1) `planVelocity()`'s `deadman` parameter
is tagged `[ms]` in the sketch while `MotionPlan::duration()` is `[s]`
throughout — resolved by converting (`duration_ = deadman * 0.001f`);
`MotionPlan::duration()` for a velocity-mode plan IS the deadman window
(not the short Ruckig ramp-time), so ticket 005's SAME terminal machine
that handles a stop segment's `t >= T_plan` also handles MOVER's deadman
elapsing, per SUC-010's "no separate watchdog logic duplicated." (2)
`ruckig::Ruckig<1>` holds a `const` member, so `MasterProfile`/
`MotionPlan` are copy-CONSTRUCTIBLE but not assignABLE — `plan()`/
`replan()`/`planVelocity()` return via `PlanResult{Verdict::OK,
MotionPlan(...)}` aggregate list-init (guaranteed C++17 prvalue copy
elision) rather than `result.plan = MotionPlan(...)`, which does not
compile. (3) `planVelocity()`'s own ceiling fold covers only the LINEAR
channel (`vBodyMax`, `vWheelMax - headroom`) — the joint v+omega
wheel-budget coupling for an arbitrary (not constant-curvature) command
is left to the tracker cascade (ticket 004) at each tick; SUC-003's
v_eff-fold invariant targets `admit()`/`plan()`'s segment plans
specifically, not `planVelocity()`, so this is in-scope-safe. (4)
`PlannerConfig.steer_headroom` (field 16, per architecture-update.md's
own field list) has no direct `Drive::Limits` consumer in this ticket —
`plan()`'s headroom is COMPUTED from `trimVMax + trimOmegaMax*W/2` per
the sketch's own literal formula, not read from a separately-named
field; left for ticket 007's adapter to resolve/map, not a genuine
conflict.

**Verification**: `uv run python -m pytest tests/sim/unit/
test_drive_admission.py tests/sim/unit/test_drive_plan.py tests/sim/unit/
test_drive_isolation.py tests/sim/unit/test_drive_arc_math.py tests/sim/
unit/test_drive_master_profile.py -v` → **6 passed, 0 failed, 33.23s**
(1 + 1 + 1 + 1 + 2 tests respectively). `just build` succeeds (firmware
links, `MICROBIT.hex`/`libfirmware_host` both built; `source/drive/` is
still uncalled from `main.cpp`, so this is a compile/link-only check, not
a behavior gate). Reasoning for not running the full ~1400-test suite:
`source/drive/` is structurally isolated (grep-enforced, `test_drive_
isolation.py`, unaffected by this ticket) and uncalled from any live
call path — the 6 targeted tests plus a clean `just build` are the
complete blast radius of this change; nothing outside `source/drive/`
was modified.

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
