---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 089 Use Cases

Parent use cases are drawn from `docs/usecases.md` (the project's master use
case list) where an existing UC applies. Several SUCs below are internal
(developer/CI-facing, or build-system) with no user-visible behavior; those
are marked `Parent: N/A`.

## SUC-001: Vendored Ruckig compiles as part of the real firmware and host-sim builds

Parent: N/A — build-system/toolchain concern, not modeled in `docs/usecases.md`.

- **Actor**: Firmware/host-sim build process (`cmake`/`mbdeploy deploy --build`
  or `just build-clean`; `tests/_infra/sim/CMakeLists.txt`).
- **Preconditions**: `libraries/ruckig/` vendored (already done, this sprint's
  foundation); the repo-root `CMakeLists.txt`'s `-std=gnu++20` override and
  `tests/_infra/sim/CMakeLists.txt`'s `CMAKE_CXX_STANDARD 20` already in place
  (already done).
- **Main Flow**:
  1. The ARM firmware CMake build compiles the vendored Ruckig sources
     (`libraries/ruckig/src/*.cpp`) and links them into the firmware binary.
  2. The host-sim CMake build (`tests/_infra/sim/CMakeLists.txt`) compiles the
     same Ruckig sources into `firmware_host`.
  3. Both builds succeed under the project's exact flags
     (`-fno-exceptions -fno-rtti`, no heap via `Ruckig<N>`).
- **Postconditions**: `#include "ruckig/ruckig.hpp"` and `Ruckig<1>` are usable
  from `source/motion/` and `source/subsystems/planner.cpp` in both builds,
  with no standalone subprocess compile step required (unlike
  `test_ruckig_smoke.py`'s current harness, which remains as a lighter-weight
  regression check).
- **Acceptance Criteria**:
  - [ ] `just build-clean` (or the project's ARM build entry point) links a
        firmware image that references Ruckig symbols with no errors.
  - [ ] `tests/_infra/sim/CMakeLists.txt`'s `firmware_host` target links with
        no errors.
  - [ ] Flash/RAM footprint delta from vendoring Ruckig into the ARM build is
        measured and recorded (ticket completion notes).

## SUC-002: `D` (Drive Robot a Specific Distance) arrives at rest with no reverse

Parent: UC-003 (Drive Robot a Specific Distance)

- **Actor**: Python host / stakeholder on the bench
- **Preconditions**: Robot on the stand, wheels off the ground. Firmware
  built with Ruckig integrated per SUC-001.
- **Main Flow**:
  1. Host sends `D <l> <r> <mm>`.
  2. Planner builds a Ruckig `InputParameter` from the current velocity
     estimate (0 at command start) to `target_position = mm`,
     `target_velocity = 0`, `target_acceleration = 0`, solves once
     (`calculate()`), and holds the resulting `Trajectory`.
  3. Each tick, Planner samples `trajectory.at_time(elapsed)` for the
     commanded velocity and hands it to `Drivetrain::setTwist()` via the
     existing `msg::DrivetrainCommand{TWIST}` edge (unchanged wire shape).
  4. The commanded velocity accelerates, cruises (if the distance allows),
     and jerk-limited-decelerates to exactly 0 as the robot reaches `mm`.
  5. Firmware emits `EVT done D reason=dist ...` once the trajectory
     completes.
- **Postconditions**: The commanded velocity trace never goes negative at any
  point in the trajectory; the robot is at rest at completion; no reverse
  wheel motion is observed after `EVT done`.
- **Acceptance Criteria**:
  - [ ] Sim: a Planner-level test samples the held trajectory across its
        whole duration and asserts the commanded velocity is `>= 0` for a
        positive `D` (mirroring `test_ruckig_smoke.py`'s own no-reverse
        assertion, but against the Planner's real goal-staging path, not a
        hand-built `InputParameter`).
  - [ ] Bench: `D 200 200 1000` on the stand — no measured reverse encoder
        motion after `EVT done`, and peak commanded/measured wheel speed does
        not exceed the commanded 200 mm/s by more than the existing
        ratio-governor/PID tolerance.

## SUC-003: `T` (Timed Duration), bare `S` (Continuous Speed), and `R` (arc) arrive at rest with no reverse when they stop

Parent: UC-001 (Drive Robot at Continuous Speed), UC-002 (Drive Robot for
Timed Duration)

- **Actor**: Python host / stakeholder on the bench
- **Preconditions**: Robot on the stand, wheels off the ground. Firmware
  built with Ruckig integrated per SUC-001.
- **Main Flow**:
  1. Host sends `T <l> <r> <ms>` (or a bare `S`/`R` that later self-terminates
     via a `stop=` clause or a duration).
  2. Planner solves a Ruckig trajectory to the commanded cruise velocity
     (`ControlInterface::Velocity`, open-ended — no target position) and
     samples it each tick; past the ramp-up trajectory's own duration,
     sampling holds at the cruise velocity (Ruckig's own past-duration
     "keep constant velocity/acceleration" extrapolation), so the commanded
     twist sustains cruise with no additional Planner bookkeeping.
  3. When a stop condition fires (`STOP_TIME`, a user `stop=` clause, or — for
     `T` — `duration` elapsing), Planner solves ONE new Ruckig trajectory from
     the CURRENT sampled state to `target_velocity = 0`,
     `target_acceleration = 0` and switches to sampling that trajectory.
  4. The commanded velocity decelerates jerk-limited to exactly 0; `EVT done`
     fires once that second trajectory completes.
- **Postconditions**: The commanded velocity trace never goes negative,
  including across the cruise-to-decel handoff; the robot is at rest at
  completion.
- **Acceptance Criteria**:
  - [ ] Sim: a Planner-level test drives a `T` goal through cruise and the
        stop-triggered re-solve, sampling the full commanded velocity trace
        and asserting it is `>= 0` throughout, matching SUC-002's assertion
        style.
  - [ ] Bench: `T 200 200 1000` on the stand — no measured reverse encoder
        motion after `EVT done`.
  - [ ] `test_motion_overshoot_regression.py`'s existing `D`/`T` bars are not
        regressed (equal or tighter than before).

## SUC-004: The jerk config surface (`j_max`/`yaw_jerk_max`) shapes the Ruckig-backed trajectories, preserving the existing "0 = unlimited jerk" sentinel

Parent: N/A — configuration/tuning surface, not modeled in `docs/usecases.md`.

- **Actor**: Firmware config author (`SET`/boot config), Planner internals.
- **Preconditions**: `msg::PlannerConfig.j_max`/`yaw_jerk_max` already exist
  in the wire schema (no proto change needed) and today feed only
  `Motion::VelocityRamp`'s optional S-curve branch, defaulting to `0.0`
  (`main.cpp`'s `defaultPlannerConfig()`).
- **Main Flow**:
  1. Planner's Ruckig-backed linear/rotational channels read
     `a_max`/`a_decel`/`v_body_max`/`j_max` (linear) and
     `yaw_rate_max`/`yaw_acc_max`/`yaw_jerk_max` (rotational) from
     `PlannerConfig` on `configure()`.
  2. A channel's `j_max`/`yaw_jerk_max == 0` (today's "trapezoid, no S-curve"
     sentinel) maps to Ruckig's `max_jerk = +infinity` for that channel — NOT
     to a literal `max_jerk = 0` (which would forbid the acceleration from
     ever changing, i.e. no motion at all).
  3. A positive `j_max`/`yaw_jerk_max` maps straight through as Ruckig's
     `max_jerk`, genuinely jerk-limiting the profile.
- **Postconditions**: Existing configs (default `j_max = yaw_jerk_max = 0.0`)
  produce the same trapezoid shape as before (modulo the terminal
  rest-arrival fix); a future bench-tuned positive `j_max` produces a
  genuinely S-curve-shaped profile with no config/wire schema change needed.
- **Acceptance Criteria**:
  - [ ] Sim: a `Motion::` wrapper unit test asserts `j_max == 0` produces the
        same trajectory shape (within tolerance) as an explicit
        `max_jerk = infinity` Ruckig solve, and that `j_max > 0` produces a
        measurably different (S-curve) profile.
  - [ ] No `protos/*.proto` change; no `scripts/gen_messages.py` regen
        required.

## SUC-005: `TURN` (absolute heading) and `RT` (relative rotation) arrive at rest with no reverse, without regressing 086/087 turn accuracy

> **[Revision, post-stakeholder-review]** This SUC replaces the original
> SUC-005 ("`TURN`/`RT`/`G` are not regressed"), which assumed `TURN`/`RT`
> stayed on the pre-existing mechanism. The stakeholder expanded scope to
> migrate `TURN`/`RT` onto Ruckig this sprint (architecture-update.md
> Decision 5's revision); `G`'s "not regressed" use case moves to the new
> SUC-006 below, narrowed to `GOTO_GOAL` alone (the one goal kind still on
> the old mechanism).

Parent: N/A for `TURN`/`RT` — no dedicated master UC exists for either in
`docs/usecases.md`.

- **Actor**: Python host / stakeholder on the bench; sim test suite.
- **Preconditions**: Robot on the stand, wheels off the ground. Firmware
  built with Ruckig integrated per SUC-001. `DISTANCE`/`TIMED`/`VELOCITY`/
  `STREAM` already migrated per SUC-002/SUC-003 (shares the same `Motion::
  JerkTrajectory` mechanism).
- **Main Flow**:
  1. Host sends `TURN <heading>` or `RT <relAngle>`.
  2. The existing wire-layer handler (`handleTURN`/`handleRT`,
     `motion_commands.cpp`, UNCHANGED) resolves the target: `TURN` reads
     LIVE fused heading and computes a shortest-path signed delta; `RT`
     computes a relative angle directly, no pose needed.
  3. `Planner::apply()`'s `TURN`/`ROTATION` cases stage a position-control
     Ruckig solve-to-rest on the ROTATIONAL channel, reading the target
     from `cmd.stops_[0].a` (`TURN`) or the existing-but-previously-unused
     `cmd.goal.rotation.angle` (`RT`) — see architecture-update.md
     Decision 9. The per-command velocity ceiling is the historical fixed
     spin rate (`kTurnOmega`/`kRotationOmega`), not the global
     `yaw_rate_max`, preserving the exact cruise-rate characteristic
     086/087's accuracy tuning was measured against.
  4. `Motion::evaluateStopCondition()`'s `STOP_HEADING` (fused-heading-based)
     and `STOP_ROTATION` (encoder-arc-based) evaluation stay UNCHANGED as
     the authoritative completion signal — Ruckig only shapes the commanded
     rotational velocity in between (Decision 4/9).
  5. The commanded rotational velocity accelerates, (if the angle is large
     enough) cruises, and jerk-limited-decelerates to exactly 0 as the
     target angle is reached — no reverse spin past the target.
- **Postconditions**: The commanded rotational-velocity trace never goes
  negative (relative to the commanded turn direction) at any point in the
  trajectory; the robot is at rest at completion; no reverse wheel motion is
  observed after `EVT done`; 086/087's existing heading/rotation accuracy
  tolerance bars are not regressed.
- **Acceptance Criteria**:
  - [ ] Sim: a Planner-level test samples the held rotational-channel
        trajectory across its whole duration for `TURN` and `RT` scenarios
        and asserts the commanded rotational velocity never reverses sign
        relative to the commanded turn direction (mirroring SUC-002's
        assertion style).
  - [ ] Bench: `TURN <heading>` (a ~90° turn) and `RT <relAngle>` on the
        stand — no measured reverse encoder motion after `EVT done`.
  - [ ] Bench: `TURN`/`RT`'s heading/rotation accuracy is re-measured
        against the SAME numeric tolerance bars 086/087 established (see
        architecture-update.md Open Question 6 — ticket execution pulls the
        exact bar from 086/087's own artifacts and the existing sim test
        assertions, not re-derived here) and found not regressed.
  - [ ] `tests/sim/unit/test_motion_commands_arc_turn.py` and
        `tests/sim/system/test_tour_geometry.py`: no NEW failure and no
        NEW `xfail` is introduced. The two currently-documented RT `xfail`s
        are permitted, but not required, to flip to passing — un-`xfail`ing
        them is an accepted possible side effect of the migration
        (Decision 9), not a requirement of this criterion.

## SUC-006: `GOTO_GOAL` (`G`) is not regressed by the Ruckig migration

> Narrowed from the original SUC-005, which also covered `TURN`/`RT` — see
> SUC-005 above for their (now expanded) use case.

Parent: UC-015 (Drive to Relative XY Position).

- **Actor**: Python host / stakeholder on the bench; sim test suite.
- **Preconditions**: `DISTANCE`/`TIMED`/`VELOCITY`/`STREAM`/`TURN`/
  `ROTATION` goal kinds have all been migrated onto Ruckig per SUC-002/
  SUC-003/SUC-005; `GOTO_GOAL` is explicitly the ONE goal kind out of scope
  for migration this sprint (architecture-update.md Decision 5's revision).
- **Main Flow**:
  1. `Subsystems::Planner` continues to route the `GOTO_GOAL` goal kind
     through the existing `Motion::VelocityRamp` + `pursueSteer()`/
     `enterPursue()`'s `PRE_ROTATE`/`PURSUE` state machine, unchanged in
     code and behavior.
  2. `Subsystems::Planner` holds both mechanisms side by side (the retained
     `ramp_` for `GOTO_GOAL` alone, and the new Ruckig channels for every
     other goal kind), dispatching on `mode_ == GO_TO` vs. not — a clean
     two-way split (architecture-update.md Grounding/Decision 5).
- **Postconditions**: `G` sim test pass status is byte-for-byte unchanged
  from pre-sprint; no new xfail is introduced by this sprint's changes.
- **Acceptance Criteria**:
  - [ ] `G`'s existing sim test suite keeps its current pass status (no
        newly broken test from a Ruckig-side regression).
  - [ ] `G` bench behavior is spot-checked unchanged on the stand (not a
        full re-verification — a smoke check that it still settles).
