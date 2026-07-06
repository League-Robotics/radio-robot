---
id: '001'
title: 'Motion executor core: Planner ramp and stop-condition engine'
status: open
use-cases: [SUC-001, SUC-002, SUC-003]
depends-on: []
github-issue: ''
issue: firmware-closed-loop-motion-verbs.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Motion executor core: Planner ramp and stop-condition engine

## Description

Build the goal-closure engine that every motion verb (`S`/`T`/`D`/`R`/
`TURN`/`RT`/`G`) will stage a goal into, ported from `source_old/
superstructure/Planner.*` + `source_old/control/BodyVelocityController.*` +
`source_old/commands/MotionCommand.*` + `source_old/control/StopCondition.*`
onto the already-generated, currently-unused `msg::PlannerCommand`/
`PlannerState`/`PlannerConfig`/`StopCondition` types
(`source/messages/planner.h`, from `protos/planner.proto` — see
architecture-update.md Grounding fact 1).

This ticket lands **no wire verb** — `Subsystems::Planner` is built and
tested in isolation. Tickets 002-004 register the verbs that stage
`msg::PlannerCommand`s into it.

Two deliberate simplifications vs. `source_old` (architecture-update.md
Decision 3, Decision 4, and Grounding):
- The ramp (`Motion::VelocityRamp`) hands a ramped `(v, omega)` straight to
  `Subsystems::Drivetrain::setTwist()`. It does **not** call
  `BodyKinematics::inverse()`/`saturate()`/a motor setter itself —
  `Drivetrain` already owns kinematics and the ratio governor; duplicating
  either here would be the exact shotgun-surgery risk Decision 3 rejects.
- `Motion::evaluateStopCondition` implements only the `TIME`/`DISTANCE`/
  `HEADING`/`POSITION`/`ROTATION` kinds. `SENSOR`/`COLOR`/`LINE_ANY` are
  recognized but rejected (`ERR badarg`-shaped rejection, not a crash) —
  no line/color sensor `Hal` leaf exists yet (Decision 4). `source_old`'s
  D-mode-specific `SAFETY_MARGIN`/`ARRIVE` refinements (sprint-072-era
  runaway safety net / stall-forced-completion) are **not** ported this
  ticket (Open Question 1 — no acceptance-bar requirement drives them yet).

`Subsystems::Planner` also does **not** port `source_old/superstructure/
Superstructure.*`'s `requestGoal(GoalRequest)` indirection or
`source_old/commands/CommandQueue.h`'s VW-push conversion — `CommandQueue`
was deleted in sprint 079, and `msg::PlannerCommand`'s oneof already *is*
the "goal request" — a command handler just constructs one directly
(ticket 002+'s job).

## Acceptance Criteria

- [ ] `protos/planner.proto`'s `DriveMode` gains `TIMED = 2` (fills the
      reserved gap between `STREAMING=1`/`DISTANCE=3`); `source/messages/
      planner.h` regenerated via `scripts/gen_messages.py`. No other
      `PlannerCommand`/`PlannerState`/`PlannerConfig`/`StopCondition` field
      changes.
- [ ] New `source/motion/motion_baseline.h`: a plain POD struct (ported
      concept from `source_old/control/StopCondition.h`'s
      `MotionBaseline`) — `t0`, `enc0`, `encDiff0`, `heading0`, `pose0X`,
      `pose0Y`, `vSign`, `omegaSign`.
- [ ] New `source/motion/stop_condition.{h,cpp}`: `Motion::
      evaluateStopCondition(const msg::StopCondition&, const
      Motion::MotionBaseline&, ...)` ported from `source_old/control/
      StopCondition.cpp`, supporting `STOP_TIME`/`STOP_DISTANCE`/
      `STOP_HEADING`/`STOP_POSITION`/`STOP_ROTATION` only.
      `STOP_SENSOR`/`STOP_COLOR`/`STOP_LINE_ANY` return a distinct
      "unsupported" result the caller can turn into `ERR badarg` (ticket
      002+) rather than silently never firing.
- [ ] New `source/motion/velocity_ramp.{h,cpp}`: `Motion::VelocityRamp`
      ported from `source_old/control/BodyVelocityController.cpp` minus
      the kinematics/saturate/motor-output tail — `setTarget(v, omega)`,
      `advance(dt_s)` (trapezoid; S-curve when `jMax > 0`, matching the
      ported logic), `reset()`, `seedCurrent(v, omega)`, `atTarget()`,
      `currentV()`/`currentOmega()`.
- [ ] New `source/subsystems/planner.{h,cpp}`: `Subsystems::Planner` with:
  - `apply(const msg::PlannerCommand& cmd, uint32_t now)` — stages the
    goal (dispatch on `goal_kind`), capturing the reply sink/corr-id
    context needed for later `EVT` emission (mirrors `MotionCommand`'s
    `setReplySink()`).
  - `tick(uint32_t now, const msg::MotorState& leftObs, const
    msg::MotorState& rightObs, const msg::PoseEstimate& fusedPose)` —
    advances the owned `Motion::VelocityRamp`, evaluates the active
    command's stop conditions via `Motion::evaluateStopCondition`, and
    holds its output (a `msg::DrivetrainCommand{TWIST}` or a neutral
    command on completion) via `hasCommand()`/`takeCommand()` — same
    held/taken discipline as `Drivetrain`/`PoseEstimator`. No stored
    `Hal::Motor`/`Drivetrain`/`PoseEstimator` reference — arguments only.
  - `hasEvent()`/`takeEvent()` — a held/taken small POD describing a
    pending `EVT done <verb> reason=<token>` (or none), carrying the
    captured reply sink + corr id from `apply()` time.
  - `state() const -> msg::PlannerState`, `configure(const
    msg::PlannerConfig&)`.
  - `hasActiveCommand() const` — mirrors `MotionCommand::active()`.
- [ ] No existing file changes: `Drivetrain`, `PoseEstimator`, `Hardware`,
      `NezhaHardware`, `SimHardware`, the `DEV` family, `dev_loop.*`,
      `main.cpp` are all untouched by this ticket.
- [ ] Wire keys stay stable: this ticket introduces no wire verb at all,
      so there is nothing to keep stable yet — verified by grep showing no
      new `CommandDescriptor` registrations land in this ticket's diff.

## Implementation Plan

**Approach:** Port math and sequencing, not wire plumbing. Work
bottom-up: `MotionBaseline` (data only) -> `evaluateStopCondition` (pure
function, host-testable without any hardware) -> `VelocityRamp` (pure,
host-testable) -> `Planner` (owns one of each, host-testable against
hand-built `msg::MotorState`/`msg::PoseEstimate` fixtures).

**Files to create:**
- `source/motion/motion_baseline.h`
- `source/motion/stop_condition.h`, `source/motion/stop_condition.cpp`
- `source/motion/velocity_ramp.h`, `source/motion/velocity_ramp.cpp`
- `source/subsystems/planner.h`, `source/subsystems/planner.cpp`

**Files to modify:**
- `protos/planner.proto` (`DriveMode` gains `TIMED = 2`)
- `source/messages/planner.h` (regenerated, not hand-edited)
- `CMakeLists.txt` / host test build config, to add `source/motion/*` and
  `source/subsystems/planner.cpp` to the build (both ARM and `HOST_BUILD`
  targets)

**Testing plan:**
- Host-built (`HOST_BUILD`) unit tests for `Motion::VelocityRamp`
  (converges to target under accel/jerk limits; `atTarget()` reports
  correctly), `Motion::evaluateStopCondition` (each of the five
  implemented kinds fires at the right threshold; the three unimplemented
  kinds return "unsupported"), and `Subsystems::Planner` (apply/tick/
  state/configure round-trip for each `goal_kind`; `hasCommand()`/
  `takeCommand()` and `hasEvent()`/`takeEvent()` held/taken discipline).
- No sim/bench test yet — no verb reaches this code from the wire until
  ticket 002.

**Documentation updates:** None this ticket (no wire-visible change).
Ticket 002+ documents the verbs that use this engine.
