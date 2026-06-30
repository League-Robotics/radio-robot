---
id: '001'
title: Planner subsystem (MotionController2)
status: done
use-cases:
- SUC-001
- SUC-004
depends-on: []
github-issue: ''
issue: message-based-subsystem-architecture.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Planner subsystem (MotionController2)

## Description

Create `MotionController2` — the Planner subsystem — as a new class in
`source/superstructure/MotionController2.h/.cpp` that wraps the existing
`MotionController` behind the 4-verb message-contract API. This is the
additive approach: the existing `MotionController` logic is unchanged; the
new class delegates to it by reference.

The Planner's role (per the issue) is **goal closure only** — it generates a
time-varying twist setpoint from a goal + pose estimate and decides when the
goal is reached. It does NOT close velocity loops (those belong to Drive2 /
MotorController).

The key design constraint is the RETURN model: `tick(now)` returns a
`msg::CommandBatch` containing a `msg::DrivetrainCommand{twist}` setpoint per
active tick. This is what makes the planner-isolation test work without a mock
sink.

Also add `toPlannerConfig(RobotConfig)` projection function in
`source/superstructure/PlannerConfig.cpp`.

## Acceptance Criteria

- [x] `source/superstructure/MotionController2.h` declares the class with:
  - `apply(const msg::PlannerCommand&)` — stages the goal command
  - `tick(uint32_t now) -> msg::CommandBatch` — advances goal logic, returns batch with `DrivetrainCommand{twist}`
  - `const msg::PlannerState& state() const` — const-ref to planner's internal state
  - `void configure(const msg::PlannerConfig&)` — delta-apply motion params
  - Constructor takes `MotionController&`, `const subsystems::Drive2&`, `const RobotConfig&`
- [x] `apply()` maps `PlannerCommand::GoalKind` to the correct `MotionController::begin*()` call:
  - `TIMED` → `beginTimed()`
  - `TURN` → `beginTurn()`
  - `DISTANCE` → `beginDistance()`
  - `VELOCITY` → `beginVelocity()`
  - `GOTO_GOAL` → `beginGoTo()`
  - `ROTATION` → `beginRotation()`
  - `STOP` → `stop()`
- [x] `tick(now)` calls `MotionController::driveAdvance()` with internal copies of
  `HardwareState`, `MotorCommands`, and `TargetState`; then extracts the resulting
  commanded twist (from internal `DesiredState` / `BodyVelocityController` output)
  and packs it into a `msg::DrivetrainCommand{twist}` inside the returned `CommandBatch`.
- [x] `state()` populates `msg::PlannerState` from `TargetState` / `DesiredState`:
  - `mode` from `DriveMode`
  - `active` from `MotionController::hasActiveCommand()`
  - `body_twist` from the last commanded body twist
- [x] `configure(PlannerConfig)` stores the config; the internal `MotorController` /
  `BodyVelocityController` reads `aMax`, `vBodyMax`, `yawRateMax` via the `RobotConfig&`
  it already holds. (Note: the existing `MotionController` takes `const RobotConfig& cfg`
  — `configure()` on `MotionController2` updates a local copy or a mutable ref; the
  implementer should document the chosen approach in a comment.)
- [x] `toPlannerConfig(const RobotConfig& rc) -> msg::PlannerConfig` in
  `source/superstructure/PlannerConfig.cpp` maps:
  - `rc.aMax` → `PlannerConfig.a_max`
  - `rc.vBodyMax` → `PlannerConfig.v_body_max`
  - `rc.yawRateMax` → `PlannerConfig.yaw_rate_max`
  - `rc.arriveTolMm` → `PlannerConfig.arrive_tol_mm`
  - Plus any other motion-only fields in `PlannerConfig` (see `msg/planner.h`)
- [x] `MotionController2` compiles under `-std=c++11 -fno-rtti -fno-exceptions` (both
  host sim and device build via `python build.py --clean`).
- [x] No virtual dispatch in the class definition. No heap allocation inside `tick()`.
- [x] Existing `MotionController` interface is unchanged.
- [x] `python build.py --clean` zero errors.

## Implementation Plan

### Approach

Additive new class. `MotionController2` holds:
- `MotionController& _mc` — existing goal-closure engine
- `const subsystems::Drive2& _drive2` — for reading `state().fused` pose/twist
- Internal `HardwareState _hw` — populated from `_drive2.state()` in `tick()`
- Internal `MotorCommands _cmds` — sink for `driveAdvance()` motor outputs (discarded;
  Drive2 owns the real motor path)
- Internal `TargetState _target` — reply sink for EVT completions; wired to a null
  sink unless a reply channel is captured via `apply()`
- Internal `DesiredState _desired` — populated by BVC via `setBvcStateRef()`
- `msg::PlannerState _state` — updated in `tick()`
- `msg::PlannerConfig _planCfg` — stored by `configure()`

`apply()` dispatches on `PlannerCommand::goal_kind` to call the appropriate
`MotionController::begin*()` entry point. The reply sink in `TargetState` should be
wired to a no-op `ReplyFn` for now (EVT completion events are routed via the bus in
ticket 003).

`tick(now)` sequence:
1. Populate `_hw` from `_drive2.state()` (copy `fused.pose.x/y/h`, `fused.twist.vx`
   into the fields `driveAdvance` reads via `getPoseFloat`).
2. Call `_mc.driveAdvance(_hw, _cmds, _target, now)`.
3. Read the commanded body twist from `_desired.bodyTwist` (wired via `setBvcStateRef`).
4. Pack `msg::DrivetrainCommand` with `twist = {vx, 0, omega}` into `CommandBatch`.
5. Update `_state.mode`, `_state.active`, `_state.body_twist`.
6. Return the `CommandBatch`.

### Files to Create

- `source/superstructure/MotionController2.h` — class declaration
- `source/superstructure/MotionController2.cpp` — implementation
- `source/superstructure/PlannerConfig.cpp` — `toPlannerConfig()` implementation
- `source/superstructure/PlannerConfig.h` — `toPlannerConfig()` declaration

### Files to Modify

- `CMakeLists.txt` — add `MotionController2.cpp` and `PlannerConfig.cpp` to firmware
  and host-sim source lists

### Testing Plan

Ticket 002 covers planner-isolation tests. This ticket's test is compilation only:

```bash
python build.py --clean        # device build must be zero errors
uv run python -m pytest tests/simulation/unit/test_architecture_seams.py -v
uv run python -m pytest tests/simulation/unit/ -x --tb=short -q
```

The full suite must pass at 2380/2 (no new failures).

### Documentation Updates

Add a `// MotionController2 — Phase 3 Planner subsystem wrapper` comment at the top of
`MotionController2.h` referencing the architecture issue.
