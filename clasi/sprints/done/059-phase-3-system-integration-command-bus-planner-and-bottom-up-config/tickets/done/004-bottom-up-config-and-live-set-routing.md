---
id: '004'
title: Bottom-up config and live SET routing
status: done
use-cases:
- SUC-004
- SUC-005
depends-on:
- 059-001
- 059-003
github-issue: ''
issue: message-based-subsystem-architecture.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bottom-up config and live SET routing

## Description

Wire the configuration flow from the top (`RobotConfig`) down to each subsystem's
typed `Config` slice (bottom-up from the subsystem's perspective):

1. **Init-time**: call `configure()` on each subsystem after construction, using the
   projection functions `toDriveConfig()`, `toSensorsConfig()`, and
   `toPlannerConfig()`.
2. **Live SET**: extend `handleSet` in `ConfigRegistry.cpp` to route a changed field
   to its owning subsystem's `configure(delta)` call, driven by a new `"subsystem":`
   annotation in `robot_config.schema.json`.
3. **SetPose / SI routing**: route the `SI` (Set Pose) verb to
   `drive2.apply(DrivetrainCommand{SetPose{x,y,h}})` so pose re-anchoring goes
   through the message contract rather than directly calling `estimate.resetPose()`.

Also, add `drive2`, `sensors`, and `planner` (`MotionController2`) as value members
on `Robot` (they may already be there from Phase 2 work; if so, verify they are
wired to `configure()` in the constructor). This ticket does NOT yet change
`loopTickOnce` — those members exist but are called only in the ordered tick (ticket
005).

## Acceptance Criteria

- [x] `Robot::Robot(Hardware& hal, const RobotConfig& cfg)` constructor calls:
  - `drive2.configure(toDriveConfig(cfg))` after `drive2` is constructed
  - `sensors.configure(toSensorsConfig(cfg).line, toSensorsConfig(cfg).color)` after `sensors` is constructed
  - `planner.configure(toPlannerConfig(cfg))` after `planner` is constructed
- [x] `data/robots/robot_config.schema.json` has a `"subsystem"` annotation for
  at minimum these fields:
  - `"vel.kP"`, `"vel.kI"`, `"vel.kFf"`, `"vel.iMax"`, `"vel.kAw"` → `"drive"`
  - `"aMax"`, `"vBodyMax"`, `"yawRateMax"`, `"arriveTolMm"` → `"planner"`
  - `"lagLineMs"`, `"lagColorMs"` → `"sensors"`
- [x] `ConfigRegistry.cpp::handleSet` reads the `"subsystem"` annotation for the
  changed field and calls the appropriate `configure()` method:
  - `"drive"` → constructs a `msg::DrivetrainConfig` delta and calls `drive2.configure(delta)`
  - `"planner"` → constructs a `msg::PlannerConfig` delta and calls `planner.configure(delta)`
  - `"sensors"` → constructs a sensor config delta and calls `sensors.configure(delta)`
  - Fields with no `"subsystem"` annotation continue to use the existing `kRegistry[]`
    direct-write path (backward compatibility preserved)
- [x] `MotorController::updateVelGains` is still called internally from `drive2.configure()`
  when velocity gain fields change (it is NOT deleted, just called from a new path).
- [x] `handleSI` (`SI` verb handler) routes to `drive2.apply(DrivetrainCommand{SetPose{x,y,h}})`
  instead of calling `estimate.resetPose()` directly. Behavior is identical.
- [x] Unit tests in `tests/simulation/unit/test_059_config_routing.py`:
  - `test_set_vel_kp_routes_to_drive2` — issue `SET vel.kP 2.0`; verify `drive2`
    applies the updated gain on the next `tickUpdate/tickAction`.
  - `test_set_amax_routes_to_planner` — issue `SET aMax 1500`; verify planner's
    internal `aMax` is updated.
  - `test_setpose_via_si_verb` — call the SI handler; verify `drive2.state()` pose
    is updated on the next `tickUpdate`.
  - `test_init_configure_called` — construct `Robot` on `MockHAL`; verify that
    `drive2.state()` and `sensors.state()` reflect the default `RobotConfig`
    values (not zero-initialized).
- [x] `python build.py --clean` zero errors.
- [x] `uv run python -m pytest -x --tb=short -q` at 2380/2 plus new tests (2408/2 total).
- [x] Existing `test_config_registry.py` and `test_config_set.py` pass unchanged.

## Implementation Plan

### Approach

**Schema annotation**: Add a `"subsystem"` string field to each relevant entry in
`robot_config.schema.json`. The schema is JSON, so each field entry that currently
has `"key"`, `"type"`, and `"offset"` gains `"subsystem": "drive"` etc.

**`handleSet` routing**: After the existing `kRegistry[]` lookup finds the matching
`ConfigEntry`, check whether the entry has a `subsystem` annotation. If yes, build
a typed config delta and call the appropriate `configure()`. If no, fall through to
the existing direct-write path. This is a pure extension — no existing behavior
changes for unannotated fields.

The `CfgCtx` struct in `ConfigRegistry.h` needs to gain pointers to `Drive2*`,
`MotionController2*`, and `Sensors*` (alongside the existing `cfg` and `mc` pointers).
Alternatively, add a `SubsystemCtx` sub-struct. The implementer should choose the
approach that minimizes the change surface; adding three optional pointers to
`CfgCtx` is the simplest.

**SI verb routing**: Find `handleSI` (likely in `source/commands/SystemCommands.cpp`
or `MotionCommands.cpp`). Change the call from `robot.estimate.resetPose(x, y, h)` to
`robot.drive2.apply(msg::DrivetrainCommand().setSetPose({x_mm, y_mm, h_rad}))`. The
behavior must be identical — Drive2's `tickUpdate` calls `estimate.resetPose` internally
when it processes the `SetPose` command.

**Init wiring in Robot constructor**: `drive2`, `sensors`, and `planner`
(`MotionController2`) were added as test-only members in Phase 2. Promote them to full
live members in `Robot.h` (if not already promoted) by adding `configure()` calls in
the constructor body after the init-list phase.

### Files to Modify

- `data/robots/robot_config.schema.json` — add `"subsystem"` annotation per field
- `source/robot/ConfigRegistry.h` — extend `CfgCtx` with subsystem pointers
- `source/robot/ConfigRegistry.cpp` — `handleSet` routing branch
- `source/robot/Robot.h` — promote `drive2`, `sensors`, `planner` to live members
- `source/robot/Robot.cpp` — add `configure()` calls in constructor; route `SI` via Drive2
- `source/commands/SystemCommands.cpp` (or wherever `handleSI` lives) — route SI
  to `drive2.apply()`

### Files to Create

- `tests/simulation/unit/test_059_config_routing.py` — config routing unit tests

### Testing Plan

```bash
python build.py --clean
uv run python -m pytest tests/simulation/unit/test_059_config_routing.py -v
uv run python -m pytest tests/simulation/unit/test_config_registry.py -v
uv run python -m pytest tests/simulation/unit/test_config_set.py -v
uv run python -m pytest -x --tb=short -q
```

### Documentation Updates

Add a comment to `CfgCtx` in `ConfigRegistry.h` explaining the `"subsystem":`
annotation convention and the routing precedence (annotated → configure(); unannotated
→ direct write).
