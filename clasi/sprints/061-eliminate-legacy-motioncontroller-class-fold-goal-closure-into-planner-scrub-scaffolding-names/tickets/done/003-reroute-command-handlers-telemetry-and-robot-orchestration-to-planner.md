---
id: '003'
title: Reroute command handlers, telemetry, and Robot orchestration to Planner
status: done
use-cases:
- SUC-003
depends-on:
- '001'
- '002'
github-issue: ''
issue: internalize-legacy-motioncontroller-into-planner.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 003 — Reroute command handlers, telemetry, and Robot orchestration to Planner

## Description

After tickets 001 and 002, `Planner` has the full API and `Superstructure`
routes through it. This ticket reroutes the remaining direct call sites that
still reach `motionController` by name:

1. **`source/commands/MotionCommands.h`** — `MotionCtx::mc`:
   - Change `class MotionController;` forward-declaration to `class Planner;`.
   - Change `MotionController* mc;` to `Planner* mc;` in `MotionCtx`.
   - All handler lambda bodies that call `ctx->mc->beginX(...)` continue to
     compile unchanged because `Planner` now has the same signatures.

2. **`source/commands/MotionCommands.cpp`** (if it includes `MotionController.h`):
   - Update `#include` to use `"superstructure/Planner.h"`.

3. **`source/robot/Robot.cpp`** — setup block:
   - `motionController.setHardwareState(&state.actual)` ->
     `planner.setHardwareState(&state.actual)`
   - `motionController.setRobotCtx(this)` -> `planner.setRobotCtx(this)`
   - `motionController.setBvcStateRef(&state.desired)` ->
     `planner.setBvcStateRef(&state.desired)`
   - `_motionCtx.mc = &motionController` -> `_motionCtx.mc = &planner`
   - The `_motionCtx.mc = const_cast<MotionController*>(&motionController)`
     line in SystemCommands.cpp setup path also needs updating (see #5).

4. **`source/robot/Robot.cpp::otosCorrect`**:
   - `motionController.hasActiveCommand()` -> `planner.hasActiveCommand()`
   - `motionController.emitToActiveChannel(...)` -> `planner.emitToActiveChannel(...)`

5. **`source/robot/Robot.cpp::distanceDrive`**:
   - `motionController.beginDistance(...)` -> `planner.beginDistance(...)`

6. **`source/robot/RobotTelemetry.cpp`**:
   - `motionController.mode()` -> `planner.mode()` in the switch (line ~66).
   - `motionController.mode()` -> `planner.mode()` in the `_lastActiveMs`
     guard (line ~175).
   - Remove or update `#include "superstructure/MotionController.h"` if present;
     `planner` is a `Robot` public member so no extra include is needed.

7. **`source/commands/SystemCommands.cpp`**:
   - `robot->motionController.disableSafetyOneShot()` ->
     `robot->planner.disableSafetyOneShot()` (two call sites: ~line 614 and ~631).
   - `_motionCtx.mc = const_cast<MotionController*>(&motionController)` ->
     `_motionCtx.mc = &robot->planner` (the setup path in SystemCommands).

After this ticket, `robot.motionController` is still a valid member (not
removed yet) but no code outside `Planner.*` / `PlannerBegin.cpp` calls
methods on it by name.

## Acceptance Criteria

- [x] `MotionCtx::mc` is `Planner*` (not `MotionController*`).
- [x] `MotionCommands.h` forward-declares `Planner`, not `MotionController`.
- [x] `RobotTelemetry.cpp` calls `planner.mode()` in both locations.
- [x] `Robot.cpp` setup calls `planner.setHardwareState`, `planner.setRobotCtx`,
      `planner.setBvcStateRef`, and `_motionCtx.mc = &planner`.
- [x] `Robot.cpp::otosCorrect` uses `planner.hasActiveCommand()` and
      `planner.emitToActiveChannel(...)`.
- [x] `Robot.cpp::distanceDrive` calls `planner.beginDistance(...)`.
- [x] `SystemCommands.cpp` calls `robot->planner.disableSafetyOneShot()`
      in both call sites and assigns `_motionCtx.mc = &robot->planner`.
- [x] `grep -n "motionController\." source/robot/Robot.cpp
      source/robot/RobotTelemetry.cpp source/commands/SystemCommands.cpp
      source/commands/MotionCommands.h` returns zero hits.
- [x] `cmake --build build_sim` succeeds with zero errors.
- [x] `uv run python -m pytest tests/simulation/unit/test_golden_tlm.py
      tests/simulation/unit/test_059_ordered_tick_parity.py
      tests/simulation/unit/test_planner_subsystem.py` all pass.
- [x] `robot.motionController` still compiles (member not yet removed).

## Implementation Plan

### Approach

Edit files in dependency order: `MotionCommands.h` first (affects call type);
then `Robot.cpp` (three separate change sites: setup block, `otosCorrect`,
`distanceDrive`); then `RobotTelemetry.cpp`; then `SystemCommands.cpp`.
Rebuild after each file.

### Files to modify

- `source/commands/MotionCommands.h`
- `source/commands/MotionCommands.cpp` (if has `MotionController.h` include)
- `source/robot/Robot.cpp`
- `source/robot/RobotTelemetry.cpp`
- `source/commands/SystemCommands.cpp`

### Files NOT to touch

- `source/robot/Robot.h` (`motionController` member — ticket 004)
- `source/superstructure/Planner.h/.cpp` (no changes needed)
- Any test files

### Testing plan

After each file change, rebuild and run the three key tests. After all changes,
run the full suite once to confirm no regressions:
```
cmake --build build_sim && uv run python -m pytest \
  tests/simulation/unit/test_golden_tlm.py \
  tests/simulation/unit/test_059_ordered_tick_parity.py \
  tests/simulation/unit/test_planner_subsystem.py
```

### Documentation updates

Update `MotionCommands.h` file-level comment to reference `Planner` instead
of `MotionController`. Update any comments in `SystemCommands.cpp` that
describe the `_motionCtx.mc` assignment.
