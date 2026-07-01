---
id: '002'
title: Reroute Superstructure from MotionController to Planner
status: done
use-cases:
- SUC-002
depends-on:
- '001'
github-issue: ''
issue: internalize-legacy-motioncontroller-into-planner.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 002 — Reroute Superstructure from MotionController to Planner

## Description

`Superstructure` currently holds a `MotionController& _mc` member and its
`requestGoal` switch dispatches to `_mc.beginX(...)`. After ticket 001,
`Planner` exposes identical method signatures. This ticket changes
`Superstructure` to hold `Planner& _planner` instead.

Changes:

1. `source/superstructure/Superstructure.h`:
   - Replace `class MotionController;` forward-declaration with `class Planner;`.
   - Change constructor signature from `Superstructure(MotionController& mc, ...)`
     to `Superstructure(Planner& planner, ...)`.
   - Rename private member `_mc` to `_planner` with type `Planner&`.
   - Rename accessor `mc()` to `planner()` returning `Planner&`.

2. `source/superstructure/Superstructure.cpp`:
   - Add `#include "Planner.h"` (or update include from `MotionController.h`).
   - In `requestGoal`: change `_mc.beginX(...)` to `_planner.beginX(...)` for
     every case in the switch.
   - In `evaluateSafety`: verify no `_mc` references remain (this method touches
     `_hc` and `cmd`/`queue`, not motion). Update any if found.
   - Any local `MotionController& mc = _mc;` aliases become
     `Planner& mc = _planner;` (or rename the alias).

3. `source/robot/Robot.h`:
   - `Superstructure` declaration must come AFTER `Planner planner;` in the
     member list (already the case — verify and do not reorder).
   - Change `superstructure(motionController, haltController, config)` in the
     constructor initializer (`Robot.cpp`) to
     `superstructure(planner, haltController, config)`.

4. Audit `source/robot/LoopScheduler.h` for any `superstructure.mc()` call
   or `MotionController` reference. If found, update to `superstructure.planner()`.
   If only a comment, update the comment.

`motionController` member in `Robot.h` is NOT removed in this ticket — it
still exists and is still used by other call sites.

## Acceptance Criteria

- [x] `Superstructure.h` forward-declares `Planner`, not `MotionController`.
- [x] `Superstructure` constructor takes `Planner&`.
- [x] `Superstructure::_planner` (was `_mc`) is `Planner&`.
- [x] `Superstructure::planner()` accessor returns `Planner&`.
- [x] `requestGoal` switch calls `_planner.beginX(...)` for all Goal cases.
- [x] `Robot.cpp` initializer uses `superstructure(planner, haltController, config)`.
- [x] `LoopScheduler.h` has no remaining `MotionController` reference or
      `superstructure.mc()` call.
- [x] `robot.motionController` is still a valid member (not removed yet).
- [x] `cmake --build build_sim` succeeds with zero errors.
- [x] `uv run python -m pytest tests/simulation/unit/test_golden_tlm.py
      tests/simulation/unit/test_059_ordered_tick_parity.py
      tests/simulation/unit/test_planner_subsystem.py` all pass.

## Implementation Plan

### Approach

Edit `Superstructure.h` first (type changes); then `Superstructure.cpp`
(call site updates); then `Robot.cpp` (constructor initializer). Finally
audit `LoopScheduler.h`.

### Files to modify

- `source/superstructure/Superstructure.h`
- `source/superstructure/Superstructure.cpp`
- `source/robot/Robot.cpp` (initializer list only)
- `source/robot/LoopScheduler.h` (if needed)

### Files NOT to touch

- `source/commands/MotionCommands.h` (ticket 003)
- `source/commands/SystemCommands.cpp` (ticket 003)
- `source/robot/RobotTelemetry.cpp` (ticket 003)
- `source/robot/Robot.h` `motionController` member (ticket 004)

### Testing plan

After each file edit, rebuild:
```
cmake --build build_sim && uv run python -m pytest \
  tests/simulation/unit/test_golden_tlm.py \
  tests/simulation/unit/test_059_ordered_tick_parity.py \
  tests/simulation/unit/test_planner_subsystem.py
```

### Documentation updates

Update `Superstructure.h` class comment to reference `Planner` rather than
`MotionController`.
