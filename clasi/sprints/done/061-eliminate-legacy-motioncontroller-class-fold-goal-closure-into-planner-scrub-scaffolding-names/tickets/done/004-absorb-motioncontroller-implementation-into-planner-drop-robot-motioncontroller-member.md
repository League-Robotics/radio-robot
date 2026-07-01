---
id: '004'
title: Absorb MotionController implementation into Planner; drop Robot motionController
  member
status: done
use-cases:
- SUC-004
depends-on:
- '003'
github-issue: ''
issue: internalize-legacy-motioncontroller-into-planner.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 004 — Absorb MotionController implementation into Planner; drop Robot motionController member

## Description

After ticket 003, no code outside `Planner.*` calls methods on
`motionController` by name. This ticket performs the structural absorb:
moves all `MotionController` implementation into `Planner`, changes `Planner`'s
constructor to take `MotorController&` and `Odometry&` directly (instead of
`MotionController&`), and removes the `motionController` value member from
`Robot`.

This is the highest-risk ticket in the sprint. The S/T/D/G state machines and
all motion logic move files. After this ticket `MotionController.h/.cpp` and
`MotionControllerBegin.cpp` are no longer needed (but NOT deleted yet —
ticket 005 deletes them to make any lingering include errors visible separately).

### Step-by-step changes

1. **`source/superstructure/Planner.h`** — restructure the private section:
   - Remove `MotionController& _mc;` member.
   - Add all private members that `MotionController` owned:
     `MotorController& _mc_ctrl`, `Odometry& _odo`, `HardwareState* _hwState`,
     `struct Robot* _robot`, `BodyVelocityController _bvc`, `MotionCommand _activeCmd`,
     `bool _safeOneShotDisable`, `DriveMode _mode`, `float _tgtL`, `float _tgtR`,
     `float _dDistTarget`, `float _dOmega`, `float _dEnc0`,
     `GPhase _gPhase` (G state machine enum), `_gTargetXWorld`, `_gTargetYWorld`,
     `_gSpeed`, and any PURSUE re-gate counter and related vars.
   - CRITICAL: `_bvc` must be declared before `_activeCmd` (constructor passes
     `&_bvc` to `_activeCmd.configure()`).
   - Add private static helper declarations: `_checkSafeOneShot`, `_startPreRotate`,
     `emitEvt`, `fullStop`, `getPoseFloat` (all previously in MotionController).
   - Change constructor declaration:
     `Planner(MotorController& mc_ctrl, Odometry& odo, const subsystems::Drive& drive, const RobotConfig& cfg)`
   - Remove the delegation method block added in ticket 001 (methods are now
     native, not delegated).
   - Remove `#include "MotionController.h"`.
   - Add `#include "control/Odometry.h"` (or forward-declare if possible),
     `#include "commands/MotionCommand.h"`, and any other headers previously
     pulled in via `MotionController.h`.

2. **`source/superstructure/Planner.cpp`** — rebuild the implementation:
   - Change constructor signature to match new header.
   - In the member-initializer list: `_mc_ctrl(mc_ctrl)`, `_odo(odo)`,
     `_bvc(mc_ctrl, cfg)`, `_activeCmd()` (with `_bvc` configure call in body).
   - Move the `driveAdvance()` body from `MotionController.cpp` into `Planner.cpp`
     (replacing references to `_mc` members with direct `_bvc`, `_activeCmd`, etc.).
   - Move `stop()`, `cancel()`, `softStop()`, `beginRawVelocity()`,
     `disableSafetyOneShot()`, `emitEvt()`, `fullStop()`, `getPoseFloat()` bodies
     from `MotionController.cpp` into `Planner.cpp`.
   - All formerly-delegation methods now point to the local members directly.

3. **`source/control/PlannerBegin.cpp`** (new file):
   - Copy the entire content of `source/control/MotionControllerBegin.cpp`.
   - Change `#include "superstructure/MotionController.h"` to
     `#include "superstructure/Planner.h"`.
   - Rename all method signatures from `MotionController::beginX(...)` to
     `Planner::beginX(...)`.
   - Replace all `_mc.` references with direct `_bvc.`, `_activeCmd.`, `_odo.`,
     `_mode`, `_tgtL`, `_tgtR`, `_gPhase`, `_robot`, etc.
   - Add `PlannerBegin.cpp` to `CMakeLists.txt`; do NOT remove
     `MotionControllerBegin.cpp` yet (ticket 005).

4. **`source/robot/Robot.h`**:
   - Remove `#include "superstructure/MotionController.h"`.
   - Remove `MotionController motionController;` member declaration.
   - Update `Planner planner;` declaration with new constructor comment.
   - Update the declaration-order comment block: step 7 (`motionController`)
     is removed; `planner` now depends on `motorController`, `estimate.odometry()`,
     and `drive`.
   - Verify `superstructure` still comes after `planner` and `haltController`
     in the member list.

5. **`source/robot/Robot.cpp`** — constructor:
   - Remove `motionController(motorController, estimate.odometry(), config)` from
     the initializer list.
   - Change `planner(motionController, drive, config)` to
     `planner(motorController, estimate.odometry(), drive, config)`.
   - Remove `#include "superstructure/MotionController.h"` if it is in `Robot.cpp`
     at the top (it may be there directly or transitively via `Robot.h`).

6. **`tests/_infra/sim/planner_api.cpp`** — update `PlannerHandle`:
   - Remove `MotionController motion_ctrl;` member.
   - Remove `motion_ctrl(mc_ctrl, est.odometry(), cfg)` from initializer list.
   - Change `planner(motion_ctrl, drive, cfg)` to
     `planner(mc_ctrl, est.odometry(), drive, cfg)`.
   - Remove `#include "superstructure/MotionController.h"`.
   - Update file-level comment.

## Acceptance Criteria

- [x] `Planner.h` has no `_mc` member and no `#include "MotionController.h"`.
- [x] `Planner.h` private section declares `_bvc` before `_activeCmd`.
- [x] `Planner` constructor takes `(MotorController&, Odometry&, const subsystems::Drive&, const RobotConfig&)`.
- [x] `Planner.cpp` contains `driveAdvance()`, `stop()`, `cancel()`, `softStop()`,
      `beginRawVelocity()`, `emitEvt()`, `fullStop()`, `getPoseFloat()` bodies,
      all operating on local members.
- [x] `PlannerBegin.cpp` exists with all `Planner::beginX(...)` bodies.
- [x] `PlannerBegin.cpp` is in `CMakeLists.txt` (sim target).
- [x] `Robot.h` has no `MotionController motionController;` member.
- [x] `Robot.h` has no `#include "superstructure/MotionController.h"`.
- [x] `Robot.cpp` initializer constructs `planner(motorController, estimate.odometry(), drive, config)`.
- [x] `planner_api.cpp` `PlannerHandle` has no `motion_ctrl` member.
- [x] `cmake --build build_sim` succeeds with zero errors.
- [x] `uv run python -m pytest tests/simulation/unit/test_golden_tlm.py
      tests/simulation/unit/test_059_ordered_tick_parity.py
      tests/simulation/unit/test_planner_subsystem.py` all pass.

## Implementation Plan

### Approach

Do NOT attempt this ticket in one pass. Work in layers:
1. Create `PlannerBegin.cpp` as a copy of `MotionControllerBegin.cpp` with
   method names changed but bodies left as-is (they may initially reference
   `_mc.` — fix these). Add to CMakeLists.txt.
2. Add private members to `Planner.h` (copy from `MotionController.h` private
   section). Keep the `_mc` reference temporarily to make the build
   incremental-friendly.
3. Move `driveAdvance()` and helpers into `Planner.cpp`.
4. Remove `_mc` from `Planner.h`; remove delegation methods from ticket 001.
5. Update `Robot.h` / `Robot.cpp` to drop `motionController`.
6. Update `planner_api.cpp`.
7. Rebuild and test.

### Files to modify

- `source/superstructure/Planner.h` (major restructure)
- `source/superstructure/Planner.cpp` (absorb driveAdvance + helpers)
- `source/control/PlannerBegin.cpp` (new — begin* bodies)
- `CMakeLists.txt` (sim target: add PlannerBegin.cpp)
- `source/robot/Robot.h` (drop motionController member + include)
- `source/robot/Robot.cpp` (update initializer)
- `tests/_infra/sim/planner_api.cpp` (update PlannerHandle)

### Files NOT to touch in this ticket

- `source/superstructure/MotionController.h/.cpp` (keep for now; deleted in 005)
- `source/control/MotionControllerBegin.cpp` (keep for now; deleted in 005)

### Testing plan

```
cmake --build build_sim && uv run python -m pytest \
  tests/simulation/unit/test_golden_tlm.py \
  tests/simulation/unit/test_059_ordered_tick_parity.py \
  tests/simulation/unit/test_planner_subsystem.py
```

Run after every major sub-step. If golden-TLM fails, this is a regression —
do NOT update the golden capture; debug the implementation.

### Documentation updates

- Update `Planner.h` class comment to reflect that it now owns BVC and
  MotionCommand directly (no longer wraps `MotionController`).
- Update `PlannerBegin.cpp` file-level comment.
- Update the `Robot.h` declaration-order comment block.
