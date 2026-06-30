---
id: '006'
title: Rename bvc2/Drive2/MotionController2 to clean names (de-scaffold)
status: done
use-cases:
- SUC-006
depends-on:
- '005'
github-issue: ''
issue: make-ordered-tick-the-default-close-parity-gaps.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Rename bvc2/Drive2/MotionController2 to clean names (de-scaffold)

## Description

The `2`-suffix identifiers (`bvc2`, `Drive2`, `MotionController2`) were temporary
migration scaffolding while the legacy classes existed alongside. With the legacy
classes deleted in ticket 005, the `2` suffixes are now misleading. This ticket
renames them to their permanent canonical names.

### Rename map (as implemented)

| Old name | New name | Scope |
|----------|----------|-------|
| `bvc2` | `bvc` | `Robot.h/.cpp` member name only (class `BodyVelocityController` unchanged) |
| `subsystems::Drive2` | `subsystems::Drive` | Class name, files (`Drive2.h/.cpp` → `Drive.h/.cpp`), all references |
| `MotionController2` | `Planner` | Class name, files (`MotionController2.h/.cpp` → `Planner.h/.cpp`), all references |
| `robot.drive2` | `robot.drive` | Robot member name |
| `robot.planner` | `robot.planner` | UNCHANGED — already the clean name |

**Note on `Planner` vs `MotionController`:** The ticket plan originally said
`MotionController2` → `MotionController`, but that name collides with the existing
legacy `MotionController` class (still a public Robot value member, deferred by
ticket 005). The wrapper class was therefore renamed `Planner` instead. The legacy
`MotionController` class and files are unchanged.

### `motionController.mode()` in `RobotTelemetry.cpp`

`RobotTelemetry.cpp` keeps `motionController.mode()` — `Planner` has no `mode()`
method. The legacy `MotionController` is still a public Robot member, so this
call is valid and unchanged.

### Robot.h declaration order (load-bearing — do not change)

After rename, the order is:
1. `BodyVelocityController bvc;`
2. `subsystems::Drive drive;`
3. `subsystems::Sensors sensors;`
4. `Planner planner;`

### C ABI shim function names preserved

Function names in `drive2_api.cpp` (`drive2_api_*`) and `bus_drain_api.cpp`
(`bus_drain_api_drive2_*`) were NOT renamed — Python tests call them by string
name via ctypes. Only the internal C++ type names were updated.

## Acceptance Criteria

- [x] `grep -rn "Drive2\|bvc2\|MotionController2" source/ tests/` returns nothing (only Python docstring/comment references remain, no C++ types).
- [x] `source/subsystems/drive/Drive.h` and `Drive.cpp` are the canonical files (no `Drive2.*` remain).
- [x] `source/superstructure/Planner.h` and `Planner.cpp` are the canonical files (no `MotionController2.*` remain).
- [x] `Robot.h` declaration order: `bvc` before `drive` before `sensors` before `planner`.
- [x] `RobotTelemetry.cpp` keeps `motionController.mode()` — `Planner` has no `mode()` method; legacy `MotionController` is still a public Robot member.
- [x] Codebase compiles cleanly with no warnings about renamed identifiers.
- [x] `uv run python -m pytest` — green except the 2 known-baseline failures.
- [x] `test_golden_tlm.py` remains green.
- [x] `test_059_ordered_tick_parity.py` remains green.

## Implementation Plan

### Files to modify

- `source/robot/Robot.h` — rename `bvc2`→`bvc`, `drive2`→`drive`, update `Drive2`→`Drive`, `MotionController2`→`MotionController`, update `#include` paths.
- `source/robot/Robot.cpp` — rename member references throughout.
- `source/robot/LoopTickOnce.cpp` — rename `robot.drive2`→`robot.drive`, `robot.planner` type reference.
- `source/robot/BusDrain.h/.cpp` — rename `Drive2`→`Drive`, `MotionController2`→`MotionController`.
- `source/robot/RobotTelemetry.cpp` — `motionController.mode()`→`planner.mode()`.
- `source/robot/ConfigRegistry.cpp` — rename type references.
- `source/subsystems/drive/Drive2.h` → rename to `Drive.h`; class `Drive2`→`Drive`.
- `source/subsystems/drive/Drive2.cpp` → rename to `Drive.cpp`; update class name.
- `source/superstructure/MotionController2.h` → rename to `MotionController.h`; class name updated.
- `source/superstructure/MotionController2.cpp` → rename to `MotionController.cpp`; class name updated.
- Any test file referencing `Drive2` or `MotionController2` by name.

### Testing plan

1. Rebuild sim: `cd tests/_infra/sim && python3 build.py`
2. `grep -rn "Drive2\|bvc2\|MotionController2" source/ tests/` — must return nothing.
3. `uv run python -m pytest` — green except 2 known-baseline failures.

### Documentation updates

Update the comment block at the top of `LoopTickOnce.cpp` and `Robot.h` to use
the new names. No changes to architecture docs needed (architecture-update.md
for this sprint already documents the end-state names).
