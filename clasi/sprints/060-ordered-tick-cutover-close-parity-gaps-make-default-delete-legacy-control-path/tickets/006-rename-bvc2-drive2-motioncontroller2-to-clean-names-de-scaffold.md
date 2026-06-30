---
id: '006'
title: Rename bvc2/Drive2/MotionController2 to clean names (de-scaffold)
status: open
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

### Rename map

| Old name | New name | Scope |
|----------|----------|-------|
| `bvc2` | `bvc` | `Robot.h/.cpp` member name only (class `BodyVelocityController` unchanged) |
| `subsystems::Drive2` | `subsystems::Drive` | Class name, files (`Drive2.h/.cpp` → `Drive.h/.cpp`), all references |
| `MotionController2` | `MotionController` | Class name, files (`MotionController2.h/.cpp` → `MotionController.h/.cpp`), all references |
| `robot.drive2` | `robot.drive` | Robot member name |
| `robot.planner` | `robot.planner` | UNCHANGED — already the clean name for the Planner role |

### Robot.h declaration order (load-bearing — do not change)

After rename, the order must be:
1. `BodyVelocityController bvc;`
2. `subsystems::Drive drive;`
3. `subsystems::Sensors sensors;`
4. `MotionController planner;`

### Call site scope

`drive2` appears in: `Robot.h/.cpp`, `LoopTickOnce.cpp`, `BusDrain.h/.cpp`,
`ConfigRegistry.cpp`, `RobotTelemetry.cpp`, and any test files referencing the
ordered-tick path. Use `grep -rn "drive2\|Drive2\|bvc2\|MotionController2" source/ tests/`
to find all sites before starting.

`MotionController2` also appears in `#include` directives. All includes of
`MotionController2.h` must be updated to `MotionController.h`.

### `motionController.mode()` in `RobotTelemetry.cpp`

After the rename, `robot.motionController` no longer exists (the member is
`robot.planner`). `telemetryEmit` at `RobotTelemetry.cpp:163` calls
`motionController.mode()`. This must become `robot.planner.mode()`. Confirm that
the renamed `MotionController` class (formerly `MotionController2`) exposes
`mode() -> DriveMode`. If not, add it.

### Approach

Rename in this order to keep the compiler clean:
1. `bvc2` → `bvc` in `Robot.h/.cpp` (member name only).
2. `Drive2.h/.cpp` files → `Drive.h/.cpp`; class name `Drive2` → `Drive`; all
   `drive2` member references → `drive`.
3. `MotionController2.h/.cpp` → `MotionController.h/.cpp`; class name → `MotionController`;
   update `motionController.mode()` → `planner.mode()` in `RobotTelemetry.cpp`.
4. Rebuild and fix compile errors.
5. Final grep check.

## Acceptance Criteria

- [ ] `grep -rn "Drive2\|bvc2\|MotionController2" source/ tests/` returns nothing.
- [ ] `source/subsystems/drive/Drive.h` and `Drive.cpp` are the canonical files (no `Drive2.*` remain).
- [ ] `source/superstructure/MotionController.h` and `MotionController.cpp` are the canonical files (no `MotionController2.*` remain).
- [ ] `Robot.h` declaration order: `bvc` before `drive` before `sensors` before `planner`.
- [ ] `RobotTelemetry.cpp` calls `planner.mode()` (not `motionController.mode()`).
- [ ] Codebase compiles cleanly with no warnings about renamed identifiers.
- [ ] `uv run python -m pytest` — green except the 2 known-baseline failures.
- [ ] `test_golden_tlm.py` remains green.
- [ ] `test_059_ordered_tick_parity.py` remains green.

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
