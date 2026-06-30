---
id: '001'
title: 'Namespace generated messages to msg::'
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: message-based-subsystem-architecture.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Namespace generated messages to msg::

## Description

Phase 1 (sprint 056) generated C++11 POD message types into `source/messages/*.h`
at global scope. This creates a name collision with the HAL types `::Pose2D` and
`::BodyTwist3` in `source/hal/capability/Pose2D.h` — the collision is documented
in `source/messages/bridges.h` as "resolved in Phase 2 via namespace migration."

This ticket wraps all generated types in `namespace msg { ... }` so `msg::Pose2D`
and `::Pose2D` are distinct names and can appear in the same TU. This is the
foundation for all Phase 2 work: `Drive2.h` needs both generated message types and
HAL types in the same translation unit.

## Approach

1. Update `scripts/gen_messages.py` to emit `namespace msg {\n...\n}` wrapping
   around all generated type definitions in each `*.h` file. The `Opt<T>` template,
   all message structs, and enums (`Neutral`, `DriveMode`, etc.) go inside `msg::`.
2. Regenerate `source/messages/*.h` — all 7 files (`common.h`, `motor.h`,
   `drivetrain.h`, `sensors.h`, `gripper.h`, `ports.h`, `planner.h`). The files are
   auto-generated so re-running `gen_messages.py` (or `build.py`) produces the
   updated headers.
3. Update `source/messages/bridges.h`: the `#include "hal/capability/Pose2D.h"`
   stays; update any references to generated type names to use `msg::` prefix in
   comments and in any `using` aliases or `static_assert` expressions.
4. Update `tests/_infra/sim/message_test_api.cpp`: all generated type references
   gain `msg::` prefix (e.g., `DrivetrainCommand` → `msg::DrivetrainCommand`,
   `MotorCommand` → `msg::MotorCommand`, `CommandBatch` → `msg::CommandBatch`,
   `PlannerConfig` → `msg::PlannerConfig`).
5. Rebuild the host sim library and re-run `test_messages.py` to confirm all
   5 existing tests pass. The test file itself does not need code changes (it tests
   via ctypes shims, not direct C++ access) but add a test 6b confirming the `msg::`
   round-trip at the C++ shim level by renaming the existing twist-roundtrip shim
   call to use `msg::DrivetrainCommand` and verifying the `ControlKind` enum is
   still `msg::DrivetrainCommand::ControlKind::TWIST == 1`.

## Files to Create/Modify

- `scripts/gen_messages.py` — add `namespace msg` wrapper emission
- `source/messages/common.h` — regenerated (auto)
- `source/messages/motor.h` — regenerated (auto)
- `source/messages/drivetrain.h` — regenerated (auto)
- `source/messages/sensors.h` — regenerated (auto)
- `source/messages/gripper.h` — regenerated (auto)
- `source/messages/ports.h` — regenerated (auto)
- `source/messages/planner.h` — regenerated (auto)
- `source/messages/bridges.h` — regenerated + updated comments/static_asserts
- `tests/_infra/sim/message_test_api.cpp` — add `msg::` prefixes

## Acceptance Criteria

- [x] All `source/messages/*.h` wrap their types in `namespace msg { ... }`.
- [x] `msg::DrivetrainCommand`, `msg::Pose2D`, `msg::BodyTwist3`, `msg::Opt<T>`,
      `msg::CommandBatch`, `msg::Capabilities`, and all other generated types are
      accessible with the `msg::` prefix.
- [x] A single TU that includes both `messages/drivetrain.h` and
      `hal/capability/Pose2D.h` compiles without error — the name `Pose2D` resolves
      to `::Pose2D` (HAL) and `msg::Pose2D` (generated) as distinct types.
- [x] `uv run python -m pytest tests/simulation/unit/test_messages.py` passes all
      existing 5 tests plus any new round-trip test added (no regressions).
- [x] `python build.py --clean` exits 0 — generated headers compile under
      `-std=c++11 -fno-rtti -fno-exceptions`.
- [x] `bridges.h` `static_assert` layout checks compile and pass.

## Testing Plan

- **Regression**: `uv run python -m pytest tests/simulation/unit/test_messages.py -v`
  — all existing tests pass.
- **Namespace isolation compile test**: add a minimal `.cpp` fixture in
  `tests/_infra/sim/` that includes both `messages/drivetrain.h` and
  `hal/capability/Pose2D.h` and references both `msg::Pose2D` and `::Pose2D` in
  a `static_assert` (or function body). Verify it compiles as part of the sim build.
- **Full suite**: `uv run python -m pytest` — 2367 + 2 pre-existing baseline green.
- **Device build**: `python build.py --clean` zero errors.

## Verification Command

`uv run python -m pytest tests/simulation/unit/test_messages.py -v && python build.py --clean`
