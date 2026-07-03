---
id: '003'
title: 'DesiredState/OutputState field renames: strip unit suffixes from commanded
  and actuator-output state'
status: done
use-cases:
- SUC-003
depends-on:
- '001'
github-issue: ''
issue: remove-units-from-identifier-names.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# DesiredState/OutputState field renames: strip unit suffixes from commanded and actuator-output state

## Description

Rename `DesiredState`'s and `OutputState`'s unit-suffixed fields,
resolving `source/state/DesiredState.h`'s whole-struct `FIXME` marker (the
issue's own worked example: `OutputState.h`'s `tgtMms` → `tgtSpeed`). Per
`architecture-update.md` Decision 4, this is a separate ticket from ticket
002 (`RobotConfig`) despite both having FIXME markers, because the two
structs have disjoint consumer sets: `RobotConfig` is a SET/GET-
configurable blob read by nearly every subsystem, while
`DesiredState`/`OutputState` are per-tick commanded/actuator state written
by the motion-command layer with a small, disjoint set of readers (5
references outside their own header, per `architecture-update.md`'s own
grep).

Fields renamed:
- `source/state/DesiredState.h`: `wheelMms` → `wheelSpeeds`,
  `targetSpeedMms` → `targetSpeed`, `distanceTargetMm` → `distanceTarget`,
  `deadlineMs` → `deadline`. Whole-struct `FIXME` comment removed.
- `source/state/OutputState.h`: `tgtMms` → `tgtSpeed` (the issue's own
  worked example, applied verbatim).

Consumers updated: `PlannerBegin.cpp`, `Planner.cpp`,
`MotionCommandHandlers`, `BodyVelocityController`.

This ticket depends only on ticket 001 (the comment convention) — it does
not depend on ticket 002, since `DesiredState`/`OutputState` share no
fields or files with `RobotConfig`. Both 002 and 003 may in principle run
in either order; this sprint sequences 002 first only because it is the
larger, higher-attention change (`architecture-update.md` Decision 4
Consequences).

See `architecture-update.md` Step 5 ("003 — DesiredState/OutputState
renames"), Decision 4; `usecases.md` SUC-003.

## Acceptance Criteria

- [x] `source/state/DesiredState.h`: `wheelMms`→`wheelSpeeds`,
      `targetSpeedMms`→`targetSpeed`, `distanceTargetMm`→`distanceTarget`,
      `deadlineMs`→`deadline`; each carries a `// [unit]` comment.
- [x] `grep -rn "FIXME" source/state/DesiredState.h` returns zero results.
- [x] `source/state/OutputState.h`: `tgtMms`→`tgtSpeed`, carries a
      `// [mm/s]` comment (the issue's own worked example, matched
      verbatim).
- [x] Consumers updated with no stray reference to an old field name
      remaining: `source/control/PlannerBegin.cpp`,
      `source/superstructure/Planner.cpp`, `MotionCommandHandlers`,
      `source/control/BodyVelocityController.cpp`. (Implementation note:
      `MotionCommandHandlers` was renamed to `MotionCommands` in an
      earlier sprint and does not reference these fields directly;
      `BodyVelocityController` also does not reference them directly. A
      full member-access sweep found the actual `OutputState::tgtMms`
      consumers to be `source/control/MotorController.cpp`,
      `source/subsystems/drive/Drive.cpp`, `source/robot/MecanumHAL.cpp`,
      and `source/robot/NezhaHAL.cpp` — all updated, no stray references
      remain anywhere in `source/`.)
- [x] `tests/simulation/unit/test_body_velocity_controller.py` and the
      `S`/`T`/`D`/`VW` system/unit test tiers pass with unchanged numeric
      assertions (only identifiers in test code/fixtures change, not
      expected values). (No test code/fixtures referenced the old field
      names, so zero test files needed changes.)
- [x] Motor commands and stop timing are byte-identical to pre-ticket for
      the same command sequence (spot-check at least one `S`/`T`/`D`/`VW`
      scenario before/after). (`tests/simulation/unit/test_golden_tlm.py`
      drives a `T 100 100 10000` sequence — exercising both
      `OutputState::tgtSpeed` and `DesiredState::deadline` — and asserts
      byte-identical TLM frames against the committed golden capture;
      passed unchanged.)
- [x] Full test suite green (`uv run python -m pytest`), no new failures
      against the 2620-passed baseline. (2621 passed, 0 failed — same
      count as post-002 baseline on this branch.)
- [x] `--clean` sim build performed before running tests.

## Testing

- **Existing tests to run**: `tests/simulation/unit/
  test_body_velocity_controller.py`, the `S`/`T`/`D`/`VW` system/unit test
  tiers, full default suite.
- **New tests to write**: none required — pure rename, no new behavior. If
  no existing test currently exercises `OutputState::tgtSpeed`'s value
  end-to-end, add a minimal regression asserting the renamed field carries
  the same value as `tgtMms` did pre-rename (belt-and-suspenders for the
  issue's own worked example).
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Rename `DesiredState.h`'s four fields and `OutputState.h`'s
one field, add `// [unit]` comments per `docs/coding-standards.md`, then
update the four consumer files. Grep for each old name across `source/`
after the rename to confirm no stray reference remains.

**Files to modify**:
- `source/state/DesiredState.h`
- `source/state/OutputState.h`
- `source/control/PlannerBegin.cpp`
- `source/superstructure/Planner.cpp`
- motion-command handler file(s) (`MotionCommandHandlers` — confirm exact
  path at implementation time)
- `source/control/BodyVelocityController.cpp`
- `tests/simulation/unit/test_body_velocity_controller.py` (any
  mock/kwarg mirroring these field names)

**Testing plan**: `--clean` sim build, then `test_body_velocity_controller.py`
and the S/T/D/VW test tier in isolation, then the full suite.

**Documentation updates**: none in this ticket (ticket 008's final sweep
covers prose docs).
