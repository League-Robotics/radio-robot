---
id: '056'
title: Phase 1 - Proto message definitions and C++ codegen
status: done
branch: sprint/056-phase-1-proto-message-definitions-and-c-codegen
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
issues:
- message-based-subsystem-architecture.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 056: Phase 1 - Proto message definitions and C++ codegen

## Goals

Define every subsystem message in proto3 as the canonical SSOT and generate
C++11 POD structs the embedded firmware uses. This is Phase 1 of the three-
phase message-based subsystem architecture (issue: message-based-subsystem-
architecture.md). It delivers the schema layer and code-generation tooling —
no subsystem wiring, no serialize/deserialize, no wire-protocol change.

## Problem

The firmware currently has scattered per-subsystem state (ActualState,
DesiredState, RobotConfig) with no machine-readable canonical definition of
what each subsystem's message contract looks like. Phase 2 (subsystem wiring)
and Phase 3 (integration) both depend on a shared, generated type vocabulary.
Without a codegen step, the types must be maintained by hand across C++ headers
and Python test harnesses, which drifts.

## Solution

Extend the `scripts/gen_default_config.py` precedent: author `.proto` schema
files as the SSOT, parse them on the HOST via `protoc`/`grpcio-tools` to obtain
a `FileDescriptorSet`, and emit header-only C++11 POD structs to
`source/messages/*.h`. The device never sees protobuf; it only sees the generated
`.h` files. The generator runs automatically inside `build.py` alongside the
existing `gen_default_config.py` call.

## Success Criteria

- `protoc` parses all 7 `.proto` files without error (CI lint).
- `gen_messages.py` runs in `build.py`; emits `source/messages/*.h`.
- Generated headers compile under real firmware flags (`-std=c++11 -fno-rtti
  -fno-exceptions`) in BOTH the host-sim build and `python build.py --clean`.
- A host unit test (`uv run python -m pytest`) exercises fluent builders, getters,
  `Opt<T>` present/absent, and confirms no heap/RTTI — green alongside the
  existing "2363 passed, 2 failed" baseline.
- `static_assert` layout-compat bridges for `Pose2D` and `BodyTwist3` compile.
- `docs/design/message-inventory.md` traceability table generated; spot-checks
  pass for DrivetrainState↔ActualState, PlannerCommand↔DesiredState/GoalRequest,
  MotorCommand↔portable-motor-interface, `*Config`↔RobotConfig.

## Scope

### In Scope

- `protos/{common,motor,drivetrain,sensors,gripper,ports,planner}.proto` — full
  message inventory as specified in the issue Phase 1 section.
- `scripts/gen_messages.py` — host-side codegen script (protoc/grpcio-tools).
- `source/messages/*.h` — generated C++11 POD headers.
- `build.py` edit — add gen_messages.py call beside gen_default_config.py.
- `tests/_infra/sim/CMakeLists.txt` edit — add `source/messages/` to include
  paths (device CMakeLists picks it up via RECURSIVE_FIND automatically).
- `docs/design/message-inventory.md` — generated traceability table.
- Host unit tests for generated messages.
- `grpcio-tools` added as dev dependency in `pyproject.toml` (host-only, device never sees it).

### Out of Scope

- Phase 2: subsystem wiring (Drive, Sensors subsystem implementations).
- Phase 3: integration, run loop, bus drain.
- Binary wire / serialize-deserialize — the ASCII wire is unchanged.
- System/Config-registry/Debug command families — deferred.
- Goal logic (stays in MotionController / Phase 3).

## Test Strategy

Host test suite: `uv run python -m pytest` (NOT `uv run pytest` — see
tests/CLAUDE.md). Tests land in `tests/simulation/unit/test_messages.py`.
Baseline: "2363 passed, 2 failed" (2 pre-existing `tag_offset_mm.z` failures,
tracked separately). New tests must be green.

Device build verification: `python build.py --clean` — zero errors.

## Architecture Notes

- Generated headers are header-only (no `.cpp`); no `.cpp` sources to add to
  the sim glob — only include-path additions needed.
- The device CMakeLists uses `RECURSIVE_FIND_FILE` over `source/` so
  `source/messages/` is picked up automatically for include dirs and any `.cpp`
  that lands there.
- `Opt<T>` is a generated template (`bool has; T val;`) — NOT `std::optional`
  (unavailable in C++11 no-exceptions builds).
- Existing types (`Pose2D`, `BodyTwist3`, `RobotGeometry` from
  `source/hal/capability/Pose2D.h`) are reused via `using` aliases plus
  `static_assert` layout checks in the generated bridge header.
- The `(units)` custom proto option is metadata only (stored in the
  FileDescriptorSet, surfaced in the traceability doc) — it never emits C++ code.
- `(max_count)=N` on every `repeated` field dictates the fixed array size in the
  generated C++ (e.g., `OutCommand cmds[K]; uint8_t count;`).

## GitHub Issues

(none linked yet)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Proto schema files (7 .proto) | — |
| 002 | gen_messages.py codegen script + build integration | 001 |
| 003 | Generated headers compile + static_assert bridges + host unit tests | 002 |
| 004 | Traceability doc (message-inventory.md) | 003 |

Tickets execute serially in the order listed.
