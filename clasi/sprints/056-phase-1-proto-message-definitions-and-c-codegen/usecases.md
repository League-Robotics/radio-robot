---
sprint: '056'
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 056 Use Cases

## SUC-001 — Author proto3 schema files as the canonical message SSOT

- **Actor**: Firmware developer
- **Preconditions**: Phase 0 (module reorg) has landed; `source/hal/capability/`,
  `source/com/`, and `source/robot/` are in their post-reorg locations.
- **Main Flow**:
  1. Developer writes `protos/common.proto`, `protos/motor.proto`,
     `protos/drivetrain.proto`, `protos/sensors.proto`, `protos/gripper.proto`,
     `protos/ports.proto`, and `protos/planner.proto` using proto3 syntax.
  2. Each file uses custom option `(units)` for SI/unit metadata on fields.
  3. Every `repeated` field carries `(max_count)=N` to bound the fixed array.
  4. CI runs `protoc --proto_path=protos` (or grpcio-tools equivalent) as a lint check.
- **Postconditions**: All 7 `.proto` files are present under `protos/` and parse
  without errors.
- **Acceptance Criteria**:
  - [ ] `protos/common.proto`, `motor.proto`, `drivetrain.proto`, `sensors.proto`,
        `gripper.proto`, `ports.proto`, `planner.proto` all exist.
  - [ ] `protoc` parses all 7 files without error.
  - [ ] Every `repeated` field has `(max_count)=N` annotation.
  - [ ] All value-type messages defined in the issue inventory are present.

---

## SUC-002 — Generate C++11 POD headers from proto schemas

- **Actor**: Build system (triggered automatically by `build.py`)
- **Preconditions**: SUC-001 complete; `grpcio-tools` installed on the host; `protos/*.proto` valid.
- **Main Flow**:
  1. `build.py` invokes `scripts/gen_messages.py` (beside the existing
     `gen_default_config.py` call).
  2. `gen_messages.py` uses `grpcio-tools` to obtain a `FileDescriptorSet` from
     the `.proto` files without running a gRPC server.
  3. The generator emits one header per `.proto` to `source/messages/*.h`.
  4. Each header contains: plain struct members for scalar fields; `Kind` enum +
     union for `oneof`; `Opt<T>` template struct for nullable fields; fixed arrays
     `T field[N]; uint8_t count;` for `repeated`; getters for all; chainable
     setters for Command/Config messages.
  5. No heap, no exceptions, no STL containers appear in the output.
- **Postconditions**: `source/messages/*.h` exist and are idempotent on re-run.
- **Acceptance Criteria**:
  - [ ] `scripts/gen_messages.py` exists and runs without error.
  - [ ] One `.h` file exists under `source/messages/` per `.proto` file.
  - [ ] Generated headers contain no `new`, `delete`, `std::`, or `throw` keywords.
  - [ ] `Opt<T>` struct template is defined exactly once (in `common.h` or a shared
        generated preamble header).
  - [ ] Chainable setters exist on DrivetrainCommand, MotorCommand, PlannerCommand,
        and all `*Config` message types.

---

## SUC-003 — Generated headers compile in firmware and host-sim builds

- **Actor**: Build system (CI)
- **Preconditions**: SUC-002 complete; `source/messages/*.h` generated.
- **Main Flow**:
  1. Device build: `python build.py --clean` runs gen_messages.py then compiles
     firmware targeting micro:bit v2 with `-std=c++11 -fno-rtti -fno-exceptions`.
  2. Host-sim build: `cmake` + `make` for the `libfirmware_host` target includes
     `source/messages/` in the include path.
  3. A bridge header `source/messages/bridges.h` provides `using` aliases binding
     the generated types to the existing hand-authored types in
     `source/hal/capability/Pose2D.h`, protected by `static_assert` size checks.
- **Postconditions**: Both builds exit 0; `static_assert` checks pass at compile time.
- **Acceptance Criteria**:
  - [ ] `python build.py --clean` exits 0 with `source/messages/*.h` present.
  - [ ] Host sim library builds cleanly (`cmake` in `tests/_infra/sim/build/`).
  - [ ] `static_assert(sizeof(Pose2D) == ...)` checks for hand vs. generated types pass.
  - [ ] No compiler warnings about undefined behavior, RTTI, or exceptions in the
        generated headers.

---

## SUC-004 — Host unit tests exercise generated message types

- **Actor**: CI / developer running `uv run python -m pytest`
- **Preconditions**: SUC-002 and SUC-003 complete; host sim library loadable.
- **Main Flow**:
  1. `tests/simulation/unit/test_messages.py` exercises the generated types.
  2. Tests cover: fluent builder round-trip, `Opt<T>` present/absent, fixed-array
     `CommandBatch`, `static_assert` bridge, and absence of heap calls.
- **Postconditions**: All new tests pass; existing suite baseline unchanged.
- **Acceptance Criteria**:
  - [ ] `uv run python -m pytest tests/simulation/unit/test_messages.py` passes.
  - [ ] Full `uv run python -m pytest` run is green except the 2 pre-existing
        `tag_offset_mm.z` failures.
  - [ ] Fluent builder chain: `DrivetrainCommand.setTwist(vx, vy, omega)` then
        `.twist()` getter returns the same values.
  - [ ] `Opt<T>` present: field is `has=true`, value accessible.
  - [ ] `Opt<T>` absent: field is `has=false`.
  - [ ] `CommandBatch` `count` field reflects number of appended commands.

---

## SUC-005 — Traceability table maps every message field to existing code

- **Actor**: Developer / reviewer
- **Preconditions**: SUC-002 complete; `gen_messages.py` can emit the inventory.
- **Main Flow**:
  1. Developer runs `gen_messages.py` (or `build.py`) with a flag that also
     writes `docs/design/message-inventory.md`.
  2. The table lists every message field alongside its corresponding member in
     `ActualState`, `DesiredState`, `RobotConfig`, or the portable-motor-interface.
  3. Reviewer spot-checks key mappings.
- **Postconditions**: `docs/design/message-inventory.md` exists with full coverage.
- **Acceptance Criteria**:
  - [ ] `docs/design/message-inventory.md` exists after running the generator.
  - [ ] `DrivetrainState` fields map to `ActualState` members.
  - [ ] `PlannerCommand` fields map to `DesiredState`/`GoalRequest` fields.
  - [ ] `MotorCommand` control modes map to portable-motor-interface verbs.
  - [ ] All `*Config` fields map to `RobotConfig` members.
