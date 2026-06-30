---
id: '003'
title: Generated headers compile + static_assert bridges + host unit tests
status: open
use-cases:
- SUC-003
- SUC-004
depends-on:
- '002'
github-issue: ''
issue: message-based-subsystem-architecture.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Generated headers compile + static_assert bridges + host unit tests

## Description

Verify that the generated `source/messages/*.h` headers from ticket 002 compile
cleanly under the real firmware flags in both the device build and the host-sim
build. Fix any generator bugs surfaced by compilation. Write the `static_assert`
layout-compat bridges in `source/messages/bridges.h`. Write host unit tests for
the generated types.

This ticket makes the generated types usable — tickets 001 and 002 only produce
text; this ticket is the first compile and the first test.

## Compile verification tasks

### Device build

Run `python build.py --clean` and confirm zero errors and zero relevant warnings
in the generated headers. The compiler flags are `-std=c++11 -fno-rtti
-fno-exceptions` (inherited from the CODAL toolchain).

If the device build does not include `source/messages/` in its include path by
default: the main `CMakeLists.txt` uses `RECURSIVE_FIND_DIR` over `source/` to
collect include directories, so `source/messages/` should be picked up
automatically once the directory exists. Verify and fix if not.

### Host-sim build

Edit `tests/_infra/sim/CMakeLists.txt` to add `source/messages/` to
`target_include_directories`:
```cmake
"${REPO_ROOT}/source/messages"
```
Place it alongside the other `source/` includes in the `target_include_directories`
block (after `source/superstructure`).

Then rebuild the host sim library:
```bash
cd tests/_infra/sim/build && cmake .. && make -j$(nproc)
```

### Generated header compile-time constraints to verify

After both builds succeed, spot-check these properties:
- No `std::` identifier in any `source/messages/*.h` file.
- No `new` or `delete` keyword in any `source/messages/*.h` file.
- `static_assert` in `bridges.h` fires if types diverge (test by temporarily
  adding a padding field to `Pose2D` and confirming the assert fails).

## `bridges.h` static_assert content

`source/messages/bridges.h` must:
1. `#include "hal/capability/Pose2D.h"` (the hand-authored types).
2. `#include "messages/common.h"` (generated types).
3. Emit `static_assert(sizeof(::Pose2D) == sizeof(msg::Pose2D), "Pose2D layout mismatch");`
   (where `msg::` is the namespace or a generated prefix if used).
4. Emit similar checks for `BodyTwist3` and `RobotGeometry`.
5. Optionally add `using` aliases: `using msg::Pose2D = ::Pose2D;` so callers can
   use either name interchangeably during the Phase 2 migration.

Note: if the generator emits types into a namespace (e.g. `namespace msg {}`),
bridges.h uses the qualified names. If types are global, the aliases are
straightforward type synonyms. The choice is the implementer's; both approaches
satisfy the contract.

## Host unit tests

Create `tests/simulation/unit/test_messages.py`. The test file exercises the
generated C++ types through the host sim library (`tests/_infra/sim/firmware.py`
ctypes wrapper). If the ctypes wrapper does not yet expose individual message
constructors, use a thin C shim compiled into the sim library that exercises the
message API and returns verifiable values.

### Minimum test coverage

**Test 1: DrivetrainCommand fluent builder round-trip**
```python
def test_drivetrain_command_fluent_builder():
    # Construct DrivetrainCommand, call setTwist(100.0, 0.0, 1.5), read back twist
    # Assert vx_mmps == 100.0, vy_mmps == 0.0, omega_rads == 1.5
    # Assert control_kind == TWIST
```

**Test 2: Opt<T> present**
```python
def test_motor_command_opt_present():
    # Construct MotorCommand, call setFeedforward(0.25)
    # Assert feedforward.has == True, feedforward.val == 0.25
```

**Test 3: Opt<T> absent**
```python
def test_motor_command_opt_absent():
    # Construct default MotorCommand
    # Assert feedforward.has == False
```

**Test 4: CommandBatch repeated field**
```python
def test_command_batch_count():
    # Construct CommandBatch, append 2 OutCommands
    # Assert cmds_count == 2
```

**Test 5: PlannerConfig chainable setter**
```python
def test_planner_config_chained_setters():
    # cfg.setAMax(300.0).setVBodyMax(400.0)
    # Assert a_max == 300.0, v_body_max == 400.0
```

**Test 6: static_assert bridges compile** (this is a compile-time test verified
by the build step; add a comment noting this was verified by `build.py --clean`
and the host sim build).

### Note on test implementation approach

If calling into C++ structs directly via ctypes is difficult (ctypes requires
knowing the struct layout), the preferred approach is to write a small C-linkage
shim in `tests/_infra/sim/sim_api.cpp` (or a separate `message_test_api.cpp`)
that exposes testable functions:

```cpp
extern "C" int test_drivetrain_command_twist_roundtrip(
    float vx, float vy, float omega,
    float* out_vx, float* out_vy, float* out_omega);
```

These shim functions are then called from Python via ctypes. This is the same
pattern used by the existing `sim_api.cpp` shims.

## Acceptance Criteria

- [ ] `python build.py --clean` exits 0 with `source/messages/*.h` present (device build).
- [ ] Host sim library builds cleanly after adding `source/messages/` to include path.
- [ ] `tests/_infra/sim/CMakeLists.txt` has `source/messages/` in `target_include_directories`.
- [ ] `source/messages/bridges.h` has `static_assert` checks for `Pose2D`, `BodyTwist3`,
      and `RobotGeometry` size equality vs. hand-authored types.
- [ ] `tests/simulation/unit/test_messages.py` exists and contains at minimum the 5
      test functions listed above (or equivalent).
- [ ] `uv run python -m pytest tests/simulation/unit/test_messages.py` exits 0.
- [ ] Full `uv run python -m pytest` run is green except the 2 pre-existing
      `tag_offset_mm.z` failures (no new failures introduced).
- [ ] No `std::`, `new`, `delete`, `throw` in any `source/messages/*.h` file.

## Implementation Plan

### Approach

1. Run `python build.py --clean` with the output of ticket 002. Note any compile errors.
2. Fix generator bugs in `gen_messages.py` (ticket 002) as needed — these fixes go
   into this ticket's commit since they are surfaced by compilation.
3. Add `source/messages/` to sim CMakeLists.
4. Write `bridges.h` content.
5. Write test shims in `sim_api.cpp` as needed.
6. Write `test_messages.py`.
7. Run `uv run python -m pytest` and iterate until green.

### Files to create

- `tests/simulation/unit/test_messages.py`
- Optional: `tests/_infra/sim/message_test_api.cpp` (C shim for ctypes tests)

### Files to modify

- `tests/_infra/sim/CMakeLists.txt` — add `source/messages/` include path,
  and add `message_test_api.cpp` to `firmware_host` sources if shim is needed.
- `scripts/gen_messages.py` — bug fixes surfaced by compilation (if any).
- `source/messages/bridges.h` — authored (not generated) content for the
  `static_assert` checks and `using` aliases.

### Testing plan

Primary: `uv run python -m pytest tests/simulation/unit/test_messages.py -v`
Regression: `uv run python -m pytest` (full suite).
Device: `python build.py --clean`.

### Documentation updates

None in this ticket.

## Verification Command

`uv run python -m pytest`
