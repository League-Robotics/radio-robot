---
id: "004"
title: "Retire Mock* files and delete obsolete sim objects"
status: open
use-cases: [SUC-007]
depends-on: ["040-003"]
github-issue: ""
issue: "migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md"
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Retire Mock* files and delete obsolete sim objects

## Description

With `SimHardware` + `PhysicsWorld` + observation models wired in (T2/T3),
the old `Mock*` files are dead code. This ticket removes them, confirms the host
sim builds clean, and verifies all tests still pass. This is a deletion-only
ticket: no new behavior, no new files, no ABI changes.

### Files to delete from source/io/sim/

Confirm each file has zero `#include` references outside `sim_api.cpp` (which was
already updated in T2). Then delete:

- `MockMotor.h` / `MockMotor.cpp`
- `MockHAL.h` / `MockHAL.cpp` (which contains the `ExactPoseTracker` struct)
- `MockOtosSensor.h` / `MockOtosSensor.cpp`
- `MockLineSensor.h` / `MockLineSensor.cpp`
- `MockColorSensor.h` / `MockColorSensor.cpp`
- `MockPortIO.h` / `MockPortIO.cpp`

**`MockServo.h` / `MockServo.cpp`:** Inspect whether `SimHardware` still
`#include`s `MockServo.h` for its `_servo` member. If so, rename the pair
to `SimServo.h` / `SimServo.cpp` and update the include in `SimHardware.h`.
If the programmer prefers, keep the `MockServo` name unchanged (functional;
just cosmetically inconsistent). Either choice is acceptable.

**`IMotor.h` shim and `IOtosSensor.h` shim** (in `source/hal/` or `source/io/`):
These alias shims from Phase A must not be deleted in this ticket — they may
still be referenced by `source/control/` headers and are scheduled for Phase F
cleanup. Do not touch them here.

**`BenchOtosSensor.h` / `.cpp`** (in `source/io/real/`): **Do not delete.** This
is a real-hardware bench firmware device; it is out of scope for Phase B.

### sim_api.cpp cleanup

After deletion, `sim_api.cpp` may have dead `#include` directives for `MockHAL.h`,
`MockMotor.h`, `MockOtosSensor.h`. Remove those includes. The file should now
only `#include` `SimHardware.h` and the firmware headers.

### CMakeLists.txt

Verify the glob `file(GLOB SIM_SOURCES ".../*.cpp")` no longer picks up the
deleted files (they are gone, so no action needed), and that it does pick up
all new `Sim*.cpp` files.

## Acceptance Criteria

- [ ] `MockMotor.h/cpp`, `MockHAL.h/cpp`, `MockOtosSensor.h/cpp`,
      `MockLineSensor.h/cpp`, `MockColorSensor.h/cpp`, `MockPortIO.h/cpp`
      are deleted from `source/io/sim/`.
- [ ] Host sim builds clean after deletion: `cmake --build tests/_infra/sim/build` succeeds.
- [ ] No dead `#include` directives for retired files remain in `sim_api.cpp`.
- [ ] `uv run --with pytest python -m pytest -q` ≥ 1957 passed, 0 errors.
- [ ] All canaries green: golden-TLM, field-pin, vendor grep.

## Implementation Plan

### Approach

1. Check for any remaining references to `Mock*` headers: `grep -r "MockMotor\|MockHAL\|MockOtosSensor" source/ tests/_infra/`.
2. Remove any residual includes from `sim_api.cpp`.
3. Delete the six `.h/.cpp` pairs.
4. Rebuild: `cmake --build tests/_infra/sim/build`.
5. Run full suite.

### Files to Delete

- `source/io/sim/MockMotor.{h,cpp}`
- `source/io/sim/MockHAL.{h,cpp}`
- `source/io/sim/MockOtosSensor.{h,cpp}`
- `source/io/sim/MockLineSensor.{h,cpp}`
- `source/io/sim/MockColorSensor.{h,cpp}`
- `source/io/sim/MockPortIO.{h,cpp}`

### Files to Modify

- `tests/_infra/sim/sim_api.cpp` — remove dead `#include` directives.
- `source/io/sim/SimHardware.h` — if `MockServo.h` is renamed, update include.

### Testing Plan

```
cmake --build tests/_infra/sim/build 2>&1 | grep -E "error:|warning:"
uv run --with pytest python -m pytest -q
```

Zero build errors. Full suite ≥ 1957 passed, 0 errors.

### Documentation Updates

None required.
