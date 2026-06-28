---
id: "001"
title: "Model-selection consolidation: IKinematics.h and main.cpp unconditional differential"
status: open
use-cases:
  - SUC-048-002
  - SUC-048-003
depends-on: []
issue: eliminate-ifdef-robot-drivetrain-mecanum-everywhere.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

## Description

Remove the `#ifdef ROBOT_DRIVETRAIN_MECANUM` selectors from the two top-level
model-selection files: `IKinematics.h` (kinematics namespace alias + `kWheelCount`)
and `main.cpp` (HAL include + HAL instantiation + `WHEEL_TEST_MAIN` block).

After this ticket the macro still exists in CMakeLists.txt / build.py (those are
removed in ticket 004), but the `#ifdef` branches in these two files are gone. Since
`kWheelCount` and the kinematics namespace are already evaluated at compile time by the
existing `#else` (differential) path, removing the mecanum branch here leaves all
consumers unchanged.

**Sequencing rationale:** This ticket is first because it is the smallest, most
self-contained change. It removes the macro from the kinematics selector (which
all state types depend on) and from `main.cpp` (which is leaf-level). No control
layer file is touched yet, so compilation remains clean throughout.

## Acceptance Criteria

- [ ] `source/kinematics/IKinematics.h` contains no `#ifdef`. Body is unconditional:
  `#include "BodyKinematics.h"`, `namespace Kinematics = BodyKinematics;`,
  `constexpr int kWheelCount = 2;`.
- [ ] `IKinematics.h` carries a comment marking this file and `main.cpp` as the
  places to edit when building a mecanum robot.
- [ ] `source/main.cpp` `#include "NezhaHAL.h"` is unconditional (no `#ifdef`
  guard around it).
- [ ] `main.cpp` HAL instantiation `static NezhaHAL hardware(...)` is unconditional.
- [ ] `main.cpp` `#define WHEEL_TEST_MAIN 0` line and the entire `#if defined(...)
  && WHEEL_TEST_MAIN` block are deleted.
- [ ] `main.cpp` has a comment on the HAL instantiation line noting this is the
  mecanum re-introduction point.
- [ ] `MecanumKinematics.h` / `MecanumKinematics.cpp` are untouched (retain).
- [ ] `MecanumHAL.cpp` is untouched (retain).
- [ ] `uv run pytest` passes (differential sim build, no behavior change).
- [ ] Firmware compiles cleanly (CMake macro still defined; `#ifdef` branches are
  now gone from these files, so the macro evaluating true/false does not matter here).

## Implementation Plan

### Approach

Minimal targeted edits to two files. The macro is still defined by CMakeLists.txt
during this ticket — it simply has no `#ifdef` sites left in these two files.

### Files to Modify

**`source/kinematics/IKinematics.h`**

Replace the entire `#ifdef ROBOT_DRIVETRAIN_MECANUM ... #endif` block with:

```cpp
// Differential-only build. To build for a mecanum robot:
//   1. Replace the include + alias + kWheelCount below with:
//        #include "MecanumKinematics.h"
//        namespace Kinematics = MecanumKinematics;
//        constexpr int kWheelCount = 4;
//   2. In source/main.cpp, replace NezhaHAL with MecanumHAL.
// (git history preserves the full mecanum integration prior to sprint 048.)
#include "BodyKinematics.h"
namespace Kinematics = BodyKinematics;
constexpr int kWheelCount = 2;
```

Update the file-level doc comment to remove references to `ROBOT_DRIVETRAIN_MECANUM`.

**`source/main.cpp`**

- Lines 3–7: Remove `#ifdef ROBOT_DRIVETRAIN_MECANUM` + `#include "MecanumHAL.h"` +
  `#else`/`#endif`; keep only `#include "NezhaHAL.h"`.
- Lines 17–19: Delete `#define WHEEL_TEST_MAIN 0` and the comment block above it.
- Lines 167–171: Remove `#ifdef ROBOT_DRIVETRAIN_MECANUM` HAL `#ifdef` block;
  keep only `static NezhaHAL hardware(uBit.i2c, uBit.io, cfg);`. Add comment:
  `// To build for mecanum: replace NezhaHAL with MecanumHAL and update IKinematics.h`
- Lines 179–186: Delete the entire `#if defined(ROBOT_DRIVETRAIN_MECANUM) &&
  WHEEL_TEST_MAIN` block (including the `wheelTestMain(...)` call).

### Testing Plan

- `uv run pytest` — all existing differential sim tests must pass.
- Inspect that `MecanumKinematics.h` still compiles in the sim build (it always
  compiled unconditionally; this is a no-op for it).
- Spot-check: `grep ROBOT_DRIVETRAIN_MECANUM source/kinematics/IKinematics.h source/main.cpp`
  must return zero matches.

### Documentation Updates

None beyond the in-code comments added as part of this ticket.
