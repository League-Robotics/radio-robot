---
id: '001'
title: Atomic source/io rename, file moves, include sweep, and build-file updates
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: reorganize-source-io-into-hal-comms-robot-configs.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 055-001: Atomic source/io rename, file moves, include sweep, and build-file updates

## Description

Rename `source/io/` to `source/hal/`, extract two CODAL-dependent robot-config
classes to `source/robot/`, and extract five communication transports to a new
`source/com/`. Then sweep all path-prefixed `#include "io/..."` lines to
`#include "hal/..."` and update both CMakeLists files.

This is a pure structural refactor — no behavior, logic, or interface changes.
All four steps must land together; any partial state breaks the build.

The consolidated architecture (`docs/architecture/architecture-034.md`) already
documents the target layout using `hal/`, `com/`, and `robot/`. This ticket
makes the physical directory tree match the docs.

## Acceptance Criteria

- [x] `uv run python -m pytest` passes with exactly **2361 passed, 2 failed**
      (the 2 pre-existing failures — `tag_offset_mm.z` schema mismatch — are
      not newly introduced or changed; use `uv run python -m pytest`, NOT
      `uv run pytest`, to avoid spurious collection errors)
- [x] `python build.py --clean` compiles and links the device firmware with zero
      missing-header or duplicate-symbol errors (use `--clean`; incremental
      builds go stale silently on `/Volumes`)
- [x] `grep -r 'include "io/' source/` returns no results
- [x] `grep -r '"io/' CMakeLists.txt tests/_infra/sim/CMakeLists.txt` returns
      no results

## Implementation Plan

### Approach

Use `git mv` throughout to preserve file history. The four steps below must all
land in one commit (or a single logical sequence) — the build is broken between
steps 1 and 4.

### Step 1 — Extract files before the directory rename

Extract the two robot-config classes and five transport pairs from
`source/io/real/` before renaming `source/io/`:

**Robot configs → `source/robot/`** (CODAL-dependent `Hardware` subclasses):
```
git mv source/io/real/NezhaHAL.h   source/robot/NezhaHAL.h
git mv source/io/real/NezhaHAL.cpp source/robot/NezhaHAL.cpp
git mv source/io/real/MecanumHAL.h   source/robot/MecanumHAL.h
git mv source/io/real/MecanumHAL.cpp source/robot/MecanumHAL.cpp
```

**Comms transports → `source/com/`** (new directory; no robot dependency):
```
mkdir source/com
git mv source/io/real/Communicator.h   source/com/Communicator.h
git mv source/io/real/Communicator.cpp source/com/Communicator.cpp
git mv source/io/real/Radio.h          source/com/Radio.h
git mv source/io/real/Radio.cpp        source/com/Radio.cpp
git mv source/io/real/RadioChannel.h   source/com/RadioChannel.h
git mv source/io/real/RadioChannel.cpp source/com/RadioChannel.cpp
git mv source/io/real/SerialPort.h     source/com/SerialPort.h
git mv source/io/real/SerialPort.cpp   source/com/SerialPort.cpp
git mv source/io/real/I2CBus.h         source/com/I2CBus.h
git mv source/io/real/I2CBus.cpp       source/com/I2CBus.cpp
```

Files that **stay** in `io/real/` (renamed to `hal/real/` in the next step):
`Motor`, `OtosSensor`, `ColorSensor`, `LineSensor`, `Servo`, `PortIO`,
`MotorBusDiagnostics`, `I2CBusRawAccess`, `BenchOtosSensor`.

### Step 2 — Rename the HAL directory

```
git mv source/io source/hal
```

After this step the tree is:
- `source/hal/` — `Hardware.h`, `NoopDevices.h`, `Sensor.h`, `ReplayHAL.{h,cpp}`,
  `capability/`, `real/` (devices only), `sim/`
- `source/com/` — five transport pairs
- `source/robot/` — now includes `NezhaHAL` and `MecanumHAL`

### Step 3 — Rewrite path-prefixed includes

Mechanical sweep of all ~52 `#include "io/..."` lines across `source/`:

```
grep -rl 'include "io/' source | xargs sed -i '' 's|include "io/|include "hal/|g'
```

Verify nothing remains:
```
grep -r 'include "io/' source/
```

Bare-filename includes (`#include "NezhaHAL.h"`, `#include "I2CBus.h"`) need no
change — they resolve through the include path. Sim files' relative
`../Hardware.h` includes are unaffected (the `hal/sim/` → `hal/` relationship is
preserved).

### Step 4 — Update the device firmware build (`CMakeLists.txt`)

Three `FILTER EXCLUDE` regex strings near lines 287–314:

| Old pattern | New pattern |
|-------------|-------------|
| `.*/io/sim/.*` | `.*/hal/sim/.*` |
| `.*/io/ReplayHAL\.cpp$` | `.*/hal/ReplayHAL\.cpp$` |
| `.*/io/real/MecanumHAL\.cpp$` | `.*/robot/MecanumHAL\.cpp$` |

Also update the comment block above these lines (lines ~250, 267–272, 282) that
references `io/` paths, for consistency.

### Step 5 — Update the sim/host build (`tests/_infra/sim/CMakeLists.txt`)

**Glob and explicit source paths** (lines ~78–98):
- `source/io/sim/*.cpp` → `source/hal/sim/*.cpp`
- `source/io/ReplayHAL.cpp` → `source/hal/ReplayHAL.cpp`
- `source/io/real/BenchOtosSensor.cpp` → `source/hal/real/BenchOtosSensor.cpp`

**Include directories** (lines ~115–118):
- `source/io` → `source/hal`
- `source/io/capability` → `source/hal/capability`
- `source/io/real` → `source/hal/real`
- `source/io/sim` → `source/hal/sim`
- **Add** `source/com` (so `I2CBus.h` and transport headers resolve for
  `MotorBusDiagnostics` and any transitively-included comms headers)

**New CODAL-exclusion FILTER lines** (add after the existing WedgeTest /
LoopScheduler exclusions, lines ~63–64): `NezhaHAL.cpp` and `MecanumHAL.cpp`
moved into `source/robot/` are now picked up by `ROBOT_SOURCES` but are
CODAL-dependent and must not compile host-side:
```cmake
list(FILTER ROBOT_SOURCES EXCLUDE REGEX ".*/NezhaHAL\\.cpp$")
list(FILTER ROBOT_SOURCES EXCLUDE REGEX ".*/MecanumHAL\\.cpp$")
```

Also update the comment block (lines ~19–24, 60–72) that references `io/` paths.

### Files to Create

- `source/com/` (new directory — created via `mkdir` before `git mv`)

### Files to Modify

- `CMakeLists.txt` — three FILTER EXCLUDE regexes + comments
- `tests/_infra/sim/CMakeLists.txt` — globs, explicit paths, include dirs,
  two new FILTER EXCLUDE lines, add `source/com` include dir

### Files Moved (git mv — history preserved)

From `source/io/real/` to `source/robot/`:
- `NezhaHAL.h`, `NezhaHAL.cpp`
- `MecanumHAL.h`, `MecanumHAL.cpp`

From `source/io/real/` to `source/com/`:
- `Communicator.h`, `Communicator.cpp`
- `Radio.h`, `Radio.cpp`
- `RadioChannel.h`, `RadioChannel.cpp`
- `SerialPort.h`, `SerialPort.cpp`
- `I2CBus.h`, `I2CBus.cpp`

Directory renamed:
- `source/io/` → `source/hal/` (all remaining contents move with it)

### Testing Plan

No new tests. Verification is build-gate-only — this is a pure structural
rename with no behavior changes.

1. **Host/sim suite** (fast, catches include-path and sim-CMake errors):
   ```
   uv run python -m pytest
   ```
   Expected: `2361 passed, 2 failed`. The 2 failures are pre-existing
   (`tag_offset_mm.z` schema mismatch). The reorg must not change this count.
   Use `uv run python -m pytest` — NOT `uv run pytest` — to avoid spurious
   collection errors from the wrong Python.
   If `pytest` is missing from `.venv`: `uv pip install pytest` first.

2. **Device firmware clean build** (catches step-3/4 regressions):
   ```
   python build.py --clean
   ```
   Must compile and link with no missing-header or duplicate-symbol errors.
   `--clean` is required; incremental builds go stale silently on `/Volumes`.

### Documentation Updates

Update the comment blocks in both CMakeLists files that reference `io/` paths
(inline documentation within the build files). No external docs need updating —
`architecture-034.md` already uses the target layout names.
