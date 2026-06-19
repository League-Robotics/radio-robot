---
id: '005'
title: Rename hal/ to io/; establish real/ sim/ subdirs; ROBOT_RUN_MODE CMake; ReplayHAL
  stub
status: done
use-cases:
- SUC-039-006
depends-on:
- 039-001
- 039-002
- 039-003
- 039-004
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# T5 — Rename hal/ to io/; establish real/ sim/ subdirs; ROBOT_RUN_MODE CMake; ReplayHAL stub

## Description

Perform the `source/hal/` → `source/io/` directory rename and establish the
`io/real/`, `io/sim/`, and `io/capability/` (already exists from T1) subdirectory
layout. Move files to their target subdirectories. Update all `#include` paths and
CMake source globs. Replace the `list(FILTER … EXCLUDE REGEX ".*/hal/mock/.*")`
pattern with `ROBOT_RUN_MODE` (SIM | REAL | REPLAY). Add a `ReplayHAL` stub.

This ticket is atomic: ALL file moves, ALL include-path updates, and ALL CMake changes
must happen in a single commit so that the suite does not pass through a partially-
moved state. The verification command is run ONCE at the end.

**This ticket has NO algorithmic changes.** Every file is moved verbatim; only paths
in `#include` directives and CMake glob patterns change.

**Host-verifiable:** The sim build (CMakeLists.txt update) is host-verifiable.
**ARM build:** The firmware CMake / `build.py` is ARM-verifiable; textually verify if
no ARM toolchain. All files that previously compiled under `hal/` still compile under
`io/real/` (same content, new path).

## Approach

### Step 1 — Audit current include structure

Before moving, grep for all `#include "hal/` and `#include "../hal/` patterns to know
every include that must be updated:

```bash
grep -rn '#include.*hal/' source/ tests/
grep -rn '#include.*"I2CBus\|"Motor\|"NezhaHAL\|"MockHAL\|"Mock\|"Bench\|"Otos\|"Servo\|"Line\|"Color\|"PortIO\|"Hardware' source/
```

The CMake include directories currently list `source/hal` and `source/hal/mock`.
After the move, these become `source/io`, `source/io/capability`, `source/io/real`,
`source/io/sim`.

### Step 2 — File moves (git mv to preserve history)

Move real (firmware) device files to `source/io/real/`:
```
git mv source/hal/Motor.h         source/io/real/
git mv source/hal/Motor.cpp        source/io/real/
git mv source/hal/OtosSensor.h    source/io/real/
git mv source/hal/OtosSensor.cpp  source/io/real/
git mv source/hal/NezhaHAL.h      source/io/real/
git mv source/hal/NezhaHAL.cpp    source/io/real/
git mv source/hal/I2CBus.h        source/io/real/
git mv source/hal/I2CBus.cpp      source/io/real/
git mv source/hal/LineSensor.h    source/io/real/
git mv source/hal/LineSensor.cpp  source/io/real/
git mv source/hal/ColorSensor.h   source/io/real/
git mv source/hal/ColorSensor.cpp source/io/real/
git mv source/hal/Servo.h         source/io/real/
git mv source/hal/Servo.cpp       source/io/real/
git mv source/hal/PortIO.h        source/io/real/
git mv source/hal/PortIO.cpp      source/io/real/
git mv source/hal/BenchOtosSensor.h   source/io/real/
git mv source/hal/BenchOtosSensor.cpp source/io/real/
git mv source/hal/MotorBusDiagnostics.h   source/io/real/   # created in T1
git mv source/hal/MotorBusDiagnostics.cpp source/io/real/
```

Move comms files to `source/io/real/` (they are HAL-level real-hardware comms):
```
git mv source/hal/Communicator.h/.cpp   source/io/real/
git mv source/hal/SerialPort.h/.cpp     source/io/real/
git mv source/hal/Radio.h/.cpp          source/io/real/
git mv source/hal/RadioChannel.h/.cpp   source/io/real/
```

Move `Hardware.h` to `source/io/`:
```
git mv source/hal/Hardware.h  source/io/
```

Move shim headers to `source/io/` (they are backward-compat aliases, not canonical):
```
git mv source/hal/IMotor.h     source/io/
git mv source/hal/IServo.h     source/io/
git mv source/hal/IOtosSensor.h source/io/
git mv source/hal/ILineSensor.h source/io/
git mv source/hal/IColorSensor.h source/io/
git mv source/hal/IPortIO.h    source/io/
git mv source/hal/Sensor.h     source/io/
```

Move mock (sim) files to `source/io/sim/`:
```
git mv source/hal/mock/MockHAL.h/.cpp        source/io/sim/
git mv source/hal/mock/MockMotor.h/.cpp      source/io/sim/
git mv source/hal/mock/MockOtosSensor.h/.cpp source/io/sim/
git mv source/hal/mock/MockLineSensor.h/.cpp source/io/sim/
git mv source/hal/mock/MockColorSensor.h/.cpp source/io/sim/
git mv source/hal/mock/MockServo.h/.cpp      source/io/sim/
git mv source/hal/mock/MockPortIO.h/.cpp     source/io/sim/
```

After moves, `source/hal/` should be empty. Remove it:
```
rmdir source/hal/mock source/hal
```

### Step 3 — Update #include paths in all source files

The CMake include directories will be updated (step 5), so files that include via bare
filename (e.g., `#include "Motor.h"`) will continue to work as long as `source/io/real/`
is in the include path. However, files that use `#include "hal/Motor.h"` (path-qualified)
must be updated. Similarly, files in `source/control/` that include `#include "IMotor.h"`
are fine as long as `source/io/` is in the include path (the shim is there).

Check for any path-qualified includes (e.g., `#include "../hal/..."` or
`#include "hal/..."`) and update them. Flat-name includes like `#include "Motor.h"`
just need the CMake include-path to be updated.

### Step 4 — Add ReplayHAL stub

Create `source/io/ReplayHAL.cpp`:
```cpp
// ReplayHAL.cpp — stub for ROBOT_RUN_MODE=REPLAY.
// Phase F will implement TLM replay; for now, this compiles as an empty HAL.
// Included only when ROBOT_RUN_MODE=REPLAY in CMake.
#include "Hardware.h"
// RobotMode::REPLAY stub: empty impl satisfying the linker.
```
And `source/io/ReplayHAL.h` with a minimal empty `ReplayHAL : public Hardware`.
All Hardware pure-virtual methods have stub no-op implementations.

### Step 5 — Update CMakeLists.txt (sim build)

**`tests/_infra/sim/CMakeLists.txt`** — full rewrite of source collection:

```cmake
set(ROBOT_RUN_MODE "SIM" CACHE STRING "Build target: REAL | SIM | REPLAY")

# Collect common sources (app/, control/, robot/)
file(GLOB APP_SOURCES     "${REPO_ROOT}/source/app/*.cpp")
file(GLOB CONTROL_SOURCES "${REPO_ROOT}/source/control/*.cpp")
file(GLOB ROBOT_SOURCES   "${REPO_ROOT}/source/robot/*.cpp")

# CODAL-only exclusions (same as before, just kept)
list(FILTER APP_SOURCES     EXCLUDE REGEX ".*/WedgeTest\\.cpp$")
list(FILTER CONTROL_SOURCES EXCLUDE REGEX ".*/LoopScheduler\\.cpp$")

# SIM mode: include io/sim/ sources; exclude CODAL-only real/ sources
file(GLOB SIM_SOURCES "${REPO_ROOT}/source/io/sim/*.cpp")

add_library(firmware_host SHARED
    ${APP_SOURCES}
    ${CONTROL_SOURCES}
    ${ROBOT_SOURCES}
    ${SIM_SOURCES}
    "${REPO_ROOT}/source/io/real/BenchOtosSensor.cpp"   # needed in HOST_BUILD
    "${CMAKE_SOURCE_DIR}/sim_api.cpp"
)

target_include_directories(firmware_host PRIVATE
    "${REPO_ROOT}/source"
    "${REPO_ROOT}/source/app"
    "${REPO_ROOT}/source/control"
    "${REPO_ROOT}/source/io"
    "${REPO_ROOT}/source/io/capability"
    "${REPO_ROOT}/source/io/real"
    "${REPO_ROOT}/source/io/sim"
    "${REPO_ROOT}/source/robot"
    "${REPO_ROOT}/source/types"
)
```

Remove the old `list(FILTER MOCK_SOURCES EXCLUDE REGEX ".*/hal/mock/.*")` pattern
(it no longer exists — `SIM_SOURCES` already globs only the sim dir).

### Step 6 — Update firmware CMake (build.py path)

The firmware build's CMakeLists.txt (root-level or `source/CMakeLists.txt`) currently
globs `source/hal/` and excludes mock. After the move, update it to glob
`source/io/real/` for REAL mode. Pass `-DROBOT_RUN_MODE=REAL` from `build.py`. This is
an ARM-build concern; update textually and verify with `python3 build.py` if available.

### Step 7 — Update vendor-confinement gate

In `tests/simulation/unit/test_vendor_confinement.py`, update the scope to check
"above `source/io/`" (instead of "above `source/hal/`"). The vendor tokens should only
appear in `source/io/real/`, `source/io/sim/`, `source/io/capability/` — not in
`source/control/`, `source/app/`, `source/robot/`, or `source/types/`.

Update the grep target paths accordingly. Regenerate `vendor_baseline.txt` if needed
(it should be empty or near-empty after T1–T4 sealed the leaks).

### Step 8 — Verify atomicity

Before committing, run the full suite to confirm zero regressions. Check that `source/hal/`
does not exist (`ls source/hal` should fail).

## Files to Create

- `source/io/ReplayHAL.h` — empty stub
- `source/io/ReplayHAL.cpp` — empty stub

## Files to Move (git mv)

All files under `source/hal/` and `source/hal/mock/` as listed in Step 2.
`source/hal/Hardware.h` → `source/io/Hardware.h`.
Shim headers → `source/io/`.
Real device files → `source/io/real/`.
Mock files → `source/io/sim/`.

## Files to Modify

- `tests/_infra/sim/CMakeLists.txt` — `ROBOT_RUN_MODE=SIM`; glob `io/sim/`; update include paths; remove hal/mock filter
- `build.py` — update `cmake -S` target if needed; pass `ROBOT_RUN_MODE=REAL`
- Firmware-level CMakeLists (if separate) — glob `io/real/` instead of `hal/`
- Any source file with path-qualified `#include "hal/..."` — update to `"io/..."` or rely on flat-name include with updated include path
- `tests/simulation/unit/test_vendor_confinement.py` — update scope from "above source/hal/" to "above source/io/"
- `tests/_infra/vendor_baseline.txt` — confirm empty or regenerate

## Acceptance Criteria

- [x] `source/hal/` directory does not exist.
- [x] `source/io/capability/` contains all 8 headers (7 + Pose2D.h).
- [x] `source/io/real/` contains all real-device files (Motor, OtosSensor, NezhaHAL, I2CBus, Servo, BenchOtosSensor, MotorBusDiagnostics, Communicator, SerialPort, Radio, RadioChannel, LineSensor, ColorSensor, PortIO).
- [x] `source/io/sim/` contains all mock files (MockHAL, MockMotor, MockOtosSensor, MockLineSensor, MockColorSensor, MockServo, MockPortIO).
- [x] `source/io/Hardware.h`, shim `I*.h` headers, `Sensor.h` live at `source/io/`.
- [x] `source/io/ReplayHAL.h/.cpp` stub exists and compiles.
- [x] `tests/_infra/sim/CMakeLists.txt` uses `ROBOT_RUN_MODE=SIM`; no `hal/mock` filter.
- [x] Vendor-confinement test scope updated to "above `source/io/`".
- [x] Vendor-confinement canary passes.
- [x] Golden-TLM canary passes byte-exact.
- [x] `defaultRobotConfig()` field-pin unchanged.
- [x] Simulation tier green: `uv run --with pytest python -m pytest -q` — count >= 1957.
- [x] No new heap allocation or fiber introduced.
- [x] **Sprint 039 DoD check:** all items from the sprint.md `## Definition of Done` list are met (T1–T4 items verified satisfied in the current tree; the alias `I*.h` shims and the vendor-gate boundary-token tightening are deliberately DEFERRED to Phase F per architecture-update.md §2, which schedules shim deletion for Phase F — the shims were moved to `source/io/`, not deleted).

## Testing Plan

- This ticket must be committed and tested ATOMICALLY — do not run intermediate tests
  on a partially-moved tree.
- Run `uv run --with pytest python -m pytest -q` (full simulation tier).
- Run all three canaries:
  - `uv run --with pytest python -m pytest tests/simulation/unit/test_vendor_confinement.py -v`
  - `uv run --with pytest python -m pytest tests/simulation/unit/test_default_config_pin.py -v`
  - `uv run --with pytest python -m pytest tests/simulation/unit/test_golden_tlm.py -v`
- If ARM toolchain present: `python3 build.py --clean` — confirms real device files compile from `io/real/`.
- **ARM-only files:** Everything in `source/io/real/` — particularly `NezhaHAL.cpp`,
  `Motor.cpp`, `LoopScheduler.cpp`. Verify textually that include paths are consistent.
- After passing all tests, confirm `ls source/hal` returns "no such file or directory".
