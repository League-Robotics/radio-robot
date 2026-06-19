---
id: '002'
title: Move MotionController under source/superstructure/ and fix include paths and
  build system
status: in-progress
use-cases:
- SUC-004
depends-on:
- 042-001
github-issue: ''
issue: migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 042-002: Move MotionController under source/superstructure/ and fix include paths and build system

## Description

Move `source/control/MotionController.{h,cpp}` to `source/superstructure/MotionController.{h,cpp}`
using `git mv`. Bodies are verbatim — no changes to any logic, signatures, or comments.
Leave an alias shim at `source/control/MotionController.h` so existing `#include "MotionController.h"`
paths continue to compile without modification (deleted in Phase F).

Update both build systems:
- `tests/_infra/sim/CMakeLists.txt`: the `source/superstructure/` glob added in T1 already
  covers `MotionController.cpp` after the move. Confirm the `source/control/*.cpp` glob no
  longer picks up `MotionController.cpp` (it will not — file has been moved).
- Firmware build (`build.py` / firmware `CMakeLists.txt`): audit whether it globs
  `source/control/*.cpp` or uses an explicit file list. If glob: the move silently removes
  `MotionController.cpp` from the firmware build — add `source/superstructure/` to the
  firmware glob or add `MotionController.cpp` explicitly. Confirm by running the ARM build
  immediately after the move.

Update the vendor-confinement baseline: `MotionController` entries under `source/control/`
move to `source/superstructure/` in `tests/_infra/vendor_baseline.txt`.

After this ticket the Superstructure class created in T1 and MotionController live in the
same directory. No behavioral change — the shim ensures all callers resolve identically.

## Acceptance Criteria

- [x] `git mv source/control/MotionController.h source/superstructure/MotionController.h`
      and `git mv source/control/MotionController.cpp source/superstructure/MotionController.cpp`
      executed; bodies identical to pre-move (verified by `git diff`).
      Byte-exact: blob hashes unchanged (`3828e50…` / `2fe8bbd…`); rename detected, 0 line changes.
- [x] `source/control/MotionController.h` exists as an alias shim containing only
      `#pragma once` and `#include "../superstructure/MotionController.h"`.
- [x] Firmware build: `python3 build.py --fw-only` → 0 errors. Then
      `git checkout -- source/robot/DefaultConfig.cpp`.
      `MotionController.cpp.obj` built from `source/superstructure/`; `[100%] Built target MICROBIT`.
- [x] Simulation tier green: `uv run --with pytest python -m pytest -q` ≥ 2001 passed,
      0 errors. (`2001 passed in 34.39s`.)
- [x] Golden-TLM canary byte-exact. (`test_golden_tlm_unchanged` PASSED.)
- [x] `tests/_infra/vendor_baseline.txt` updated: MotionController entries point to
      `source/superstructure/` path. — N/A: baseline contains only 4 `source/app/` entries;
      MotionController has no vendor tokens, so it produces zero hits in either dir. No baseline
      change required (per ticket: "update if needed"; both `control` and `superstructure` are
      INSPECT_DIRS, gate stays green).
- [x] Vendor-confinement grep gate passes with `source/superstructure/` in INSPECT_DIRS.
      (`INSPECT_DIRS` already includes `superstructure` from T1; `test_vendor_confinement_no_new_leaks` PASSED.)
- [x] `field-pin` canary (`defaultRobotConfig()` diff) empty. (`test_default_robot_config_unchanged` PASSED.)

## Implementation Plan

### Approach

1. Run `git mv source/control/MotionController.h source/superstructure/MotionController.h`.
2. Run `git mv source/control/MotionController.cpp source/superstructure/MotionController.cpp`.
3. Create `source/control/MotionController.h` (the shim):
   ```cpp
   #pragma once
   #include "../superstructure/MotionController.h"
   ```
4. Check `source/superstructure/MotionController.h` — it currently includes
   `"BodyVelocityController.h"`, `"MotionCommand.h"`, `"Config.h"`, `"Protocol.h"`,
   `"RobotState.h"`, `"CommandTypes.h"` with relative paths (no directory prefix,
   relying on include-path flags). Verify the compiler still resolves these via
   `-I source/control` or the equivalent CMake `include_directories`. If the moved
   header uses `"BodyVelocityController.h"` (relative) and CMake adds `source/control`
   to the include path, no path fixup is needed in the moved header. Confirm by
   attempting a sim build immediately after the `git mv`.
5. Audit the firmware build system:
   - Open `build.py` and the firmware `CMakeLists.txt`.
   - If it uses `file(GLOB ... "source/control/*.cpp")`: add `source/superstructure/*.cpp`
     to the same glob or add `source/superstructure/MotionController.cpp` explicitly.
   - If it uses an explicit source list: add `source/superstructure/MotionController.cpp`
     and remove `source/control/MotionController.cpp` from the list.
6. Update `tests/_infra/vendor_baseline.txt`: replace any `source/control/MotionController`
   path entries with `source/superstructure/MotionController`.
7. Run ARM build (`python3 build.py --fw-only`) as the hard gate. On success, run
   `git checkout -- source/robot/DefaultConfig.cpp`.
8. Run full simulation tier.

### Files to Create

- `source/control/MotionController.h` (the shim — replaces the moved original)

### Files to Modify (via git mv)

- `source/control/MotionController.h` → `source/superstructure/MotionController.h`
- `source/control/MotionController.cpp` → `source/superstructure/MotionController.cpp`

### Files to Modify (content)

- Firmware `CMakeLists.txt` or `build.py` — add `source/superstructure/` to build glob
  or explicit source list (if needed; confirm by build attempt first)
- `tests/_infra/vendor_baseline.txt` — repoint MotionController entries

### Testing Plan

- `python3 build.py --fw-only` immediately after `git mv` and shim creation.
- `uv run --with pytest python -m pytest -q` full simulation tier.
- `test_golden_tlm.py` explicitly.
- `test_watchdog_exemption.py`, `test_goto_bounds.py`, `test_incident_scenarios.py`.
- `test_vendor_confinement.py` (grep gate).
- Field-pin canary.

### Documentation Updates

None beyond sprint artifacts.
