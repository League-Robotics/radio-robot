---
id: '005'
title: Swap Odometry to EKFTiny and delete old EKF.h/EKF.cpp
status: open
use-cases: ["SUC-003"]
depends-on: ["050-004"]
github-issue: ''
issue: consolidate-control-code-onto-vendored-libraries-cmon-pid-tinyekf.md
completes_issue: false
---

# Swap Odometry to EKFTiny and delete old EKF.h/EKF.cpp

## Description

With parity confirmed in ticket 004, swap `Odometry`'s private `_ekf` member from
`EKF` to `EKFTiny` and delete the old `source/state/EKF.{h,cpp}` files. This is the
point at which TinyEKF becomes the live production EKF backend.

**This ticket must not begin until ticket 004 is in `done` status.**

### Changes required

**`source/control/Odometry.h`** (two lines only):
- Change `#include "state/EKF.h"` to `#include "state/EKFTiny.h"`.
- Change `EKF _ekf;` to `EKFTiny _ekf;`.
- No other changes — the public API of EKFTiny is identical to EKF.

**`source/control/Odometry.cpp`**: No changes required. All call sites use
method names that are identical between EKF and EKFTiny.

**`source/state/EKF.h`** and **`source/state/EKF.cpp`**: Delete both files.

**`source/state/PhysicalStateEstimate.{h,cpp}`**: No changes required (it delegates
to Odometry and does not reference EKF directly).

If any other file in `source/` references `EKF` by name (grep before deleting),
update those references too. Likely none, but verify.

### Verification that deletion is safe

Before deleting EKF.h and EKF.cpp:
1. `grep -rn "EKF" /path/to/source/ --include="*.h" --include="*.cpp"` — confirm
   the only references are in `Odometry.h` (which you are changing) and EKF.{h,cpp} itself.
2. After the Odometry.h change, `grep -rn "#include.*EKF.h"` — should return zero hits.

## Acceptance Criteria

- [ ] `source/state/EKF.h` does not exist.
- [ ] `source/state/EKF.cpp` does not exist.
- [ ] `source/control/Odometry.h` includes `"state/EKFTiny.h"` and declares `EKFTiny _ekf`.
- [ ] `source/control/Odometry.cpp` is unchanged.
- [ ] `source/state/PhysicalStateEstimate.{h,cpp}` are unchanged.
- [ ] `python build.py --clean` completes successfully (firmware + host-sim builds).
- [ ] `uv run --with pytest python -m pytest tests/simulation -q` shows exactly 2 failures (pre-existing config-schema tests) and no others.
- [ ] `test_vendor_confinement.py` remains green.

## Implementation Plan

### Approach

1. Grep for all `EKF` references in `source/` to confirm scope.
2. Edit `source/control/Odometry.h` — change include and field type.
3. Rebuild sim: `cd tests/_infra/sim/build && cmake .. && make` (or let pytest rebuild via `build_lib` fixture).
4. Run `uv run --with pytest python -m pytest tests/simulation -q` to confirm clean.
5. Delete `source/state/EKF.h` and `source/state/EKF.cpp`.
6. Rebuild again to confirm both the firmware (RECURSIVE_FIND_FILE no longer picks up EKF.cpp) and sim builds are clean.
7. Run full test suite again.

### Files to modify

- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/source/control/Odometry.h` — 2-line change

### Files to delete

- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/source/state/EKF.h`
- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/source/state/EKF.cpp`

### Testing plan

**Verification command:** `uv run --with pytest python -m pytest tests/simulation -q`

Expected: exactly 2 failures (config-schema), 0 new. Additionally, run:
`python build.py --clean` from the repo root to verify firmware build.

### Documentation updates

None required.
