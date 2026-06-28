---
id: '002'
title: Wire libraries/tinyekf include dirs into both CMakeLists.txt build paths
status: open
use-cases: ["SUC-001"]
depends-on: ["050-001"]
github-issue: ''
issue: consolidate-control-code-onto-vendored-libraries-cmon-pid-tinyekf.md
completes_issue: false
---

# Wire libraries/tinyekf include dirs into both CMakeLists.txt build paths

## Description

Add `libraries/tinyekf` to the include search path in both build systems
(firmware and host-sim), mirroring the cmon-pid pattern from Sprint 049.
After this ticket, any source file can `#include <tinyekf.h>` (after
`#define EKF_N` and `#define EKF_M`) and the include will resolve in both builds.

This ticket makes no source-code changes — it only modifies the two CMakeLists.txt
files.

### Root `CMakeLists.txt` change

After the existing cmon-pid entry at line 213:
```cmake
include_directories(${PROJECT_SOURCE_DIR}/libraries/cmon-pid)
```
Add:
```cmake
# libraries/tinyekf: header-only EKF LA backend (Sprint 050, Phase B).
include_directories(${PROJECT_SOURCE_DIR}/libraries/tinyekf)
```

### `tests/_infra/sim/CMakeLists.txt` change

Inside `target_include_directories(firmware_host PRIVATE ...)`, after the existing cmon-pid entry:
```cmake
    # 049-002: cmon-pid float-adapted vendored header (Sprint 049, Phase A).
    "${REPO_ROOT}/libraries/cmon-pid"
```
Add:
```cmake
    # 050-002: TinyEKF header-only LA backend (Sprint 050, Phase B).
    "${REPO_ROOT}/libraries/tinyekf"
```

## Acceptance Criteria

- [ ] Root `CMakeLists.txt` contains `include_directories(${PROJECT_SOURCE_DIR}/libraries/tinyekf)` immediately after the cmon-pid line.
- [ ] `tests/_infra/sim/CMakeLists.txt` contains `"${REPO_ROOT}/libraries/tinyekf"` inside `target_include_directories(firmware_host PRIVATE ...)` immediately after the cmon-pid entry.
- [ ] No other changes to either CMakeLists.txt.
- [ ] `uv run --with pytest python -m pytest tests/simulation -q` still passes with no new failures beyond the 2 pre-existing baseline.

## Implementation Plan

### Approach

Minimal additive edits to exactly two CMakeLists.txt files. The pattern is a copy
of the cmon-pid entries with `tinyekf` substituted. No source or test changes.

### Files to modify

- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/CMakeLists.txt` — add one `include_directories` line after line 213.
- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/tests/_infra/sim/CMakeLists.txt` — add one path inside `target_include_directories`, after the cmon-pid entry at line 131.

### Testing plan

**Verification command:** `uv run --with pytest python -m pytest tests/simulation -q`

The sim build rebuilds automatically when the CMakeLists changes. Confirm no new
failures beyond the 2 pre-existing baseline. EKFTiny does not exist yet, so the
include is not exercised until ticket 003.

### Documentation updates

None required beyond the inline CMake comments added in this ticket.
