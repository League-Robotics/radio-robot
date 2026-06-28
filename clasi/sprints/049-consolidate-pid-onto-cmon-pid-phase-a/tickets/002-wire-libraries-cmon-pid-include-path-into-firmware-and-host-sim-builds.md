---
id: '002'
title: Wire libraries/cmon-pid/ include path into firmware and host-sim builds
status: open
use-cases:
  - SUC-001
depends-on:
  - '001'
github-issue: ''
issue: consolidate-control-code-onto-vendored-libraries-cmon-pid-tinyekf.md
completes_issue: false
---

# Wire libraries/cmon-pid/ include path into firmware and host-sim builds

## Description

Add `libraries/cmon-pid/` to the include search paths in both build systems so
that `#include "cmon-pid.h"` resolves without qualification in any source file
in either the ARM firmware build or the host simulation build.

The firmware CODAL build uses `CMakeLists.txt` at the repo root; the host sim
build uses `tests/_infra/sim/CMakeLists.txt`. Both must be updated.

## Acceptance Criteria

- [ ] `CMakeLists.txt` (root) contains an explicit
      `include_directories(${PROJECT_SOURCE_DIR}/libraries/cmon-pid)` line with
      an inline comment explaining it is the Sprint 049 vendored header.
- [ ] `tests/_infra/sim/CMakeLists.txt` contains
      `"${REPO_ROOT}/libraries/cmon-pid"` in the `target_include_directories`
      block for `firmware_host`.
- [ ] The host sim rebuilds without error after the include-path change.
- [ ] `uv run --with pytest python -m pytest tests/simulation -q` introduces no
      new failures beyond the 2 pre-existing baseline failures.

## Implementation Plan

### Approach

**Root CMakeLists.txt** (`/Volumes/Proj/proj/RobotProjects/radio-robot-elite/CMakeLists.txt`):
Insert near the existing `include_directories(${PROJECT_SOURCE_DIR}/${LIB_DEST})`
line (around line 209). Add after it:

```cmake
# libraries/cmon-pid: float-adapted vendored header (Sprint 049, Phase A).
# Added explicitly because cmon-pid is not a CODAL dependency managed via
# add_subdirectory — it is a plain vendored-in-place header.
include_directories(${PROJECT_SOURCE_DIR}/libraries/cmon-pid)
```

**tests/_infra/sim/CMakeLists.txt** (`/Volumes/Proj/proj/RobotProjects/radio-robot-elite/tests/_infra/sim/CMakeLists.txt`):
In the `target_include_directories(firmware_host PRIVATE ...)` block (starts
around line 110), add a new entry:

```cmake
    "${REPO_ROOT}/libraries/cmon-pid"
```

alongside the existing source-tree entries.

### Files to modify

- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/CMakeLists.txt`
- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/tests/_infra/sim/CMakeLists.txt`

### Testing plan

After the include-path changes, trigger a host sim rebuild:

```
cd /Volumes/Proj/proj/RobotProjects/radio-robot-elite/tests/_infra/sim
cmake -B build . && cmake --build build
```

Then run the canonical test suite:

```
uv run --with pytest python -m pytest tests/simulation -q
```

Expected: exactly 2 pre-existing failures unchanged. No source file has been
changed yet so no test should be affected.

### Documentation

The CMake comment added in the root CMakeLists.txt documents the rationale
inline. No other doc changes needed.
