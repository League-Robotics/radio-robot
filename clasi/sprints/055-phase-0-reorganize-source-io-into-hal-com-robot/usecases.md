---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 055 Use Cases

## SUC-001: Developer builds firmware and runs host tests after source tree reorganization

- **Actor**: Developer (CI system or human)
- **Preconditions**: The source tree reorganization (source/io → hal/com/robot) has
  been applied. Both CMakeLists.txt files have been updated. All path-prefixed
  includes have been rewritten.
- **Main Flow**:
  1. Developer runs `uv run python -m pytest` to build the firmware simulation
     library and execute the full host/sim test suite.
  2. Developer runs `python build.py --clean` to compile the device firmware from
     a clean state.
- **Postconditions**:
  - Host/sim test suite reports "2361 passed, 2 failed" (two pre-existing
    unrelated failures; count unchanged from pre-reorganization baseline).
  - Device firmware compiles and links with no missing-header or
    duplicate-symbol errors.
  - No source file under `source/` contains a `#include "io/..."` include.
  - No `CMakeLists.txt` file contains an `io/` path reference.
- **Acceptance Criteria**:
  - [ ] `uv run python -m pytest` passes with exactly 2361 passed, 2 failed
  - [ ] `python build.py --clean` completes with zero errors
  - [ ] `grep -r 'include "io/' source/` returns no results
  - [ ] `grep -r '"io/' CMakeLists.txt tests/_infra/sim/CMakeLists.txt` returns no results
