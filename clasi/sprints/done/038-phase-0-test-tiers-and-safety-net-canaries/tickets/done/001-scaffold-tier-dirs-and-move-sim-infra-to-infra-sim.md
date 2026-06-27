---
id: '001'
title: Scaffold tier dirs and move sim infra to _infra/sim/
status: done
use-cases:
- SUC-001
- SUC-002
depends-on: []
github-issue: ''
issue: migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Scaffold tier dirs and move sim infra to _infra/sim/

## Description

Create the Sprint 038 §7 tier directory scaffold and move the sim build infrastructure
(`tests/sim/`) into `tests/_infra/sim/`. This is the foundation move that all subsequent
tickets depend on. Fix the REPO_ROOT cmake path and update all five path-wiring files
atomically. Verify the full simulation suite stays green before committing.

**Why this ticket first**: the sim infra move is the most disruptive path change —
it touches cmake, conftest, build.py, and sim_conn.py. Doing it while the test files
remain in `tests/unit/` (pyproject.toml unchanged) gives the smallest blast radius.

## Implementation Plan

### Step 1: Scaffold empty tier directories

Create these directories with `.gitkeep` files so they are committable:
```
tests/simulation/unit/
tests/simulation/system/
tests/bench/unit/
tests/bench/system/
tests/field/unit/
tests/field/system/
tests/_infra/
tests/_infra/sim/
```

### Step 2: Move sim infra (`git mv`)

```bash
git mv tests/sim/CMakeLists.txt  tests/_infra/sim/CMakeLists.txt
git mv tests/sim/sim_api.cpp     tests/_infra/sim/sim_api.cpp
git mv tests/sim/firmware.py     tests/_infra/sim/firmware.py
```

Inspect `tests/sim/conftest.py` — if it only duplicates root conftest fixtures, delete it.
If it has unique content, merge into `tests/conftest.py` first, then delete.

### Step 3: Fix REPO_ROOT in CMakeLists.txt

`tests/_infra/sim/` is 3 levels below the repo root (was 2). Change line:

```cmake
# Before (2 hops from tests/sim/):
get_filename_component(REPO_ROOT "${CMAKE_SOURCE_DIR}/../.." ABSOLUTE)

# After (3 hops from tests/_infra/sim/):
get_filename_component(REPO_ROOT "${CMAKE_SOURCE_DIR}/../../.." ABSOLUTE)
```

Verify by confirming `source/` resolves correctly from the new path.

### Step 4: Update `tests/conftest.py`

```python
# Before:
_SIM_DIR   = _TESTS_DIR / "sim"

# After:
_SIM_DIR   = _TESTS_DIR / "_infra" / "sim"
```

`_BUILD_DIR = _SIM_DIR / "build"` follows automatically. The `build_lib` fixture
cmake arguments derive from these constants — no other fixture edits needed.

### Step 5: Update `build.py` `build_host_sim()`

Change cmake source and build dir references from `tests/sim` / `tests/sim/build`
to `tests/_infra/sim` / `tests/_infra/sim/build` (both the `-S` and `-B` arguments
and any summary printouts).

### Step 6: Update `host/robot_radio/io/sim_conn.py` `_DEFAULT_LIB`

Current value (from sprint 037 arch-update): resolves relative to `host/robot_radio/io/`
as `../../../tests/sim/build/`. Change to `../../../tests/_infra/sim/build/`.

### Step 7: Verify green suite

`pyproject.toml` is NOT changed in this ticket (testpaths still `["tests"]`). Run:
```
uv run --with pytest python -m pytest -q
```
Confirm: cmake build succeeds at new paths; `sim` + `sim_field_profile` fixtures work;
test count ≥ 1954 passed, 0 errors. Only commit once green.

## Files to Create

- `tests/simulation/unit/.gitkeep`, `tests/simulation/system/.gitkeep`
- `tests/bench/unit/.gitkeep`, `tests/bench/system/.gitkeep`
- `tests/field/unit/.gitkeep`, `tests/field/system/.gitkeep`
- `tests/_infra/.gitkeep` (or let `_infra/sim/` serve as anchor)

## Files to Move (`git mv`)

- `tests/sim/CMakeLists.txt` → `tests/_infra/sim/CMakeLists.txt`
- `tests/sim/sim_api.cpp` → `tests/_infra/sim/sim_api.cpp`
- `tests/sim/firmware.py` → `tests/_infra/sim/firmware.py`

## Files to Modify

- `tests/_infra/sim/CMakeLists.txt` — REPO_ROOT 2→3 hops
- `tests/conftest.py` — `_SIM_DIR` constant
- `build.py` — `build_host_sim()` cmake paths
- `host/robot_radio/io/sim_conn.py` — `_DEFAULT_LIB`

## Files to Delete

- `tests/sim/conftest.py` — inspect first; delete if redundant

## Acceptance Criteria

- [x] `tests/_infra/sim/CMakeLists.txt` exists; REPO_ROOT computed with 3 `..` hops.
- [x] `tests/_infra/sim/sim_api.cpp` and `tests/_infra/sim/firmware.py` exist.
- [x] `tests/conftest.py` `_SIM_DIR` points to `tests/_infra/sim/`.
- [x] `build.py` references `tests/_infra/sim/` in `build_host_sim()`.
- [x] `sim_conn.py` `_DEFAULT_LIB` resolves to `tests/_infra/sim/build/`.
- [x] All tier scaffold dirs exist: `simulation/unit/`, `simulation/system/`,
      `bench/unit/`, `bench/system/`, `field/unit/`, `field/system/`.
- [x] `uv run --with pytest python -m pytest -q` passes ≥ 1954 tests, 0 errors.
- [x] `git diff source/` is empty.

## Testing Plan

```bash
# Primary verification (must stay green):
uv run --with pytest python -m pytest -q

# Spot-check build path:
python3 build.py
ls tests/_infra/sim/build/libfirmware_host.*

# Spot-check a sim-dependent test:
uv run --with pytest python -m pytest tests/unit/test_sim_realtime.py -v
```

No new tests are written in this ticket.
