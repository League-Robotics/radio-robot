---
id: '006'
title: Scrub test-infra drive2/mc2 scaffolding names (C ABI + Python, atomic rename)
status: open
use-cases:
- SUC-006
depends-on:
- "005"
github-issue: ''
issue: internalize-legacy-motioncontroller-into-planner.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 006 — Scrub test-infra drive2/mc2 scaffolding names (C ABI + Python, atomic rename)

## Description

The test infrastructure retains sprint-059/060 scaffolding names that should be
canonical: `drive2_api_*` C-ABI symbols, `bus_drain_api_drive2_*` symbols,
`Drive2Ctx` Python struct, and test filenames `test_drive2_subsystem.py` /
`test_motioncontroller2_smoke.py`. Python ctypes binds C functions by string
name — so the C++ `extern "C"` symbol rename and the Python ctypes call-site
update MUST happen in the same commit. This ticket renames everything atomically.

### Changes

1. **`tests/_infra/sim/drive2_api.cpp`** -> **`tests/_infra/sim/drive_api.cpp`**:
   - Rename the file (git mv).
   - Rename all `drive2_api_*` function names to `drive_api_*` in the C++
     `extern "C"` block.
   - Update the file-level comment.
   - Add `drive_api.cpp` to `CMakeLists.txt` sim target; remove `drive2_api.cpp`.

2. **`tests/_infra/sim/bus_drain_api.cpp`**:
   - Rename `bus_drain_api_drive2_get_fused_x` -> `bus_drain_api_drive_get_fused_x`
     and any other `bus_drain_api_drive2_*` symbols.
   - Update the comment on line 14 referencing `drive2`.

3. **`tests/simulation/unit/test_drive2_subsystem.py`** ->
   **`tests/simulation/unit/test_drive_subsystem.py`**:
   - Rename the file (git mv).
   - Replace all `lib.drive2_api_*` ctypes bindings with `lib.drive_api_*`.
   - Rename the `Drive2Ctx` Python ctypes struct to `DriveCtx` (including all
     instantiation sites in the file).
   - Update the module docstring.

4. **`tests/simulation/unit/test_motioncontroller2_smoke.py`** ->
   **`tests/simulation/unit/test_planner_subsystem_smoke.py`**:
   - Rename the file (git mv).
   - Update module docstring (calls `planner_api_*` already — no binding changes).
   - Update any internal reference to `MotionController2`.

5. **`tests/simulation/unit/test_059_ordered_tick_parity.py`**:
   - Find any `drive2_api_*` ctypes bindings or `Drive2Ctx` references.
   - Update to `drive_api_*` / `DriveCtx`.

6. **`tests/_infra/sim/config_routing_api.cpp`**:
   - Line 212: update comment from "drive2_api pattern" to "drive_api pattern".
   - Find any `drive2_api_*` function calls and update to `drive_api_*`.

7. **Confirm `tests/simulation/unit/test_059_config_routing.py`**:
   - Check methods `test_drive2_vel_kp_non_zero_at_init` and
     `test_drive2_planner_sensors_default_consistent`.
   - If they bind C symbols via `drive2_api_*` ctypes, update the bindings AND
     rename the Python test methods.
   - If they are pure Python test method names with no C binding, rename the
     methods for consistency (drop the `drive2` prefix).

### What stays the same

- `planner_api.cpp` and `planner_api_*` symbols — already canonical.
- `tests/simulation/unit/test_planner_subsystem.py` — already canonical.
- `tests/simulation/unit/test_motioncontroller2_smoke.py` functions that call
  `planner_api_*` — bindings unchanged; only the filename and docstring change.

## Acceptance Criteria

- [ ] `tests/_infra/sim/drive2_api.cpp` does not exist;
      `tests/_infra/sim/drive_api.cpp` exists.
- [ ] All `drive2_api_*` C++ `extern "C"` symbols are renamed `drive_api_*`.
- [ ] All `bus_drain_api_drive2_*` C++ symbols are renamed `bus_drain_api_drive_*`.
- [ ] `tests/simulation/unit/test_drive2_subsystem.py` does not exist;
      `tests/simulation/unit/test_drive_subsystem.py` exists.
- [ ] `tests/simulation/unit/test_motioncontroller2_smoke.py` does not exist;
      `tests/simulation/unit/test_planner_subsystem_smoke.py` exists.
- [ ] `Drive2Ctx` does not appear in any `.py` file.
- [ ] `grep -rIn "drive2_api\|bus_drain_api_drive2\|Drive2Ctx\|mc2" tests/`
      returns zero hits.
- [ ] `cmake --build build_sim` succeeds with zero errors.
- [ ] `uv run python -m pytest` passes (all except 2 baseline config-golden).

## Implementation Plan

### Approach

Do C++ renames first (file rename + symbol rename + CMakeLists.txt), then do
Python renames (file renames + ctypes binding updates). Rebuild after C++
changes. Run the full test suite at the end.

IMPORTANT: The C++ file rename and Python binding update must be committed
atomically. Do not commit the C++ rename without the Python update — the tests
will fail between those two states.

### Files to modify / rename

- `tests/_infra/sim/drive2_api.cpp` -> `drive_api.cpp` (rename + symbol changes)
- `tests/_infra/sim/bus_drain_api.cpp` (symbol renames)
- `tests/_infra/sim/config_routing_api.cpp` (comment + possible call sites)
- CMakeLists.txt (update file reference)
- `tests/simulation/unit/test_drive2_subsystem.py` -> `test_drive_subsystem.py`
  (rename + binding updates + DriveCtx rename)
- `tests/simulation/unit/test_motioncontroller2_smoke.py` ->
  `test_planner_subsystem_smoke.py` (rename + docstring)
- `tests/simulation/unit/test_059_ordered_tick_parity.py` (binding updates)
- `tests/simulation/unit/test_059_config_routing.py` (method name review)

### Testing plan

After all changes:
```
cmake --build build_sim && uv run python -m pytest
```
Expected: all pass except 2 baseline config-golden failures.

### Documentation updates

Update file-level comments in all renamed/modified files.
