---
id: '004'
title: Flip USE_ORDERED_TICK to default; full host suite green
status: done
use-cases:
- SUC-004
depends-on:
- '003'
github-issue: ''
issue: make-ordered-tick-the-default-close-parity-gaps.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Flip USE_ORDERED_TICK to default; full host suite green

## Description

With all 3 parity gaps closed (tickets 001-003), the ordered-tick path is now
feature-complete and correct. This ticket makes `USE_ORDERED_TICK` the compile-time
default so that no explicit flag is required to activate the ordered-tick path.

The sim build is controlled by `tests/_infra/sim/CMakeLists.txt`. Find where
compile definitions are set (likely a `target_compile_definitions` or
`add_compile_definitions` call) and add `-DUSE_ORDERED_TICK`.

The firmware build (for the micro:bit) has its own CMakeLists. Add
`-DUSE_ORDERED_TICK` there too. Check `CMakeLists.txt` at the repo root or
`source/CMakeLists.txt`.

After adding the define, the full host test suite is run WITHOUT any explicit
`-DUSE_ORDERED_TICK` flag on the command line — it should activate automatically
from the CMake definition.

This ticket confirms that the sprint's acceptance gate is met: the ordered-tick
path is live as the default, and all tests that previously ran against the legacy
path now run against the ordered-tick path and pass.

## Acceptance Criteria

- [x] `tests/_infra/sim/CMakeLists.txt` defines `USE_ORDERED_TICK` unconditionally.
- [x] The firmware `CMakeLists.txt` defines `USE_ORDERED_TICK` unconditionally.
- [x] The sim shared library rebuilds cleanly (`cd tests/_infra/sim && python3 build.py`).
- [x] `uv run python -m pytest` — green except the 2 known-baseline config-golden failures.
- [x] `test_golden_tlm.py` passes (using the regenerated capture from ticket 001).
- [x] `test_059_ordered_tick_parity.py` passes.
- [x] `test_planner_subsystem.py` passes.

## Implementation Plan

### Approach

1. Find the sim CMakeLists: `tests/_infra/sim/CMakeLists.txt`. Add
   `add_compile_definitions(USE_ORDERED_TICK)` (or equivalent). Rebuild the sim lib.
2. Find the firmware CMakeLists (repo root or `source/`). Add the same define.
3. Run the full host suite and confirm the acceptance gate.

### Files to modify

- `tests/_infra/sim/CMakeLists.txt` — add `USE_ORDERED_TICK` compile definition.
- Firmware `CMakeLists.txt` — add `USE_ORDERED_TICK` compile definition.

### Files to read first

- `tests/_infra/sim/CMakeLists.txt` — find the existing compile-definitions block.
- Repo-root `CMakeLists.txt` (or `source/CMakeLists.txt`) — firmware build defs.

### Testing plan

1. Rebuild: `cd tests/_infra/sim && python3 build.py`
2. Full suite: `uv run python -m pytest` — green except 2 known-baseline failures.
3. Spot-check: `uv run python -m pytest tests/simulation/unit/test_golden_tlm.py tests/simulation/unit/test_059_ordered_tick_parity.py -v`

### Documentation updates

None. The architecture-update.md documents the intent. The CMake change is self-documenting.
