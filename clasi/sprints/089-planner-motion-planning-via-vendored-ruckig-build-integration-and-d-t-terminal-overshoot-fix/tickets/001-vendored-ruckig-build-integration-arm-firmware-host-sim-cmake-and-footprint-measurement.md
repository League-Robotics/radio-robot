---
id: '001'
title: Vendored-Ruckig build integration (ARM firmware + host-sim CMake) and footprint
  measurement
status: open
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: planner-motion-planning-via-vendored-ruckig.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Vendored-Ruckig build integration (ARM firmware + host-sim CMake) and footprint measurement

## Description

`libraries/ruckig/` (16 headers + 11 core `.cpp` solver sources) is already
vendored and proven to compile under the firmware's exact flags via
`tests/sim/unit/test_ruckig_smoke.py`'s standalone subprocess harness — but
it is not yet part of either real build. Neither the repo-root
`CMakeLists.txt` (the ARM firmware image) nor `tests/_infra/sim/CMakeLists.txt`
(the `firmware_host` shared library the Python test harness loads) reaches
`libraries/ruckig/src/`: the ARM build's `RECURSIVE_FIND_FILE(SOURCE_FILES
...)` only scans `source/`, and the host-sim build's `FIRMWARE_SOURCES` list
is explicit and does not mention `libraries/ruckig`. This ticket is the hard,
mechanical prerequisite every other ticket in this sprint depends on
(architecture-update.md Decision 7, Migration Concerns).

## Implementation Plan

**Approach** (architecture-update.md Decision 7 — append to each build's
existing flat source list, not a new CMake target, matching the pattern
every other `source/` translation unit already uses; do NOT
`add_subdirectory()` Ruckig — its own build files were deliberately not
vendored):
1. Repo-root `CMakeLists.txt`: add
   `include_directories(${PROJECT_SOURCE_DIR}/libraries/ruckig/include)`
   next to the existing `cmon-pid`/`tinyekf` `include_directories()` calls
   (around line 221-226). Add a `file(GLOB RUCKIG_SOURCES
   "${PROJECT_SOURCE_DIR}/libraries/ruckig/src/*.cpp")` and
   `list(APPEND SOURCE_FILES ${RUCKIG_SOURCES})` before the
   `if("${SOURCE_FILES}" STREQUAL "")` guard.
2. `tests/_infra/sim/CMakeLists.txt`: the same two additions applied to
   `FIRMWARE_SOURCES` and `target_include_directories(firmware_host
   PRIVATE ...)`.
3. Build both targets (`just build-clean` or the project's ARM build entry
   point; the host-sim build via its own CMake invocation / `uv run
   pytest` collection) and confirm both link with no errors.
4. Measure and record the ARM image's flash/RAM footprint delta
   (before/after this change) — architecture-update.md Open Question 5.
   Use the project's existing size-reporting step (`arm-none-eabi-size` on
   the built ELF, or whatever the build already prints) rather than adding
   new tooling.
5. Do NOT write any code that USES Ruckig yet (`#include "ruckig/
   ruckig.hpp"` anywhere in `source/`) — this ticket is build-integration
   only; ticket 002 is the first consumer. A trivial, temporary
   `#include`-and-discard in a scratch file is acceptable ONLY as a
   build-verification step and must not be committed.

**Files to modify**: `CMakeLists.txt` (repo root), `tests/_infra/sim/
CMakeLists.txt`. No `source/` files change.

**Testing plan**: this ticket has no new pytest content of its own (no
`source/` behavior changes) — verification is that both CMake targets
configure and build cleanly, and that `tests/sim/unit/test_ruckig_smoke.py`
(the existing standalone-subprocess check) keeps passing unmodified,
proving the flags this ticket's integration uses match what that harness
already validated.

**Documentation updates**: `libraries/ruckig/README.vendored.md`'s "Build
constraints (verified)" section should note the two build targets Ruckig is
now integrated into (it currently only mentions the smoke test).

## Acceptance Criteria

- [ ] Repo-root `CMakeLists.txt` compiles `libraries/ruckig/src/*.cpp` into
      the ARM firmware image; a full ARM build (`just build-clean` or
      equivalent) succeeds with no new warnings/errors attributable to
      Ruckig.
- [ ] `tests/_infra/sim/CMakeLists.txt`'s `firmware_host` target compiles
      the same sources and links with no errors.
- [ ] Flash/RAM footprint delta from vendoring Ruckig into the ARM image is
      measured and recorded in this ticket's completion notes (nRF52833:
      512 KB flash / 128 KB RAM shared with CODAL — record both the
      absolute delta and whether it is a concern).
- [ ] `tests/sim/unit/test_ruckig_smoke.py` still passes unmodified.
- [ ] No `source/` file added or modified — this ticket is build-system
      only.
- [ ] Full sim suite (`uv run pytest`) stays green — no regression from
      the CMake changes.

## Testing

- **Existing tests to run**: `uv run pytest` (full suite, sanity check
  that the CMake change doesn't break host-sim collection/build);
  `tests/sim/unit/test_ruckig_smoke.py` specifically.
- **New tests to write**: none — this ticket adds no `source/` behavior.
- **Verification command**: `uv run pytest tests/sim/unit/test_ruckig_smoke.py`
  then the full `uv run pytest`; separately, a clean ARM build via the
  project's standard build entry point.
