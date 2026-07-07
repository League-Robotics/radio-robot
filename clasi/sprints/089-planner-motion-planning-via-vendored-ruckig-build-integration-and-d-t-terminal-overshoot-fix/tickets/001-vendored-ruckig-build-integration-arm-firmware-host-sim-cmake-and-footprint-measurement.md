---
id: '001'
title: Vendored-Ruckig build integration (ARM firmware + host-sim CMake) and footprint
  measurement
status: done
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

- [x] Repo-root `CMakeLists.txt` compiles `libraries/ruckig/src/*.cpp` into
      the ARM firmware image; a full ARM build (`just build-clean` or
      equivalent) succeeds with no new warnings/errors attributable to
      Ruckig.
- [x] `tests/_infra/sim/CMakeLists.txt`'s `firmware_host` target compiles
      the same sources and links with no errors.
- [x] Flash/RAM footprint delta from vendoring Ruckig into the ARM image is
      measured and recorded in this ticket's completion notes (nRF52833:
      512 KB flash / 128 KB RAM shared with CODAL — record both the
      absolute delta and whether it is a concern).
- [x] `tests/sim/unit/test_ruckig_smoke.py` still passes unmodified.
- [x] No `source/` file added or modified — this ticket is build-system
      only.
- [x] Full sim suite (`uv run pytest`) stays green — no regression from
      the CMake changes.

## Completion Notes (2026-07-07)

**Build integration.** Repo-root `CMakeLists.txt`: added
`include_directories(${PROJECT_SOURCE_DIR}/libraries/ruckig/include)` next
to the `cmon-pid`/`tinyekf` calls, plus a `file(GLOB RUCKIG_SOURCES
"${PROJECT_SOURCE_DIR}/libraries/ruckig/src/*.cpp")` /
`list(APPEND SOURCE_FILES ${RUCKIG_SOURCES})` immediately before the
`if("${SOURCE_FILES}" STREQUAL "")` guard. `tests/_infra/sim/CMakeLists.txt`:
the same `file(GLOB RUCKIG_SOURCES ...)` appended to `FIRMWARE_SOURCES`, and
`${REPO_ROOT}/libraries/ruckig/include` added to `firmware_host`'s
`target_include_directories()`. Exactly Decision 7's chosen approach — no
`add_subdirectory()`, no new CMake target. No `source/` file touched.

Both a clean ARM build (`just build-clean`) and the host-sim
`firmware_host` target compiled all 11 vendored Ruckig `.cpp` sources and
linked with **zero** warnings or errors attributable to Ruckig (the build
log's only "error"-substring hits are pre-existing vendor-SDK filenames like
`app_error_handler_gcc.c`, not actual diagnostics).

**Flash/RAM footprint (measured 2026-07-07, `arm-none-eabi-size` on
`build/MICROBIT` after `just build-clean`; nRF52833: 512 KB flash / 128 KB
RAM shared with CODAL, FLASH partition specifically is 364 KB):**

- **As committed by this ticket** (Ruckig compiled in, but no `source/` call
  site — ticket 002 is the first consumer): **delta = 0 bytes**, both flash
  and RAM. Confirmed byte-identical `arm-none-eabi-size` output before vs.
  after this ticket's CMake change (FLASH 177764 B / 47.69%, RAM 120768 B /
  98.33%, both builds) and zero Ruckig object code surviving in the linked
  ELF. Cause: the vendored codal `target.json`'s linker flags already carry
  `-Wl,--gc-sections` (combined with `-ffunction-sections -fdata-sections`),
  so with no reachable call site the entire vendored library is discarded at
  link time. **Not a concern** — this is the true, honest number for what
  this ticket alone changes.
- **Worst case once something calls it** — measured via a temporary,
  uncommitted scratch probe (a `scratch_ruckig_footprint_probe.cpp` calling
  `Ruckig<1>::calculate()` once, wired into `main()`, both deleted/reverted
  before this commit, per the ticket's own allowance for a "trivial,
  temporary … build-verification step"): **+151,512 bytes flash**
  (177764 B -> 329276 B, 47.69% -> 88.34% of the 364 KB FLASH region),
  **~0 bytes RAM delta** (120768 B unchanged — `Ruckig<1>`'s working state
  is fully stack-local via `std::array`, no heap, no added static/global
  instance). **This IS flagged as a real concern**: linking in *any* single
  call to `Ruckig<1>::calculate()` pulls in the full quartic/quintic
  position/velocity step-solver code (e.g. `position_third_step2.cpp` alone
  is 55 KB of source) rather than just the branch a given input takes, and
  it leaves only ~43 KB (11.7%) FLASH headroom once ticket 002 lands. This
  is expected to be a **one-time fixed cost** (the same `Ruckig<1>`
  instantiation is reused by every later Planner call site, so it should not
  multiply per call site), but every later ticket in this sprint should
  budget flash against this number, not against the ticket-001 zero-delta.
  Documented in `libraries/ruckig/README.vendored.md`'s new "Build
  integration" section.

**On-target solve-time (Open Question 4):** NOT measured by this ticket —
not one of this ticket's acceptance criteria (only the footprint delta is),
and no `source/` call site exists yet to time. Deferred to ticket 002 (the
first real consumer), which will have an actual call site to instrument.

**Tests.** `tests/sim/unit/test_ruckig_smoke.py` passes unmodified. Full
`uv run pytest`: 671 passed, 4 xfailed, 1 failed
(`tests/testgui/test_set_origin.py::test_set_origin_button_resets_fused_pose_to_world_origin_against_real_sim`,
"D never reached mode=I" within an 8s timeout). Verified this failure is
**pre-existing and unrelated to this ticket**: reproduced identically
(same assertion, same timeout) with the CMake changes stashed out and the
host-sim library rebuilt against the unmodified baseline. It is the exact
D-drive terminal-overshoot behavior this whole sprint exists to fix
(tickets 002+), not a regression from build-integration.

## Testing

- **Existing tests to run**: `uv run pytest` (full suite, sanity check
  that the CMake change doesn't break host-sim collection/build);
  `tests/sim/unit/test_ruckig_smoke.py` specifically.
- **New tests to write**: none — this ticket adds no `source/` behavior.
- **Verification command**: `uv run pytest tests/sim/unit/test_ruckig_smoke.py`
  then the full `uv run pytest`; separately, a clean ARM build via the
  project's standard build entry point.
