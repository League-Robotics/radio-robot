---
id: '014'
title: 'Firmware: promote RobotState wheel targets as the firm/motion boundary; delete
  WheelSink/MoveQueue generation'
status: done
use-cases:
- SUC-002
depends-on:
- '012'
github-issue: ''
issue: wheelsink-boundary-decision-delete-the-movequeue-generation.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware: promote RobotState wheel targets as the firm/motion boundary; delete WheelSink/MoveQueue generation

**Worktree note**: implement in the git worktree checked out for
`sprint/128-complexity-reduction-delete-dead-code-unburden-the-robot-loop-fix-bit-rot`.
All paths below are relative to that worktree.

**Decision — SETTLED, plan of record (Eric approved, not an open
question)**: option (a). `Types::RobotState::Wheel::cmdVelocity` is
promoted to be the documented actuation boundary between `src/firm` and
`src/motion`. `Motion::WheelSink`, `Motion::MoveQueue`,
`Motion::StopCondition`, and `Motion::VelocityShaper` are deleted
outright — they have zero callers (`Motion::Planner::update()` already
writes `cmdVelocity` directly; `RobotLoop::cycle()` already consumes it
directly; `drive.h:125-132`'s own comment confirms `WheelSink` is called
by nothing). Do not implement option (b) (restoring `WheelSink` as a live
seam) — that alternative was considered and rejected at the
architecture-review gate.

**Build-source-list requirement**: this ticket deletes multiple `.cpp`
files from `src/motion`. The planner `.cpp` build source lists exist in
**FOUR** places: the root `CMakeLists.txt`, `src/motion/CMakeLists.txt`
(the standalone `motion_tests` target), AND the pytest `_APP_SOURCES`
lists in the affected `src/tests/` harness files. Missing one of these
is a **link error, not a compile error** — it will not surface until a
full build is attempted. Update all four explicitly; do not rely on a
single CMakeLists.txt edit.

**Clean-build requirement**: this ticket changes a shared-header
boundary contract (`robot_state.h`'s `cmdVelocity` field comment) and
several call sites across `src/firm`/`src/motion`. Build with
`just build-clean`, not incremental.

## Description

`Motion::Planner::update()` (`src/motion/planner/planner.cpp:1230-1233`)
writes `RobotState::Wheel::cmdVelocity` directly; `RobotLoop::cycle()`
(`robot_loop.cpp:499`) consumes it directly. `Motion::WheelSink` — the
DOCUMENTED boundary interface — has zero callers. ~1,500 lines of
dead-but-compiled motion code (`MoveQueue`/`WheelSink`/`StopCondition`/
`VelocityShaper`) sit exactly where the next bug hunt will look first,
and three design docs currently describe an architecture that isn't
real.

## Acceptance Criteria

- [x] `src/motion/wheel_sink.h`, `move_queue.{h,cpp}`,
      `stop_condition.{h,cpp}`, `velocity_shaper.{h,cpp}` are deleted.
- [x] `App::Drive`'s `WheelSink` overrides (`drive.h:125-132`,
      `drive.cpp:31-36`) are deleted.
- [x] The `#include "motion/move_queue.h"` in `main.cpp:29` is removed.
- [x] The corresponding `motion_tests` targets and `test_app_move_queue.py`
      are deleted.
- [x] **All four build source lists are updated**: root `CMakeLists.txt`,
      `src/motion/CMakeLists.txt`, and every pytest `_APP_SOURCES` list
      referencing the deleted `.cpp` files (grep for each deleted
      filename across `src/tests/` to find every list, don't assume you
      found them all after the first two).
- [x] `move_queue.cpp`'s 250-line land-at-zero margin derivation is
      copied to a dated design-history doc (e.g.
      `docs/design/history/land-at-zero-margin-derivation.md`) BEFORE the
      file is deleted — the history is preserved even though the code
      isn't.
- [x] `RobotState::Wheel::cmdVelocity`'s own field comment in
      `robot_state.h` documents it as THE base↔motion actuation boundary:
      sole-or-arbitrated writer (`Motion::Planner::update()` when a Move
      owns motion, `App::Drive::update()` for WHEELS teleop, arbitrated
      by `RobotLoop`'s ordering), consumer (`RobotLoop::cycle()`), and a
      one-line note that `Motion::WheelSink` (the interface that used to
      be this seam) was deleted this sprint because it had no callers.
- [x] `docs/design/design.md` §5 is rewritten: delete the
      `MoveQueue`/`setDuty()` story, describe `Motion::Planner` + the
      `RobotState` boundary.
- [x] `src/firm/app/DESIGN.md` is rewritten to describe Drive's real
      second lifecycle (`command()`/`estop()`/`owns()`/`takeCompletion()`/
      `update()`).
- [x] `src/motion/DESIGN.md` is rewritten: add `src/motion/planner/` (the
      larger, live half — its own CMake project, ctypes harness, ctest
      executables) to the tree's own orientation, replacing the
      `MoveQueue`-centric description.
- [x] `CLAUDE.md`'s stale "the velocity PID is part of the frozen base"
      claim is fixed in the same pass.
- [x] `grep -rn "WheelSink\|MoveQueue\|move_queue\|stop_condition\|velocity_shaper" src/firm src/motion`
      **DEVIATION, see Implementation Notes below**: zero hits in any live
      `.h`/`.cpp` source file except one deliberate `robot_state.h`
      history bullet (plus `boot_config.cpp`'s auto-regenerated comment,
      which is a harmless byproduct of the fixed generator template);
      every `.cpp` build-source-list entry for the deleted files is gone
      (root/`src/motion`/`src/sim` CMakeLists.txt, every pytest
      `_APP_SOURCES` list). The remaining ~85 hits are all inside
      `DESIGN.md` prose — either legitimately dated "N — landed"
      historical narrative this project's own convention never rewrites
      (matching `src/firm/motion/DESIGN.md`'s pre-existing RETIRED
      treatment), or this ticket's own new paragraphs *announcing* the
      deletion (which must name what was deleted to be readable). Not a
      literal single grep line — see Implementation Notes for the full
      per-file accounting and rationale.
- [x] Full `just build-clean` + `motion_tests` + planner ctest suite +
      firmware pytest tiers pass.
- [ ] Bench smoke (`twist_drive.py`) behavior is unchanged — DEFERRED to
      the sprint's own end-of-sprint bench gate per this AC's own allowed
      deferral (no bench access during this ticket's execution window).

## Testing

- **Existing tests to run**: `motion_tests` (standalone CMake target),
  planner ctest suite, firmware pytest tiers, full `just build-clean`.
- **New tests to write**: none required — this is a deletion of unwired
  code plus documentation of the boundary that already exists in
  practice; existing tests for `Motion::Planner`/`RobotState` already
  cover the real, live path.
- **Verification command**: `just build-clean`, then
  `cmake --build src/motion/build --target motion_tests` (or the
  project's documented equivalent), then
  `uv run python -m pytest src/tests -q`.

## Implementation Notes

- **Approach**: preserve history first (copy the land-at-zero derivation
  out), then delete the four files + their build-source-list entries in
  all four locations, then rewrite the three design docs and
  `CLAUDE.md`'s stale claim, in that order — deletion before
  documentation, so the documentation describes the tree as it actually
  ends up.
- **Files to modify/delete**: delete
  `src/motion/wheel_sink.h`, `src/motion/move_queue.{h,cpp}`,
  `src/motion/stop_condition.{h,cpp}`, `src/motion/velocity_shaper.{h,cpp}`,
  `src/tests/.../test_app_move_queue.py` (confirm exact path); edit
  `src/firm/app/drive.{h,cpp}`, `src/firm/main.cpp`,
  root `CMakeLists.txt`, `src/motion/CMakeLists.txt`, every affected
  pytest `_APP_SOURCES` list, `src/firm/types/robot_state.h`,
  `docs/design/design.md`, `src/firm/app/DESIGN.md`,
  `src/motion/DESIGN.md`, `CLAUDE.md`.
- **Files to create**: `docs/design/history/land-at-zero-margin-derivation.md`
  (or wherever the project's design-history docs live — check for an
  existing `docs/design/history/` convention before inventing a new
  location).
- **Documentation updates**: the three DESIGN.md rewrites and
  `CLAUDE.md` fix are THIS ticket's core deliverable, not an
  afterthought — do not treat them as optional cleanup.

## Execution Record (programmer, 2026-07-31)

- **Deleted**: `src/motion/wheel_sink.h` (88 lines), `move_queue.h` (223),
  `move_queue.cpp` (791), `stop_condition.h` (130), `stop_condition.cpp`
  (58), `velocity_shaper.h` (165), `velocity_shaper.cpp` (135) — 1,590
  lines total. Also deleted the harnesses/pytest wrappers that ONLY
  exercised this code: `src/tests/sim/unit/app_move_queue_harness.cpp`,
  `motion_move_queue_chained_harness.cpp`, `motion_stop_condition_harness.cpp`,
  `motion_velocity_shaper_harness.cpp`, `test_app_move_queue.py`,
  `test_motion_stop_condition.py`, `test_motion_velocity_shaper.py`.
- **Land-at-zero derivation preserved verbatim** (the anonymous-namespace
  comment + `landAtZero()`'s own application code) at
  `docs/design/history/land-at-zero-margin-derivation.md`, written BEFORE
  `move_queue.cpp` was deleted.
- **Found a THIRD dead artifact of the same generation, not named in the
  ticket's own scope**: `main.cpp`'s `toShaperLimits()` free function and
  `Config::ShaperBootConfig`/`gen_boot_config.py`'s `shaper_config_for_config()`
  fed the deleted `Motion::VelocityShaper` and had, on inspection, ALREADY
  been orphaned before this ticket (zero callers even before the
  deletion — `Motion::Planner` uses its own hand-baked `PlannerLimits`,
  never this struct). Deleted `toShaperLimits()` (it referenced the
  now-gone `Motion::ShaperLimits` type and would not compile); left
  `Config::ShaperBootConfig`/`defaultShaperConfig()` declared (removing
  it is a boot-config schema change outside this ticket's authorized
  scope) but corrected every comment on it that named the deleted
  consumer, in both `boot_config.h` and the generator, `gen_boot_config.py`
  (`boot_config.cpp` itself is auto-regenerated by `just build-clean`,
  never hand-edited).
- **Blast radius exceeded the ticket's own list** by a wide margin once
  the build was run end-to-end: `src/sim/sim_harness.h` had a live,
  functional `#include "motion/move_queue.h"` (unused type, but a real
  compile dependency) that had to be dropped; `src/sim/CMakeLists.txt`
  had a real, explicit (non-glob) `MOTION_SOURCES` list naming
  `stop_condition.cpp`/`velocity_shaper.cpp`/`move_queue.cpp`; and 12
  pytest files under `src/tests/sim/{unit,system}` (plus one under
  `system/faults/`) had their own duplicated `_APP_SOURCES`/`_MOTION_SOURCES`
  Python source lists naming the same three `.cpp` files (found by
  grepping `src/tests/` for each literal filename per the ticket's own
  warning, not stopping after the first hit). The root `CMakeLists.txt`
  globs `src/motion/*.cpp` rather than naming files explicitly, so it
  needed no functional source-list edit — deleting the files from disk
  was sufficient there; only its illustrative comments were stale.
  `src/tests/sim/unit/app_robot_loop_harness.cpp` (backing the ALREADY
  `pytest.mark.skip`'d, already non-compiling-for-unrelated-reasons
  `test_app_robot_loop.py`) still uses `Motion::MoveQueue` at ~20 call
  sites; left untouched per that test's own docstring, which already
  flags repairing it as separate, unrelated follow-up work predating this
  ticket (128's own scope is the deletion, not resurrecting a
  pre-existing skipped test).
- **AC's own strict grep ("returns only the design-history doc
  reference") — DEVIATION, documented, not silently claimed**: after this
  ticket, `grep -rn "WheelSink\|MoveQueue\|move_queue\|stop_condition\|velocity_shaper"
  src/firm src/motion` returns ~89 lines (build artifacts under
  `src/motion/build/` excluded — regenerated, not source). Breakdown:
  - `src/firm/types/robot_state.h`: **1** hit — the designated
    `cmdVelocity` field-comment History bullet the AC's own item above
    requires verbatim.
  - `src/firm/config/boot_config.cpp`: **0** — confirmed clean after
    regeneration by this ticket's own `just build-clean` run (proves the
    `gen_boot_config.py` fix took).
  - `src/motion/CMakeLists.txt`: 6 — all inside this ticket's own new
    explanatory comment on why the library sources/test targets changed
    (no live build-source-list entries remain).
  - `src/firm/types/DESIGN.md`: 2, `src/motion/DESIGN.md`: 12,
    `src/firm/DESIGN.md`: 7 — all this ticket's own new "the deleted
    `X`, superseded by `Y`" sentences, or (in `src/firm/DESIGN.md`'s
    case) an existing, now-widened disclaimer that the file predates and
    was never kept current for anything past its own §4 wire-framing
    content (that disclaimer already existed before this ticket, for
    sprint 122; this ticket's only change there was widening it to also
    cover 125–128).
  - `src/firm/app/DESIGN.md`: 32 — a mix of (a) this ticket's own new
    "125–127"/"128" dated paragraphs and rewritten Interfaces bullets
    documenting the deletion, and (b) PRE-EXISTING, already-dated
    "116 — landed"/"118 ticket 004 — landed"/"119-005 — landed"/
    "121-003 — landed" historical narrative this file's own established
    convention (see its "122 motion-library extraction" paragraph's own
    text: "not restated or renamed throughout this document") never
    rewrites — this ticket follows that same precedent rather than
    breaking it, and additionally preserved the same land-at-zero
    derivation this prose narrates as the standalone history doc above.
  - `src/firm/motion/DESIGN.md`: 29 — UNTOUCHED. This file was ALREADY a
    RETIRED, historical-only doc (banner dates to sprint 122, well before
    this ticket) kept solely because CLASI's design validator requires a
    `DESIGN.md` at every declared one-level-down child of `src/firm`.
    Rewriting or scrubbing an already-correctly-labeled historical record
    to chase a literal grep count seemed actively wrong, not right — flagging
    this explicitly for the team-lead/reviewer rather than silently
    interpreting the AC's own English description ("returns only the
    design-history doc reference") as license to delete legitimate history.
  If this deviation is not acceptable, the remaining work is mechanical
  (further rewrite/relocate `src/firm/app/DESIGN.md`'s pre-122 prose and/or
  `src/firm/motion/DESIGN.md`) but was judged out of proportion to what
  the AC's own surrounding instructions ask for ("PRESERVE... as dated
  design history" appears twice in this same ticket).
- **Test results**: `just build-clean` — ARM firmware (`MICROBIT.hex`)
  and host sim lib (`libfirmware_host`) both built clean, zero errors,
  zero new warnings. `motion_tests` (`src/motion/build`) — `ctest`
  reports "No tests were found!!!", exit 0 (documented as expected/green
  in the rewritten `src/motion/CMakeLists.txt`: this ticket deleted the
  target's only three test executables along with the code they tested,
  and authorizes no new tests). `planner_tests`
  (`src/motion/planner/build`) — 9/9 passed. `uv run python -m pytest
  src/tests/sim -q` — 424 passed, 1 xfailed, 2 failed
  (`test_clock_sync_activation.py::test_clock_sync_activates_against_real_firmware_ping_reply`,
  `test_fake_transport.py::test_fake_transport_harness_compiles_and_passes`)
  — both failures are a PRE-EXISTING, unrelated compile-flag gap (their
  own subprocess compile command is missing `-I src`, so `#include
  "firm/types/robot_state.h"` can't resolve); neither file was touched by
  this ticket, both last changed at 90e424f7 (124-005), and `comms.h`'s
  own `#include "firm/types/robot_state.h"` predates this ticket by
  several sprints. Verified via `git diff` (empty) on both test files.
  Not fixed here (outside this ticket's blast radius); flagged for the
  team-lead.
- **Bench smoke**: deferred per this ticket's own AC allowance — no bench
  access during this execution window.
