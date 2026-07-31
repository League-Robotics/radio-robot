---
id: '014'
title: 'Firmware: promote RobotState wheel targets as the firm/motion boundary; delete
  WheelSink/MoveQueue generation'
status: open
use-cases: [SUC-002]
depends-on: ['012']
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

- [ ] `src/motion/wheel_sink.h`, `move_queue.{h,cpp}`,
      `stop_condition.{h,cpp}`, `velocity_shaper.{h,cpp}` are deleted.
- [ ] `App::Drive`'s `WheelSink` overrides (`drive.h:125-132`,
      `drive.cpp:31-36`) are deleted.
- [ ] The `#include "motion/move_queue.h"` in `main.cpp:29` is removed.
- [ ] The corresponding `motion_tests` targets and `test_app_move_queue.py`
      are deleted.
- [ ] **All four build source lists are updated**: root `CMakeLists.txt`,
      `src/motion/CMakeLists.txt`, and every pytest `_APP_SOURCES` list
      referencing the deleted `.cpp` files (grep for each deleted
      filename across `src/tests/` to find every list, don't assume you
      found them all after the first two).
- [ ] `move_queue.cpp`'s 250-line land-at-zero margin derivation is
      copied to a dated design-history doc (e.g.
      `docs/design/history/land-at-zero-margin-derivation.md`) BEFORE the
      file is deleted — the history is preserved even though the code
      isn't.
- [ ] `RobotState::Wheel::cmdVelocity`'s own field comment in
      `robot_state.h` documents it as THE base↔motion actuation boundary:
      sole-or-arbitrated writer (`Motion::Planner::update()` when a Move
      owns motion, `App::Drive::update()` for WHEELS teleop, arbitrated
      by `RobotLoop`'s ordering), consumer (`RobotLoop::cycle()`), and a
      one-line note that `Motion::WheelSink` (the interface that used to
      be this seam) was deleted this sprint because it had no callers.
- [ ] `docs/design/design.md` §5 is rewritten: delete the
      `MoveQueue`/`setDuty()` story, describe `Motion::Planner` + the
      `RobotState` boundary.
- [ ] `src/firm/app/DESIGN.md` is rewritten to describe Drive's real
      second lifecycle (`command()`/`estop()`/`owns()`/`takeCompletion()`/
      `update()`).
- [ ] `src/motion/DESIGN.md` is rewritten: add `src/motion/planner/` (the
      larger, live half — its own CMake project, ctypes harness, ctest
      executables) to the tree's own orientation, replacing the
      `MoveQueue`-centric description.
- [ ] `CLAUDE.md`'s stale "the velocity PID is part of the frozen base"
      claim is fixed in the same pass.
- [ ] `grep -rn "WheelSink\|MoveQueue\|move_queue\|stop_condition\|velocity_shaper" src/firm src/motion`
      returns only the design-history doc reference.
- [ ] Full `just build-clean` + `motion_tests` + planner ctest suite +
      firmware pytest tiers pass.
- [ ] Bench smoke (`twist_drive.py`) behavior is unchanged — the deleted
      code was unwired, so ANY behavior change means the deletion cut
      something live: stop and investigate, do not proceed. (This bench
      check may be deferred to the sprint's own end-of-sprint bench gate
      rather than run per-ticket, per this sprint's Test Strategy — but
      if you have bench access during this ticket, a quick smoke is
      cheap insurance before moving to ticket 015/016.)

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
