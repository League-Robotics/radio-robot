---
id: '016'
title: 'Firmware: RobotState::pose single writer; retire StateEstimator from the loop'
status: done
use-cases:
- SUC-002
depends-on:
- '014'
github-issue: ''
issue: robot-state-pose-needs-exactly-one-writer.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware: RobotState::pose single writer; retire StateEstimator from the loop

**Worktree note**: implement in the git worktree checked out for
`sprint/128-complexity-reduction-delete-dead-code-unburden-the-robot-loop-fix-bit-rot`.
All paths below are relative to that worktree.

**Decision (sprint-planner call, not one of the two stakeholder-flagged
decisions — settled per the source issue's own recommendation)**:
**delete** `state_estimator.{h,cpp}` outright, rather than parking it.
`PoseTracker` (inside `Motion::Planner`) is the live fusion path; a
future estimator rebuild (`estimator-v2-otos-fusion-sim-first.md`) can
start fresh from the planner's own math rather than reviving this class.
If this turns out to be wrong (e.g. something depends on
`StateEstimator` this ticket's implementer discovers mid-work), stop and
flag it — don't silently switch to parking without recording why.

**Build-source-list requirement**: deleting `state_estimator.{h,cpp}`
removes another `.cpp` from `src/motion`. The build source lists exist
in **FOUR** places (root `CMakeLists.txt`, `src/motion/CMakeLists.txt`,
and the pytest `_APP_SOURCES` lists) — missing one is a link error, not a
compile error. Update all four explicitly, the same as tickets 014/015.

**Clean-build requirement**: this ticket changes `robot_state.h` (the
`pose` field's writer comment) and removes a per-cycle call from
`RobotLoop::tick()`. Build with `just build-clean`, not incremental.

## Description

`robot_state.h:164-169` documents `pose`'s writer as
`Motion::Odometry::integrate()`, "never OTOS-blended." But
`Planner::update()` (`planner.cpp:1247-1252`) ALSO writes `state.pose`,
from its own `PoseTracker`, which blends OTOS heading whenever
`limits_.headingOtosWeight > 0`. Telemetry happens to read the odom value
only because `tlm_.update()` runs before `planner_.tick()` in today's
cycle order — an accident of ordering, not a contract. Separately,
`StateEstimator::update()` runs every cycle (`robot_loop.cpp:536`)
computing a value with no consumer — its own header says so.
`robot_state.h:103-104` also still names `App::Drive::tick()` as
`cmdVelocity`'s writer, stale since the planner integration (ticket 014
already updates this comment as part of its own boundary-documentation
work — confirm it's current before re-editing here, don't clobber it).

## Acceptance Criteria

- [x] The `state.pose.*` write block in `Planner::update()`
      (`planner.cpp:1247-1252`) is deleted. `PoseTracker` remains the
      planner's internal working estimate; it is NOT wired to write
      `RobotState::pose` — if the wire ever needs `PoseTracker`'s output,
      that is future work through the existing `state.estimate` section,
      never by overwriting `pose`.
- [x] `Motion::Odometry::integrate()` (via `publishPose()`) is confirmed
      as `state.pose`'s ONE remaining writer.
- [x] The every-cycle `stateEstimator_.update(state_, ...)` call is
      removed from `RobotLoop::cycle()`/`tick()`.
- [x] `state_estimator.{h,cpp}` are deleted (per the settled decision
      above), along with their entries in all four build source lists.
- [x] `robot_state.h`'s `pose` field comment is fixed: writer is
      `Motion::Odometry::integrate()`, "never OTOS-blended" now stays
      TRUE once the planner's write is removed.
- [x] `robot_state.h`'s `cmdVelocity` writer comment names both writers
      and the ownership-switch mechanism, mirroring the existing accurate
      `Command.mode`/`moveActive` treatment — coordinate with ticket 014,
      which may have already written this comment as part of its own
      boundary documentation; do not produce a conflicting second edit.
- [x] `src/motion/DESIGN.md`'s estimator roster is updated: `Odometry` =
      telemetry trip odometer (the one remaining pose writer);
      `PoseTracker` = the planner's internal working estimate (the one
      that actually drives the robot, never wired to `RobotState::pose`);
      `StateEstimator` = deleted, no replacement in this tree (a future
      estimator rebuild is separate, tracked work, not part of this
      ticket).
- [x] `grep -n "state.pose" src/motion/planner/planner.cpp` shows reads
      only.
- [x] `grep -rn "stateEstimator_\." src/firm/app/` returns nothing.
- [x] A firmware test asserts a full cycle leaves `state.pose` equal to
      `Odometry`'s integration even with `headingOtosWeight > 0`
      configured (this is the regression guard for the exact hazard this
      ticket closes — a nonzero weight must no longer make telemetry
      silently flip pose sources).
- [x] `just build-clean` + `motion_tests` + planner ctest suite +
      firmware pytest tiers pass.
- [x] Bench: telemetry `pose` on the stand matches pre-change values for
      the standard smoke moves — the change removes an unread write, so
      drift here means something was live: stop and investigate. (Per
      this sprint's Test Strategy, the full hardware bench pass happens
      at the sprint-level gate after all firmware tickets land; a quick
      smoke here is cheap insurance if bench access is available during
      this ticket.)

## Testing

- **Existing tests to run**: `motion_tests`, planner ctest suite,
  firmware pytest tiers, full `just build-clean`.
- **New tests to write**: the `headingOtosWeight > 0` regression test
  named in the acceptance criteria above — a full-cycle test asserting
  `state.pose` matches `Odometry`'s integration regardless of the
  planner's internal `PoseTracker` blend weight.
- **Verification command**: `just build-clean`, then
  `cmake --build src/motion/build --target motion_tests`, then
  `uv run python -m pytest src/tests -q`.

## Implementation Notes

- **Approach**: remove the `Planner::update()` pose write first (the
  actual bug being fixed — telemetry's pose source becoming
  ordering-dependent), add the regression test, THEN remove
  `StateEstimator`'s per-cycle call and delete the class — sequencing
  the correctness fix ahead of the unrelated cleanup so a bisect, if ever
  needed, isolates them.
- **Files to modify/delete**: edit `src/motion/planner/planner.cpp`
  (delete the `state.pose.*` write block), `src/firm/app/robot_loop.cpp`
  or `.h` (remove the `stateEstimator_.update()` call site); delete
  `src/motion/state_estimator.{h,cpp}`; edit
  `src/firm/types/robot_state.h` (writer comments — coordinate with
  ticket 014), root `CMakeLists.txt`, `src/motion/CMakeLists.txt`,
  affected pytest `_APP_SOURCES` lists, `src/motion/DESIGN.md`.
- **Documentation updates**: `src/motion/DESIGN.md`'s estimator roster
  (this ticket's own acceptance criterion) — a deliverable, not optional.
