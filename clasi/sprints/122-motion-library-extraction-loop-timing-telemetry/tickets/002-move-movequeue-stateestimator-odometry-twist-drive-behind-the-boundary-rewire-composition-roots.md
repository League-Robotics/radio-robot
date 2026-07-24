---
id: '002'
title: Move MoveQueue/StateEstimator/Odometry/twist-Drive behind the boundary; rewire
  composition roots
status: open
use-cases: [SUC-001, SUC-002]
depends-on: ["001"]
github-issue: ''
issue: extract-motion-library-to-src-motion.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Move MoveQueue/StateEstimator/Odometry/twist-Drive behind the boundary; rewire composition roots

## Description

Second and final half of the mechanical extraction. With `src/motion`,
its CMakeLists, and the boundary header already standing (ticket 001),
move the remaining four responsibility groups — `move_queue.*`,
`state_estimator.*`, `odometry.*`, and the twist-decomposition half of
`drive.*` (`setTwist()` + its `BodyKinematics` call) — into `src/motion`,
rewire `App::RobotLoop`, `main.cpp`, and `SimHarness` construction to
compose base + motion through the boundary from ticket 001, and prove
zero behavior change against the baseline ticket 001 recorded.

`App::Drive` (base-side) keeps `setWheels()`/`stop()`/`tick()` — the
wheel-target sink — and now implements ticket 001's boundary interface so
`Motion::MoveQueue` can hold it by that interface instead of by concrete
`Drive&`. `Motion::MoveQueue` calls `BodyKinematics::inverse()` itself
(that math moved to `src/motion` in ticket 001) for the twist path, then
hands the resulting `vLeft`/`vRight` down through the boundary sink.

**Locked scope reminders (see sprint.md Design Rationale Decisions 1-2):**
- Boundary stays a velocity sink. Do not introduce a duty type here.
- `devices/velocity_pid.*` and `NezhaMotor`'s PID ownership are untouched.
- `RobotLoop`'s existing per-object references (Drive&, Odometry&,
  MoveQueue&, StateEstimator&, etc.) do NOT need to collapse into a
  single composed object this sprint — sprint.md's architecture review
  explicitly notes that reference-web collapse is sprint 2's job
  (`docs/design/base-explicit-loop-sketch.md`'s "no reference webs"
  principle), not required here. Only include paths and construction
  change, per the extraction issue's own instruction.

## Acceptance Criteria

- [ ] `move_queue.{h,cpp}`, `state_estimator.{h,cpp}`, `odometry.{h,cpp}`
      moved (`git mv`) from `src/firm/app/` into `src/motion/`.
- [ ] `drive.*`'s twist half (`setTwist()`, the `BodyKinematics::inverse()`
      call) moves into `src/motion`; `src/firm/app/drive.*` keeps only
      `setWheels()`/`stop()`/`tick()`, now implementing ticket 001's
      boundary interface.
- [ ] `Motion::MoveQueue` no longer holds a concrete `App::Drive&` —
      it holds the boundary interface type instead.
- [ ] `App::RobotLoop`, `main.cpp`, and `src/sim/sim_harness.h`
      (`SimHarness`) are rewired to construct the moved objects from
      `src/motion` and wire them through the boundary; `RobotLoop::cycle()`
      dispatch logic (the schedule, the runAndWait blocks, the command
      switch) is otherwise textually unchanged.
- [ ] `src/motion`'s include graph remains clean (only `messages/` from
      `src/firm`) after this larger move — re-verify the grep check from
      ticket 001.
- [ ] `src/sim/CMakeLists.txt`'s `APP_SOURCES` no longer lists
      `move_queue.cpp`/`odometry.cpp`/`state_estimator.cpp` at their old
      path; they're covered by the new `src/motion` source list instead.
- [ ] Root `CMakeLists.txt`'s ARM build links the now-larger `src/motion`
      tree correctly (re-verify after this ticket's move, not just
      ticket 001's smaller one).
- [ ] `motion_tests` gains the end-to-end model-plant scenario: enqueue
      two chained moves against `wheel_plant.{h,cpp}` (reused from
      `src/tests/sim/plant/`) and verify the completion sequence —
      proving the boundary is sufficient with no `SimHarness`, no
      `libfirmware_host`, no Python.
- [ ] **Refactor gate**: re-run the full sim/closure suite and diff
      against ticket 001's recorded baseline — every compared number is
      unchanged. Any discrepancy is a bug in this move, not an accepted
      side effect; do not close this ticket with an unexplained diff.
- [ ] `firmware` (ARM), `motion_tests`, and `libfirmware_host` (sim) all
      build green.

## Testing

- **Existing tests to run**: full `uv run pytest` (`src/tests/sim/`,
  including `app_move_queue_harness`/`app_state_estimator_harness`/
  `app_odometry_harness`/`app_drive_harness`/`app_robot_loop_harness`
  suites, now compiled against the new `src/motion` path where
  applicable); ARM `firmware` build; `libfirmware_host` (sim) build.
- **New tests to write**: the `motion_tests` two-chained-moves end-to-end
  scenario (new `.cpp` scenario file under `src/motion`'s test sources,
  linking `wheel_plant.cpp`).
- **Verification command**: `uv run pytest`; fresh `cmake --build` +
  run of `motion_tests`; diff against ticket 001's baseline artifact.

## Implementation Plan

**Approach**: move the four remaining pieces with `git mv` to preserve
history, adjust every `#include` path in both moved and base-side files,
change `RobotLoop`/`main.cpp`/`SimHarness` construction to build the
moved objects from their new namespace/location and pass the boundary
interface where a concrete `Drive&` was passed before, then run the full
before/after parity check.

**Files to create**:
- `src/motion/move_queue.{h,cpp}`, `state_estimator.{h,cpp}`,
  `odometry.{h,cpp}` (moved)
- The twist-decomposition piece split out of `drive.cpp` (new file under
  `src/motion`, name at implementer's discretion — e.g.
  `src/motion/twist_decomposition.{h,cpp}` or folded directly into
  `move_queue.cpp` if that reads more cohesively; sprint.md's module
  boundary is normative, not the exact file split)
- New `motion_tests` scenario source for the two-chained-moves
  end-to-end test

**Files to modify**:
- `src/firm/app/drive.{h,cpp}` (shrink to wheel-target sink; implement
  boundary interface)
- `src/firm/app/robot_loop.{h,cpp}` (construction signature/includes)
- `src/firm/main.cpp` (construction wiring)
- `src/sim/sim_harness.h` (construction wiring)
- `src/sim/CMakeLists.txt` (drop the four moved files from `APP_SOURCES`)
- Root `CMakeLists.txt` (re-verify `src/motion` glob covers the larger tree)
- `src/motion/CMakeLists.txt` (add the four moved modules +
  the new end-to-end scenario source)

**Testing plan**: baseline diff is the primary gate (see Acceptance
Criteria). Run the full suite immediately before this ticket's final
commit and immediately after; the two runs' numeric outputs must match
ticket 001's originally recorded baseline. Build and run `motion_tests`
standalone to confirm the new scenario passes with no Python/sim-library
involvement.

**Documentation updates**: none required by this ticket beyond what's
needed to keep the build green — full `DESIGN.md`/`docs/design/design.md`
reconciliation is ticket 004's job, deliberately sequenced after this
ticket so it documents the FINAL post-move shape rather than an
intermediate state.
