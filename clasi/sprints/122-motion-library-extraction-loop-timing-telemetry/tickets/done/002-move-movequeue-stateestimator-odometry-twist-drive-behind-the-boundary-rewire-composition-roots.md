---
id: '002'
title: Move MoveQueue/StateEstimator/Odometry/twist-Drive behind the boundary; rewire
  composition roots
status: done
use-cases:
- SUC-001
- SUC-002
depends-on:
- '001'
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

- [x] `move_queue.{h,cpp}`, `state_estimator.{h,cpp}`, `odometry.{h,cpp}`
      moved (`git mv`) from `src/firm/app/` into `src/motion/`.
- [x] `drive.*`'s twist half (`setTwist()`, the `BodyKinematics::inverse()`
      call) moves into `src/motion`; `src/firm/app/drive.*` keeps only
      `setWheels()`/`stop()`/`tick()`, now implementing ticket 001's
      boundary interface.
- [x] `Motion::MoveQueue` no longer holds a concrete `App::Drive&` —
      it holds the boundary interface type instead.
- [x] `App::RobotLoop`, `main.cpp`, and `src/sim/sim_harness.h`
      (`SimHarness`) are rewired to construct the moved objects from
      `src/motion` and wire them through the boundary; `RobotLoop::cycle()`
      dispatch logic (the schedule, the runAndWait blocks, the command
      switch) is otherwise textually unchanged.
- [x] `src/motion`'s include graph remains clean (only `messages/` from
      `src/firm`) after this larger move — re-verify the grep check from
      ticket 001.
- [x] `src/sim/CMakeLists.txt`'s `APP_SOURCES` no longer lists
      `move_queue.cpp`/`odometry.cpp`/`state_estimator.cpp` at their old
      path; they're covered by the new `src/motion` source list instead.
- [x] Root `CMakeLists.txt`'s ARM build links the now-larger `src/motion`
      tree correctly (re-verify after this ticket's move, not just
      ticket 001's smaller one).
- [x] `motion_tests` gains the end-to-end model-plant scenario: enqueue
      two chained moves against `wheel_plant.{h,cpp}` (reused from
      `src/tests/sim/plant/`) and verify the completion sequence —
      proving the boundary is sufficient with no `SimHarness`, no
      `libfirmware_host`, no Python.
- [x] **Refactor gate**: re-run the full sim/closure suite and diff
      against ticket 001's recorded baseline — every compared number is
      unchanged. Any discrepancy is a bug in this move, not an accepted
      side effect; do not close this ticket with an unexplained diff.
- [x] `firmware` (ARM), `motion_tests`, and `libfirmware_host` (sim) all
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

## Implementation Notes (closing)

**What moved, verbatim `git mv` + edit**: `move_queue.{h,cpp}`,
`odometry.{h,cpp}`, `state_estimator.{h,cpp}` from `src/firm/app/` into
`src/motion/`, each rewritten `namespace App` → `namespace Motion`.

**The boundary structs/methods settled on** (deliberately smaller than
ticket 001's own `WheelState`/`WheelSinkConfig` — see "Scope decisions"
below):
- `Motion::WheelSink` (ticket 001's header, now genuinely load-bearing):
  `App::Drive` implements it (`setWheels(v_left, v_right)`/`stop()`);
  `Motion::MoveQueue` holds a `WheelSink&` instead of a concrete `Drive&`.
- `Motion::Odometry`: constructor becomes
  `Odometry(float trackWidth, float initialLeftPosition, float initialRightPosition)`;
  `integrate()`/`reset()` take the current wheel positions as explicit
  float parameters instead of reading a held `Devices::Motor&`. The base
  (`App::RobotLoop::cycle()`) reads `motorL_.position()`/`motorR_.position()`
  at the exact point in the cycle `Odometry` used to read them itself, so
  the values flowing in are bit-identical, just handed in rather than
  pulled.
- `Motion::StateEstimator::Input` (new, nested struct): a plain
  field-for-field mirror of what `update()` used to read off
  `App::Telemetry::Frame` (`encLeft*`/`encRight*`/`pose*`/`twist*`/`otos*`),
  since `Motion::StateEstimator` may not depend on `App::Telemetry::Frame`.
  `App::RobotLoop` copies the same frame fields into an `Input` each cycle,
  immediately before calling `update()`.
- `App::applyOtosSample()` (OTOS-only perception step) split OUT of
  `odometry.cpp` into a NEW base-side file, `src/firm/app/otos_sample.{h,cpp}`
  — it needs `Devices::Otos&`/`App::Telemetry::Frame&`, neither of which
  `src/motion` may depend on. Zero logic change, pure relocation.
- `Motion::MoveQueue`: `enqueue()` gained an explicit `uint64_t now`
  parameter (it used to read `Devices::Clock::nowMicros()` off a held
  `const Devices::Clock&`, which `src/motion` may not depend on).
  `App::RobotLoop::handleMove()` now reads `clock_.nowMicros()` at the
  exact call site `MoveQueue::enqueue()` used to read it internally.
  `MoveQueue`'s twist path now calls `Motion::BodyKinematics::inverse()`
  itself (moving that computation from `App::Drive::tick()`'s deferred
  call to `MoveQueue::activate()`/`shapeAndStage()`'s immediate call) and
  hands the resulting `vLeft`/`vRight` to the sink via `setWheels()` —
  numerically identical, since nothing else touches `trackWidth`/`v_x`/
  `omega` between the old and new call points within a cycle.

**How `RobotLoop` shuttles data across the boundary each cycle**:
1. `odom_.integrate(motorL_.position(), motorR_.position())` (was
   `odom_.integrate()`).
2. `stateEstimator_.update(estimatorInput, nowMs)` where `estimatorInput`
   is a `Motion::StateEstimator::Input` built from `frame_`'s fields
   immediately before the call (was `stateEstimator_.update(frame_, nowMs)`).
3. `moveQueue_.enqueue(move, corrId, clock_.nowMicros())` from
   `handleMove()` (was `moveQueue_.enqueue(move, corrId)`).
4. `moveQueue_.tick(nowUs, odom_)` — unchanged signature (already took
   `now`/`const Odometry&` explicitly pre-ticket).

**Gate numbers vs. baseline** (ticket 001 recorded, on the clean sprint
branch tip before ticket 001's own moves; `0981773b`):
- Baseline: `uv run python -m pytest -q` → 1407 passed, 2 skipped,
  9 xfailed, 2 xpassed, 0 failed (~549s).
- Post-ticket-002 rerun: `uv run python -m pytest -q` → **1407 passed,
  2 skipped, 9 xfailed, 2 xpassed, 0 failed** (~486s) — numerically
  IDENTICAL. `uv run python3 build.py --clean` → both `firmware` (ARM,
  `MICROBIT.hex`) and `libfirmware_host` build green, same version
  `v0.20260723.4`.

**`motion_tests` run** (fresh configure + build, this ticket's larger
tree):
```
cmake -S src/motion -B src/motion/build
cmake --build src/motion/build --target motion_tests
```
Result: 3/3 tests passed (`motion_stop_condition_tests`,
`motion_velocity_shaper_tests`, and the new
`motion_move_queue_chained_tests` — two chained WHEELS+DISTANCE Moves
against `TestSim::WheelPlant`, verifying A completes, B chain-advances
the SAME `tick()` call with no intervening stop (SUC-051), then B
completes on its own baseline). No Python, no `libfirmware_host`, no
`SimHarness` anywhere in that binary's build or run path.

**Include-graph grep-verify** (re-run after this ticket's larger move):
`grep -rn '#include "' src/motion --include="*.h" --include="*.cpp" | grep -v 'messages/'`
shows only `motion/*` self-references — zero `devices/`, `app/`, `com/`,
`config/` includes anywhere in `src/motion`.

**Scope decisions (documented, not silently made)**:
- **`WheelState`/`WheelSinkConfig` (ticket 001's own boundary structs)
  are NOT used this ticket.** `Motion::Odometry`/`Motion::MoveQueue` take
  plain `float trackWidth`/position parameters directly instead of a
  bundled config/state struct. Rationale: the call-site count for this
  move turned out to be ~130+ construction/integrate/enqueue sites across
  10 test harnesses plus the 3 composition roots — bundling into
  `WheelSinkConfig` (whose own `linearAMax`/`angularAMax`/etc. fields
  don't match `MoveQueue`'s existing `ShaperLimits` field names 1:1)
  would have added a translation layer with no behavioral benefit, at
  real risk across that many call sites. `wheel_sink.h`'s `WheelSink`
  interface itself (the actually load-bearing half of the boundary) IS
  fully wired; `WheelState`/`WheelSinkConfig` remain declared, unused,
  available for a future ticket. Flagging for the team-lead/sprint-planner
  in case ticket 003/004 expected them consumed here.
- **`App::Drive` keeps its name and its `trackWidth()` accessor**
  (Open Question 2, left to implementer) even though it no longer does
  its own kinematics — `App::RobotLoop::updateTlm()` still needs
  `trackWidth` to fuse `frame_.twist` via `BodyKinematics::forward()`,
  and `Drive` was already the natural holder of that value.
- **`app_drive_harness.cpp`'s `setTwist()`/last-wins scenarios were
  DELETED, not adapted** (6 of 8 scenarios) — `App::Drive::setTwist()` no
  longer exists (the whole point of the move). Their coverage did not
  disappear: it moved WITH the code, into `app_move_queue_harness.cpp`'s
  own TWIST-Move scenarios (which already exercise
  `BodyKinematics::inverse()` end-to-end via `appliedDuty()` readback).
  The 2 remaining `app_drive_harness.cpp` scenarios (`setWheels()` stages
  raw values; `stop()` zeroes both targets) fully cover the narrowed
  `Drive` API.
- **`app_state_estimator_harness.cpp` no longer references `App::`
  at all** — its `makeFrame()` helper now builds a
  `Motion::StateEstimator::Input` directly and the file drops its
  `app/telemetry.h` include entirely, an incidental improvement (this
  harness was already advertised as having "no I2C bus/Devices:: leaf
  dependency"; now it has no `App::` dependency either).

**Gotcha hit, worth flagging**: building `motion_tests` standalone via
`cmake -S src/motion -B src/motion/build` leaves a `src/motion/build/`
directory that the ROOT ARM build's own `RECURSIVE_FIND_FILE` glob over
`src/motion` (root `CMakeLists.txt`) picks up — including CMake's own
`CompilerIdCXX` probe `.cpp` files, which then fail to compile
(`AvailabilityMacros.h` missing) and break the ARM `firmware` build. Hit
this firsthand: `uv run python3 build.py --clean` failed until
`src/motion/build/` was removed first. `src/motion/build/` IS gitignored
(bare `build` pattern, `.gitignore` line 1) so it never lands in a commit,
but `FILE(GLOB_RECURSE ...)` doesn't respect `.gitignore` — this is a
latent footgun for any future developer who builds `motion_tests` locally
in the same checkout as an ARM build. Not fixed here (would touch shared
root `CMakeLists.txt`/`util.cmake` beyond this ticket's remit) — flagging
for a follow-up issue: either `RECURSIVE_FIND_FILE` should exclude `build`
directories, or `src/motion/CMakeLists.txt`'s own doc comment should tell
developers to build motion_tests OUTSIDE the tree (e.g.
`cmake -B /tmp/motion-build`) or always `rm -rf src/motion/build` before
an ARM build.
