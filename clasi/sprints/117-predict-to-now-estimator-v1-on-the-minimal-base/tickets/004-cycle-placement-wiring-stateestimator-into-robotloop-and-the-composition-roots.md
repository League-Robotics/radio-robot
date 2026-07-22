---
id: '004'
title: "Cycle placement wiring — StateEstimator into RobotLoop and the composition roots"
status: open
use-cases:
- SUC-059
depends-on:
- '002'
- '003'
github-issue: ''
issue: predict-to-now-odometry-estimator-ring-capture-dump-validation-trajectory-controller.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Cycle placement wiring — StateEstimator into RobotLoop and the composition roots

## Description

Wire `App::StateEstimator` (ticket 002) — constructed from the baked
`Config::defaultEstimatorConfig()` (ticket 003) — into the two real
composition roots (`src/firm/main.cpp` and `src/sim/sim_harness.h`, the
same pattern `App::MoveQueue` followed in sprint 116) and into
`RobotLoop::cycle()`'s trailing `kPace` block. Per this sprint's overlay
(`design/DESIGN.md`, §2's new "Predict-to-now estimation" paragraph), the
call site is `stateEstimator_.update(frame_, nowUs)`, placed immediately
after `frame_.pose` is staged (i.e. after `applyOtosSample()` and
`odom_.integrate()` — the source issue's own "after applyOtosSample()/
odom_.integrate(), before pilot_.plan()" positioning, with `Pilot` long
deleted so this is simply the end of that block).

This is the integration ticket: it depends on BOTH ticket 002 (the class)
and ticket 003 (the baked config the constructor needs) landing first.
Bounded work only — pure float math over already-staged `frame_` data, no
new bus transaction, no new `runAndWait`/`sleepUntil`.

## Acceptance Criteria

- [ ] `RobotLoop`'s constructor gains a `StateEstimator&` parameter
      (mirrors how `MoveQueue&`/`Preamble&` etc. are already threaded
      through); `main.cpp` and `src/sim/sim_harness.h` each construct one
      `StateEstimator` instance from `Config::defaultEstimatorConfig()`
      and pass it in.
- [ ] `RobotLoop::cycle()`'s trailing `kPace` block calls
      `stateEstimator_.update(frame_, nowUs)` immediately after
      `frame_.pose = {odom_.x(), odom_.y(), odom_.theta()};` and before
      `updateLineColor(nowUs)`.
- [ ] `grep 'runAndWait\|sleepUntil' src/firm/app/robot_loop.cpp` is
      byte-for-byte unchanged by this ticket — confirms no new wait was
      introduced.
- [ ] A sim/unit test on `App::RobotLoop` (extending
      `app_robot_loop_harness.cpp`/`test_app_robot_loop.py`) asserts that
      after several cycles of motion, `stateEstimator_`'s wheel and body
      estimates are `valid = true` and track the commanded motion in the
      expected direction/magnitude.
- [ ] A bench-comparable timing check (measured the same way existing
      cycle-timing assertions in this test suite are measured) shows no
      regression in encoder-tracking-vs-commanded-speed accuracy
      attributable to the estimator's addition to the schedule.
- [ ] `RobotLoop::handleConfig()`'s `PatchKind::ESTIMATOR` branch (ticket
      003) is confirmed to reach the SAME `stateEstimator_` instance this
      ticket wires in (i.e. ticket 003's branch and this ticket's
      construction/wiring agree on one shared instance, not two).

## Implementation Plan

**Approach.** Additive, narrow-surface composition-root change: one new
constructor parameter, one new call in an already-existing `runAndWait`
body. No change to the schedule's timing constants (`kSettle`/`kClear`/
`kCycle`/`kPace`) — the estimator's `update()` call is bounded work
inside the EXISTING `kPace` block budget, the same way `applyOtosSample()`/
`odom_.integrate()`/`updateLineColor()` already share that block.

**Files to modify:**
- `src/firm/app/robot_loop.h` / `robot_loop.cpp` — constructor signature,
  `cycle()`'s `kPace` block call site.
- `src/firm/main.cpp` — construct `StateEstimator` from
  `Config::defaultEstimatorConfig()`; wire into `RobotLoop`'s
  constructor call.
- `src/sim/sim_harness.h` — same wiring for the host-build sim
  composition root (mirrors how `MoveQueue`/`Deadman` were wired
  previously).
- `src/tests/sim/unit/app_robot_loop_harness.cpp` /
  `test_app_robot_loop.py` — extend to exercise the new constructor
  parameter and assert post-cycle estimator state.

**Documentation updates:** none beyond what tickets 002/003's own
overlay/direct-edit content already covers — this ticket only wires
already-documented modules together.

## Testing

- **Existing tests to run**: `src/tests/sim/unit/test_app_robot_loop.py`,
  `src/tests/sim/system/` full sweep, full `uv run python -m pytest`.
- **New tests to write**: `RobotLoop` construction/wiring test asserting
  `stateEstimator_` is ticked every cycle and reaches `valid` state after
  warm-up; timing-regression check.
- **Verification command**: `uv run python -m pytest src/tests/sim/unit/test_app_robot_loop.py`;
  full suite `uv run python -m pytest`.
