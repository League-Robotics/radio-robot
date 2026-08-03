---
id: '004'
title: 'positionEpoch consumers: Odometry + planner pose channels re-anchor across
  rebaseline'
status: open
use-cases: [SUC-131-004]
depends-on: []
github-issue: ''
issue: A-position-rebaseline-destroys-the-pose.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# positionEpoch consumers: Odometry + planner pose channels re-anchor across rebaseline

## Description

`App::RobotLoop::publishWheel()` (`robot_loop.cpp:364-382`) rebaselines a
wheel once its raw position crosses +/-30,000 mm and bumps
`positionEpoch`. The same cycle's `odom_.integrate(motorL_.position(),
motorR_.position())` (`robot_loop.cpp:541`) feeds the ALREADY-rebaselined
absolute position into `Motion::Odometry::integrate()`
(`odometry.cpp:17-21`), which computes a bare delta against last cycle's
pre-rebaseline value — a ~-30,000 mm step: heading jumps ~234 rad, x/y by
~15 m. `Motion::Planner`'s `PoseTracker::integrate()`
(`estimation.cpp:32-57`) has the identical raw-delta shape, so a Move in
flight across the boundary is corrupted too. `positionEpoch` was invented
for exactly this and is grep-verified to have ZERO motion-side consumers
today — only `App::Telemetry` copies it onto the wire.

Give `positionEpoch` real consumers: both `Motion::Odometry` and the
planner's `PoseTracker`/`WheelChannel` track the last epoch they have seen
per wheel, and when a call arrives with a changed epoch, re-anchor their
own last-position baseline (using each type's existing reset-style
primitive — `Odometry::reset()`, `PoseTracker::reset()` — as the pattern to
follow) instead of differencing across the discontinuity. Fix the
secondary defect in the same ticket: `Devices::NezhaMotor::softRebaseline()`
currently sets `velocity_ = 0.0f`, so the wheel reports 0 mm/s for 1-2
cycles mid-motion purely because of a software re-anchor — remove that
line so `velocity_` holds its last computed value across the boundary.

## Acceptance Criteria

- [ ] `Motion::Odometry::integrate()` re-anchors `lastLeft_`/`lastRight_`
      to the current position (crediting zero delta that call) exactly on
      the cycle a wheel's `positionEpoch` changes; it differences
      normally on every other cycle. Left and right wheels can rebaseline
      independently (each `publishWheel()` call is per-wheel), so each
      wheel's epoch is tracked separately.
- [ ] `Motion::PoseTracker`/`Motion::WheelChannel` (whichever holds the
      raw-delta shape consuming positions in `Motion::Planner::tick()`)
      does the same.
- [ ] `NezhaMotor::softRebaseline()` no longer sets `velocity_ = 0.0f`; a
      test asserts wheel velocity does not read 0 during the tick a
      rebaseline fires.
- [ ] Sim pose-sanity test: drive a wheel past +/-30,000 mm (equivalent
      of a ~75s/400mm/s soak); assert `state.pose.x/y/heading` show no
      discontinuity across the triggering cycle and total accumulated
      travel matches the commanded distance within existing sim exactness
      tolerance.
- [ ] Sim test: a Move in flight AT the rebaseline boundary (trigger the
      margin crossing mid-Move) completes at the correct place (within
      existing exactness tolerance), not corrupted by the epoch change.
- [ ] A soak-equivalent sim run past 30 m of cumulative travel shows no
      pose discontinuity anywhere in the logged trajectory (not just at
      one engineered boundary).
- [ ] Every existing call site of the two `integrate()` methods
      (`App::RobotLoop::cycle()`, `Motion::Planner::tick()`) is updated to
      pass the new epoch argument; any sim/test harness constructing
      `Types::RobotState` directly and calling these methods supplies a
      stable epoch (0) so pre-existing tests keep their current behavior
      unless explicitly exercising the new epoch-change path.
- [ ] Full sim suite stays green.

## Testing

- **Existing tests to run**: `Motion::Odometry` unit tests,
  `Motion::Planner`'s `estimation_test.cpp`/`pose_ownership_test.cpp`,
  full `src/tests/sim` suite.
- **New tests to write**:
  - Sim pose-sanity test across the rebaseline boundary (heading/x/y
    continuity, travel-matches-commanded).
  - Sim test: Move in flight at the boundary completes correctly.
  - Sim/firmware test: wheel velocity does not read 0 during a rebaseline
    tick.
  - Sim soak test past 30 m with pose logged, checked for discontinuities.
- **Verification command**: `uv run python -m pytest src/tests/sim`;
  `planner_tests`' own build for the estimation-layer unit tests.

## Implementation Plan

**Approach**: Widen `Motion::Odometry::integrate(float leftPosition, float
rightPosition)` to also accept each wheel's `positionEpoch` (e.g.
`integrate(float leftPosition, float rightPosition, uint8_t leftEpoch,
uint8_t rightEpoch)`), tracking `lastLeftEpoch_`/`lastRightEpoch_`
internally (initialized to match the constructor's initial-epoch
assumption, 0). On a per-wheel epoch mismatch, re-anchor that wheel's
`lastLeft_`/`lastRight_` to the incoming position instead of computing a
delta for it that cycle. Apply the analogous change to
`Motion::PoseTracker::integrate()`/`Motion::WheelChannel`. Update
`App::RobotLoop::cycle()`'s call
(`odom_.integrate(motorL_.position(), motorR_.position())`,
`robot_loop.cpp:541`) to also pass
`state_.wheelLeft.positionEpoch`/`state_.wheelRight.positionEpoch`, and the
planner's equivalent ingestion call in `Motion::Planner::tick()`. In
`nezha_motor.cpp`'s `softRebaseline()`, remove the `velocity_ = 0.0f;`
line.

**Files to modify**:
- `src/motion/odometry.h`/`.cpp`
- `src/motion/planner/estimation.h`/`.cpp`
- `src/firm/app/robot_loop.cpp` (the `odom_.integrate()` call site)
- `src/motion/planner/planner.cpp` (wherever it ingests positions into
  `PoseTracker`/`WheelChannel`)
- `src/firm/devices/nezha_motor.cpp` (`softRebaseline()`)
- Any sim/test harness constructing `Types::RobotState` and calling these
  `integrate()` methods directly.

**Testing plan**: as listed above.

**Documentation updates**: `robot_state.h`'s `positionEpoch` field doc
comment gets a line noting it now has real motion-side consumers;
`odometry.h`/`estimation.h`'s file headers get a short note on the
epoch-aware re-anchor contract.
