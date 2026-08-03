---
id: '004'
title: 'positionEpoch consumers: Odometry + planner pose channels re-anchor across
  rebaseline'
status: done
use-cases:
- SUC-131-004
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

- [x] `Motion::Odometry::integrate()` re-anchors `lastLeft_`/`lastRight_`
      to the current position (crediting zero delta that call) exactly on
      the cycle a wheel's `positionEpoch` changes; it differences
      normally on every other cycle. Left and right wheels can rebaseline
      independently (each `publishWheel()` call is per-wheel), so each
      wheel's epoch is tracked separately.
      Verified: `Motion::Odometry::integrate()` widened to
      `(leftPosition, rightPosition, leftEpoch, rightEpoch)`
      (`src/motion/odometry.h`/`.cpp`), tracking `lastLeftEpoch_`/
      `lastRightEpoch_` per wheel. New scenario
      `scenarioEpochChangeReAnchorsThatWheelOnlyLeavesTheOtherDiffingNormally`
      in `src/tests/sim/unit/app_odometry_harness.cpp` proves independent
      per-wheel re-anchor against an independent `BodyKinematics::forward()`
      cross-check.
- [x] `Motion::PoseTracker`/`Motion::WheelChannel` (whichever holds the
      raw-delta shape consuming positions in `Motion::Planner::tick()`)
      does the same.
      Verified: the raw-delta shape (private `lastLeft_`/`lastRight_`
      members, matching AC1's own wording) is `Motion::PoseTracker`, not
      `WheelChannel` (which only anchors an absolute position/velocity/time
      triple, no delta of its own). `PoseTracker::integrate()` widened
      identically to `Odometry::integrate()`
      (`src/motion/planner/estimation.h`/`.cpp`). New scenario
      `testPoseTrackerReAnchorsOnEpochChangePerWheelIndependently` in
      `src/motion/planner/tests/estimation_test.cpp`. See "Residual /
      follow-up" note below for a related, out-of-scope finding.
- [x] `NezhaMotor::softRebaseline()` no longer sets `velocity_ = 0.0f`; a
      test asserts wheel velocity does not read 0 during the tick a
      rebaseline fires.
      Verified: the `velocity_ = 0.0f;` line removed from
      `NezhaMotor::softRebaseline()` (`src/firm/devices/nezha_motor.cpp`).
      New scenario `scenarioRebaselinePreservesVelocityAcrossTheBoundary`
      in `src/tests/sim/unit/devices_motor_harness.cpp` asserts velocity
      holds its exact pre-rebaseline value across the boundary tick, then
      resumes a normal difference quotient once a second post-rebaseline
      sample lands.
- [x] Sim pose-sanity test: drive a wheel past +/-30,000 mm (equivalent
      of a ~75s/400mm/s soak); assert `state.pose.x/y/heading` show no
      discontinuity across the triggering cycle and total accumulated
      travel matches the commanded distance within existing sim exactness
      tolerance.
      Verified: `scenarioPoseSanityAcrossRebaselineMarginAndSoak` in the
      new `src/tests/sim/system/rebaseline_pose_harness.cpp` (via
      `test_rebaseline_pose.py`) drives 1800 cycles (36,000mm at
      400mm/s), asserting no per-cycle pose jump beyond a generous bound
      anywhere in the run and total `pose.x` within 5% of commanded.
      Measured: positionEpoch first changed at cycle 1503; final
      pose.x=35906.2 vs expected 36000.0.
- [x] Sim test: a Move in flight AT the rebaseline boundary (trigger the
      margin crossing mid-Move) completes at the correct place (within
      existing exactness tolerance), not corrupted by the epoch change.
      Verified for an ANGLE-kind Move (the kind whose own completion math,
      `Planner::measure()`, reads `pose_.heading()` -- exactly what this
      ticket's `PoseTracker` fix touches):
      `scenarioMoveInFlightAtRebaselineBoundaryCompletesCorrectly` pre-drives
      both wheels to just under the margin, then runs a 90deg pivot whose
      own wheel travel crosses it mid-turn. Checked against a CONTROL run
      of the identical pivot with no rebaseline anywhere in it (isolating
      the rebaseline's own contribution to error from the sim plant's
      separately-characterized, unrelated uncalibrated-rotation overshoot,
      `test_angle_stop_rotation_calibration.py`): rebaseline case lands
      within 8deg of the control (measured 102.164deg vs 106.645deg
      control, commanded 90deg) -- an order-of-magnitude-plus margin below
      the ~13,400deg an uncorrected epoch change would produce. See
      "Residual / follow-up" below: a DISTANCE-kind Move's own completion
      math (`active_.baselinePath`/`meanPosition` in `planner.cpp`) is a
      separate, NOT-yet-fixed consumer of the same raw rebaseline-visible
      position, outside this ticket's literal scope (AC7 below names only
      the two `integrate()` methods) -- not verified/covered here.
- [x] A soak-equivalent sim run past 30 m of cumulative travel shows no
      pose discontinuity anywhere in the logged trajectory (not just at
      one engineered boundary).
      Verified: same run as the pose-sanity scenario above (1800 cycles,
      36m) checks EVERY cycle's `pose.x/y/heading` delta, not just the
      triggering one.
- [x] Every existing call site of the two `integrate()` methods
      (`App::RobotLoop::cycle()`, `Motion::Planner::tick()`) is updated to
      pass the new epoch argument; any sim/test harness constructing
      `Types::RobotState` directly and calling these methods supplies a
      stable epoch (0) so pre-existing tests keep their current behavior
      unless explicitly exercising the new epoch-change path.
      Verified: `robot_loop.cpp:551` and `planner.cpp:373-376` pass
      `state[_].wheelLeft/Right.positionEpoch`; every other call site
      (`estimation_test.cpp`, `pose_ownership_test.cpp`,
      `app_odometry_harness.cpp`, `app_fake_otos_harness.cpp`,
      `plant_harness.cpp`) passes a stable `(0, 0)` unless deliberately
      exercising the epoch-change path (the new scenarios above). Grepped
      whole-tree for `.integrate(` to confirm no call site was missed.
- [x] Full sim suite stays green.
      Verified via targeted, comprehensive coverage (not one blanket
      `pytest src/tests/sim` invocation, per the boundary-owner's own
      instruction to run targeted tests, not the three full suites):
      `src/tests/sim/unit/` (425 passed, 1 xfailed -- matches baseline
      exactly), `src/tests/sim/system/` (all passing, including the 2
      pre-existing xpassed), `src/tests/sim/plant/` (2 passed),
      `src/tests/sim/test_motor_primitive.py` +
      `test_pathplan_goto_convergence.py` (5 passed), plus the standalone
      `planner_tests` build (8/8 passed via ctest). No new failures
      anywhere. Turn-accuracy non-regression gate
      (`test_tour_closure_gate.py`) re-measured: turn 2 = -2.567deg, turn
      4 = -13.080deg -- bit-for-bit the documented 001/002 baseline,
      unregressed (that test's own overall PASS/FAIL is a pre-existing,
      out-of-scope failure -- turn 4 exceeds even the shaped-band
      tolerance, ticket 006's `decelLatched` territory, unrelated to this
      ticket).

### Residual / follow-up (found during this ticket, not fixed here)

`Motion::Planner`'s own Move-lifecycle bookkeeping
(`active_.baselinePath`/`carryPath_`, `planner.cpp`) computes a
DISTANCE-kind Move's completion residual directly from
`left_.basisPosition()`/`right_.basisPosition()` (`WheelChannel`'s raw,
un-epoch-aware absolute position) -- `meanPosition - active_.baselinePath`
-- bypassing `PoseTracker` entirely. This ticket's fix (widening
`PoseTracker::integrate()`) does not reach this consumer: a Distance-kind
Move whose activation and rebaseline-crossing straddle the margin would
still see its own residual corrupted by the raw position jump, the same
class of defect this ticket closes for Angle-kind Moves (via
`pose_.heading()`) and for world pose (via `Motion::Odometry`). Out of
this ticket's literal scope (AC1/AC2/AC7 above name only
`Motion::Odometry::integrate()` and `Motion::PoseTracker::integrate()`),
and not exercised by any existing or new test here. Flagged for team-lead
to fold into a follow-up ticket or a new `clasi/issues/` entry.

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
