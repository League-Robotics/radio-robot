---
id: '002'
title: Subsystems::PoseEstimator -- encoder dead-reckoning + OTOS fusion
status: open
use-cases: [SUC-002]
depends-on: ['001']
github-issue: ''
issue: plan-revive-testgui-against-the-new-tree-simulator.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Subsystems::PoseEstimator -- encoder dead-reckoning + OTOS fusion

## Description

New `Subsystems::PoseEstimator` (`source/subsystems/pose_estimator.{h,cpp}`)
-- a Subsystems-tier peer of `Subsystems::Drivetrain`, deliberately **not**
folded into `Drivetrain` itself (architecture-update.md Decision 1: control-
law tuning and sensor-fusion-noise tuning change for different reasons; this
is a cohesion decision, not an oversight of the fact that
`msg::DrivetrainState`/`DrivetrainConfig` already scaffold pose/EKF fields --
see "Grounding," fact 1, in the architecture doc).

`PoseEstimator` owns one `Hal::EkfTiny` (ticket 001) plus its own
encoder-only dead-reckoning accumulator (arc-segment integration, ported in
concept from `source_old/control/Odometry.cpp`'s encoder half). It exposes
two independent readings:

- `encoderPose()` -- pure dead-reckoning from wheel encoder deltas, no OTOS
  involved, ever.
- `fusedPose()` -- the EKF's belief: predicted every tick from the same
  encoder deltas, corrected by the odometer's reading when one is present and
  fresh.

## Acceptance Criteria

- [ ] `configure(const msg::DrivetrainConfig&)` reads `trackwidth`,
      `rotational_slip`, `ekf_q_xy`, `ekf_q_theta`, `ekf_r_otos_xy`,
      `ekf_r_otos_theta` -- the SAME `msg::DrivetrainConfig` type
      `Drivetrain::configure()` already takes (no new config message; no
      proto change).
- [ ] Any of the four EKF noise fields arriving as exactly `0.0f` (the proto
      zero-default, meaning "never configured") is substituted with a small,
      hardcoded, documented default before being passed to
      `Hal::EkfTiny::init()` -- mirroring the existing `effectiveSlip()`
      zero-as-unset-sentinel pattern already present in the ported
      `Odometry` source (see architecture-update.md Decision 4). A non-zero
      configured value passes through unchanged.
- [ ] `tick(now, leftObs, rightObs, otosObs)` -- `leftObs`/`rightObs` are
      `msg::MotorState` (the SAME per-wheel observation shape
      `Drivetrain::tick()` already takes); `otosObs` is
      `const msg::PoseEstimate*`, nullable.
- [ ] Encoder delta is computed from `leftObs.position`/`rightObs.position`
      (`Opt<float>`) -- if either lacks `.has`, this tick's update is skipped
      (no update, no crash, no stale-data corruption).
- [ ] Midpoint-arc integration into `encoderPose()`'s accumulator matches the
      same `dCenter = (dL+dR)/2`, `dTheta = ((dR-dL)/trackwidth) *
      clampedRotationalSlip` math as the ported `Odometry::predict()`
      (`effectiveSlip()`'s clamp semantics preserved: 0/negative -> 1.0,
      (0, 0.5) -> 0.5 floor, >1.0 -> 1.0 ceiling).
- [ ] `Hal::EkfTiny::predict(dCenter, dTheta, thetaBefore, dt)` runs every
      tick unconditionally (dead-reckoning always advances, whether or not an
      odometer is present).
- [ ] `Hal::EkfTiny::updatePosition()`/`updateHeading()` run ONLY when
      `otosObs != nullptr` AND `otosObs->stamp.valid` is true.
- [ ] Host unit test: with `otosObs` always `nullptr` across a multi-tick
      sequence, `fusedPose()` equals `encoderPose()` exactly at every tick
      (no correction applied when there is nothing to correct with).
- [ ] Host unit test: with a synthetic `otosObs` diverging from the
      encoder-only path (e.g. offset by a known amount), `fusedPose()`
      differs measurably from `encoderPose()` after several ticks -- proves
      the correction step actually executes when an odometer is present.
- [ ] Host unit test: a `DrivetrainConfig` with all four EKF fields at
      `0.0f` still produces a finite, non-NaN, non-degenerate `fusedPose()`
      after several predict+correct ticks (the sentinel-default fallback
      prevents the degenerate `Q=0, R=0` case).
- [ ] `PoseEstimator` holds no `Hal::Motor`/`Hal::Odometer` reference or
      pointer as a member -- observations are tick() arguments only, matching
      `Drivetrain`'s own no-stored-HAL-reference discipline (see
      `drivetrain.h`'s class comment).
- [ ] All new identifiers are lowerCamelCase (methods/functions), no
      unit-suffixed names, units in `// [unit]` comment tags; uses only
      `msg::` pose types (never `kinematics/pose2d.h`'s parallel family).

## Implementation Plan

### Approach

1. Read `source_old/control/Odometry.{h,cpp}`'s `predict()` in full;
   confirm exactly which lines are the encoder-only dead-reckoning half
   (independent of the EKF calls) -- that half becomes `PoseEstimator`'s own
   accumulator logic, ported directly (not through `Hal::EkfTiny` at all,
   since `encoderPose()` must exist even with no EKF involvement).
2. Write `Subsystems::PoseEstimator` wrapping one `EkfTiny` member (ticket
   001) plus the encoder accumulator's own float x/y/heading state.
3. Implement the zero-as-unset-sentinel substitution in `configure()` as a
   small private helper (e.g. `static float sentinelOr(float configured,
   float fallback)`), applied to exactly the four fields listed in
   Acceptance Criteria -- document the chosen fallback constants' values and
   provenance (a reasonable starting point, not a tuned value) in a comment.
4. Implement `tick()` per the Acceptance Criteria's exact sequencing
   (encoder delta -> dead-reckoning accumulate -> EKF predict -> conditional
   EKF correct).
5. Write host unit tests covering the three acceptance-criteria scenarios
   (no-odometer identity, odometer-present divergence, zero-config
   sanity).

### Files to create

- `source/subsystems/pose_estimator.h`
- `source/subsystems/pose_estimator.cpp`
- `tests/sim/unit/pose_estimator_harness.cpp` (ad hoc-compile convention,
  matching ticket 001's own new harness and the existing `*_harness.cpp`
  files).

### Files to modify

- None. `Subsystems::Drivetrain` is unaffected (Decision 1) -- this ticket
  does not touch `drivetrain.{h,cpp}`.

### Testing plan

- New standalone-compiled harness exercising the three scenarios in
  Acceptance Criteria, run the same ad hoc way as ticket 001's harness and
  the project's existing `tests/sim/unit/*_harness.cpp` tier.
- No hardware-bench-gate item for this ticket specifically (no wiring into
  `devLoopTick`/`main.cpp` yet -- that is ticket 003).

### Documentation updates

- None required this ticket (not wire-visible yet). Document the sentinel-
  default fallback constants and their rationale directly in
  `pose_estimator.cpp`'s `configure()` implementation.
