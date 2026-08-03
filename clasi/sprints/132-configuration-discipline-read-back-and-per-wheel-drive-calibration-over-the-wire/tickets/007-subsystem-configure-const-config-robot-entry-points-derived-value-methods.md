---
id: '007'
title: Subsystem configure(const Config::Robot&) entry points + derived-value methods
status: in-progress
use-cases:
- SUC-002
- SUC-006
depends-on:
- '006'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Subsystem configure(const Config::Robot&) entry points + derived-value methods

## Description

Add `void configure(const Config::Robot&)` to `Drive`, `Motion::Planner`,
`Devices::Motor` (both instances get the call), `Devices::Otos`,
`App::StateEstimator`, and `RobotLoop` (for geometry/rotation
calibration). Each pulls only the fields it owns, reusing setters that
already exist: `Drive::setWheelCorrection`/`setControlGains`/
`setAdaptationBounds`/`setCrawlPulse`; `Motor::applyTravelCalib`;
`RobotLoop::setRotationCalibration`. `Devices::Motor::configure()`
additionally returns `bool`, `false` while the robot is moving
(`Configurator` maps this to `ERR_BUSY` — that mapping is ticket 009's
job; this ticket's scope is the `configure()` method itself returning the
correct bool). Add `effectiveTrackWidth()` and `rotationOffsetPos()` as
methods on `Config::Robot` (not stored fields), replacing the duplicate
derivation currently fanned out in `boot_calibration.cpp:25-29` to four
places.

## Acceptance Criteria

- [ ] `Drive::configure(const Config::Robot&)`,
      `Motion::Planner::configure(const Config::Robot&)`,
      `Devices::Motor::configure(const Config::Robot&) -> bool`,
      `Devices::Otos::configure(const Config::Robot&)`,
      `App::StateEstimator::configure(const Config::Robot&)`,
      `RobotLoop::configure(const Config::Robot&)` all exist.
- [ ] Each `configure()` call reuses an EXISTING setter (no new firmware
      control logic invented) — verified by a diff review showing each
      `configure()` body is a thin pull-and-forward.
- [ ] `Devices::Motor::configure()` returns `false` when the motor
      reports itself in motion (reusing whatever "is moving" signal
      `NezhaMotor`/`MotorArmor` already exposes), `true` otherwise.
- [ ] `Config::Robot::effectiveTrackWidth()` and
      `Config::Robot::rotationOffsetPos()` exist as `const` methods, not
      stored fields; both match `boot_calibration.cpp:25-29`'s current
      derivation formula exactly.
- [ ] Every call site that previously read `boot_calibration.cpp`'s
      derived trackWidth/rotation values now calls the new methods
      instead (Drive, Odometry, Planner, RobotLoop — the four fan-out
      points named in the issue's own audit).
- [ ] Compiles under `HOST_BUILD`.

## Testing

- **Existing tests to run**: Drive/Planner/Motor/Otos unit test suites
  continue to pass unmodified where they don't touch `configure()`
  directly.
- **New tests to write**: one test per subsystem's `configure()`
  confirming it applies the expected setter calls given a sample
  `Config::Robot`; a test confirming `effectiveTrackWidth()`/
  `rotationOffsetPos()` match the old `boot_calibration.cpp` formula
  bit-for-bit for a sample input.
- **Verification command**: `uv run python -m pytest <relevant unit test
  paths> -q`.

## Implementation Plan

**Approach**: One `configure()` method per subsystem header/source pair,
each a small, focused addition. Derived-value methods added directly to
the `Config::Robot` header (ticket 002's generated file, or a small
hand-written extension alongside it — confirm whether the generator
supports attaching methods to generated structs, or whether `Config::Robot`
needs to be a hand-written wrapper embedding the generated group structs
by value, with derived methods on the wrapper; this is an implementation
call to resolve and document, not pre-decided at planning time).

**Files to modify**: `src/firm/app/drive.{h,cpp}`,
`src/firm/motion/planner/planner.{h,cpp}` (confirm exact path),
`src/firm/devices/motor.{h,cpp}`, `src/firm/devices/otos.{h,cpp}`,
`src/firm/app/state_estimator.{h,cpp}` (confirm exact filename),
`src/firm/app/robot_loop.{h,cpp}`, `src/firm/config/` (`Config::Robot`'s
own header, for the derived methods).

**Testing plan**: as above.

**Documentation updates**: each subsystem's file header gets a one-line
addendum noting its `configure()` entry point and what group(s) it reads.
