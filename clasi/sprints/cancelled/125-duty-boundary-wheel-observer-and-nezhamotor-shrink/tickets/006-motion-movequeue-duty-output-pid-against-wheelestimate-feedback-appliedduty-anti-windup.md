---
id: '006'
title: 'Motion::MoveQueue duty output: PID against WheelEstimate feedback, appliedDuty
  anti-windup'
status: open
use-cases:
- SUC-003
- SUC-004
- SUC-007
depends-on:
- '004'
- '005'
github-issue: ''
issue: firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Motion::MoveQueue duty output: PID against WheelEstimate feedback, appliedDuty anti-windup

## Description

`Motion::MoveQueue` (`src/motion/move_queue.{h,cpp}`) gains two
`Motion::WheelVelocityPid` instances (ticket 005) as direct members,
invoked at the end of `shapeAndStage()`: convert the shaped velocity
target into a `DutyCommand` using hand-fed `WheelEstimate` feedback
(measured velocity) and `appliedDuty` (anti-windup — reads the ACTUATOR's
truth, post-dwell/deadband, not the PID's own last commanded output),
then call `sink_.setDuty(left, right)` instead of `setWheels()`.
`tick()`'s signature gains explicit `WheelEstimate` parameters (left,
right), matching its existing hand-fed-reading convention (`now`, `odom`)
rather than a new held reference. Also add `commandedTwist()` (returns
`v_x`/`omega`): `TWIST` moves return the shaped `cruiseVX`/`cruiseOmega`
directly; `WHEELS` moves derive via `BodyKinematics::forward(cruiseVLeft,
cruiseVRight, trackWidth)` — the SAME function already used elsewhere to
fuse the two leaves' measured velocities, applied here to the commanded
pair instead. See sprint architecture Step 3 (`Motion::MoveQueue`) and
Design Rationale Decisions 1 and 4.

## Acceptance Criteria

- [ ] `MoveQueue::shapeAndStage()` calls `sink_.setDuty()`, never
      `setWheels()` (deleted from the interface by ticket 002).
- [ ] **[off-hardware]** A `motion_tests` scenario forces a dwell-shaped
      write (a commanded reversal) and asserts the PID's integrator
      reflects the SHAPED `appliedDuty`, not the pre-shaping commanded
      value; the SAME scenario against a deliberately-reverted build
      (feedback wired to the commanded value) FAILS the assertion — a
      real tripwire, not a vacuous pass.
- [ ] **[off-hardware]** A sim test drives one `TWIST` and one `WHEELS`
      `Move` in turn and asserts `commandedTwist()`'s `v_x`/`omega` is
      nonzero and correct in both cases.
- [ ] `MoveQueue`'s own cohesion holds: no new wrapping controller class
      introduced (Design Rationale Decision 1) — the PID instances are
      direct `MoveQueue` members.

## Testing

- **Existing tests to run**: `move_queue_harness.cpp`/chained-`Wheels`-
  Move scenarios (expect them to need updating for the new `setDuty()`
  call shape — not a silent behavior change, an intentional one).
- **New tests to write**: the anti-windup regression test above (with its
  required-to-fail reverted-build companion), the `commandedTwist()`
  correctness test for both `Move` kinds.
- **Verification command**: `cmake --build src/motion/build --target
  motion_tests`.
