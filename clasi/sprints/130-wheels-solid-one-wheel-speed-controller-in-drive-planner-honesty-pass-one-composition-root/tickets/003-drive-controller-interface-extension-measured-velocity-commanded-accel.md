---
id: '003'
title: 'Drive controller interface extension: measured velocity + commanded accel'
status: done
use-cases:
- SUC-001
depends-on:
- '002'
github-issue: ''
issue: wheel-speed-controller-moves-into-drive.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Drive controller interface extension: measured velocity + commanded accel

## Description

Extend `App::Drive`'s `tick()`/`update()` contract per
`wheel-speed-controller-moves-into-drive.md`'s "Drive interface today"
section: the unified controller (built in ticket 004) needs `tick()`
to see the measured wheel velocity (already available via
`Devices::Motor`) AND the commanded acceleration (new). Proposal:
`tick(const Types::RobotState&)` — `Drive` already includes
`robot_state.h` — with `Motion::Planner` publishing a new
`Wheel::cmdAccel` blackboard field beside the existing `cmdVelocity`.

This ticket is plumbing only — behavior-preserving, no controller logic
changes. It exists as its own ticket so ticket 004 can implement the
Stage A/B/C algorithm against a stable, already-landed signature rather
than co-developing the interface and the algorithm at once. Depends on
ticket 002 (composition root) so the extended interface is wired
through `composeRobot()` for both sim and hardware from the start,
never hand-wired twice.

## Acceptance Criteria

- [x] `Types::RobotState::Wheel::cmdAccel` field added (`// [mm/s^2]`
      unit tag); `Motion::Planner` publishes it beside `cmdVelocity`
      every tick.
- [x] `App::Drive::tick()`'s signature is extended to see measured wheel
      velocity + commanded accel (e.g. via `const Types::RobotState&`),
      with sample-time/freshness gating available for future stages to
      consume.
- [x] Zero behavior change: all existing bench/sim acceptance tests pass
      unchanged — this ticket changes no control logic.
- [x] `composeRobot()` (ticket 002) is the only place this interface is
      wired for both sim and hardware — no duplicate wiring in
      `main.cpp`/`SimHarness` directly.

## Testing

- **Existing tests to run**: full `app_drive`/`planner` unit and sim
  test suites — must show zero behavior change.
- **New tests to write**: a smoke test asserting `cmdAccel` round-trips
  correctly through the `RobotState` blackboard from `Planner::update()`
  to `Drive::tick()`.
- **Verification command**: `uv run pytest`

## Implementation Plan

**Approach**: narrowest possible interface change — add the field and
the signature, wire it through, change no control behavior. This keeps
ticket 004's diff focused on the algorithm alone.

**Files to create/modify**:
- `src/firm/types/robot_state.h` (`Wheel::cmdAccel`)
- `src/firm/app/drive.{h,cpp}` (signature extension)
- `src/motion/planner/planner.cpp` (publish `cmdAccel`)
- `src/firm/app/robot_loop.cpp` (call site update)

**Testing plan**: existing `app_drive`/`planner` tests re-run unchanged;
new `cmdAccel` round-trip smoke test.

**Documentation updates**: `drive.h`'s header comment gets a forward
note that Stage A/B/C (ticket 004) will consume this interface — full
rewrite of the "there is no controller here" comment happens in ticket
004/005, not here.
