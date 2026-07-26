---
id: '002'
title: Retool Motion::WheelSink as a duty sink (WheelEstimate replaces WheelState)
status: open
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Retool Motion::WheelSink as a duty sink (WheelEstimate replaces WheelState)

## Description

Retool `src/motion/wheel_sink.h`: replace `WheelSink::setWheels(v_left,
v_right)`/`stop()` with `setDuty(left, right)`/`stop()` (both `[-1,1]`).
Replace the unused `WheelState` struct with `WheelEstimate` (`position`,
`velocity`, `age`, `rejects`, `wedged`) — the plain struct `Drive`'s
published `RobotState` wheel-section values get read into and hand-fed
to `Motion::MoveQueue::tick()`, matching `MoveQueue`'s existing hand-fed
`(now, odom)` convention rather than a live cross-tree reference.
`WheelSinkConfig` is unaffected. Pure type/interface change — no
concrete implementation lands in this ticket (`App::Drive` gets a
placeholder/no-op `setDuty()` just enough to keep the tree linking; the
real implementation is ticket 007). See sprint architecture Step 3
(`Motion::WheelSink`) and Design Rationale Decision 1.

This is a foundation ticket: zero behavior change to anything downstream
yet, just the shared vocabulary every later core ticket in this sprint
codes against. Expect `App::Drive`/`Motion::MoveQueue` call sites to go
red until tickets 006-007 land (Migration Concerns: "CI goes red then
green mid-sprint, by design").

## Acceptance Criteria

- [ ] `Motion::WheelSink::setWheels()` no longer exists; `setDuty(left,
      right)`/`stop()` exist in its place, `[-1,1]` per the base
      contract's plausibility bound.
- [ ] `WheelState` is removed; `WheelEstimate` exists with the fields
      above, documented as the base→motion per-wheel reading shape.
- [ ] `src/motion/DESIGN.md`'s own boundary description is NOT yet
      updated in this ticket (that is ticket 017's job) — do not
      partially update it here and leave it inconsistent with tickets
      still in flight.
- [ ] `motion_tests` still configures and builds (may have red tests
      downstream until later core tickets land — not a regression in
      this ticket's own scope).

## Testing

- **Existing tests to run**: `cmake --build src/motion/build --target
  motion_tests` (expect some downstream failures until tickets 006-007
  land — confirm THIS ticket's own diff compiles cleanly, not that the
  whole battery is green yet).
- **New tests to write**: none yet — this is a type-only change; real
  behavioral tests land with tickets 004-007.
- **Verification command**: `cmake --build src/motion/build` (configure
  step alone must succeed).
