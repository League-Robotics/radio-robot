---
id: '007'
title: 'App::Drive duty wiring: implement the duty WheelSink, own WheelObserver pair
  (motor ownership unchanged)'
status: open
use-cases:
- SUC-001
depends-on:
- '002'
- '003'
- '004'
- '006'
github-issue: ''
issue: firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# App::Drive duty wiring: implement the duty WheelSink, own WheelObserver pair (motor ownership unchanged)

## Description

`App::Drive` (`src/firm/app/drive.{h,cpp}`) implements the retooled
`Motion::WheelSink` as a duty sink: `setDuty(left, right)` clamps
(`|duty| <= 1`, NaN → 0 — defense-in-depth, mirroring
`clampToPositionWireBound()`'s existing posture) and stages `lastCmd_`;
`stop()` stages zero. Zero-on-silence is INHERITED structurally from
`MoveQueue`'s own unconditional per-cycle tick (already ends every cycle
in either a shaped `setDuty()` call or `stop()` on an empty queue) — no
new watchdog logic needed here. `Drive` owns two `App::WheelObserver`
instances (ticket 004) and feeds them each cycle from the raw
`WheelSample` it collects. **Motor ownership is UNCHANGED this ticket**
(Design Rationale Decision 6's valve) — `Drive` still holds
`Devices::Motor&` REFERENCES exactly as today; `RobotLoop` still calls
`motorL_.requestSample()`/`motorL_.tick()` directly (the ownership MOVE
is ticket 013, deferrable). This ticket's own `RobotLoop::cycle()` touch
is the minimum needed to feed `Drive`'s observers and thread
`WheelEstimate`s to `MoveQueue::tick()` — not the full sense/observe/
decide/act/report rewrite (ticket 015).

See sprint architecture Step 3 (`App::Drive`) and Design Rationale
Decision 6.

## Acceptance Criteria

- [ ] `App::Drive` implements `Motion::WheelSink::setDuty()`/`stop()`.
- [ ] **[off-hardware]** A sim test with NO `Move` enqueued asserts the
      wheel receives exactly duty 0 every cycle — positive evidence for
      zero-on-silence, not just "no crash observed."
- [ ] **[off-hardware]** A `motion_tests`/sim scenario drives a chained
      `WHEELS` `Move` end to end through `Drive`'s duty sink and asserts
      the two `Devices::Motor` leaves only ever receive `setDuty()`
      calls (never a resurrected `setVelocity()`).
- [ ] `Drive`'s wheel-section publish (raw + observed + `appliedDuty` +
      `positionEpoch` + glitch/wedge) happens once, immediately after
      both collects — same coherence point 124 established (grep/review
      check against `RobotState::Wheel`'s own publish-rule doc comment).
- [ ] `App::Drive` still holds `Devices::Motor&` by reference, NOT by
      ownership — confirm this ticket does not accidentally pre-empt
      ticket 013's own scope.

## Testing

- **Existing tests to run**: `app_robot_loop_harness.cpp`/`drive`-focused
  sim scenarios (expect updates for the duty-sink call shape).
- **New tests to write**: zero-on-silence positive-evidence test, the
  chained-Move duty-only-calls test, a wheel-section publish-coherence
  test (both L and R fresh before the section is read).
- **Verification command**: `uv run pytest` plus the sim `ctest` suite.
