---
id: 009
title: 'RobotLoop/Telemetry restructure: one-assembly-point state->update->emit, TelemetrySecondary
  deletion'
status: done
use-cases:
- SUC-004
- SUC-005
depends-on:
- '007'
- 008
github-issue: ''
issue: robot-state-blackboard-one-struct-for-all-shared-state-and-telemetry.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# RobotLoop/Telemetry restructure: one-assembly-point state->update->emit, TelemetrySecondary deletion

## Description

Restructure `cycle()` to build `RobotState` (ticket 007's struct) once
per cycle, publishing each section at its earliest coherent point (per
the blackboard issue's publish rule: wheels immediately after both L/R
collects, sensors/odom/estimate in dependency order in the pace block),
then call `Telemetry::update(state)`/`emit(now)` exactly once —
replacing the ten-argument `assembleFrame()`, the ten scattered
`setFlag()` calls, and the hand-copied `StateEstimator::Input`
construction.

Delete `TelemetrySecondary` outright: the frame type, the wire schema
arm, and `telemetry.cpp`'s tie-break/alternation machinery. Fold any
secondary field the host genuinely used (per the blackboard issue's own
review — candidates: cmd velocities, glitch counts) into the pruned
primary frame during this same change.

**Device ownership is UNCHANGED this sprint** (architecture Decision 1 —
the scope valve, pulled): `motorL_`/`motorR_`/`otos_`/`line_`/`color_`
stay direct `RobotLoop` members, exactly as today. Do NOT introduce
named bus-phase methods (`requestLeft()`/`collectLeft()`/etc.) or move
device ownership to `Drive`/a new `Sensors` subsystem — that reshuffle
is explicitly sprint 125's scope. This ticket only changes HOW state is
assembled and published, not WHO owns the devices producing it.

## Acceptance Criteria

- [x] `cycle()` contains exactly one `tlm_.update(state)` call and zero
      `setFlag()` calls anywhere else (grep-enforceable: `grep -n
      "setFlag" src/firm/app/robot_loop.cpp` returns nothing outside
      `Telemetry::update()`'s own implementation).
- [x] `TelemetrySecondary`'s frame type, wire schema arm, and
      tie-break/alternation machinery are removed from the diff, not
      merely unused.
- [x] `Motion::StateEstimator` consumes `RobotState` directly (ticket
      007) — no private near-duplicate `Input` struct remains.
- [x] `enc_left.age`/`enc_right.age` (built on ticket 002's
      `sampleTime()`) differ by roughly the `kClear`+`kSettle`
      separation under the virtual clock, never equal, never zero — a
      sim test asserts this directly (a test asserting they're equal is
      the bug, not the spec).
- [x] Device ownership in `RobotLoop` is byte-for-byte unchanged from
      before this ticket (no Drive/Sensors reshuffle) — verify via
      `git diff` showing no change to which references `RobotLoop`
      holds.
- [x] Regenerated `kReplyEnvelopeMaxEncodedSize` ≤ 130 B (co-verified
      with ticket 008).

## Testing

- **Existing tests to run**: `app_robot_loop_harness.cpp`,
  `app_telemetry_harness.cpp`, `app_comms_harness.cpp`.
- **New tests to write**: the single-assembly-point grep/structural
  check; the age-differential sim test; a `TelemetrySecondary`-removal
  regression test (asserting no secondary wire arm is ever emitted).
- **Verification command**: `uv run pytest` plus the C++ sim-tests
  build and the firmware ARM build (`just build`).
