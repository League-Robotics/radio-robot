---
id: '010'
title: 'Slew-cap disposition: unmodified code, visible present-and-uncharacterized
  marker'
status: open
use-cases:
- SUC-008
depends-on:
- '003'
github-issue: ''
issue: firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Slew-cap disposition: unmodified code, visible present-and-uncharacterized marker

## Description

NO code change to `NezhaMotor`'s existing slew cap (`writeRawDuty()`'s
slew section, `kDefaultSlewRate`/`MotorConfig::slewRate`) — ship it
byte-for-byte unmodified. Add a visible "present, uncharacterized"
marker (a new `RobotState`/telemetry bool or flag bit) so the open
decision (delete vs. characterize into the observer's model, per the
issue's own item 5) is honest and visible rather than silently landed
either way. Sprint 126 inherits the bench step-response test that would
actually decide it. See sprint architecture Design Rationale Decision 5
— deliberately NOT deciding by assertion with zero bench evidence.

## Acceptance Criteria

- [ ] A diff against pre-sprint `nezha_motor.cpp`'s `writeRawDuty()` slew
      section shows NO change.
- [ ] A new present-and-uncharacterized marker is readable in
      `RobotState`/telemetry every frame.
- [ ] Sprint 125's own closing notes (or this ticket's own completion
      notes) state explicitly that the bench step-response test needed
      to decide delete-vs-characterize does NOT run this sprint — a
      stated, deliberate gap for 126, not implied done.

## Testing

- **Existing tests to run**: any test asserting `writeRawDuty()`'s slew
  behavior — must pass UNCHANGED (this ticket's whole point is zero
  behavior change to that path).
- **New tests to write**: a telemetry/state test confirming the new
  marker is present and readable every frame.
- **Verification command**: `uv run pytest`.
