---
id: 008
title: RobotState wheel-section schema + Telemetry projection; split CONFIG-patch
  routing
status: open
use-cases:
- SUC-001
- SUC-002
- SUC-008
depends-on:
- '006'
- '007'
github-issue: ''
issue: firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# RobotState wheel-section schema + Telemetry projection; split CONFIG-patch routing

## Description

Extend `Types::RobotState::Wheel` (`src/firm/types/robot_state.h`):
`appliedDuty` `[-1,1]` (post-shaping, ticket 006's anti-windup feedback
value), `glitchCount` (cumulative, from the old `encGlitchCount_`'s
role), `wedged` (bool, now `WheelObserver`-sourced), and a raw-vs-observed
pair for `position`/`velocity` (exact field split at implementation time —
the base contract's "commanded AND observed AND raw, all visible"
requirement). Rename/repurpose `cmdVelocity` → `cmdDuty` `[-1,1]` (the
primitive changed; the field's role — "what `Drive` last staged" — does
not). Wire `App::Telemetry`'s projection for the new fields (scaled-field
conversion, wire-budget-aware per Migration Concerns).

Split `CONFIG`-patch application routing in `RobotLoop::handleConfig()`
— SAME wire fields (`MotorConfigPatch.kp/ki/kff/i_max/kaw/travel_calib`),
no protocol/schema change: `kp`/`ki`/`kff`/`i_max`/`kaw` now route to
`MoveQueue`'s `WheelVelocityPid` instances (a new `MoveQueue` setter);
`travel_calib` stays with `Devices::Motor::applyGains()`, narrowed to
just that one field. See sprint architecture Step 3 (`Types::RobotState`,
`App::Telemetry`) and Design Rationale Decision 7.

## Acceptance Criteria

- [ ] `RobotState::Wheel` carries `appliedDuty`/`glitchCount`/`wedged`/
      `cmdDuty` and a raw-vs-observed position/velocity pair; all
      projected into telemetry with a scale/bound consistent with the
      existing wire-budget accounting.
- [ ] `MotorConfigPatch`'s wire schema is UNCHANGED (no `.proto` field
      added/removed/retyped) — confirm via `git diff` on
      `src/protos/config.proto` showing no changes from this ticket.
- [ ] A `CONFIG` patch carrying `kp`/`ki` reaches `MoveQueue`'s PID
      instances (not `Devices::Motor`); a patch carrying `travel_calib`
      reaches `Devices::Motor::applyGains()` (not `MoveQueue`) — both
      demonstrated in a test, not just code-reviewed.
- [ ] `Config::TuningSnapshot`'s persisted-format shape is UNCHANGED
      (Migration Concerns) — only WHERE `handleConfig()` applies the gain
      subset changes.

## Testing

- **Existing tests to run**: `app_robot_loop_harness.cpp`'s CONFIG-patch
  scenarios (expect updates for the new routing destination, not deletion).
- **New tests to write**: a routing-split test asserting gain fields land
  on `MoveQueue`'s PID and `travel_calib` lands on `Devices::Motor`,
  independently; a telemetry round-trip test for each new `RobotState`
  field.
- **Verification command**: `uv run pytest`.
