---
id: 009
title: 'Narrow MotorArmor: delete wedge detection, keep standstill-guarded reset dispatch'
status: open
use-cases:
- SUC-002
depends-on:
- '004'
github-issue: ''
issue: firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Narrow MotorArmor: delete wedge detection, keep standstill-guarded reset dispatch

## Description

Narrow `Devices::MotorArmor` (`src/firm/devices/motor_armor.h`): delete
`updateWedgeDetector()`, `wedged()`, `wedgeSuspect()` (now
`App::WheelObserver`'s job, ticket 004 — its innovation logic is the one
wedge signal, read by `Drive` and published into `RobotState` directly).
KEEP `processResetIfPending()`/`updateRestTracking()` (the standstill-
guarded reset dispatch) UNCHANGED — its production-caller count stays
zero, same as before this sprint (Design Rationale Decision 8: kept as a
decorator, not deleted or folded into `Drive`, since it wraps exactly one
`Motor` transparently and doesn't violate the "no reference webs"
principle). See sprint architecture Step 3 (`App::MotorArmor`).

## Acceptance Criteria

- [ ] `Devices::MotorArmor::wedged()`/`wedgeSuspect()`/
      `updateWedgeDetector()` are deleted, not stubbed.
- [ ] `processResetIfPending()`/`updateRestTracking()` and their existing
      tests are UNCHANGED in behavior — a diff review confirms no logic
      change to the reset-dispatch path.
- [ ] `Devices::MotorArmor`'s construction shape
      (`explicit MotorArmor(Motor& inner)`) is unchanged.

## Testing

- **Existing tests to run**: `devices_motor_armor_harness.cpp` — delete
  or update wedge-detection-specific scenarios; confirm reset-dispatch
  scenarios stay green unchanged.
- **New tests to write**: none (this is a deletion + confirmation
  ticket, not new behavior).
- **Verification command**: `uv run pytest`.
