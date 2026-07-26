---
id: '005'
title: Relocate MotorVelocityPid to Motion::WheelVelocityPid
status: open
use-cases:
- SUC-003
depends-on:
- '002'
github-issue: ''
issue: firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Relocate MotorVelocityPid to Motion::WheelVelocityPid

## Description

Move `Devices::MotorVelocityPid` (`src/firm/devices/velocity_pid.{h,cpp}`)
verbatim into `src/motion/wheel_velocity_pid.{h,cpp}` as
`Motion::WheelVelocityPid` — rename only (`Devices::` → `Motion::`), zero
control-law behavior change. Add it to `src/motion/CMakeLists.txt`'s
`motion` library sources and give it its own `motion_tests` unit test
carrying the SAME assertions the pre-move `Devices::MotorVelocityPid`
test had (a literal parity check, not a rewrite — the whole point of
"verbatim"). See sprint architecture Step 3 (`Motion::WheelVelocityPid`)
and Design Rationale Decision 1 (why this stays a standalone class rather
than folding into `MoveQueue` directly — kept separately unit-testable).

Do NOT wire it into `MoveQueue` yet — that's ticket 006. This ticket is
the mechanical relocation only.

## Acceptance Criteria

- [ ] `Devices::MotorVelocityPid` no longer exists;
      `Motion::WheelVelocityPid` exists with an identical public
      interface (`compute()`, `restGateEngaged()`).
- [ ] **[off-hardware]** `motion_tests` runs a `WheelVelocityPid` unit
      test with the SAME assertions as the pre-move
      `Devices::MotorVelocityPid` test (byte-for-byte comparable
      expected outputs) — zero behavior change, verified not assumed.
- [ ] `src/firm/devices/velocity_pid.{h,cpp}` are deleted, not left as
      dead/duplicate code.

## Testing

- **Existing tests to run**: whatever test currently exercises
  `Devices::MotorVelocityPid` — ported, not deleted-and-forgotten.
- **New tests to write**: none beyond the ported test (this ticket adds
  no new behavior).
- **Verification command**: `cmake --build src/motion/build --target
  motion_tests`.
