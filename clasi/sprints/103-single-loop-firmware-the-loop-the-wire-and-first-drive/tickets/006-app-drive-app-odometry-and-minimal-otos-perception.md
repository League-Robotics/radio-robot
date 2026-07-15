---
id: '006'
title: app/Drive, app/Odometry, and minimal OTOS perception
status: open
use-cases: [SUC-006]
depends-on: ['001', '003']
github-issue: ''
issue: single-loop-firmware-p3-p7-continuation.md
completes_issue:
  single-loop-firmware-p3-p7-continuation.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# app/Drive, app/Odometry, and minimal OTOS perception

## Description

Build `source/app/drive.{h,cpp}` and `source/app/odometry.{h,cpp}`: `Drive`
converts a body twist into wheel velocity targets via the unchanged
`BodyKinematics::inverse()` and stages them onto the two `NezhaMotor`
leaves; `Odometry` integrates wheel motion back into a world pose estimate
via the unchanged `BodyKinematics::forward()`. Also add a minimal,
OTOS-only perception step (owned by this ticket, not a separate module —
see architecture-update.md Step 7 Open Question 1): one `Otos` sample per
cycle feeding `Telemetry`'s `otos`/`otos_connected` fields. The archived
plan's full 3-way `Perception` round-robin (otos|line|color) is
deliberately NOT built this sprint — `line`/`color` have no telemetry
field to feed yet.

Depends on ticket 001 (`msg::Twist` type) and ticket 003 (bare leaves, not
`DeviceBus`).

## Acceptance Criteria

- [ ] `Drive::setTwist(v_x, omega)` stores the target; `Drive::stop()`
      zeroes it; `Drive::tick()` calls `BodyKinematics::inverse(v_x, omega,
      trackWidth, vL, vR)` and stages `vL`/`vR` onto the two `NezhaMotor`
      leaves via their existing `setVelocity()` setter — no additional
      scaling/sign logic duplicated in `Drive` beyond what `inverse()`
      already computes.
- [ ] `Drive::stop()` results in both wheel targets reaching 0 within one
      cycle of the next `tick()`.
  and `NezhaMotor`'s own `pidEnabled_` stays at its default `true` (PID
  path) — this ticket does not touch PID enable/disable.
- [ ] `Odometry::integrate()` reads both motors' position (or per-cycle
      delta), calls `BodyKinematics::forward()` (not a hand-rolled
      equivalent), and accumulates world `x`/`y`/`theta`.
- [ ] A host-buildable test proves `Odometry::integrate()` accumulates
      correctly for (a) a straight-line case (equal `vL`/`vR`) and (b) a
      pure-rotation case (`vL == -vR`), against `BodyKinematics::forward()`'s
      own known-correct output for those inputs.
- [ ] No new state duplicated between `Drive`/`Odometry` and the
      `NezhaMotor` leaves' own cached position/velocity — `Odometry` reads
      the leaves' existing accessors, it does not maintain a shadow copy.
- [ ] `Otos` is sampled at least once per cycle (or per a documented slot
      schedule this ticket defines) and the result reaches `Telemetry`
      (ticket 005) before that cycle's frame is built — a direct call or a
      small shared struct, this ticket's own choice, documented.
- [ ] `line`/`color` steady-state sampling is explicitly NOT built this
      ticket (documented in code comments and completion notes, not
      silently absent) — `Preamble` (ticket 007) still detects their
      presence at boot.

## Implementation Plan

**Approach**: `Drive`/`Odometry` are thin — the actual math lives entirely
in the unchanged `BodyKinematics::inverse()`/`forward()`
(`source/kinematics/body_kinematics.{h,cpp}`), confirmed during this
sprint's own planning to already match these two classes' needs exactly
(no kinematics code changes required). Write `Drive`/`Odometry` against
`NezhaMotor`'s ACTUAL public surface (`setVelocity(float)`,
`position()`/`velocity()` accessors — read `nezha_motor.h` directly, not
just the archived plan's prose) and `Otos`'s actual `begin()`/tick/read
surface (`source/devices/otos.h`).

**Files to create/modify**:
- `source/app/drive.h`, `source/app/drive.cpp` (new)
- `source/app/odometry.h`, `source/app/odometry.cpp` (new)

**Testing plan**:
- Existing tests to run: none directly (new files); confirm
  `BodyKinematics`'s own existing tests (if any under `tests/sim/unit/`)
  stay untouched/green, since `Drive`/`Odometry` are new callers, not
  modifiers, of that code.
- New tests to write: the straight-line and pure-rotation `Odometry`
  accumulation tests (Acceptance Criteria above); a `Drive::tick()` test
  confirming staged wheel targets match `BodyKinematics::inverse()`'s
  direct output for a representative `(v_x, omega)` pair, using a
  `HOST_BUILD` fake/scripted `NezhaMotor` or a direct assertion against
  the leaf's `setVelocity()` call (whichever the leaf's own `HOST_BUILD`
  seam supports — confirm during implementation).
- Verification command: `uv run python -m pytest tests/sim/unit/ -k "drive or odometry"`
  (once the test files exist).

**Documentation updates**: a code comment on the minimal-OTOS-perception
decision (cite architecture-update.md Step 7 Open Question 1) so a future
reader knows the 3-way round-robin was deliberately deferred, not
forgotten.
