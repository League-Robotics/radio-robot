---
id: '007'
title: 'RobotState struct: src/firm/types/robot_state.h, the dependency-free blackboard'
status: done
use-cases:
- SUC-004
depends-on:
- '002'
github-issue: ''
issue: robot-state-blackboard-one-struct-for-all-shared-state-and-telemetry.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# RobotState struct: src/firm/types/robot_state.h, the dependency-free blackboard

## Description

Define `src/firm/types/robot_state.h` — the one dependency-free struct
that becomes the sole cross-subsystem and cross-tree data contract
(sprint 124 architecture, Step 3). Dependency-free means cstdint-level
includes only, no `msg::`/`messages/` types — `firm/types` becomes a
second shared floor both `src/firm` and `src/motion` stand on, alongside
`Motion::WheelSink` (a deliberate second base↔motion crossing, per
Decision in the architecture's Step 4 dependency graph).

Sectioned per the blackboard issue's struct sketch: `Time`, `Wheel`
(×2, left/right — sensed position/velocity/sampleTime/connected +
commanded velocity, PLUS the new `positionEpoch` field this sprint adds
per Decision 6), `Otos`, `Perception`, `Pose`, `Estimate` (ZOH bases:
value + velocity + basisTime, so "predict to time t" is a pure function
over the state), `Command`, `Health`. Field lists are finalized in this
ticket against `Motion::StateEstimator::Input`'s existing 16 fields (the
natural seed — already float-typed, already covers most of what
`RobotState` needs) plus what `App::Telemetry` needs to project.

This ticket is the struct DEFINITION only. Wiring `RobotLoop`/
`Telemetry` to actually build and consume it is ticket 009 — do not
restructure `cycle()` here.

## Acceptance Criteria

- [x] `robot_state.h` compiles standalone with no `messages/`/`msg::`
      includes (grep-enforceable: `grep -n "messages/\|msg::"
      src/firm/types/robot_state.h` returns nothing).
- [x] Every field of `Motion::StateEstimator::Input` (current 16 fields:
      `encLeftPosition`, `encLeftVelocity`, `encLeftTime`,
      `encRightPosition`, `encRightVelocity`, `encRightTime`, `poseX`,
      `poseY`, `poseHeading`, `twistVX`, `twistVY`, `twistOmega`,
      `otosPresent`, `otosHeading`, `otosOmega`, `otosTime`) is
      representable in `RobotState` — no information loss vs. today.
- [x] `RobotState::Wheel` carries `positionEpoch` (new, per architecture
      Decision 6) alongside `position`/`velocity`/`sampleTime`/
      `connected`/`cmdVelocity`.
- [x] `src/motion/state_estimator.h` is updated to `#include
      "firm/types/robot_state.h"` in place of its own private `Input`
      struct declaration (the struct itself may still alias/typedef for
      compatibility if the ticket's implementation finds that cleaner —
      implementer's call, as long as no duplicate near-identical struct
      remains).
- [x] All fields are plain data (no methods beyond trivial accessors),
      trivially copyable — no pointers, no heap.

## Testing

- **Existing tests to run**: `motion_tests` build (the standalone,
  Python-free `src/motion` test suite) must stay green.
- **New tests to write**: a standalone compile check asserting
  `robot_state.h` has no forbidden includes (can be a simple build-time
  or lint-time assertion rather than a runtime test).
- **Verification command**: `uv run pytest` plus the `motion_tests`
  build and the firmware ARM build (`just build`).

## Completion Notes (2026-07-25)

**Final field list, by section, and why:**

- `Time { cycleStart [ms], cycleBusy [us], cyclePeriod [us] }` — writer:
  `RobotLoop::cycle()`. Matches the sketch verbatim; these three already
  existed on `App::Telemetry::Frame`.
- `Wheel { position [mm], velocity [mm/s], sampleTime [ms], connected,
  positionEpoch (uint8_t), cmdVelocity [mm/s] } wheelLeft, wheelRight` —
  sensed fields mirror `Devices::Motor::position()/velocity()/connected()`
  (today read directly by `RobotLoop`, per Decision 1); `cmdVelocity` is
  `App::Drive`'s staged target; `positionEpoch` is the new field
  Decision 6 requires (an 8-bit wrap counter, incremented whenever
  `RobotLoop` calls the wheel's existing, unmodified
  `Devices::Motor::rebaseline()` — no device-layer change, per the ticket's
  own AC).
- `Otos { present, connected, x/y [mm], heading [rad], v_x/v_y [mm/s],
  omega [rad/s], sampleTime [ms] }` — mirrors `msg::OtosReading` field
  names (`v_x`/`v_y`, not `vx`/`vy` — matches the existing wire type and
  `naming-and-style.md`'s subscript convention) plus the `present`/
  `connected` freshness/health bits `App::applyOtosSample()` already
  tracks.
- `Perception { line, color (packed uint32), lineFresh, colorFresh }` —
  matches the sketch; `line`/`color` hold the already-packed wire-layout
  words `RobotLoop`'s existing `packLine()`/`packColor()` helpers produce.
- `Pose { x/y [mm], heading [rad], v_x/v_y [mm/s], omega [rad/s] }` —
  `Motion::Odometry`'s dead-reckoned world pose plus the same-cycle fused
  body twist (today `BodyKinematics::forward()`, computed inline in
  `RobotLoop`).
- `Estimate { wheelLeft/wheelRight: WheelEstimate{distance,velocity,
  basisTime,valid}; body: BodyEstimate{x,y,heading,v_x,v_y,omega,
  basisTime,valid}; innovations: Innovations{heading,omega,valid} }` —
  copied field-for-field from `Motion::StateEstimator`'s own private
  `WheelEstimate`/`BodyEstimate`/`Innovations` structs (the ZOH-basis
  shape the issue calls out explicitly), now the canonical shape those
  three structs conceptually mirror.
- `Command { mode (Types::Mode), moveActive, v_x [mm/s], omega [rad/s] }`
  — `mode` is a project-owned enum mirroring `msg::DriveMode`'s value set
  without depending on it (`Types::Mode`, UpperCamelCase enumerators, per
  `naming-and-style.md`). `v_x`/`omega`, not the issue sketch's
  `targetVx`/`targetOmega`: reused the same names `Pose`/`BodyEstimate`
  already use, disambiguated by the `command.` section prefix rather than
  a `target`-prefixed compound name — avoids mashing a word prefix onto a
  mathematical subscript and stays consistent with how every other
  section disambiguates by section, not per-field prefix.
- `Health { i2cSafetyNetCount, commsMalformedCount, wedgeLatch,
  moveTimeout, shapingDisabled }` — the issue's own sketch lists
  `deadmanExpired` instead of the last two; `App::Deadman` is fully
  retired (no live signal exists — its old telemetry flag is declared but
  permanently unwired), so per this ticket's own instruction to derive
  the field list from what genuinely exists, `deadmanExpired` was dropped
  and `moveTimeout`/`shapingDisabled` substituted — both ARE genuinely
  live today (`RobotLoop`'s own `kFlagFaultMoveTimeout`/
  `kFlagFaultShapingDisabled` derivation).

**`Motion::StateEstimator::Input` was rewired NOW, not deferred to 009.**
AC 4 explicitly requires `state_estimator.h` to `#include
"firm/types/robot_state.h"` in place of its private `Input` struct this
ticket — that isn't optional, so it had to happen regardless of the
008/009 split. Implementation: `Input` is now `using Input =
Types::RobotState;` (a type alias, not a second struct) — no duplicate
near-identical struct remains, satisfying the AC's own explicit allowance
for an alias. Because `RobotState` reorganizes the same 16 values into
sections (`wheelLeft.position` vs. the old flat `encLeftPosition`, etc.),
every place that previously constructed a flat `Input` had to move to the
new sectioned field paths for the build to compile at all — this is why
`src/motion/state_estimator.cpp` (the `update()` body), one construction
site in `src/firm/app/robot_loop.cpp` (the cycle-local `estimatorInput`
variable — still cycle-local and throwaway, NOT `RobotLoop`'s future
`state_` member; that promotion is still 009's job), and
`app_state_estimator_harness.cpp`'s `makeFrame()` builder all needed their
field accesses updated to the new paths. This is a mechanical field-path
adaptation, not a `cycle()` restructure: same call site, same number of
statements, same control flow — nothing about ownership, phase
sequencing, or the loop schedule changed. Doing it now (rather than
carrying the alias-vs-old-struct decision into 009) means 009 starts from
a tree where there is exactly one `RobotState`-shaped type in existence,
not two.

**Decision 1 compliance, confirmed explicitly:** this ticket did **not**
move device ownership out of `RobotLoop` — `motorL_`/`motorR_`/`otos_`/
`line_`/`color_` remain direct `RobotLoop` members, read directly in
`cycle()`, exactly as before. Did **not** create a `Sensors` subsystem.
Did **not** add named bus-phase methods (`requestLeft()`/`collectLeft()`/
`requestRight()`/`collectRight()`) to `Drive` — `App::Drive` is untouched
by this ticket. The only `robot_loop.cpp` change is the field-path
adaptation described above inside the existing `estimatorInput`
construction block; no lines were added, removed, or reordered around it,
and `cycle()`'s own phase sequencing is byte-for-byte unchanged. The
Drive/Sensors device-ownership reshuffle remains sprint 125's scope in
full, per Decision 1.

**What's NOT yet true (for 008/009 to know):** `RobotState` is defined
but not yet published anywhere — no `RobotLoop`-owned `state_` member
exists yet; the only place any of its fields are populated today is the
cycle-local `estimatorInput` variable, and only its `wheelLeft`/
`wheelRight`/`pose`/`otos` fields at that (the fields `Motion::
StateEstimator::update()` actually reads). `Wheel::cmdVelocity`/
`positionEpoch`, all of `Command`, and all of `Health` are defined but
never assigned by any production code path yet — ticket 008 (packed
telemetry + the `positionEpoch` rebaseline trigger) and ticket 009 (the
`RobotLoop`/`Telemetry` restructure to one persistent, once-per-cycle-
published `state_` member, plus `TelemetrySecondary`'s deletion) are
where that lands. `App::Telemetry` has not been touched at all by this
ticket — no projection method exists yet.

**Verification, verbatim (independently re-run and confirmed by
team-lead):**
- `just build-sim` — clean, `Built target firmware_host`.
- `just build` (ARM) — clean: `firmware hex v0.20260724.2 -> MICROBIT.hex`,
  `host sim lib v0.20260724.2 (HOST_BUILD)`.
- `motion_tests` (standalone CMake build,
  `cmake -S src/motion -B src/motion/build && cmake --build
  src/motion/build --target motion_tests`) — 3/3 passed
  (`motion_stop_condition_tests`, `motion_velocity_shaper_tests`,
  `motion_move_queue_chained_tests`), verified both incrementally and
  from a clean configure.
- Full suite `uv run python -m pytest` — **1478 passed, 2 skipped, 9
  xfailed, 2 xpassed**, zero failures (up 2 from the 1476 pre-ticket
  baseline, both new tests in
  `src/tests/sim/unit/test_firm_types_robot_state.py`).
