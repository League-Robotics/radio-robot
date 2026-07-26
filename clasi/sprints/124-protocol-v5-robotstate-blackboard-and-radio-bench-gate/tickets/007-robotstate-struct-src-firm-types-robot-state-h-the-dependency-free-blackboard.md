---
id: '007'
title: 'RobotState struct: src/firm/types/robot_state.h, the dependency-free blackboard'
status: in-progress
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

- [ ] `robot_state.h` compiles standalone with no `messages/`/`msg::`
      includes (grep-enforceable: `grep -n "messages/\|msg::"
      src/firm/types/robot_state.h` returns nothing).
- [ ] Every field of `Motion::StateEstimator::Input` (current 16 fields:
      `encLeftPosition`, `encLeftVelocity`, `encLeftTime`,
      `encRightPosition`, `encRightVelocity`, `encRightTime`, `poseX`,
      `poseY`, `poseHeading`, `twistVX`, `twistVY`, `twistOmega`,
      `otosPresent`, `otosHeading`, `otosOmega`, `otosTime`) is
      representable in `RobotState` — no information loss vs. today.
- [ ] `RobotState::Wheel` carries `positionEpoch` (new, per architecture
      Decision 6) alongside `position`/`velocity`/`sampleTime`/
      `connected`/`cmdVelocity`.
- [ ] `src/motion/state_estimator.h` is updated to `#include
      "firm/types/robot_state.h"` in place of its own private `Input`
      struct declaration (the struct itself may still alias/typedef for
      compatibility if the ticket's implementation finds that cleaner —
      implementer's call, as long as no duplicate near-identical struct
      remains).
- [ ] All fields are plain data (no methods beyond trivial accessors),
      trivially copyable — no pointers, no heap.

## Testing

- **Existing tests to run**: `motion_tests` build (the standalone,
  Python-free `src/motion` test suite) must stay green.
- **New tests to write**: a standalone compile check asserting
  `robot_state.h` has no forbidden includes (can be a simple build-time
  or lint-time assertion rather than a runtime test).
- **Verification command**: `uv run pytest` plus the `motion_tests`
  build and the firmware ARM build (`just build`).
