---
id: '016'
title: Robot facade to AppContext struct
status: done
branch: sprint/016-robot-facade-to-appcontext-struct
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
- SUC-007
issues:
- replace-robot-facade-with-appcontext-struct.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 016: Robot facade to AppContext struct

> **Roadmap sprint** — lightweight plan only. Full artifacts (use cases,
> architecture, tickets) are produced in detail planning before execution.
> First of three sequenced sprints: **016 (this) → 017 → 018**.

## Goals

Replace the leaky `Robot` facade (`source/robot/Robot.{h,cpp}`) with an open
`struct AppContext` (single global `robot`) whose subsystem members are public,
so callers reach subsystems directly (`robot.driveController.stop(...)`) instead
of through delegation methods and nullable-pointer accessors. This is a
structural refactor with **no behavior change** — it lands first so the motion
work in 017/018 is written once against the final `robot.driveController.beginX()`
structure rather than the about-to-be-deleted facade.

## Problem

Of `Robot`'s ~38 methods, ~43% are pure passthroughs to MotorController/Odometry/
DriveController/sensors and ~28% are trivial getters; every subsystem it "owns" is
a singleton, so the encapsulation buys nothing while forcing callers through
indirection. See issue `replace-robot-facade-with-appcontext-struct.md`.

## Solution

`struct AppContext` with public members, wired in an AppContext constructor
(subsystems keep their existing constructors). Delete pure passthroughs; keep
genuine cross-cutting orchestration (`controlCollectSplitPhase`, `otosCorrect`,
sensor reads, `distanceDrive`, telemetry) as AppContext member functions. Push
device-specific logic down into the HAL class that owns it
(`OtosSensor::readTransformed`, `Servo` angle tracking). Migrate callers
(`CommandProcessor` ~55–60 sites, `LoopScheduler`, `WedgeTest`, `main.cpp`)
lowest-risk-first, then delete `Robot.{h,cpp}`.

## Success Criteria

- Clean build (`python3 build.py --clean`); host tests pass.
- Bench (robot on stand): all drive verbs (S/T/D/G/VW/STOP), GRIP, ZERO, OTOS
  verbs, PORT, STREAM/SNAP telemetry behave identically to pre-refactor.
- `caps=` still lists otos/line/color/gripper correctly (the `is_initialized()`
  rewrite). No `Robot` references remain in the tree.

## Scope

### In Scope

- New `source/robot/AppContext.{h,cpp}` replacing `Robot.{h,cpp}`.
- `OtosSensor::readTransformed(const RobotConfig&)`; `Servo` angle tracking +
  `currentAngle()`.
- Caller migration: `CommandProcessor`, `LoopScheduler`, `WedgeTest`, `main.cpp`.
- Delete `Robot.{h,cpp}` and dead methods.

### Out of Scope

- Any motion-control / velocity-profile / MotionCommand work (sprints 017–018).
- Behavior changes to drive verbs, telemetry, or sensors.

## Test Strategy

Lean on the existing host test suite for regression (no behavior change expected).
Standing bench-acceptance gate: encoders + wheels + sensors all read; drive verbs
exercised on the stand via `uv run rogo`. Clean build mandatory before any bench
flash.

## Architecture Notes

Member declaration order in `AppContext` is load-bearing (must match init order).
HAL devices stay external statics in `main.cpp` bound to AppContext reference
members. `Communicator` is dropped from the struct (zero callers). Detailed
disposition table and sequencing live in the issue and will be finalized in the
architecture-update during detail planning.

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | HAL additions: OtosSensor::readTransformed and Servo angle tracking | — |
| 002 | Add AppContext struct and wire in main.cpp | 001 |
| 003 | Migrate LoopScheduler to AppContext | 002 |
| 004 | Migrate WedgeTest to AppContext | 003 |
| 005 | Migrate CommandProcessor to AppContext | 004 |
| 006 | Delete Robot.h/Robot.cpp and dead code; bench regression gate | 005 |

Tickets execute serially in the order listed.
