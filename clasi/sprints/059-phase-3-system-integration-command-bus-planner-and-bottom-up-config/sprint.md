---
id: 059
title: Phase 3 - System integration, command bus, planner, and bottom-up config
status: planning-docs
branch: sprint/059-phase-3-system-integration-command-bus-planner-and-bottom-up-config
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
issues:
- message-based-subsystem-architecture.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 059: Phase 3 - System integration, command bus, planner, and bottom-up config

## Goals

Wire the Phase 2 subsystems (`subsystems::Drive2` and `subsystems::Sensors`) into
the running robot by introducing the Planner (`MotionController2`), the command-queue
bus drain/route layer, bottom-up configuration via typed config projections, and
finally cutting over `loopTickOnce` to the message-driven ordered tick. This sprint
completes the umbrella issue `message-based-subsystem-architecture.md`.

## Problem

Phase 2 built `Drive2` and `Sensors` as standalone message-contract units, but the
live robot still runs through the old `Drive::periodic()`/`loopTickOnce` imperative
path. There is no message-driven Planner, no command-queue bus routing outbound
`CommandBatch`es from subsystem ticks, no projection-based subsystem configuration,
and no ordered tick that enforces the sense-before-actuate discipline at the
subsystem boundary.

## Solution

Introduce the Planner (`MotionController2`) as a new class wrapping the existing
`MotionController` logic behind the `apply(PlannerCommand)` / `tick(now) ->
CommandBatch` / `state()` / `configure(PlannerConfig)` contract. Add a bounded bus
drain+route step that drains `OutCommand`s from returned `CommandBatch`es through the
existing `CommandProcessor` verb router with a `push_front` safety priority path. Add
a `toPlannerConfig(RobotConfig)` projection. Generalize live config routing via a new
`subsystem:` annotation on `robot_config.schema.json`. Then swap `loopTickOnce` to
the message-driven ordered tick (parity-gated final ticket).

## Success Criteria

- `MotionController2` (Planner) builds and passes planner-isolation tests for timed,
  turn, and distance goals asserting the returned `CommandBatch{DrivetrainCommand{twist}}`
  sequence matches the expected trapezoid/heading profile.
- Bus drain+route handles `OutCommand`s with bounded-cascade guard and
  `priority=true` push_front safety priority.
- Bottom-up config: `configure()` called on each subsystem post-construction from
  typed projections; live SET routes to the owning subsystem via `subsystem:` schema
  annotation.
- `loopTickOnce` rewired to the ordered tick; VW and TURN walk end-to-end with
  byte-plausible parity against the pre-sprint baseline.
- `uv run python -m pytest` green at baseline 2380/2 (the 2 pre-existing failures
  are `tag_offset_mm.z` schema tests, unrelated); new planner-isolation tests added.
- `python build.py --clean` zero errors.
- Bench smoke on tovez confirms telemetry is consistent and a safe on-stand spin
  matches the old path.

## Scope

### In Scope

- `MotionController2` (Planner) class in `source/superstructure/` composing the
  existing `MotionController` logic behind the message-contract API.
- `toPlannerConfig(RobotConfig)` projection function.
- Planner-isolation sim tests under `tests/simulation/unit/`.
- Command-queue bus drain+route: bounded cascade (`max_iters=8`), `priority=true`
  push_front, routing `OutCommand`s through existing `CommandProcessor` verb dispatch.
- Bottom-up config: post-construction `configure()` per subsystem; `subsystem:`
  annotation on `robot_config.schema.json`; SET routing to `drive.configure()` /
  `planner.configure()` / `sensors.configure()`.
- `SetPose` / `SI` verb routing to `drive2.apply(SetPose)`.
- Ordered-tick cutover of `loopTickOnce` (parity-gated, FINAL ticket).
- Bench smoke on tovez (safe: telemetry + on-stand rotation only).

### Out of Scope

- Binary / protobuf serialize-deserialize (ASCII wire unchanged).
- System, Config-registry, Debug command message families (deferred).
- Swerve extension.
- Mecanum / Togov field driving (bench-stand validation only in this sprint).
- Any modification of existing `Drive::periodic()` or `MotionController` imperative
  logic other than wrapping it for the new contract.

## Test Strategy

All tests run via `uv run python -m pytest` from the project root.

- **Planner-isolation tests** (new, in `tests/simulation/unit/test_planner_subsystem.py`):
  construct `MotionController2` on injected pose; feed `PlannerCommand` via `apply()`;
  call `tick()` N times; assert the returned `CommandBatch` twist + yaw-rate sequence
  matches the expected trapezoid/heading profile. No robot, no comms.
- **Parity tests** (new, in `tests/simulation/unit/test_059_ordered_tick_parity.py`):
  walk a VW and a TURN end-to-end through the new `loopTickOnce`; assert byte-plausible
  parity with the pre-sprint golden-TLM baseline.
- **Existing suite**: must remain green at 2380 passed / 2 pre-existing failures.
- **Device build**: `python build.py --clean` must emit zero errors.

## Architecture Notes

See `architecture-update.md` for the full design. Key decisions:

- Planner is `MotionController2` — new class, additive, wraps existing logic.
- RETURN model for `CommandBatch`; bus drain is the caller's (scheduler's) responsibility.
- Bounded cascade: `max_iters=8` per tick; exceeding it is an EVT not a crash.
- Bottom-up config via `subsystem:` annotation + projection functions; registry stays for live SET.
- Ordered-tick cutover is a FINAL, parity-gated ticket that can be reverted.

## GitHub Issues

(None currently linked.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Planner subsystem (MotionController2) | — |
| 002 | Planner-isolation sim tests | 001 |
| 003 | Command-queue bus drain and route | 001 |
| 004 | Bottom-up config and live SET routing | 001, 003 |
| 005 | Ordered-tick cutover (parity-gated) | 002, 003, 004 |
| 006 | Bench smoke verification | 005 |

Tickets execute serially in the order listed.
