---
id: 084
title: Firmware motion verbs and config/pose-set surface
status: done
branch: sprint/084-firmware-motion-verbs-and-config-pose-set-surface
use-cases: []
issues:
- firmware-closed-loop-motion-verbs.md
- firmware-config-and-pose-set-surface.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 084: Firmware motion verbs and config/pose-set surface

## Goals

Restore closed-loop motion (`D`/`T`/`R`/`TURN`/`RT`/`G`/`S`/`STOP` + stop
clauses + the full `mode=` state machine) and the config/pose-set command
surface (`SET`/`GET`/`SI`/`ZERO`/OTOS verbs) on the new `source/` tree,
porting the relevant logic out of the parked `source_old` stack onto sprint
082's `Subsystems::Drivetrain` + `Subsystems::PoseEstimator`. This gives
TestGUI's command rows, tours, and calibration/pose-set features a firmware
command surface to bind to.

**Delivers:** the full closed-loop motion + config/pose-set command surface
that TestGUI's advanced features (084 downstream) need.

**Dependency:** depends on 082 (pose estimate for goal closure + `mode=`/
`TLM`), already done. Pairs two issues scoped together because they touch
the same firmware surface and both feed `PoseEstimator`/`Drivetrain`.

**Note:** this sprint is large and covers two related issues
(`firmware-closed-loop-motion-verbs.md` and
`firmware-config-and-pose-set-surface.md`). It may split into two sprints
(motion / config) at detail-planning time if ticket count or architecture
review indicates it should.

## Problem

TestGUI's tours, command rows, and Operations panel (Sync-Pose, Zero-
Encoders, Set-Origin, calibration push) all need firmware verbs that were
deliberately not carried into the greenfield `source/` tree — closed-loop
motion, the config registry, and the OTOS command surface only exist in
`source_old`.

## Solution

Port `source_old/superstructure/*`, `source_old/control/*`,
`source_old/commands/{MotionCommands,ConfigCommands,OtosCommands}.*`, and
`source_old/robot/ConfigRegistry.*` onto the new HAL/Drivetrain and
command-plane discipline (handlers stage into the `DevLoopState` outbox;
`devLoopTick` drains — sprint 079), restoring the verbs as top-level per
`docs/protocol-v2.md`.

## Success Criteria

Against the sim: `D 200 200 500` moves true pose ~500 mm; `RT 9000` rotates
~90° (within plant tolerance); `stop=` clauses honored; `mode=` returns to
`I` at completion. `SET tw=...` then `GET` round-trips and visibly changes
drivetrain behavior; `SI x y h` teleports the fused pose; `ZERO enc`
rezeroes `enc=`/`encpose=`; OTOS verbs ack against the sim, `ERR nodev` on
hardware. Hardware bench gate: closed-loop drive/turn on the stand, encoders
proportional, round-trip over serial.

## Scope

### In Scope

- Motion executor above `Drivetrain`: port
  `source_old/superstructure/{Planner,Superstructure,PlannerConfig}.*` and
  `source_old/control/{BodyVelocityController,HaltController,
  MotorController,VelocityController}.*`.
- Verbs restored as top-level: `D`, `T`, `R`, `TURN`, `RT`, `G`, `S`, `STOP`,
  plus `stop=<kind>:<args>` clauses (`source_old/commands/MotionCommands.*` +
  `messages/planner.h`).
- Extend `mode=` from 082's minimal `I`/`S` to the full `I/S/T/D/G/...` set.
- `SET`/`GET` config registry: port `source_old/commands/ConfigCommands.*` +
  `source_old/robot/ConfigRegistry.*` + `DefaultConfig.cpp` boot defaults,
  wired to `msg::DrivetrainConfig` and `PoseEstimator::configure()`. Wire
  keys stay stable per `.claude/rules/coding-standards.md`.
- Pose-set: `SI` (set fused pose), `ZERO enc` (rezero encoders +
  `PoseEstimator` accumulator).
- OTOS command surface: port `source_old/commands/OtosCommands.*`
  (`OZ`/`OI`/`OL`/`OA`) against the sim's `SimOdometer`; `ERR nodev` on real
  hardware until the deferred real-OTOS driver lands.

### Out of Scope

Wiring these verbs into TestGUI's UI (tours, GOTO runner, Operations panel,
calibration push) — that's sprint 085. The real-hardware OTOS driver is a
separate deferred issue
(`clasi/issues/nezha-hardware-otos-driver-for-new-source-tree.md`).

## Test Strategy

Sim-level acceptance tests for each motion verb and config/pose-set verb,
plus the hardware bench gate (stand-mounted drive/turn, encoder round-trip
over serial) per `.claude/rules/hardware-bench-testing.md`. Detail-planning
phase sizes out specific test files.

## Architecture Notes

No architecture changes finalized yet — this roadmap entry precedes
detail-planning. The architecture-update.md (written at detail-planning
time) will cover the motion-executor layering above `Drivetrain` and the
config-registry wiring into `PoseEstimator::configure()`.

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed (self-review APPROVE, `architecture_review` gate recorded 2026-07-06)
- [x] Stakeholder has approved the sprint plan (`stakeholder_approval` gate
      recorded 2026-07-06; approved as ONE sprint, no split, all 9 tickets
      as designed)

## Tickets

Stakeholder-approved 2026-07-06 (recorded via `stakeholder_approval` gate);
all nine tickets created. Tickets execute serially in the order listed.

| # | Title | Issue | Depends On |
|---|-------|-------|------------|
| [001](tickets/001-motion-executor-core-planner-ramp-and-stop-condition-engine.md) | Motion executor core: Planner ramp + stop-condition engine | firmware-closed-loop-motion-verbs.md | — |
| [002](tickets/002-velocity-and-bounded-drive-verbs-s-t-d-stop-plus-stop-clauses.md) | Velocity and bounded-drive verbs: S / T / D / STOP + `stop=` clauses | firmware-closed-loop-motion-verbs.md | 001 |
| [003](tickets/003-arc-and-turn-verbs-r-turn-rt.md) | Arc and turn verbs: R / TURN / RT | firmware-closed-loop-motion-verbs.md | 002 |
| [004](tickets/004-go-to-verb-g.md) | Go-to verb: G | firmware-closed-loop-motion-verbs.md | 003 |
| [005](tickets/005-mode-machine-extend-tlm-mode-to-i-s-t-d-g.md) | Mode machine: extend `TLM mode=` to I/S/T/D/G | firmware-closed-loop-motion-verbs.md | 004 |
| [006](tickets/006-config-registry-top-level-set-get.md) | Config registry: top-level `SET`/`GET` | firmware-config-and-pose-set-surface.md | 001 |
| [007](tickets/007-pose-set-surface-si-and-zero-enc.md) | Pose-set surface: `SI` + `ZERO enc` | firmware-config-and-pose-set-surface.md | 006 |
| [008](tickets/008-otos-command-surface-oi-oz-or-op-ov-ol-oa.md) | OTOS command surface: OI/OZ/OR/OP/OV/OL/OA | firmware-config-and-pose-set-surface.md | 007 |
| [009](tickets/009-sim-and-hardware-bench-verification.md) | Sim + hardware bench verification (completes both issues) | both | 005, 008 |

See `architecture-update.md` Step 6 (Design Rationale) and the "Sizing /
structure" section for the full rationale, key-by-key `SET`/`GET` mapping,
and the no-split recommendation.
