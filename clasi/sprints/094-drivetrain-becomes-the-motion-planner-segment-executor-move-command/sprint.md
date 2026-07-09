---
id: "094"
title: "Drivetrain becomes the motion planner — segment executor + MOVE command"
status: planning-docs
branch: sprint/094-drivetrain-becomes-the-motion-planner-segment-executor-move-command
use-cases: ["SUC-001", "SUC-002", "SUC-003", "SUC-004", "SUC-005"]
issues:
  - drivetrain-becomes-the-motion-planner-segment-executing-subsystem.md
  - communicator-drivetrain-motion-command-segment.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 094: Drivetrain becomes the motion planner — segment executor + MOVE command

## Goals

Turn `Subsystems::Drivetrain` from a message-fed faceplate into the robot's
**motion planner**: it resolves its own wheel motors from the hardware
container, owns a relocated Ruckig-based segment executor (lifted from
`Subsystems::Planner`), and executes a small FIFO of relative motion
**segments**. Add one new wire verb, **`MOVE <distance> <direction>
<finalHeading> [limits…]`**, that the command layer hands straight to the
Drivetrain via a blackboard segment queue — no Planner, no motion-loop
round-trip. Preserve, byte-for-byte, the two hard-won pieces of prior work
this sprint builds on top of: the Nezha I2C flip-flop split-phase bus
schedule (renamed `serviceBus`, body untouched) and the presolved graceful
decel-to-zero that eliminated 092/093's terminal reverse-creep.

This sprint builds directly on sprint 093's gutted loop (`Rt::MainLoop` =
Hardware + Drivetrain only, four verbs `S`/`STOP`/`PING`/`HELLO`,
`bb.driveIn` the one drive mailbox). It formalizes the design already laid
out in the two issues below (marked "plan/land together").

## Problem

Today, motion planning lives in `Subsystems::Planner` (parked, unwired since
093), which owned the Ruckig trajectory machinery and emitted a
`msg::DrivetrainCommand{TWIST}` routed through the blackboard to
`Subsystems::Drivetrain` — a thin faceplate holding no motor references.
That message plumbing between planner and drivetrain was pure overhead, and
093 deliberately left the seam clean for this sprint to close: fold motion
planning directly into the Drivetrain, and give the Communicator one compact
command surface to drive it with, "with no hiccups" (stakeholder,
2026-07-08).

## Solution

- Lift the non-GOTO internals of `Planner` (`JerkTrajectory` linear +
  rotational channels, stop-condition evaluation, `MotionBaseline`, the
  divergence replan, the compile-split dead-time, the presolved
  decel-to-zero) into a new, Drivetrain-owned `Motion::SegmentExecutor`.
- Add a new pose-free, encoder-only `Motion::Segment` type
  (`source/motion/segment.h`): `distance` / `direction` / `finalHeading` +
  per-segment motion-limit overrides. A differential drive decomposes one
  segment into up to three phases: PRE_PIVOT → TRANSLATE → TERMINAL_PIVOT.
- Rename `NezhaHardware`/`SimHardware`/`Hardware`'s `tick()` to
  `serviceBus()` — a pure rename; the flip-flop body, dt=0 guard, and the
  093 sim-40ms/hw-80ms `kDeadTime` compile split are untouched.
- Rewrite `Drivetrain` to hold `Hardware&`, resolve its bound wheel pair via
  `hardware.motor(port)`/`hardware.state(port)`, own the executor + an
  8-slot segment ring, and stage wheel setpoints via `motor(p).apply()` —
  flushed at `serviceBus`'s own cadence (timing unchanged).
- Add `MOVE` (new verb) + rewrite `STOP` to trigger the graceful
  decel-to-zero instead of an instant brake. Add a minimal pull-based `TLM`
  verb for measured `enc=`/`vel=` telemetry (synchronous reply, no
  loop-originated output — consistent with 093 Decision 1).
- Delete `Motion::VelocityRamp` (GOTO-only, no longer needed). Physically
  relocate `Subsystems::Planner` out of `source/` (parked, not deleted) —
  `codal.json`'s `"application": "source"` glob means a file left in
  `source/` still compiles, and deleting `velocity_ramp.h` breaks
  `planner.cpp`'s include.

## Success Criteria

- `MOVE` drives a straight segment, a pure in-place turn, and a
  translate-then-terminal-pivot segment in sim, each draining to a graceful
  stop (no reverse-creep).
- `S`/`STOP` still work as the direct escape hatch; `STOP` now triggers the
  graceful decel-to-zero.
- `arm-none-eabi-size build/MICROBIT` is measured before/after re-linking
  Ruckig into the live tick path; firmware fits flash.
- The I2C flip-flop's timing (compile-split dead-time, per-pass cadence) is
  provably unchanged.
- HITL bench gate (093-style, stand-mounted) confirms the above on real
  hardware.

## Scope

### In Scope

- `Motion::Segment` + `Motion::SegmentExecutor` (the executor lift).
- `Hardware`/`NezhaHardware`/`SimHardware`: `tick()` → `serviceBus()` rename.
- `Drivetrain` rewrite: motor refs, executor + ring ownership, staged writes.
- `MainLoop`/composition roots (`main.cpp`, `sim_api.cpp`)/`Blackboard`:
  `segmentIn` queue, loop reorder, boot jerk-config defaults.
- Command surface: `MOVE`, graceful `STOP`, pull-based `TLM`.
- Delete `velocity_ramp.{h,cpp}`; relocate `planner.{h,cpp}` out of `source/`.
- HITL standing bench gate.

### Out of Scope

- Absolute `TURN`, `STOP_HEADING`/`STOP_POSITION`, GOTO/pursuit — all need
  the fused pose 093 removed with `PoseEstimator`. Deferred until pose
  estimation returns (see the GOTO-revival follow-up issue this sprint
  files).
- The push `done` completion event (needs the loop-originated-output seam
  093 Decision 1 declared deferred-or-obsolete) — segment completion is
  polling-observable via `TLM` this sprint instead.
- Global `SET jmax`/`GET jmax` wire keys + `Rt::Configurator` revival — 094
  ships the jerk knob as per-segment `MOVE` args plus baked boot defaults.
- Re-parsing `D`/`TURN`/`RT` onto segments, or reviving them as live verbs —
  `MOVE` is the one new verb this sprint adds; the others stay unregistered
  exactly as 093 left them.

## Test Strategy

Each ticket except the HITL bench ticket is independently verifiable in sim:
`just build-sim` + `uv run python -m pytest` (collects `tests/sim/`). New
host unit tests cover the executor in isolation (straight / in-place-turn /
translate-then-pivot / auto-decel-on-drain / stop-mid-segment / no
reverse-creep) and the command surface end-to-end (`MOVE`/`S`/`STOP`/`TLM`
over `sim_command()`). The 093 four-verb focused suite must stay green
throughout. The final ticket is a stakeholder-run HITL bench gate on the
stand (`.claude/rules/hardware-bench-testing.md`) — not sim-automatable.

## Architecture Notes

See `architecture-update.md` for the full design, diagrams, and design
rationale. Key constraints: flash budget (Ruckig re-linked into the live
tick path — mandatory `arm-none-eabi-size` gate), I2C flip-flop timing
(pure rename, staging-only `Motor::apply()` verified), and the presolved
graceful decel-to-zero (lifted from `planner.cpp`'s `armDistanceStopDecel`/
`armRotationalStopDecel`/`armVelocityStopDecel` plus the literal-0.0f snap).

## GitHub Issues

(None linked yet.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Segment type + SegmentExecutor (the lift) | — |
| 002 | Delete VelocityRamp, park Planner | 001 |
| 003 | Hardware container: `tick` → `serviceBus` | — |
| 004 | Drivetrain owns motors + executor + ring | 001, 003 |
| 005 | Loop + composition roots + blackboard (flash-size gate) | 002, 004 |
| 006 | Command surface: MOVE + graceful STOP + TLM | 005 |
| 007 | Standing hardware bench gate [HITL] | 006 |

Tickets execute serially in the order listed.
