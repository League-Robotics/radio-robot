---
id: "094-004"
title: "Drivetrain owns motors + executor + ring"
status: open
use-cases: ["SUC-001", "SUC-002", "SUC-004"]
depends-on: ["094-001", "094-003"]
issue: drivetrain-becomes-the-motion-planner-segment-executing-subsystem.md
---

# 094-004: Drivetrain owns motors + executor + ring

## Description

Rewrite `Subsystems::Drivetrain` to hold a `Hardware&`, resolve its bound
wheel pair via `hardware.motor(port)`/`hardware.state(port)` instead of
receiving observations as `tick()` arguments, and own one
`Motion::SegmentExecutor` plus an 8-slot segment ring. Writes are staged
via `hardware.motor(port).apply(cmd)` — flushed at `serviceBus`'s own
cadence (094-003) — instead of held in a `Hal::DrivetrainToHardwareCommand`
for `MainLoop` to route. `hasCommand()`/`takeCommand()` are deleted;
nothing is left to route.

This ticket also carries the sprint's second hard-constraint verification:
confirming `Hal::Motor::apply()`/`NezhaMotor::apply()` do no immediate I2C
write (staging-only), which is the load-bearing assumption behind "stage
now, flush next `serviceBus` pass = identical one-pass latency to today's
`routeOutputs()` chain."

This ticket does not yet wire `Drivetrain`'s new constructor signature into
`main.cpp`/`sim_api.cpp`/`MainLoop` — that is 094-005. This ticket's own
sim/host tests construct a `Drivetrain` directly against a test double or a
real `SimHardware` instance.

## Acceptance Criteria

- [ ] `Drivetrain`'s constructor becomes `Drivetrain(Subsystems::Hardware&
      hardware)`; the reference is stored, not copied.
- [ ] `Drivetrain::tick()`'s signature becomes `tick(uint32_t now,
      Rt::WorkQueue<Motion::Segment, 8>& segmentIn,
      Rt::Mailbox<msg::DrivetrainCommand>& driveIn)` — the `motors[]`/
      `motorCount` arguments are gone; `tick()` resolves its own bound pair
      via `hardware_.state(port)` internally, with the same `- 1` base
      conversion and range assert `drivetrain.cpp:184-188` already has
      (moved to wherever the container is now indexed).
- [ ] `driveIn` is drained and applied FIRST, before `segmentIn` — a
      WHEELS/NEUTRAL command from `driveIn` clears the segment ring and
      Drivetrain switches to direct mode for that tick (escape-hatch-wins
      precedence, per the communicator issue).
- [ ] `segmentIn` (when `driveIn` did not just preempt) is drained in full
      into the internal ring each tick; the ring executes its head segment
      via the owned `Motion::SegmentExecutor`, popping on completion and
      starting the next.
- [ ] `NEUTRAL` (from `STOP`) clears the ring and arms the executor's
      graceful decel-to-zero instead of an instant zero-velocity command —
      confirmed by a test that issues NEUTRAL mid-segment and asserts the
      sampled body twist decays smoothly to exactly `0.0f`, never changing
      sign (no reverse-creep).
- [ ] Writes are staged via `hardware_.motor(port).apply(leftCmd/rightCmd)`
      — `Hal::DrivetrainToHardwareCommand`, `hasCommand()`, `takeCommand()`
      are all deleted from `Drivetrain`.
- [ ] `Drivetrain::state()` sources `enc=`/`vel=` fields from
      `hardware_.state(port)` (measured), preserving the existing
      "measured, not commanded" telemetry semantic
      (`drivetrain.cpp:239-246`'s prior commanded-target behavior is
      replaced, not kept — the new source is genuinely measured).
- [ ] `governRatio()` is kept as-is (unchanged math, `drivetrain.cpp:115-153`).
- [ ] **Staging-only verification**: a code-level check (a comment citing
      the exact call chain, or better, a host unit test that stages a
      velocity command and asserts no `I2CBus` write occurs until an
      explicit `serviceBus()` call) confirms `NezhaMotor::setVelocity()`/
      `setDutyCycle()` remain staging-only — no I2C write happens outside
      `NezhaHardware::serviceBus()`'s `COLLECT_DUE`-gated `tick()`
      dispatch. If this assumption is found to be violated, STOP and raise
      an exception rather than silently reworking the loop-order model.
- [ ] Sim tests: enqueue-then-execute-then-pop for a single segment;
      escape-hatch preemption (`S` mid-segment clears the ring
      immediately); `STOP` mid-segment triggers the graceful decel
      (asserted via `Drivetrain::state()`'s reported twist trending to
      zero, never reversing sign).
- [ ] `just build-sim` succeeds; `uv run python -m pytest` stays green.

## Implementation Plan

**Approach**: Rewrite `drivetrain.h`/`drivetrain.cpp` in place (this is a
"rewrite," not an "extend" — the class comment's own framing of the old
faceplate design, `drivetrain.h:1-57`, needs a matching rewrite describing
the new motion-planner role, mirroring how thoroughly-commented the
existing file already is). Do not change `governRatio()`'s math or
`commandedWheelTargets()`'s TWIST/WHEELS/NEUTRAL dispatch shape beyond what
is needed to source ratio-governor inputs from `hardware_.state(port)`
instead of the old `motors[]` argument.

**Files to modify**:
- `source/subsystems/drivetrain.h` — full rewrite of the class comment,
  constructor, `tick()` signature, private members (add `Hardware&
  hardware_`, `Motion::SegmentExecutor executor_`, the segment ring
  storage); remove `hasCommand()`/`takeCommand()`/`heldCommand_`.
- `source/subsystems/drivetrain.cpp` — matching implementation rewrite.

**Files to create**: none (094-001 already created `segment.h`/
`segment_executor.{h,cpp}`; this ticket only consumes them).

**Testing plan**: extend/add sim tests under `tests/sim/unit/` constructing
a `Drivetrain` against a real `SimHardware` (or a lightweight test double if
one already exists for `Hardware`) — do not yet go through
`sim_command()`/the wire (that needs 094-005/006's loop+command wiring).
Cover: single-segment enqueue/execute/pop; multi-segment queue draining in
order; escape-hatch preemption; graceful-stop-on-NEUTRAL with the
no-reverse-creep assertion; the staging-only verification test.

**Documentation updates**: `drivetrain.h`'s own file-header class comment
needs a full rewrite (it currently documents the pre-094 faceplate design
in detail — sprint 079/087/090's own commentary — and must be replaced with
094's motion-planner design, following the same level of detail the
existing file already sets as the project's convention for this file).
