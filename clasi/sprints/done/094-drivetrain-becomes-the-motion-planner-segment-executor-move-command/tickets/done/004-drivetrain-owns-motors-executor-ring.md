---
id: "094-004"
title: "Drivetrain owns motors + executor + ring"
status: done
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

- [x] `Drivetrain`'s constructor becomes `Drivetrain(Subsystems::Hardware&
      hardware)`; the reference is stored, not copied.
- [x] `Drivetrain::tick()`'s signature becomes `tick(uint32_t now,
      Rt::WorkQueue<Motion::Segment, 8>& segmentIn,
      Rt::WorkQueue<msg::DrivetrainCommand, 8>& driveIn)` (harmonization
      deviation: `driveIn` is a `WorkQueue`, not the ticket's originally
      written `Mailbox` — the freshly-rebased base's `Rt::Blackboard`
      already carries `driveIn` as a `WorkQueue<msg::DrivetrainCommand,8>`
      since sprint 093's own FIFO-queue rework; this ticket did not revert
      that) — the `motors[]`/`motorCount` arguments are gone; `tick()`
      resolves its own bound pair via `hardware_.state(port)` internally,
      with the same range assert `drivetrain.cpp:184-188` already had (the
      `- 1` base conversion itself is now GONE, not merely moved:
      `Hardware::state()`/`motor()` already take a 1-based port and do
      their own out-of-range clamping, so there is nothing left to
      convert — see drivetrain.h's tick() doc comment).
- [x] `driveIn` is drained (one command per tick, FIFO pop) and applied
      FIRST, before `segmentIn` — a WHEELS/NEUTRAL command from `driveIn`
      clears the segment ring and Drivetrain switches to direct mode for
      that tick (escape-hatch-wins precedence, per the communicator
      issue).
- [x] `segmentIn` (when `driveIn` did not just preempt) is drained in full
      into the internal ring each tick; the ring executes its head segment
      via the owned `Motion::SegmentExecutor`, popping on completion and
      starting the next.
- [x] `NEUTRAL` (from `STOP`) clears the ring and arms the executor's
      graceful decel-to-zero instead of an instant zero-velocity command —
      confirmed by `drivetrain_harness.cpp`'s
      `scenarioStopMidSegmentGracefulDecelNoReverseCreep` (the measured
      plant velocity decays smoothly toward zero and never reverses sign
      mid-segment).
- [x] Writes are staged via `hardware_.motor(port).apply(leftCmd/rightCmd)`
      — `Hal::DrivetrainToHardwareCommand`, `hasCommand()`, `takeCommand()`
      are all deleted from `Drivetrain`.
- [x] `Drivetrain::state()` sources `enc=`/`vel=` fields from
      `hardware_.state(port)` (measured), preserving the existing
      "measured, not commanded" telemetry semantic
      (`drivetrain.cpp:239-246`'s prior commanded-target behavior is
      replaced, not kept — the new source is genuinely measured).
- [x] `governRatio()` is kept as-is (unchanged math, `drivetrain.cpp:115-153`).
- [x] **Staging-only verification**: implemented as a host unit test
      (`tests/sim/unit/drivetrain_harness.cpp`'s
      `scenarioStagingOnlyNoI2CWriteUntilExplicitHardwareTick`) — a real
      `Subsystems::NezhaHardware` + `Subsystems::Drivetrain` against the
      HOST_BUILD scripted `I2CBus` fake: `Drivetrain::tick()` alone (which
      stages a WHEELS command through `hardware_.motor(port).apply()`)
      asserts `bus.txnCount() == 0`; only an explicit `hardware.tick()`
      call afterward increases it. Confirmed `NezhaMotor::setVelocity()`
      remains staging-only — no exception was needed.
- [x] Sim tests: enqueue-then-execute-then-pop for a single segment
      (`scenarioSingleSegmentEnqueueExecutePop`); escape-hatch preemption
      (`S` mid-segment clears the ring immediately,
      `scenarioEscapeHatchPreemptionClearsRingImmediately`); `STOP`
      mid-segment triggers the graceful decel
      (`scenarioStopMidSegmentGracefulDecelNoReverseCreep`, asserted via
      `Drivetrain::state()`'s measured velocity trending to zero, never
      reversing sign).
- [x] `just build-sim` succeeds; `uv run python -m pytest` stays green.

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

## Completion Notes (2026-07-09)

This ticket landed together with 094-005 against the freshly-rebased
`sprint/094` branch, whose base is the NEW comms-only bare `main()` loop
(not the old `Rt::MainLoop`-centric design this ticket was originally
written against) — see 094-005's own completion notes for the integration
shape. The Drivetrain rewrite itself:

- `source/subsystems/drivetrain.h`/`.cpp` fully rewritten per the AC above.
  `Drivetrain` now holds `Hardware& hardware_`, owns `Motion::
  SegmentExecutor executor_` + `Rt::WorkQueue<Motion::Segment, 8> ring_`,
  and a `bool segmentMode_` flag distinguishing SEGMENT mode (ring/executor
  drives output) from DIRECT mode (mode_/commandedWheelTargets() drives
  output, the pre-094 escape-hatch path). A new `configureMotion(const
  msg::PlannerConfig&)` method forwards to `executor_.configure()` — the
  boot-only jerk-limit knob 094-005 seeds from both composition roots.
- `governRatio()`'s math is byte-for-byte unchanged; it now runs uniformly
  over DIRECT-mode targets (TWIST/WHEELS) and SEGMENT-mode targets (the
  executor's body twist, converted via the same `BodyKinematics::
  inverse()` the TWIST arm always used).
- **Staging-only verification result**: PASSED, no exception raised. See
  AC above — `NezhaMotor::setVelocity()`/the whole `Hal::Motor::apply()`
  chain remains staging-only; `Drivetrain::tick()` alone issues zero I2C
  bus transactions; only an explicit `hardware.tick()` (the flip-flop's
  own `REQUEST_DUE`/`COLLECT_DUE` schedule) flushes them. This confirms
  the "stage now, flush next `Hardware::tick()` pass" latency model 094-005
  relies on.
- A design-note reference implementation existed in a stash on this branch
  (`094-004-005-wip-old-base-salvage`, targeting the OLD pre-rebase base)
  and was read for the Drivetrain rewrite's shape per the team-lead's
  explicit permission — the actual code here was written fresh against the
  new base, not copy-pasted, and required no `git stash pop`.
- Test file: `tests/sim/unit/drivetrain_harness.cpp` (new, replaces the old
  079-003-era harness) + `tests/sim/unit/test_drivetrain.py` (rewritten
  compile-and-run driver, now linking the SegmentExecutor/Ruckig chain and
  a real `SimHardware`/`NezhaHardware` instead of bare `msg::*` fixtures).
  `tests/sim/unit/configurator_harness.cpp` also needed a one-line fixup
  (`Drivetrain drivetrain;` → `Drivetrain drivetrain(hardware);`, 9
  call sites) and `tests/sim/unit/test_configurator.py` needed its compile
  flags/source list restored to the Ruckig-driven `gnu++20`/
  `-fno-exceptions`/`-fno-rtti` shape (094-002 had dropped that need when
  Planner was parked; `Rt::Configurator` holding a `Drivetrain&` means it
  transitively re-needs it now that `Drivetrain` owns a
  `Motion::SegmentExecutor`).
