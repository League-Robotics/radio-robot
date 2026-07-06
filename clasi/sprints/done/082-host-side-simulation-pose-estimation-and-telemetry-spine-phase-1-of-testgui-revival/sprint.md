---
id: 082
title: Host-side simulation pose estimation and telemetry spine (Phase 1 of TestGUI
  revival)
status: done
branch: sprint/082-host-side-simulation-pose-estimation-and-telemetry-spine-phase-1-of-testgui-revival
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
issues:
- plan-revive-testgui-against-the-new-tree-simulator.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 082: Host-side simulation pose estimation and telemetry spine (Phase 1 of TestGUI revival)

## Goals

Give the new `source/` firmware tree a **pose it can estimate** and a
**telemetry surface to report it on**. This is Phase 1 of the three-sprint
"revive TestGUI" program (`clasi/issues/plan-revive-testgui-against-the-new-tree-simulator.md`):
Phase 2 (closed-loop motion verbs + config, sprint 083) and Phase 3 (host/
TestGUI revival, sprint 084) are explicitly **out of scope** here and are not
planned by this document -- they are referenced only as downstream consumers
of what this sprint produces.

## Problem

Sprint 077's greenfield rebuild stood up a minimal `source/` tree whose only
wire surface is `PING/VER/HELP/ECHO/ID` + the `DEV` family. It has **no
telemetry stream** and **no consumer of `Hal::Odometer`** -- `Hal::SimOdometer`
(landing in sprint 081, ticket 003) exists as a concrete sensor leaf, but
nothing fuses its reading into a pose, and nothing reports any pose, encoder
state, or velocity over the wire. TestGUI (Phase 3) needs a streamed pose (for
its four colour-coded traces: camera/truth, encoder, OTOS, fused) and a
`mode=` idle signal to poll. This sprint closes that gap for the firmware
side only.

## Solution

1. **Pose estimation** -- a new `Subsystems::PoseEstimator` consumes wheel
   encoder positions (already-calibrated mm, from `Hal::Motor::position()`)
   for dead-reckoning (`encpose=`), and the raw odometer reading (from
   `Hal::Odometer::pose()`, when a leaf is present) to correct a 3-state
   (x, y, heading) EKF (`Hal::EkfTiny`, ported from `source_old/state/EKFTiny.*`
   against the already-vendored `libraries/tinyekf`) producing the fused
   estimate (`pose=`). The raw odometer reading itself is reported unfused as
   `otos=`.
2. **Telemetry surface** -- `STREAM <ms>` / `SNAP` commands (new
   `source/commands/telemetry_commands.*`) emit `TLM` frames (new
   `source/telemetry/tlm_frame.*`, ported from `source_old/robot/RobotTelemetry.cpp`)
   carrying `t= mode= seq= enc= vel= pose= encpose= otos= twist=`, wired
   through `source/dev_loop.cpp` so both the ARM path and the host sim (from
   sprint 081) emit identical frames.

See `architecture-update.md` for the full design, module list, and the
explicit list of old-tree features deliberately **not** ported this sprint
(velocity-channel EKF fusion, Mahalanobis gating, `otos_health=`, `fields=`
subscription, idle-rate streaming) -- each is a documented, named
simplification, not a silent omission.

## Success Criteria

- On the sprint-081 host sim: `TLM` `pose=`/`encpose=` track the ctypes
  ground-truth pose within the plant's tolerance; `otos=` diverges from truth
  by the configured `SimOdometer` error knob and re-converges to truth when
  all knobs are zeroed.
- `STREAM <ms>` and `SNAP` both produce well-formed `TLM` frames with all
  nine required fields (or the correct subset when hardware/odometer is
  absent), sharing one `seq=` counter.
- `mode=` reports `I` when the Drivetrain is not active and a minimal active
  value when a `DEV DT VW`/`WHEELS` drive is in progress (full state-machine
  semantics are sprint 083's job).
- Hardware bench gate (`.claude/rules/hardware-bench-testing.md`) run on the
  stand: encoders alive and incrementing proportionally to command, `enc=`/
  `encpose=` visible and moving correctly over `TLM`, round-trip over serial.
  **OTOS-specific bench checks are not satisfiable this sprint** -- no real
  `Hal::Odometer` leaf exists in the new tree yet (see Architecture Notes) --
  and this is called out explicitly in the bench report rather than silently
  skipped.

## Scope

### In Scope

- `Hal::EkfTiny` -- 3-state (x, y, heading) EKF core, ported from
  `source_old/state/EKFTiny.*` against `libraries/tinyekf`.
- `Subsystems::PoseEstimator` -- encoder dead-reckoning + EKF wiring, config
  read from the existing `msg::DrivetrainConfig` EKF fields.
- `Subsystems::Hardware::odometer()` -- a new, defaulted-nullptr virtual
  accessor so `devLoopTick` can reach whichever concrete `Hal::Odometer` leaf
  (if any) the active `Subsystems::Hardware` owner has, without an `#ifdef`.
- `source/telemetry/tlm_frame.*` + `source/commands/telemetry_commands.*` --
  `TLM` frame formatting and the `STREAM`/`SNAP` verbs.
- `source/dev_loop.{h,cpp}` and `source/main.cpp` wiring for both the
  estimator tick and periodic telemetry emission.
- `mode=` minimal idle/active value.

### Out of Scope

- Closed-loop motion verbs (`D`/`T`/`R`/`TURN`/`RT`/`G`/`S`), `stop=` clauses,
  full `mode=` state machine -- sprint 083.
- `SET`/`GET` config surface, `SI`/`ZERO`/OTOS calibration verbs (`OZ/OI/OL/OA`)
  -- sprint 083.
- Any `host/`/TestGUI code change -- sprint 084.
- A real-hardware `OtosSensor` (`Hal::Odometer` leaf for the physical OTOS
  chip) -- does not exist in the new tree and is not added by this sprint;
  see Architecture Notes and the ticket 005 gap note.
- Velocity-channel EKF fusion, Mahalanobis/chi-squared gating, P-inflation
  gate-recovery, `otos_health=`/`ekf_rej=` diagnostics, `STREAM fields=`
  subscription, D10 idle-rate/channel-rebinding refinements -- all explicitly
  deferred (see architecture-update.md Design Rationale).

## Test Strategy

- Host-side unit tests for `Hal::EkfTiny` (synthetic predict/correct
  sequences) and `Subsystems::PoseEstimator` (encoder-only dead-reckoning
  arithmetic, OTOS-absent fallback).
- Sim-level tests (`tests/sim/`, against the sprint-081 ctypes harness):
  `pose=`/`encpose=` vs. ground truth tolerance; `otos=` divergence/
  reconvergence against `SimOdometer`'s error knobs; `STREAM`/`SNAP` frame
  shape and shared `seq=` counter.
- Hardware bench gate per `.claude/rules/hardware-bench-testing.md`, scoped
  to what this sprint actually adds (encoders, `TLM` round-trip) -- OTOS bench
  checks explicitly reported as not-yet-possible.

## Architecture Notes

- **Dependency on sprint 081 (in progress)**: 081 delivers the host ctypes
  sim (`libfirmware_host`, `tests/_infra/sim/`), `Subsystems::Hardware`,
  `Subsystems::SimHardware`, `devLoopTick`, and the first concrete
  `Hal::Odometer` leaf (`Hal::SimOdometer`). **082 is verified against that
  sim and cannot execute until 081 closes.** 082 branches from `master`
  after 081 merges -- not from 081's own branch.
- **No real-hardware `Hal::Odometer` leaf exists.** `Subsystems::NezhaHardware`
  has no OTOS chip driver in the new tree (only `source_old` had one). This
  sprint's `Subsystems::Hardware::odometer()` seam defaults to `nullptr`;
  `Subsystems::PoseEstimator` degrades gracefully (dead-reckoning only, no
  fusion, `otos=` omitted from `TLM`) when it is. Porting the real OTOS I2C
  driver is out of scope and not scheduled by this document.
- **`mode=` is intentionally minimal.** Full state-machine semantics
  (I=idle vs. a motion verb executing) belong to sprint 083's motion verbs.
  This sprint defines the field and gives it an honest, minimal value
  (idle vs. an active `DEV DT` drive) -- not invented motion state.
- See `architecture-update.md` for the full module design, diagrams, and
  Design Rationale (including the scope-reduction decisions: 3-state EKF,
  no gating, no `fields=` subscription).

## GitHub Issues

(None filed directly against this sprint; tracked via the linked program
issue `clasi/issues/plan-revive-testgui-against-the-new-tree-simulator.md`,
which spans sprints 082-084 and stays open after this sprint closes.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Port `Hal::EkfTiny` -- 3-state EKF core | -- |
| 002 | `Subsystems::PoseEstimator` -- encoder dead-reckoning + OTOS fusion | 001 |
| 003 | `Subsystems::Hardware::odometer()` seam + dev-loop/main wiring | 002 |
| 004 | Telemetry surface -- `TLM` frame, `STREAM`/`SNAP` commands | 003 |
| 005 | Sim verification + hardware bench gate | 004 |

Tickets execute serially in the order listed.

**Optional split point (flagged, not decided here):** tickets 001-003 are the
estimation half; 004-005 are the telemetry half. If stand/calendar pressure or
review-cycle overhead makes one sprint awkward, closing after 003 and opening
a follow-on sprint for 004-005 is a reasonable, low-risk split -- mirroring
sprint 081's own "flagged, not decided here" sizing note. This document keeps
all five tickets in 082 because the dependency chain is already fully serial
either way.
