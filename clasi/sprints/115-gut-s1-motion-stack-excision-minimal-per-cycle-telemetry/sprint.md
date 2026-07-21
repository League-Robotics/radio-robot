---
id: '115'
title: 'Gut S1: motion-stack excision + minimal per-cycle telemetry'
status: roadmap
branch: sprint/115-gut-s1-motion-stack-excision-minimal-per-cycle-telemetry
worktree: false
use-cases: []
issues:
- telemetry-frame-tightening-amendment-to-gut-s1.md
- sim-loop-hook-registration-race-with-tick-thread.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 115: Gut S1: motion-stack excision + minimal per-cycle telemetry

## Goals

- Tag the current tree (`pre-gut-motion-stack`) and capture a baseline bench
  telemetry log (seq continuity / drop rate) for later soak comparison.
- Delete the motion stack (executor, pilot, heading_source, vendored Ruckig,
  the unused measurement-ring/interpolation scaffolding) down to a minimal
  "command controlled speed" firmware base.
- Rewrite the telemetry frame per the tightening amendment: timestamped
  `EncoderReading`/`OtosReading` objects, one `flags` bit-string, a single
  ack slot, packed `line`/`color` words — emitted **every loop iteration**
  (primary period = cycle period, 20 ms), closing the
  `kcycle-kprimaryperiod-mismatch.md` stale-label bug.
- Bump the persisted-tuning schema version 1→2 for the blob layout change
  (drop the planner slot) so old blobs don't silently misdecode.
- Do the minimum forced host touch: `protocol.py` rework, one sim-config
  file, and a new `tlm_log.py` CSV logging tool — the host-side dataset
  source all future analysis (including sprint 117's estimator) reads from.
- Verify on hardware at the stage gate: the loop runs without lockups, the
  robot drives, encoders track, and the existing deadman still neutralizes
  on host silence (S1 keeps TWIST+deadman; MOVE replaces it in sprint 116).

## Problem

Weeks of motion-control work on the executor/pilot/Ruckig stack never
produced a completing tour (turn non-termination, terminal wedge), and the
planned predict-to-now arc was pre-empted before execution. That stack —
`src/firm/motion/` (executor 914+710 lines, jerk_trajectory), `app/pilot.*`,
`app/heading_source.*`, `vendor/ruckig/` (~5,900 LOC, ~27% of the
nRF52833's flash) — is the bulk of the firmware's complexity and none of it
serves the minimal use case. Separately, the current telemetry frame is
executor-era: untimestamped flat encoder fields, a bare OTOS pose with
velocities silently dropped, nine standalone bools plus two bitmasks, an
ack ring, and a primary period that doesn't match the cycle period despite
a doc comment claiming it does — all of which undermines any future
prediction/estimation work planned on top of it.

## Solution

Delete the motion stack and ring scaffolding wholesale (the tag preserves
everything for recovery) and rebuild simplest: velocity-PID motor control,
per-cycle encoder + OTOS (+ rate-limited line/color) reads into the central
latest-value structure (`RobotLoop::frame_`), and the tightened telemetry
frame emitted every cycle. Keep the existing TWIST+deadman command surface
through this stage — it is what keeps the robot drivable at the S1 gate —
and cut it over to the bounded MOVE protocol in sprint 116. One coherent
unit (S0 tag/baseline, then S1 excision+telemetry as a single non-compiling-
in-between change), ending flashable and hardware-verified.

## Success Criteria

- Robot drives via twist commands (forward/reverse/pivot) with encoders
  tracking commanded sign and magnitude.
- The deadman still neutralizes motors within its lease after one bounded
  command then silence (unchanged in S1).
- Telemetry frame streams every cycle (~50 Hz) with per-source sample
  timestamps, OTOS velocities riding the wire, packed `line`/`color` words
  showing plausible changing values, and `flags` tracking
  status/fault/event correctly.
- `tlm_log.py` captures a drive session to CSV with per-reading timestamps.
- 10-minute soak at ≥5-10 Hz alternating commands: no reboot, seq
  monotonic at the doubled rate, drop rate at or better than the S0
  baseline, no motion-timing regression from the added sensor reads.
- Persisted-tuning version bump verified: the one-time tuning-store wipe +
  radio-channel re-pick is observed once, then a config patch survives a
  power-cycle at the new (85-byte) layout.
- `uv run python -m pytest` green on the surviving suite; `python build.py`
  builds firmware + host sim lib clean; ~164 KiB flash freed.

## Scope

### In Scope

- S0: git tag + baseline bench telemetry capture.
- S1 deletions: `src/firm/motion/`, `app/pilot.*`, `app/heading_source.*`,
  `vendor/ruckig/`, `measurement_ring.h`/`interpolation.h` + their test
  harnesses; matching CMake edits.
- Proto surgery: `envelope.proto` (delete `Move` arm 20 → reserve, delete
  `ConfigDelta.planner` → reserve 3; `Twist` arm stays through S1);
  `telemetry.proto` full rewrite per the tightening amendment; delete
  `planner.proto`/`motion.proto`; `gen_boot_config.py` planner-emission
  removal.
- Firmware: `main.cpp`/`robot_loop.{h,cpp}`/`drive.{h,cpp}`/
  `odometry.{h,cpp}`/`telemetry.{h,cpp}` reshape per the gut and amendment
  issues; rate-limited line/color reads into the frame; persisted_tuning
  version bump (1→2, 110→85-byte blob).
- Sim: strip executor/pilot/heading_source from `sim_harness.h` and the
  wire test codec.
- Host (bench-toolchain-forced minimum only): `protocol.py` decode rework,
  `sim_boot_config.py` planner-enum removal, `nezha_state.py`/
  `robot_state.py` adapter, new `src/tests/bench/tlm_log.py`.
- Test sweep: delete the ~40 executor/pilot/tour/ruckig harnesses and
  bench scripts; edit survivors (app_robot_loop, app_drive, app_telemetry,
  config_gate, persisted_tuning, sim_harness_configure, wire codec suite).
- Optional rider: `sim-loop-hook-registration-race-with-tick-thread.md` —
  a small, co-located sim-stability fix — included only if it doesn't
  jeopardize the hardware gate.

### Out of Scope

- The MOVE protocol cutover (Move arm 21, StopCondition, MoveQueue,
  deadman deletion, legacy Twist deletion) — that is sprint 116; S1
  deliberately keeps TWIST+deadman so the robot stays drivable here.
- Host motion/tour code deletion (`planner/`, `path/`, `nav/`, TestGUI
  tour/turn modules, ~30-40 files) — stays in place, dormant, as a
  separate future follow-up per the gut issue's stakeholder decision.
- Any estimator/prediction work (sprint 117) — this sprint only produces
  the dataset source (the tightened per-cycle telemetry log) that 117
  consumes.

## Test Strategy

`uv run python -m pytest` green on the surviving suite; `python build.py`
builds firmware + host sim lib clean; then the hardware gate on the stand
per `.claude/rules/hardware-bench-testing.md`: sensors alive, wheels
drive with encoders tracking, round-trip over serial, deadman
neutralize-on-silence, STOP immediate-neutral while streaming twists,
`tlm_log.py` capturing a session at ~50 Hz with plausible line/color and
OTOS velocities, a ≥10-minute soak at the doubled telemetry rate, and the
one-time tuning-store wipe + radio re-pick observed followed by a config
patch surviving a power-cycle.

## Architecture

(Architecture for this sprint's change, sized to the change — a
one-paragraph note for a trivial sprint, a fuller write-up with
component/data-model detail for a substantial one. May read "N/A —
trivial" when the change has no architectural impact.)

### Architecture Overview

(High-level structure and component relationships, if applicable.)

### Design Rationale

(Significant decisions with alternatives considered and reasoning, if
applicable.)

### Migration Concerns

(Data migration, backward compatibility, deployment sequencing — or
"None" if not applicable.)

## Use Cases

(Use cases sized to the change — may read "N/A — trivial" for small
sprints that don't warrant new or updated use cases.)

### SUC-001: (Title)
Parent: UC-XXX

- **Actor**: (Who)
- **Preconditions**: (What must be true before)
- **Main Flow**:
  1. (Step)
- **Postconditions**: (What is true after)
- **Acceptance Criteria**:
  - [ ] (Criterion)

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning document is complete (sprint.md, including its
      Architecture and Use Cases sections)
- [ ] Architecture review passed (or skipped, for changes with no
      architectural impact)
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|

Tickets execute serially in the order listed.
