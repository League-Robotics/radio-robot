---
id: "118"
title: "PoseFix revival, external pose fusion, and validation threshold ratification"
status: roadmap
branch: sprint/118-posefix-revival-external-pose-fusion-and-validation-threshold-ratification
worktree: false
use-cases: []
issues: []
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 118: PoseFix revival, external pose fusion, and validation threshold ratification

## Goals

Sprint 4 of the predict-to-now odometry arc (see
`clasi/issues/predict-to-now-odometry-estimator-ring-capture-dump-validation-trajectory-controller.md`).
Revive `PoseFix` (extended with velocity) as a firmware-consumed
`CommandEnvelope` arm publishing into the `external` ring; run the full
loop-de-loop validation suite + notebook against real bench CSVs;
characterize the OTOS `VELOCITY_XL` scale bug; give the sim `OtosPlant`
real `v_x`/`v_y` (currently hard-zeroed). This sprint's gate is also the
arc's residual-threshold ratification gate — the precondition for sprint
119.

## Problem

The estimator (116) and fake-OTOS regime (117) validate plumbing and
internal consistency, but neither has independent ground truth: fake OTOS
derives from the same encoders it complements, and the real OTOS (once
trusted) is still onboard, not external. `PoseFix` (host-timestamped
camera observations, correctly stamped into the robot's own clock domain
via sprint 115's revived clock sync) is the arc's actual independent
ground truth. This sprint also converts the still-unratified RMS numbers
from 116/117 into stakeholder-approved accept thresholds — nothing in
119 can be judged "good enough" without them.

## Solution

- `PoseFix` (`drivetrain.proto`, reserved field 7) revived as a
  `CommandEnvelope` arm, extended with velocity (`v_x, v_y, omega`). Host
  stamps `t` via `clock_sync.py`'s `to_robot_time()` at the observation
  instant (not arrival) — radio delay is exactly why the stamp rides in
  the message. Firmware handler (the 099-008 consumer that never landed)
  publishes `PoseRecord` into the `external` ring; `reset`/
  `zero_encoders` semantics preserved for hard re-anchor.
- Estimator's external-fusion weight (`w_ext`, defaulted 0 since sprint
  116) gets real values now that there is real external data to fuse
  against — plumbing landed in 116, this sprint is what finally
  exercises it above zero.
- Full loop-de-loop validation suite + notebook run against real bench
  CSVs (not sim) — this is where the arc's methodology (leave-one-out,
  one-step-ahead RMS, position-integration projection) is applied for
  real, external-source-anchored ground truth.
- OTOS `VELOCITY_XL` scale characterization: `otos.h:280-286`'s
  documented-wrong LSB scale reuse (position LSB constants applied to a
  register block that documents a different native scale) gets a real
  bench-measured fix, unblocking the estimator's linear-velocity OTOS
  fusion weight (held at 0 since 116 specifically because of this).
- Sim `OtosPlant` `v_x`/`v_y` (`sim_plant.cpp:221-226`, currently
  hard-zeroed) — real values so sim-side OTOS-velocity fusion can be
  exercised before the bench.

## Success Criteria

Host-injected external poses land in the `external` ring with correct
robot-domain stamps. Stakeholder ratifies residual accept thresholds from
real bench RMS tables — this ratification is the explicit precondition
for sprint 119 (per the arc issue's own gate table); 119 must not start
before it.

## Scope

### In Scope

- `PoseFix` revival + velocity extension + firmware `external`-ring
  consumer.
- Full validation suite + notebook run against bench CSVs.
- OTOS `VELOCITY_XL` scale characterization + fix.
- Sim `OtosPlant` `v_x`/`v_y`.
- Stakeholder threshold-ratification gate.

### Out of Scope

- Trajectory controller / `Motion::Executor` changes (sprint 119) —
  turn/straight termination behavior is still unchanged after this
  sprint closes.

## Test Strategy

Sim first (OtosPlant `v_x`/`v_y` fix is sim-testable directly), then
stand with camera-sourced `PoseFix` injection — this is the sprint that
finally closes the loop against independent ground truth, so the bench
pass is the primary evidence, not a formality. `uv run python -m pytest`
+ sim suite; `just build-clean`; `mbdeploy deploy`; hardware bench gate
per `.claude/rules/hardware-bench-testing.md`; playfield camera setup per
`.claude/rules` vision/geofence guidance where applicable.

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
