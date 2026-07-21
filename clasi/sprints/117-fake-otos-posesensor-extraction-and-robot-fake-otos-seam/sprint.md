---
id: '117'
title: 'Fake OTOS: PoseSensor extraction and ROBOT_FAKE_OTOS seam'
status: roadmap
branch: sprint/117-fake-otos-posesensor-extraction-and-robot-fake-otos-seam
worktree: false
use-cases: []
issues:
- on-chip-fake-otos-test-device.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 117: Fake OTOS: PoseSensor extraction and ROBOT_FAKE_OTOS seam

## Goals

Sprint 3 of the predict-to-now odometry arc (see
`clasi/issues/predict-to-now-odometry-estimator-ring-capture-dump-validation-trajectory-controller.md`).
Extract a `Devices::PoseSensor` interface (the union of what the app graph
calls on `Otos`), make `Otos` implement it, retype consumers to
`PoseSensor&`, and build `FakeOtos` — a build-selectable test device that
synthesizes an OTOS pose from encoder kinematics — behind a
`ROBOT_FAKE_OTOS` CMake seam. Covers
`clasi/issues/on-chip-fake-otos-test-device.md` in full.

**First action on pickup**: `on-chip-fake-otos-test-device.md`'s own
Context section states the real OTOS is "on a servo port" — this is
**wrong** and must be corrected before any other work on this issue.
Ground truth (stakeholder-corrected, 3×): the robot's OTOS is **rigidly
mounted on the robot's I2C bus, address 0x17**. The bench's current AUTO
fallback to encoder heading is because `otos.present()` reads false for
other reasons (no chip ever detected at that address in the current bench
configuration), not because the chip lives on a servo port. Never write
"OTOS on a servo/servo port" in any artifact this sprint produces.

## Problem

The real OTOS derives its pose from its own onboard optical/IMU fusion —
independent ground truth the estimator (sprint 116) can be checked
against. On the bench today, the OTOS is not present/trusted, so every
estimator OTOS-fusion path is untested against live OTOS-shaped data.
`FakeOtos` gives the bench a self-consistent "OTOS-present" regime
(matching how the sim closure gate already validates with OTOS heading)
without requiring a physical, trusted OTOS on every bench session — while
being explicit that it validates plumbing/latency/fusion math, not
accuracy (it derives from the same encoders it is meant to complement —
see the arc issue's own "Fake-OTOS circularity" risk note).

## Solution

- Extract `Devices::PoseSensor` — the interface `Otos` already implicitly
  satisfies for every app-graph call site; `Otos` retypes to implement it
  explicitly, consumers (`HeadingSource`, `RobotLoop`'s wiring, etc.)
  retype their reference to `PoseSensor&`.
- `Devices::FakeOtos` (`src/firm/devices/fake_otos.{h,cpp}`): holds
  `Motor&` L/R + trackWidth, integrates diff-drive forward kinematics
  over encoder deltas at the real chip's ~20ms cadence, publishes
  `PoseRecord` into the `otos` ring (sprint 115's container — same ring
  the real chip publishes into, so downstream consumers/dumps don't need
  to know which is live), `present()` always true, zero bus traffic.
- Build seam: `ROBOT_FAKE_OTOS` CMake option at `main.cpp`'s composition
  root; production hex byte-identical with it off.

## Success Criteria

Fake-OTOS hex on the stand: `otos.present()` true, the `otos` ring
carries plausible synthetic pose records, the sprint-116 estimator fuses
heading/omega against them the same way it would against real OTOS data.

## Scope

### In Scope

- `Devices::PoseSensor` interface extraction + `Otos`/consumer retyping.
- `Devices::FakeOtos` + `ROBOT_FAKE_OTOS` build seam.
- Correcting `on-chip-fake-otos-test-device.md`'s wrong "servo port"
  premise.

### Out of Scope

- Any change to the REAL `Otos` chip-facing behavior beyond the interface
  retype (production path stays byte-identical).
- External/camera pose (sprint 118).
- Trajectory controller (sprint 119).

## Test Strategy

Sim first (fake-OTOS path is trivially simulable — same kinematics the
sim plant already uses), then stand: flash `ROBOT_FAKE_OTOS` hex, confirm
`otos.present()`/ring contents/estimator fusion per Success Criteria;
flash the normal (fake-OTOS-off) hex and confirm a byte-identical
production image (or documented, justified diff). `uv run python -m
pytest` + sim suite; `just build-clean`; `mbdeploy deploy`; hardware
bench gate per `.claude/rules/hardware-bench-testing.md`.

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
