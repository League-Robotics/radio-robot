---
id: '125'
title: 'Firmware base hardening: characterization, gate, and freeze'
status: roadmap
branch: sprint/125-firmware-base-hardening-characterization-gate-and-freeze
worktree: false
use-cases: []
issues:
- firmware-base-hardening-characterization-gate-and-freeze.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 125: Firmware base hardening: characterization, gate, and freeze

## Goals

Third and final sprint of the firmware-base-hardening restructuring.
Characterizes the observer/duty stack sprint 124 built, scores it against
the numeric base gate, and — if green — declares the base FROZEN
(subsequent base changes require a stakeholder-signed issue).
ROADMAP-LEVEL ONLY — detail-planned once sprint 124 lands, since the gate
numbers and the characterization battery's exact per-wheel constants
depend on the concrete observer/`NezhaMotor` shape 124 ships.

## Problem

Sprint 124 lands a new base contract (duty primitive, observer,
shrunk `NezhaMotor`) but characterizing it — and deciding whether it is
good enough to freeze on — is deliberately separate work: characterization
constants must be measured and provenance-recorded, never swept into
existence, and the freeze decision is a stakeholder-visible gate, not an
implementation detail folded into the sprint that also changes the
contract being measured.

## Solution

Per `clasi/issues/firmware-base-hardening-characterization-gate-and-freeze.md`:
run the sim exactness check (observer vs. the first-order `WheelPlant`,
≤0.1 mm / ≤0.1 mm/s), the bench per-wheel step-response battery (dead
time, tau, deadband floor, reversal-dwell, both directions, several
speeds, on the stand, recorded per-robot in JSON with measurement
provenance, no-sweep rule enforced), and score the base gate (observer
fidelity, duty-write-path completion with `appliedDuty` reported,
telemetry pairing/rate/gap-free, zero-on-silence). If every item is
green, record the freeze declaration (base changes from this point
forward require a stakeholder-signed issue).

## Success Criteria

- Sim: observer tracks the first-order `WheelPlant` exactly (≤0.1 mm
  position, ≤0.1 mm/s velocity) — any residual treated as a bug, not a
  tuning target.
- Bench: per-wheel step-response battery run and recorded (dead time,
  tau, deadband floor, reversal-dwell effect), both directions, several
  speeds, with measurement provenance in the per-robot JSON.
- Base gate scored explicitly against stated numbers: observer fidelity
  (sim exact; bench within a stated band), duty-write-path completion
  (shaping visible, `appliedDuty` reported), telemetry truthfulness
  (pairing/rate/gap-free), zero-on-silence verified.
- Freeze declaration recorded if the gate is green — a stakeholder-
  visible artifact, not an implicit sprint-close side effect.

## Scope

### In Scope

- Sim exactness validation against `WheelPlant`.
- Bench step-response characterization battery (stand-mounted).
- Base-gate scoring against the stated numeric criteria.
- Freeze declaration (contingent on a green gate).

### Out of Scope

- Any further base-contract change (that is what freezing prevents,
  absent a stakeholder-signed issue) — this sprint measures and gates,
  it does not re-open sprint 124's design.
- Motion-library accuracy work (out of the base's scope entirely, per
  the original directive).

## Test Strategy

(Describe the overall testing approach for this sprint: what types of tests,
what areas need coverage, any integration or system-level testing needed.)

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
