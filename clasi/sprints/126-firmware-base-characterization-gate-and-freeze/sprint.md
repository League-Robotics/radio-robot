---
id: '126'
title: Firmware base characterization, gate, and freeze
status: roadmap
branch: sprint/126-firmware-base-characterization-gate-and-freeze
worktree: false
use-cases: []
issues:
- firmware-base-hardening-characterization-gate-and-freeze.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 126: Firmware base characterization, gate, and freeze

## Goals

Characterize the base-contract change sprint 125 lands (duty primitive +
per-wheel command observer), pass it against a numeric gate, and declare
the firmware base FROZEN: subsequent base changes require a
stakeholder-signed issue, and the motion library treats the boundary
(`Motion::WheelSink`, `RobotState`/`firm/types`) plus the gate numbers as
a stable platform contract from that point on. Third and final sprint of
the firmware-base-hardening restructuring (124 wire/state → 125 duty
boundary/observer → **126 characterization/gate/freeze**).

## Problem

Sprint 125 introduces a duty primitive and a per-wheel observer whose
correctness claims (dead time, rise shape, deadband floor, observer
fidelity) are currently assertions, not measurements. Freezing the base
before those numbers are measured and gated would lock in an unverified
contract; the motion library is about to build on top of this boundary
and needs it to be a known, stable, numerically-characterized quantity —
not "should work."

## Solution

- **Characterization**, constants with derivations, never swept: sim
  (plant is first-order by construction — the observer must track it
  EXACTLY, since any residual is a bug, not tuning) and bench (per-wheel
  step-response battery, both directions, several speeds, loaded on the
  stand: dead time, effective tau, deadband floor, reversal-dwell
  effect), recorded per-robot in JSON with measurement provenance. A
  sensitivity note vs. battery voltage is stated (the observer's
  correction step absorbs slow drift; characterize, don't chase).
- **The gate**: observer fidelity (sim exact within ≤0.1 mm / ≤0.1 mm/s
  vs. plant truth; bench within a stated band vs. encoder truth at the
  next sample), completion of the duty write path (dwell/deadband
  visible in telemetry, `appliedDuty` reported, zero-on-silence
  verified), and telemetry truthfulness (same-generation L/R pairing,
  ~25 Hz sustained rate, gap-free sequence numbers).
- **The freeze**: gate green ⇒ base frozen. Subsequent base changes
  require a stakeholder-signed issue.

## Success Criteria

- Every characterization constant is measured, derived, and recorded
  with its provenance (robot, date, conditions) — none are swept/tuned
  values presented as constants.
- Sim observer fidelity gate passes exactly (≤0.1 mm / ≤0.1 mm/s).
- Bench observer fidelity gate passes within its stated band across the
  full step-response battery.
- Duty write path and telemetry truthfulness gates both pass on the
  stand, per `.claude/rules/hardware-bench-testing.md`.
- A freeze declaration is recorded (this sprint's own closing artifact)
  stating the base is frozen and what "stakeholder-signed issue" means
  procedurally for a future base change.

## Scope

### In Scope

- The characterization battery (sim + bench).
- The numeric gate definition and its measurement run.
- The freeze declaration.

### Out of Scope

- Any base-contract change itself — this sprint measures and gates what
  125 already built; it does not itself change the base contract.
- Motion-library work built on top of the frozen boundary — that begins
  only after this sprint's gate is green.

## Test Strategy

To be detailed at detail-planning time, after sprint 125 closes and its
actual duty/observer implementation is known. Expected shape: sim
exact-tracking tests against `WheelPlant`, a bench step-response battery
script recording measurement provenance to JSON, and a stand bench-gate
run confirming the full gate (fidelity + write-path + telemetry) in one
sitting.

## Architecture

N/A — roadmap-level stub. Full architecture (or a reasoned "N/A —
trivial"/"compact" sizing, if this sprint turns out to be measurement-only
with no new module) is written when this sprint is detail-planned, after
sprint 125 lands.

### Architecture Overview

N/A — roadmap-level stub. Deferred to detail planning.

### Design Rationale

N/A — roadmap-level stub. Deferred to detail planning.

### Migration Concerns

N/A — roadmap-level stub. Deferred to detail planning.

## Use Cases

N/A — roadmap-level stub. Use cases are written when this sprint is
detail-planned.

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
