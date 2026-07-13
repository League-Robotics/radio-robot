---
id: "002"
title: "Motion Control notebook: motors follow PID (sine/square) with OTOS OFF; high-inertia motor 2; reliable encoders"
status: open
use-cases: []
depends-on: []
github-issue: ""
issue: ""
# completes_issue: Controls whether linked issues are archived when this ticket
# is moved to done. Default: true (archive when all referencing tickets are done).
# Set to false (scalar) to suppress archival for ALL linked issues on this ticket.
# Set to a mapping {filename.md: false} to suppress archival per issue filename.
# Use false for tickets that partially address a multi-sprint umbrella issue.
completes_issue: true
# exception: Written by a lower agent when it cannot proceed (see architecture §exception-protocol).
# exception:
#   thrown_by: "programmer"          # "programmer" | "sprint-planner"
#   thrown_at: "2026-05-07T14:23:00Z"
#   attempted: |
#     Description of what was attempted before giving up.
#   conflict: "architecture-update.md §3 — reason the agent is blocked"
#   surface: "internal"              # "user-visible" | "internal"
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Motion Control notebook: motors follow PID (sine/square) with OTOS OFF; high-inertia motor 2; reliable encoders

## Description

Prove the **velocity PID drives the motors accurately with OTOS OFF** —
decoupling motor control from pose sensing. Update the existing **Motion Control
notebook** so, run against the rig, it:

- Commands each motor a **sine** and a **square** velocity reference and plots
  commanded vs actual (encoder-derived) velocity — the motors must **track**.
- Includes **motor 2 (high-inertia, 3 wheels)** — the PID must still track under
  that load (characterize/bound the lag + overshoot).
- Runs with **OTOS OFF** throughout (pose sensing not involved).
- Confirms **encoders read reliably** the whole time (no dropouts, no wedge
  false-latch, plausible velocity).

No turns, no planner — per-motor PID tracking only. Depends on ticket 001.

## Acceptance Criteria

- [ ] Notebook drives sine + square velocity refs to motor 1 and motor 2 and
      plots commanded vs actual; both track within a documented tolerance.
- [ ] Motor 2's high inertia is exercised; tracking holds (lag/overshoot characterized).
- [ ] OTOS is OFF for the whole run; motors run correctly regardless.
- [ ] Encoders read reliably across the run (no stalls/wedge/garbage).

## Testing

- **Verification command**: run the Motion Control notebook against the rig over USB.
- **HITL (rig)**: execute end-to-end; inspect the tracking plots.
