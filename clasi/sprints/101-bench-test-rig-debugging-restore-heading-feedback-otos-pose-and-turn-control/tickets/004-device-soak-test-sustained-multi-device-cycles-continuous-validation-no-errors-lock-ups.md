---
id: "004"
title: "Device soak test: sustained multi-device cycles, continuous validation, no errors/lock-ups"
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

# Device soak test: sustained multi-device cycles, continuous validation, no errors/lock-ups

## Description

A scripted **soak test** that runs a repeating series of device cycles for a
sustained period and **continuously validates** results, proving the device
layer does not error out or lock up.

- Cycle both motors (incl. high-inertia motor 2) through velocity/duty patterns;
  read encoders/OTOS/line/color each cycle; sweep the servo.
- Assert per cycle: motors follow (encoder velocity tracks command), encoders
  advance, OTOS updates, line count coherent (steps mod 8 with the index), color
  plausible.
- Detect + report any lock-up, stall, wedge latch, comms timeout, or implausible
  reading; run long enough to surface intermittent faults.

Depends on tickets 001-003.

## Acceptance Criteria

- [ ] Soak runs N cycles (documented duration) with zero unhandled errors, zero
      lock-ups, zero comms timeouts.
- [ ] Per-cycle validations pass (motors track, encoders advance, OTOS updates,
      line coherent, color plausible); failures reported with detail.
- [ ] OTOS + encoders proven reliable over the sustained run (the sprint's headline goal).

## Testing

- **HITL (rig)**: run the soak over USB for the documented duration.
