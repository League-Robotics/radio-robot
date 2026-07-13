---
id: "003"
title: "Sensor exercise notebook: line-sensor 0-7 count + ch1 index, color sensor, OTOS X/Y (drum) + heading (servo)"
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

# Sensor exercise notebook: line-sensor 0-7 count + ch1 index, color sensor, OTOS X/Y (drum) + heading (servo)

## Description

A **new notebook** exercising the rig's sensors by turning **motor 1** (the drum):

- **Line sensor**: recover the 3-bit binary count on **ch2/ch3/ch4 (0..7)** and
  the **ch1 index** (black once per revolution at count 0). One full 360°
  revolution = **8 counts**. Show the count sequence and the index pulse.
- **Color sensor**: read the painted-wheel colors cycling past as motor 1 turns;
  show distinguishable colors.
- **OTOS**: servo at neutral → drum motion reads mostly on OTOS **X**; servo
  ~+90° → mostly **Y**; a servo sweep moves OTOS **heading**. Show all three.

Depends on ticket 001 (device surface + servo).

## Acceptance Criteria

- [ ] Notebook recovers the line-sensor 0..7 count + ch1 index correctly as
      motor 1 turns; 8 counts/revolution confirmed.
- [ ] Color sensor shows distinguishable colors cycling with motor 1.
- [ ] OTOS X (neutral servo), Y (servo +90°) from the drum, and OTOS heading
      from a servo sweep are all demonstrated.

## Testing

- **HITL (rig)**: execute the notebook over USB; verify count/index/color/OTOS.
