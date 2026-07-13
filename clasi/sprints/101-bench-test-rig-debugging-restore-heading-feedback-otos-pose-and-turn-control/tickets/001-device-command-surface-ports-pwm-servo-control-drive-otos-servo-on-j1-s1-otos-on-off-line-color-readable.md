---
id: "001"
title: "Device command surface + ports/PWM servo control (drive OTOS servo on J1/S1; OTOS on/off; line/color readable)"
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

# Device command surface + ports/PWM servo control (drive OTOS servo on J1/S1; OTOS on/off; line/color readable)

## Description

The rig is exercised by driving the DeviceBus **device subsystem directly** (no
planner). This ticket establishes that surface and adds servo control.

- Confirm/enable device commands reaching: **motors** (VEL / DUTY / PID /
  NEUTRAL / STATE / RING) on ports 1 and 2, **encoders**, **OTOS** (position +
  an **ON/OFF** so motors can run with pose sensing off), **line** (4 ch), and
  **color** (RGBC). The DeviceBus cutover left color/line unbridged — surface them.
- **Resurrect the old ports/PWM code** (source_old/legacy ports) and add a
  command to drive **PWM on Nezha port J1 / pin S1** — the OTOS servo. The
  cutover firmware has no PWM/servo verb.
- Decide + record the firmware image (DeviceBus bring-up DEV surface vs. extend
  the cutover). Flash over USB (rig on the bench).

## Acceptance Criteria

- [ ] A command drives PWM on port J1/S1; sweeping it visibly rotates the OTOS servo.
- [ ] OTOS **heading** changes as the servo rotates — proves OTOS heading is live
      and coupled to the servo (the key check after the mobile-robot OTOS-frozen finding).
- [ ] OTOS can be turned OFF and ON by command; with OTOS OFF the motors still run.
- [ ] Line sensor (4 ch) + color sensor (RGBC) return plausible, changing values as motor 1 turns the drum.
- [ ] Motor VEL/DUTY/PID/NEUTRAL/STATE reach both motors; encoders read and change.

## Testing

- **Existing tests to run**: `uv run python -m pytest` (host suite stays green)
- **New tests to write**: host unit test for any ports/PWM command encode/build.
- **Verification command**: `uv run python -m pytest`
- **HITL (rig)**: flash over USB; sweep servo + watch OTOS heading; toggle OTOS off/on; read line/color while turning motor 1.
