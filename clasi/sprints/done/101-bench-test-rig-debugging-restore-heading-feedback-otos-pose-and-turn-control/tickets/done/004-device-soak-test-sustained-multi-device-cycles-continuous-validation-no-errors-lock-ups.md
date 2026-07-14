---
id: '004'
title: 'Device soak test: sustained multi-device cycles, continuous validation, no
  errors/lock-ups'
status: done
use-cases: []
depends-on: []
github-issue: ''
issue: ''
completes_issue: true
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
