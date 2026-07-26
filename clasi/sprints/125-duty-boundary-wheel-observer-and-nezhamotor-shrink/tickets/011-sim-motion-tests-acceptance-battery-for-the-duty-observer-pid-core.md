---
id: '011'
title: Sim/motion_tests acceptance battery for the duty/observer/PID core
status: open
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
depends-on:
- '005'
- '006'
- '007'
- '008'
github-issue: ''
issue: firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sim/motion_tests acceptance battery for the duty/observer/PID core

## Description

Consolidate and confirm the off-hardware acceptance battery for the core
duty/observer/PID work (tickets 002-008), collecting what each prior
ticket's own testing section introduced into one coherent, CI-run suite
rather than scattered ad hoc scenarios. Required components (per sprint
Test Strategy): the relocated `Motion::WheelVelocityPid` unit test
(parity with pre-move `Devices::MotorVelocityPid`); the observer's
exact-tracking + predict-through-freeze scenarios against
`TestSim::WheelPlant`; a `motion_tests` chained-`Wheels`-Move scenario
exercising the full shape→PID→duty path end to end; the zero-on-silence
positive-evidence test; the `appliedDuty` anti-windup regression test
(with its required-to-fail reverted-build companion); a sim re-run of the
existing pairing-skew/straight-leg-crab suite. This ticket's job is
completeness and CI wiring, not inventing new coverage each prior ticket
should already have proposed.

## Acceptance Criteria

- [ ] Every off-hardware acceptance criterion tagged in sprint.md's
      Use Cases SUC-001 through SUC-004 has a corresponding automated
      test, runnable via `uv run pytest` and/or `motion_tests`' `ctest`
      target — enumerate the mapping in this ticket's own completion
      notes (criterion → test name).
  - [ ] The full off-hardware battery is green in one CI run (this is the
      ticket that confirms tickets 002-008 landed a coherent, not just
      individually-green, whole).
- [ ] The reverted-build tripwire tests (anti-windup, first-MOVE-loss
      regression from ticket 001 if not already covered there) are
      confirmed to actually fail against a deliberately-reverted build —
      not merely asserted to exist.

## Testing

- **Existing tests to run**: the full sim/unit suite plus `motion_tests`.
- **New tests to write**: only gaps found while consolidating — this
  ticket should mostly be wiring/completeness, not large new test
  authorship (if it turns out to need substantial new coverage, that is
  a signal a prior ticket under-delivered its own testing section).
- **Verification command**: `uv run pytest` and `cmake --build
  src/motion/build --target motion_tests`.
