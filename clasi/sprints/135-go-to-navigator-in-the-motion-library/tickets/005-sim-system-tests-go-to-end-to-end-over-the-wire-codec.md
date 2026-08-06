---
id: '005'
title: 'Sim system tests: GO_TO end-to-end over the wire codec'
status: open
use-cases:
- SUC-001
- SUC-002
depends-on:
- '004'
- '008'
github-issue: ''
issue: replaceable-go-to-moves-in-the-motion-library.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sim system tests: GO_TO end-to-end over the wire codec

## Description

Whole-robot scenario coverage for `GO_TO`, composed the same way every
other `src/tests/sim/system/` scenario is: `TestSim::SimHarness` wiring
the REAL `App::RobotLoop` (now including `Motion::Navigator`, ticket 004)
against `TestSim::SimPlant` — a real `Devices::I2CBus` implementation
parsing the actual wire protocol and integrating real wheel/OTOS physics —
no ARM hardware involved. This is this sprint's PRIMARY verification tier
per the sim-first stakeholder directive; it should carry most of this
sprint's confidence, not the hardware pass (ticket 006).

Model both new test files directly on the closest existing sibling,
`src/tests/sim/system/move_protocol_harness.cpp` /
`test_move_protocol.py` (116-008) — same `SimHarness`/`SimPlant`
composition, same "compile a throwaway C++ binary via subprocess, run it,
assert exit 0, print a human-readable per-scenario trace" convention (see
`src/tests/sim/system/README.md`'s own description of that file for the
closest analog: a new binary command-plane arm exercised end-to-end
through the real wire codec).

## Scope

Two scenarios, following the linked issue's Verification section:

1. **End-to-end `GO_TO`, both frames.** Send a `GO_TO` (WORLD frame) over
   the real wire codec to a booted `SimHarness`; drive cycles; assert the
   plant's simulated pose converges to within the target's arrival
   tolerance and the robot comes to rest; assert exactly one completion
   ack with the sent id appears in the decoded telemetry ack ring. Repeat
   for a `GO_TO` (ROBOT frame) target, asserting it resolves against the
   pose AT ACCEPTANCE (send it after a preceding move has changed heading,
   to prove it doesn't chase the robot as it turns — SUC-001's own
   "resolved once at acceptance" postcondition).
2. **Streamed-target EXTERNAL-mode scenario.** Send a short SEQUENCE of
   `GO_TO` commands over the wire, each replacing the previous target
   before it's reached (mimicking a host pure-pursuit loop streaming
   `pursuitTarget()` output) — assert the robot never comes to rest before
   the FINAL target (SUC-002's core postcondition: intermediate targets
   don't cause stop-and-restart), and that a deliberately delayed/dropped
   target update in the middle of the sequence does not fault or halt the
   Navigator (send nothing for several cycles mid-sequence, then resume) —
   the robot should keep converging on the last-accepted target
   throughout.

## Acceptance Criteria

- [ ] `src/tests/sim/system/goto_protocol_harness.cpp` (or similarly named
      per this directory's convention) + `test_goto_protocol.py` compile
      and run against the real `SimHarness`/`SimPlant`, exit 0.
- [ ] World-frame `GO_TO` scenario: simulated pose converges within
      arrival tolerance, robot at rest, exactly one completion ack
      observed for the sent id.
- [ ] Robot-frame `GO_TO` scenario: target resolves against the pose at
      the moment of acceptance, not a moving frame — verified by changing
      heading between send and completion and confirming the world-frame
      endpoint matches the acceptance-time resolution, not a
      re-resolved one.
- [ ] Streamed-target scenario: robot's simulated trajectory does not
      come to rest before the final target in the sequence; a mid-sequence
      gap in target updates (several cycles of silence) does not fault,
      halt, or emit an abort ack — the robot keeps converging on the
      last-accepted target.
- [ ] Both scenarios assert on DECODED telemetry (the ack ring, `flags`
      bits, `pose`), not on internal state peeked out of process — same
      discipline `test_move_protocol.py` already follows.
- [ ] New harness files added to `pyproject.toml`'s existing
      `testpaths = ["src/tests/sim"]` collection with no configuration
      change needed (this directory's existing convention already covers
      new files here automatically — confirm, don't assume).

## Testing

- **Existing tests to run**: full sim suite, to confirm the new harness
  doesn't disturb anything else:
  ```
  uv run python -m pytest
  ```
- **New tests to write**: the two scenarios above.
- **Verification command**:
  ```
  uv run python -m pytest src/tests/sim/system/test_goto_protocol.py -v -s
  ```
