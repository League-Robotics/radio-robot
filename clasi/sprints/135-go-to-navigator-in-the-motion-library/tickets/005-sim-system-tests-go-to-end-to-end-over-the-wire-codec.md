---
id: '005'
title: 'Sim system tests: GO_TO end-to-end over the wire codec'
status: done
use-cases:
- SUC-001
- SUC-002
depends-on:
- '004'
- 008
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

- [x] `src/tests/sim/system/goto_protocol_harness.cpp` (or similarly named
      per this directory's convention) + `test_goto_protocol.py` compile
      and run against the real `SimHarness`/`SimPlant`, exit 0.
- [x] World-frame `GO_TO` scenario: simulated pose converges within
      arrival tolerance, robot at rest, exactly one completion ack
      observed for the sent id.
- [x] Robot-frame `GO_TO` scenario: target resolves against the pose at
      the moment of acceptance, not a moving frame — verified by changing
      heading between send and completion and confirming the world-frame
      endpoint matches the acceptance-time resolution, not a
      re-resolved one.
- [x] Streamed-target scenario: robot's simulated trajectory does not
      come to rest before the final target in the sequence; a mid-sequence
      gap in target updates (several cycles of silence) does not fault,
      halt, or emit an abort ack — the robot keeps converging on the
      last-accepted target.
- [x] Both scenarios assert on DECODED telemetry (the ack ring, `flags`
      bits, `pose`), not on internal state peeked out of process — same
      discipline `test_move_protocol.py` already follows.
- [x] New harness files added to `pyproject.toml`'s existing
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

## Completion Notes

Added `src/tests/sim/system/goto_protocol_harness.cpp` +
`src/tests/sim/system/test_goto_protocol.py`, modeled directly on
`move_protocol_harness.cpp`/`test_move_protocol.py` (116-008) — same
`TestSim::SimHarness`/`TestSim::SimPlant` composition, same
compile-a-throwaway-binary-via-subprocess convention, same full
HOST_BUILD dependency graph `test_app_robot_loop_goto.py` (ticket 004)
already compiles (now including `src/motion/navigator/{arc_solver,
navigator}.cpp`).

Three scenarios (the ticket's two Scope items map to three functions —
the ROBOT-frame case needed its own function to keep the preceding-turn
setup readable):

1. **`scenarioWorldFrameGotoArrivesAndSettles()`** — a WORLD-frame GO_TO
   to (700, 300)mm. Asserts, purely from decoded telemetry: the enqueue
   ack (corr_id) and the completion ack (id, err==0) both land in the ack
   ring, `kFlagActive` clears and `kFlagFaultMoveTimeout` stays clear by
   the end, and the decoded OTOS position settles within ~20mm of the
   target (well inside the 100mm baked `defaultArrivalTolerance` +
   margin).
2. **`scenarioRobotFrameGotoResolvedOnceAtAcceptance()`** — a preceding
   plain MOVE (never touching the Navigator) turns the robot ~80deg and
   lands it fully at rest; the ROBOT-frame target's expected world
   endpoint is computed in the TEST ITSELF, independently mirroring
   `RobotLoop::handleGoto()`'s own resolution formula against the
   acceptance-time OTOS reading captured from decoded telemetry (not sim
   ground truth). The robot's final decoded OTOS position lands within
   ~20mm of that fixed, precomputed point — proving the target was
   resolved once at acceptance, not continuously re-resolved against the
   robot's own turning heading (SUC-001).
3. **`scenarioStreamedTargetsNeverRestBeforeFinal()`** — four waypoints
   streamed over the wire (id/corr distinct per waypoint), each replacing
   the previous target before it could be reached, plus a deliberate
   16-cycle (800ms) silent gap mid-sequence. Asserts no completion ack for
   any of the three superseded waypoints ever appears, `kFlagFaultMoveTimeout`
   never sets, and the robot never comes to rest before the final
   waypoint's own completion ack lands.

**Landmine found and fixed during verification**: `App::kFlagActive` is
NOT a valid "is the robot moving" signal across an internal-segment
replace boundary — it tracks Planner Move-SLOT occupancy, and clears for
exactly one telemetry-visible cycle on *every* internal-segment replace
(both the ones this scenario's own streamed updates provoke and the
Navigator's own ordinary material-change/half-arc-refresh reissues),
even while both wheels are still accelerating (observed directly:
`kFlagActive` cleared with `enc_left`/`enc_right` at 173.4mm/s, still
climbing). First draft of scenario 3 asserted "never rest" on
`kFlagActive` and failed reliably; diagnostic tracing showed the flag
blipping every 1-3 cycles throughout normal streamed cruising while
encoder velocity stayed continuously in the 57-180mm/s band. Scenario 3
now measures "at rest" from decoded `enc_left`/`enc_right` velocity
(`kMinMovingVelocity = 30mm/s`, chosen with margin above the observed
~58mm/s mid-route floor and well below cruise) — a more direct physical
proxy for motion, and still decoded telemetry, matching the ticket's own
"decoded telemetry only" discipline. `kFlagActive` is still checked at
the very end of every scenario (a real "settled" cross-check once
completion is observed), just not as the load-bearing "never rest"
signal mid-stream.

**Verification performed** (scoped per the sprint's updated per-ticket
policy — no bare full-repo `pytest` run):

1. `uv run python -m pytest src/tests/sim/system/test_goto_protocol.py -v -s`
   — 1 passed, run 4 times total (including the failing-then-fixed
   iteration) with no flakiness once the `kFlagActive` fix landed. Also
   confirmed compiling with `-Wall -Wextra` produces zero warnings from
   the new harness file itself (all warnings present are pre-existing,
   from `src/firm/devices/otos.h` and `src/motion/planner/planner.cpp`).
   Confirmed collection with no `pyproject.toml` change via
   `uv run python -m pytest src/tests/sim/system/ --collect-only -q`
   (shows `test_goto_protocol.py::test_goto_protocol_scenarios_pass`).
2. `uv run python src/tests/bench/square_tour.py --sim` — exit 0,
   `PASS: square tour closed`.
3. `uv run python -m pytest src/tests/sim/unit/test_app_robot_loop_goto.py -v`
   (ticket 004's own end-to-end GO_TO routing test) — 1 passed.
