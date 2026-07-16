---
id: '004'
title: Migrate whole-robot scenario tests onto SimPlant/SimHarness; verify straight-twist
  stays straight
status: done
use-cases:
- SUC-041
depends-on:
- '003'
github-issue: ''
issue: plan-pure-i2cbus-clock-interfaces-a-real-simplant-simulator.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Migrate whole-robot scenario tests onto SimPlant/SimHarness; verify straight-twist stays straight

## Description

Stage 2 (part c) of the master plan — the payoff ticket for the SimPlant/
SimHarness work: prove the divergence bug (left encoder freezes, right
runs away under an arbitrary twist stream) is gone, and migrate the 4
existing whole-robot scenario tests onto the new harness.

Migrate these 4 tests, currently built against the old `SimApi`/scripted
bus, onto `SimPlant`/`sim_harness.h`:
- `tests/sim/system/sim_api_harness.cpp` (+ `test_sim_api.py`)
- `tests/sim/system/profiled_motion_harness.cpp` (+
  `test_profiled_motion_sim.py`)
- `tests/sim/system/scripted_twist_demo_harness.cpp` (+
  `test_scripted_twist_demo.py`)
- `tests/sim/system/faults/*` (fault-knob scenarios)

Each migration replaces the old harness's `SimApi` construction with
`SimHarness` construction and its command-injection/telemetry-drain calls
with the new harness's equivalents — the SCENARIO logic (what twist is
injected, what is asserted) is preserved as-is; only the underlying
simulator changes.

Also produce the standalone verification the master plan's Stage 2 calls
for: a straight twist command, run for a realistic duration, asserting
heading stays near zero throughout (not just at the end) — this is the
direct regression test for the divergence bug described in the master
plan's Context section.

## Acceptance Criteria

- [x] All 4 migrated system tests (`sim_api`, `profiled_motion`,
      `scripted_twist_demo`, `faults/fault_knobs`) pass against
      `SimPlant`/`sim_harness.h`.
- [x] A standalone straight-twist driver/test: command a straight twist,
      step for a realistic duration (order of the tour leg lengths this
      sprint's SUC-042 targets), assert heading stays within a small bound
      of 0 for the ENTIRE run (not just the final sample) — demonstrating
      the left-freezes/right-runs-away desync is gone.
- [x] No test in `tests/sim/system/` references the deleted `SimApi`/
      `DutyPredictor` classes.

## Implementation Plan

**Approach**: One test file at a time, verifying each independently before
moving to the next — this ticket touches 4+ existing test files plus adds
one new standalone check, so keep the diff reviewable per file.

**Files to modify**:
- `tests/sim/system/sim_api_harness.cpp`, `test_sim_api.py`
- `tests/sim/system/profiled_motion_harness.cpp`, `test_profiled_motion_sim.py`
- `tests/sim/system/scripted_twist_demo_harness.cpp`, `test_scripted_twist_demo.py`
- `tests/sim/system/faults/*`

**Files to create**:
- A new standalone straight-twist regression test/harness (colocate under
  `tests/sim/system/` alongside the migrated scenarios, or
  `tests/_infra/sim/` alongside `sim_harness.h` if it more naturally
  belongs with the harness itself — programmer's call, document the
  choice).

**Testing plan**:
- Existing tests to run: the 4 migrated scenario tests themselves (this
  ticket's own subject).
- New test to write: the straight-twist-stays-straight check described
  above.
- Verification command: whatever this ticket's own new/migrated tests are
  runnable through (pytest once ticket 005's ctypes ABI exists is the
  eventual path — until then, a standalone host-compiled driver is
  acceptable for THIS ticket's own verification; ticket 005 does not block
  this ticket structurally since `SimHarness` is C++-only and does not
  need ctypes to be exercised directly).

**Documentation updates**: none beyond updating each migrated test file's
own header comment to reference `SimPlant`/`sim_harness.h` instead of the
deleted `SimApi`.
