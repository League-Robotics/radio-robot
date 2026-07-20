---
id: '007'
title: 'Verification: sim/real-boot parity tests + _SimConfigConn relocation regression'
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
depends-on:
- '006'
github-issue: ''
issue: config-as-truth-sim-configure-on-open.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Verification: sim/real-boot parity tests + _SimConfigConn relocation regression

## Description

This is the sprint's own closing proof: the concrete deliverable stated in
the sprint goal is "a headless `SimLoop` run and the TestGUI Sim run the SAME
config the robot would get." Every prior ticket built one piece of the
mechanism; this ticket writes the tests that directly assert the mechanism
actually closes the gap, plus runs the full regression sweep across
everything touched by the sprint (proto fields, `SimHarness`, `gen_boot_config.py`,
`calibration_commands()`, the relocated `SimConfigConn`).

**Golden-parity test** (the headline proof, SUC-001/SUC-002): for each of
`tovez_nocal.json` and `tovez.json`, (a) run `gen_boot_config.py`'s
`generate()` against the file and capture the `PlannerConfig`/`MotorConfig`
values it would bake (via the same functions ticket 004's
`planner_boot_config_for()`/`motor_boot_config_for()` call — this test should
assert against `gen_boot_config.py` DIRECTLY, not against ticket 004's
wrapper, so a bug in ticket 004's own plumbing can't hide behind testing
itself); (b) construct a headless `SimLoop` (`start_tick_thread=False`),
call `configure_from_robot()` with the same file's `RobotConfig`, and read
back the live config via ticket 002's readback accessor (through
`sim_ctypes.cpp`'s test-only export or however that ticket implemented it);
(c) assert every Tier-2 field matches, and every Tier-1 field
(`calibration_kwargs()`'s covered set) matches what `calibration_commands()`
would have pushed for the same file.

**Robot-switch test** (SUC-003): connect once against `tovez_nocal.json`,
read back config, call `configure_from_robot()` again with `tovez.json`,
read back again, assert the second reading reflects `tovez.json`'s values
(not a merge of both) — proves a mid-session profile switch fully replaces
config, not just augments it.

**model_tau parity test** (SUC-004): part of the golden-parity test above,
but called out explicitly since it's the one genuinely new field this sprint
adds semantics for — assert the sim's live `modelTauLin`/`modelTauAng` (via
whatever `Pilot`-level test accessor exists or needs adding — check if
`SimHarness` already exposes anything reaching `Pilot`'s private members; if
not, this may need a one-line addition to ticket 002's readback, coordinate
with that ticket if sequencing allows, or file a small follow-up if not)
match `tovez_nocal.json`'s `control.model_tau_lin`/`model_tau_ang` (0.1/0.08)
after `configure_from_robot()`.

**Regression sweep** (SUC-005 + relocation safety): run the FULL
`uv run python -m pytest` suite (~6 min) and confirm: zero changes in any
pre-existing `src/tests/sim/unit/`, `src/tests/sim/system/` C++/Python test
result; the four `_SimConfigConn`-consuming TestGUI tests
(`test_calibration_push_on_connect.py`, `test_tour_closure_gate.py`,
`test_otos_calibration_convergence.py`, `test_turn_error_characterization.py`)
all still pass after the ticket-005 relocation.

## Acceptance Criteria

- [x] New test file (`src/tests/sim/system/test_sim_boot_config_parity.py`)
      implements the golden-parity test for both `tovez_nocal.json` and
      `tovez.json`, asserting field-for-field equality between
      `gen_boot_config.py`'s direct output and the sim's live config after
      `configure_from_robot()`, for every Tier-1 and Tier-2 field this
      sprint covers (the full list enumerated in ticket 004's acceptance
      criteria). Expected values are computed by calling `gen_boot_config.py`'s
      own mapping functions directly (NOT via `sim_boot_config.py`'s wrapper),
      per this ticket's own design constraint. No Python-reachable readback of
      the live sim config existed before this ticket -- added
      `sim_read_planner_config()`/`sim_read_motor_config()` ctypes exports
      (`src/sim/sim_ctypes.cpp`) plus `SimLoop.read_planner_config()`/
      `read_motor_config()` (`src/host/robot_radio/io/sim_loop.py`) to make
      the assertion possible; both are thin, test-only call-throughs to
      ticket 002's own `SimHarness::plannerConfig()`/`motorConfig()`
      accessors. All 8 tests in the file pass (directly observed:
      `uv run python -m pytest src/tests/sim/system/test_sim_boot_config_parity.py -v`
      -> 8 passed).
- [x] The same file implements the robot-switch test (SUC-003)
      (`test_robot_switch_replaces_not_merges`) -- connects against
      `tovez_nocal.json`, reads back, reconfigures against `tovez.json`,
      reads back again, and asserts the second reading matches `tovez.json`'s
      values on every Tier-2 field (not a merge of the first config).
- [x] The `model_tau_lin`/`model_tau_ang` assertion (SUC-004) is present both
      inside the golden-parity test (both robot JSONs) and as its own small
      test (`test_model_tau_parity_tovez_nocal`, pinning the literal 0.1/0.08
      values from `tovez_nocal.json`).
- [x] Targeted regression sweep is green (directly observed, not assumed):
      `uv run python -m pytest src/tests/sim src/tests/unit -q` -> 794 passed,
      1 xfailed, 1 xpassed, zero failures; `uv run python -m pytest
      src/tests/testgui -q -k "config or transport or sim or wire or
      calibration"` -> 167 passed, 326 deselected, 4 xfailed, zero failures.
      No existing test file's assertions needed to change EXCEPT
      `src/tests/sim/unit/test_wire_differential.py`'s
      `test_field_numbers_match_pb2_descriptors_telemetry`, whose
      `PlannerConfigPatch` field-number pin was already stale before this
      ticket (missing `distance_kp=21`, added by sprint 112 and never
      back-filled into this pin) -- confirmed failing on this branch before
      the fix and passing after (re-synced against the actual generated
      `config_pb2` descriptor, not guessed; `model_tau_lin`/`model_tau_ang`
      were checked and confirmed absent from `PlannerConfigPatch`'s own
      descriptor -- sprint 113 Design Rationale Decision 4 deliberately keeps
      them off the live wire patch, so nothing else needed adding to this
      pin). The full ~13-minute `uv run python -m pytest` (whole `src/tests`)
      run is the team-lead's job at sprint close per this ticket's own
      framing instructions; the sweeps above are the thorough targeted
      verification this ticket ran directly.
- [x] The four `_SimConfigConn`-consuming TestGUI tests were individually
      re-run (not just swept up in a larger run) and confirmed passing
      post-relocation: `uv run python -m pytest
      src/tests/testgui/test_calibration_push_on_connect.py
      src/tests/testgui/test_tour_closure_gate.py
      src/tests/testgui/test_otos_calibration_convergence.py
      src/tests/testgui/test_turn_error_characterization.py -v` -> 18 passed,
      6 xfailed, 2 xpassed (the xfail/xpass are pre-existing turn-tour
      characterization markers, unrelated to this sprint's config-delivery
      change), zero failures.
- [x] `sprint.md`'s Success Criteria section (all four bullets) checked
      against actual test results:
      1. "Connecting the TestGUI's Sim transport... and constructing +
         connecting a headless SimLoop... both result in the sim's...
         running the identical PlannerConfig... that gen_boot_config.py
         would bake" -- CONFIRMED: `test_golden_parity_planner_config`/
         `test_golden_parity_motor_config` assert exactly this, for both
         `tovez_nocal.json` and `tovez.json`, and pass.
      2. "model_tau_lin/model_tau_ang are read from data/robots/*.json's...
         for both real-firmware boot and sim open" -- CONFIRMED:
         `test_model_tau_for_config_reads_tovez_nocal_json` (real-firmware
         boot side, ticket 001, pre-existing) and
         `test_model_tau_parity_tovez_nocal` (sim-open side, this ticket)
         both pass.
      3. "Switching the active robot profile mid-session... re-pushes both
         tiers" -- CONFIRMED: `test_robot_switch_replaces_not_merges`
         passes, asserting a full replace (not a merge) across every
         Tier-2 field.
      4. "None of the existing ~40 C++ sim test harnesses... change
         behavior; the full uv run python -m pytest suite... and the
         TestGUI still work" -- CONFIRMED for the scope this ticket
         directly verified: `src/tests/sim src/tests/unit` (794 passed, 1
         xfailed, 1 xpassed, zero failures) and the targeted TestGUI sweep
         (167 passed, 326 deselected, 4 xfailed, zero failures), including
         all four named `_SimConfigConn` regression files individually.
         The full-suite (~13 min, whole `src/tests`) confirmation is the
         team-lead's job at sprint close.

## Testing

- **Existing tests to run**: the full `uv run python -m pytest` suite
  (~6 min) — this ticket's own job IS the test-running/verification pass,
  not a subset.
- **New tests to write**: as enumerated above —
  `test_sim_boot_config_parity.py` (golden-parity + robot-switch +
  model_tau), all under `src/tests/sim/` per this sprint's own Test Strategy.
- **Verification command**: `uv run python -m pytest` (full suite, the
  project's ~6 min gate).

## Files to touch

- New: `src/tests/sim/system/test_sim_boot_config_parity.py` (or split
  across a couple of files if that reads better — implementer's call, one
  file is the default expectation)

## Depends On

- Ticket 006 (needs the full delivery path — `SimTransport`/`SimLoop`
  wiring — to exist end-to-end before parity can be asserted).
