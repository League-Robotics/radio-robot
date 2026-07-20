---
id: '007'
title: 'Verification: sim/real-boot parity tests + _SimConfigConn relocation regression'
status: open
use-cases: [SUC-001, SUC-002, SUC-003, SUC-004, SUC-005]
depends-on: ['006']
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

- [ ] New test file (e.g. `src/tests/sim/system/test_sim_boot_config_parity.py`)
      implements the golden-parity test for both `tovez_nocal.json` and
      `tovez.json`, asserting field-for-field equality between
      `gen_boot_config.py`'s direct output and the sim's live config after
      `configure_from_robot()`, for every Tier-1 and Tier-2 field this
      sprint covers (the full list enumerated in ticket 004's acceptance
      criteria).
- [ ] The same file (or a sibling) implements the robot-switch test (SUC-003).
- [ ] The `model_tau_lin`/`model_tau_ang` assertion (SUC-004) is present,
      either inside the golden-parity test or as its own small test.
- [ ] Full `uv run python -m pytest` run is green, with explicit
      confirmation (in this ticket's completion notes) that no existing
      test file's assertions needed to change — a byte-for-byte "diff of
      test results before/after this sprint" is empty except for the new
      test files added across tickets 001-007.
- [ ] The four `_SimConfigConn`-consuming TestGUI tests are individually
      re-run and confirmed passing post-relocation (not just swept up in the
      full-suite run — call them out by name in completion notes, since
      they're this sprint's single highest-risk regression surface per
      `sprint.md`'s Design Rationale Decision 3).
- [ ] `sprint.md`'s Success Criteria section (all four bullets) is checked
      off against the actual test results, not just assumed true.

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
