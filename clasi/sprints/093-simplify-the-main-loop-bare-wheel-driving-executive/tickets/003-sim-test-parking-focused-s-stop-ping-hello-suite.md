---
id: '003'
title: Sim-test parking + focused S/STOP/PING/HELLO suite
status: exception
use-cases:
- SUC-001
- SUC-002
- SUC-003
depends-on:
- '002'
github-issue: ''
issue: simplify-the-main-loop-strip-it-to-bare-wheel-driving.md
completes_issue: true
exception:
  thrown_by: programmer
  thrown_at: '2026-07-09T13:42:02.489914+00:00'
  attempted: 'Completed the ticket''s own explicitly-scoped work in full: triaged
    all 49 tests/sim/unit/*.py files + tests/sim/system/test_tour_geometry.py by actually
    running each (not assuming from filename); confirmed every failure traces to a
    genuine "ERR unknown" removed-surface reply (spot-checked test_mode_machine.py,
    test_watchdog_policy.py, test_stiction_and_motor_lag.py, test_otos_commands.py,
    test_protocol_roundtrips.py tracebacks), not a ticket-002 regression; git mv''d
    26 unit files + 3 harness .cpp siblings + 1 system file into tests/sim/parked-093/{unit,system}/,
    wrote parked-093/README.md naming what must be re-wired per file; added "parked-093"
    to pyproject.toml''s norecursedirs (verified empirically via pytest --collect-only
    that the bare basename form, matching tests_old/source_old''s own convention,
    excludes it); removed conftest.py''s dead `DEV WD` widen call with an explanatory
    docstring; wrote tests/sim/unit/test_bare_loop_commands.py (7 tests) asserting
    S drives both wheels to commanded targets/direction via vel()/true_velocity(),
    S with opposite-sign l/r spins wheels oppositely, STOP neutralizes regardless
    of prior S state (checked hard across 5 subsequent ticks), PING/HELLO reply correctly,
    and an out-of-surface verb (DEV WD 100, GET drivetrainConfig) replies exactly
    "ERR unknown". `uv run python -m pytest tests/sim` is 37/37 green. Then ran the
    ticket''s literal acceptance-bullet command, full `uv run python -m pytest` (which
    pyproject.toml''s testpaths actually scopes to tests/sim + tests/unit + tests/testgui,
    not just tests/sim as tests/CLAUDE.md''s stale doc claims) and found 16 failures,
    all in tests/testgui/ (tests/unit is 4/4 green). To rule out a ticket-093 regression
    vs. pre-existing breakage, built a clean git worktree at the sprint''s own merge-base
    commit (3f788090, pre-093 master tip) and ran tests/testgui there in isolation:
    364/364 pass. This proves the 16 failures are new fallout specifically caused
    by sprint 093''s wire-surface gutting (TestGUI/robot_radio host-side code driving
    DEV/SET/GET/OTOS/telemetry verbs that now reply ERR unknown), not a defect in
    this ticket''s own tests/sim work.'
  conflict: "architecture-update.md Step 5 (\"Impact on Existing Components\") explicitly\
    \ states: \"no EVT, no TLM, no GET/SET, no STREAM/SNAP, no pose/OTOS verbs. Any\
    \ host tool (TestGUI, robot_radio) that depends on those verbs will get ERR unknown\
    \ \u2014 this sprint does not touch host-side code; that fallout is explicitly\
    \ accepted for the bench-bring-up phase this sprint targets.\" Ticket 003's own\
    \ Description, Acceptance Criteria, and Implementation Plan \"Files to create/modify\"\
    \ list are scoped exclusively to tests/sim/ (parked-093/, conftest.py, pyproject.toml\
    \ norecursedirs, a new tests/sim/unit/ focused suite) \u2014 tests/testgui/ never\
    \ appears anywhere in the ticket's scope, sizing, or triage criterion (which is\
    \ itself framed only in terms of tests/sim/unit's 49 files + 1 system file). Yet\
    \ the ticket's final acceptance bullet reads \"`uv run python -m pytest` (full\
    \ scoped run, matching pyproject.toml's testpaths) is 100% green\" \u2014 and\
    \ pyproject.toml's testpaths is `[\"tests/sim\", \"tests/unit\", \"tests/testgui\"\
    ]`, not tests/sim alone. Satisfying that bullet literally requires either (a)\
    \ triaging and parking/fixing 16 tests/testgui failures across 7 files (test_calibration_push_on_connect.py,\
    \ test_error_divergence.py, test_goto.py, test_set_origin.py, test_sim_errors_panel.py,\
    \ test_tour1_geometry.py, test_traces.py, test_transport.py) \u2014 a testgui-domain\
    \ undertaking this ticket never scoped, sized, or authorized, and which architecture-update.md\
    \ frames as a DIFFERENT sprint's problem (\"this sprint does not touch host-side\
    \ code\") \u2014 or (b) restoring some host-facing wire commands, which would\
    \ directly reverse architecture-update.md's Decision 2/Step-5 removal. Either\
    \ path overrides an explicit upstream architecture decision; I did neither, to\
    \ avoid unauthorized scope expansion or unilaterally reversing a stakeholder-owned\
    \ decision. Recommend: either descope/reword this ticket's final acceptance bullet\
    \ to \"uv run python -m pytest tests/sim\" (matching the ticket's own actual scope\
    \ and tests/CLAUDE.md's \u2014 stale but directionally intended \u2014 description\
    \ of the close gate), or spin up a follow-up ticket to triage/park tests/testgui/\
    \ fallout the same way this ticket did for tests/sim/."
  surface: internal
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sim-test parking + focused S/STOP/PING/HELLO suite

## Description

After tickets 001-002, `tests/sim/unit/` (49 files) and `tests/sim/system/`
(1 file) are mostly red: most tests dispatch a wire verb outside
`{PING, HELLO, S, STOP}` via `sim_command()`/`sim_command_on()`, or assert
on `Planner`/`PoseEstimator`/`Rt::Configurator`/watchdog behavior being
driven BY `Rt::MainLoop::tick()` — all now gone from the loop. Per
architecture-update.md Decision 3, obsoleted files are PARKED (moved, not
deleted), and a small, curated, currently-green suite is kept/written for
the new four-verb surface. This ticket restores the CLASI close-gate to
green — it is the last purely-software ticket before the hardware bench
gate (ticket 004).

**Triage criterion** (apply per-file, do not skip files by assumption —
verify by running them): PARK a test if it dispatches any wire verb outside
`{PING, HELLO, S, STOP}`, or asserts on `Planner`/`PoseEstimator`/
`Rt::Configurator`/watchdog behavior mediated by `MainLoop::tick()`. KEEP a
test as-is if it exercises a class in isolation via its own harness (e.g.
`drivetrain_harness.cpp`, `pose_estimator_harness.cpp`,
`configurator_harness.cpp`, `stop_condition_harness.cpp` — these test the
parked classes' own internal correctness, independent of whether `MainLoop`
ticks them) or a HAL/plant-level primitive untouched by the gut (encoders,
PID, I2C, Ruckig math, `Rt::Blackboard`/`Rt::Queue` structure, boot-config
generation).

This ticket must also fix `tests/sim/conftest.py`'s `sim` fixture: it
currently issues `s.command(f"DEV WD {_WATCHDOG_WIDE_WINDOW}")` at setup to
widen the (now-removed) serial-silence watchdog before every test. Since
`DEV` is unregistered (ticket 001), this call becomes a silent
`ERR unknown` at the top of every single test using the `sim` fixture —
harmless in that it doesn't raise, but dead and misleading. Remove the
widen call and its surrounding comment; there is no watchdog left to widen.

## Acceptance Criteria

- [ ] Run the full `tests/sim/` suite post-002, and produce a triage: for
      each of the 49 `tests/sim/unit/*.py` files + `tests/sim/system/
      test_tour_geometry.py`, classify PARK or KEEP per the criterion
      above (verified by actually running each, not assumed from the file
      name).
- [ ] Create `tests/sim/parked-093/` (a new leaf directory inside the
      `sim/` domain, alongside `unit/`/`system/`) and `git mv` every PARKed
      file into it, preserving its original relative structure (e.g.
      `tests/sim/unit/test_tlm_stream_snap.py` →
      `tests/sim/parked-093/unit/test_tlm_stream_snap.py`).
- [ ] Add a short `tests/sim/parked-093/README.md` (mirroring
      `tests_old/`'s own documentation precedent): names the sprint (093),
      the reason (four-verb gut removed `Planner`/`PoseEstimator`/
      `Configurator`/telemetry/config/pose/otos from the live wire
      surface), and what would need to be re-wired (which classes need a
      live command family again) before a given parked file could return.
- [ ] `pyproject.toml`'s `[tool.pytest.ini_options]` `norecursedirs` gains
      `"tests/sim/parked-093"` (or the bare leaf name if `norecursedirs`
      matches by directory name rather than path — verify against the
      existing `tests_old`/`source_old` entries' own matching behavior)
      so `pytest` never collects it.
- [ ] `tests/sim/conftest.py`'s `sim` fixture no longer issues
      `DEV WD ...`; its docstring/comments referencing the watchdog-widen
      rationale are updated to say why it's gone (not silently deleted
      with no trace, so a future reader isn't confused about why the
      fixture looks unusually bare compared to its own doc comment).
- [ ] A focused suite exists and is green, covering (at minimum, may live
      in existing KEPT files if they already fit, or a new
      `tests/sim/unit/test_bare_loop_commands.py`):
      - `S <l> <r>` drives both wheels to the commanded targets in the
        commanded direction (check via `sim_get_vel_l/r` or
        `sim_get_true_vel_l/r`, not just the `OK` reply).
      - `S` with differing-sign `l`/`r` spins the wheels in opposite
        directions.
      - `STOP` neutralizes both wheels regardless of prior `S` state.
      - `PING` → `OK`.
      - `HELLO` → a `DEVICE:...`-shaped reply.
      - A wire verb outside the four (e.g. `GET drivetrainConfig` or
        `DEV WD 100`) → `ERR unknown` (proves the table reduction from
        ticket 001, not just that the four verbs work).
- [ ] `uv run python -m pytest` (full scoped run, matching
      `pyproject.toml`'s `testpaths`) is 100% green.

## Implementation Plan

**Approach**: Run-triage-move-fix-verify, in that order — do not move files
based on their names alone; run the suite first (post-002) and read each
failure to confirm it's a removed-surface failure, not an unrelated
regression ticket 002 introduced by accident (if the latter, that's a
ticket 002 bug to fix, not a test to park).

**Files to create/modify**:
- `tests/sim/parked-093/` (new directory) + moved files + `README.md`.
- `pyproject.toml` (`norecursedirs`).
- `tests/sim/conftest.py` (`sim` fixture).
- New or extended focused-suite test file(s) under `tests/sim/unit/`.

**Testing plan**:
- Existing: every KEPT file must still pass; this ticket's own acceptance
  IS the full green suite run.
- New: the focused four-verb suite above.
- Verification command: `uv run python -m pytest`.

**Documentation updates**: `tests/sim/parked-093/README.md` (new, required
by acceptance criteria above). No changes to `docs/protocol-v2.md` (deferred
per architecture-update.md Step 7 item 2).
