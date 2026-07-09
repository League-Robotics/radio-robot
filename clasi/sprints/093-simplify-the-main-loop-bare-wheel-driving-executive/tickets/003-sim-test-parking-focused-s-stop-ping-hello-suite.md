---
id: '003'
title: Sim-test parking + focused S/STOP/PING/HELLO suite
status: open
use-cases: [SUC-001, SUC-002, SUC-003]
depends-on: ['002']
github-issue: ''
issue: simplify-the-main-loop-strip-it-to-bare-wheel-driving.md
completes_issue: true
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
