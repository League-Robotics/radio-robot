---
id: '004'
title: 'Tour test suite rewrite: FakeTransport-backed, re-added to testpaths'
status: done
use-cases:
- SUC-035
depends-on:
- '002'
- '003'
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Tour test suite rewrite: FakeTransport-backed, re-added to testpaths

## Description

`tests/testgui/test_tour1_geometry.py`/`test_tour_stop.py`/
`test_tour_idle_detection.py` target the OLD `tests/_infra/sim` ctypes
firmware sim, deleted wholesale at sprint 102 ticket 005 (`git show
72d8be7e --stat`). `test_tour1_geometry.py`'s own `_LIB_PRESENT` guard
means it silently SKIPS every `uv run python -m pytest` run today — it has
not actually executed since before the single-loop rebuild — and
`tests/testgui/` is not even in `pyproject.toml`'s `testpaths` (dropped at
102). None of these tests currently exercise anything against the
current, post-102 architecture, or against tickets 002/003's new tour
driver / TestGUI rewire.

This ticket rewrites the tour-behavior tests against a `FakeTransport`-
backed harness (mirroring `tests/unit/test_planner_executor.py`'s own
established double convention) instead of the deleted ctypes sim, proving
the GUI's tour buttons correctly drive ticket 002's tour driver and
correctly handle Stop — without requiring a rebuilt sim library this
sprint (explicitly out of scope — architecture-update.md Decision 1).
`test_tour_idle_detection.py`, which tests the now-removed
`_wait_for_idle()`/SNAP-poll mechanism specifically, is deleted (nothing
in this sprint needs a poll-based completion check any more).
`tests/testgui/` (the rewritten subset) rejoins `pyproject.toml`'s
`testpaths`. Serves SUC-035.

## Acceptance Criteria

- [x] `test_tour1_geometry.py` (or its rewritten equivalent) passes under
      `uv run python -m pytest`, using a `FakeTransport`/double instead of
      the deleted `tests/_infra/sim` ctypes library — no skip, an actual
      pass.
- [x] `test_tour_stop.py` (or its rewritten equivalent) passes the same
      way, confirming Stop Tour re-enables buttons synchronously (ticket
      003's own regression-tested contract) against the new tour driver.
- [x] `test_tour_idle_detection.py` is deleted (its own subject —
      `_wait_for_idle()` — no longer exists per ticket 003).
- [x] `pyproject.toml`'s `testpaths` gains the rewritten `tests/testgui/`
      subset (or the whole directory, implementer's call, as long as every
      test that runs actually passes — no newly-collected-but-skipped or
      newly-collected-but-failing file).
- [x] Full suite (`uv run python -m pytest`) stays green with the new
      tests collected and passing, not skipped.

## Implementation Plan

### Approach

1. Replace `test_tour1_geometry.py`'s `_LIB_PRESENT`-gated, real-ctypes-sim-
   backed fixture with a `FakeTransport` double satisfying
   `planner.executor.TwistTransport`'s protocol (`twist()`/`stop()`/
   `read_pending_binary_tlm_frames()`), driven through the REAL GUI Qt
   objects (tour buttons, `_TourRunner`, the QThread it runs on) exactly as
   the existing file already does — only the BACKING transport changes,
   not the "drive it through the real GUI" testing philosophy that made
   the original file valuable. The fake's `read_pending_binary_tlm_frames()`
   should synthesize a plausible, monotonically-advancing encoder pose per
   tick so `run_tour()`'s own closure computation has something meaningful
   to compute against.
2. Assert what's actually testable without real motion physics: each tour
   runs to completion (no leg timing out), `[TOUR]` log narration appears
   for each leg, Stop Tour mid-run re-enables buttons synchronously
   (`test_tour_stop.py`'s own existing assertions, retargeted at the fake).
   Drop the old file's ground-truth-span/fused-pose-distance assertions
   that depended on the deleted sim's actual physics — those become
   ticket 002's own unit tests (`test_planner_tour.py`) and ticket 005's
   bench-verified closure numbers instead; this file's job is CONTROL FLOW
   correctness (does the GUI drive the right calls in the right order),
   not physical accuracy.
3. Delete `test_tour_idle_detection.py` outright (confirm via grep that
   nothing else references anything it defines before deleting).
4. Update `pyproject.toml`'s `testpaths`.

### Files to Modify

- `tests/testgui/test_tour1_geometry.py` — rewritten fixture/assertions.
- `tests/testgui/test_tour_stop.py` — rewritten fixture (fake transport
  instead of ctypes sim); assertions largely unchanged in spirit (the
  behavior contract itself doesn't change).
- `pyproject.toml` — `testpaths`.

### Files to Delete

- `tests/testgui/test_tour_idle_detection.py`.

### Testing Plan

- The rewritten files ARE the tests for this ticket — `uv run python -m
  pytest tests/testgui/test_tour1_geometry.py tests/testgui/
  test_tour_stop.py -v` as the direct verification, then the full suite.
- Confirm via `uv run python -m pytest --collect-only` (or equivalent) that
  the rewritten files are actually COLLECTED (not skipped) once
  `testpaths` is updated — the whole point of this ticket is ending the
  silent-skip status quo.

### Documentation Updates

- Update the module docstrings of both rewritten test files to explain the
  new fake-transport convention and why the old ctypes-sim dependency was
  dropped (mirroring the existing files' own thorough "what changed from
  the pre-rebuild version" documentation style — this project's
  established convention for this exact kind of file).

## Completion Notes

**Scope grew beyond the plan's "Files to Modify" list** — re-adding
`tests/testgui/` to `testpaths` surfaced real, independent staleness the
plan didn't anticipate (all fixed, not worked around):

- `test_binary_bridge.py` (sprint 097/100-007 vintage) tested R/TURN/G
  translating to real `segment`/`replace` envelopes and `legacy_render`-
  specific reply text — both dead: `legacy_render`/`legacy_verbs` were
  deleted at 104-002 (binary_bridge.py's own "107-003 launch-unblock"
  section), and `envelope_pb2`'s `body`/`cmd` oneofs independently shrank to
  `{ok,err,tlm}`/`{config,stop,twist}`. Rewrote to test the CURRENT
  degraded-mode contract (every verb -> one fixed `_LEGACY_UNAVAILABLE_REPLY`,
  `render_log_line()` falls back to `text_format`) and locked in both
  preconditions so a future fix to either is a loud, deliberate test change.
- `test_canvas.py`'s three asset-path tests failed for a real reason: a
  later reorg (`{tests_old => archive/tests_old}`, commit `ea9b3e28`) moved
  the parked pre-rebuild tree but never updated `canvas.py`'s three
  `_PLAYFIELD_*` constants to match — every playfield-calibration load was
  silently falling back to hardcoded defaults. Fixed in `canvas.py` itself
  (production code, one three-line change).
- Every `qapp` fixture in `tests/testgui/` (14 files, including this
  ticket's own rewrite) now opens with `pytest.importorskip("PySide6")` —
  `gui` is not in `pyproject.toml`'s `default-groups`, so a fresh/CI clone
  that hasn't `uv sync --group gui`'d would otherwise hit a hard
  collection/run `ModuleNotFoundError` across ~15 files the moment
  `testgui` rejoined `testpaths`, not a clean skip.
- 15 individual tests across 6 pre-existing files
  (`test_calibration_push_on_connect.py`, `test_error_divergence.py`,
  `test_goto.py`, `test_set_origin.py`, `test_traces.py`,
  `test_transport.py`) still skip on `_sim_lib_path().exists()` — they
  need the deleted ctypes sim (rebuild explicitly out of scope this sprint,
  architecture-update.md Decision 1). Judged NOT a violation of AC4's
  "no newly-collected-but-skipped ... file": these are narrowly-scoped,
  individually-documented skips inside files that are otherwise fully
  green, the opposite of the whole-file silent-skip status quo
  (`test_tour1_geometry.py`'s own `_LIB_PRESENT` module-level `pytestmark`)
  this ticket exists to end. `pyproject.toml`'s `testpaths` comment records
  this reasoning for the next reader.

**Post-review fix (test isolation)**: the team-lead's full-suite run caught
`test_tour2_runs_to_completion_with_per_leg_log_narration` failing under
full-suite ordering (1089 passed / 1 failed) while passing in isolation.
Root cause: `_FakeTwistTransport.twist()` integrated its synthesized pose
using REAL `time.monotonic()`-measured elapsed time between calls — correct
on an idle machine, but under the full suite's load (1000+ preceding tests)
scheduling jitter could stretch one tick's actual gap well past the nominal
150ms, over-advancing that tick's distance enough to trip
`StreamingExecutor`'s own bounded-overshoot check. Fixed at the root: the
fake now integrates against a FIXED nominal interval read from
`PlannerParams().streaming_interval` (valid because `_TourRunner.run()`
always constructs a fresh, default `PlannerParams()` — no override reaches
this file), removing the dependency on real-time scheduling fidelity
entirely rather than papering over it with a retry. Verified: full suite
green twice consecutively post-fix (1090 passed, 15 skipped, both runs).

**Slow tests**: the two full-tour completion tests are real-wall-clock-paced
(`run_tour()`'s own `StreamingExecutor` sleeps `params.streaming_interval`
per tick; `_TourRunner.run()` injects no faster clock) — ~45s each, ~90s of
the suite's ~185s total. Marked `@pytest.mark.slow` (registered in
`pyproject.toml`) so `pytest -m "not slow"` can deselect them for a fast
local loop; they stay in the DEFAULT run per AC5 (never excluded, never
skipped).

**Final suite totals** (two consecutive full runs, `uv run python -m
pytest -q`, with `tests/testgui/` counted): **1090 passed, 15 skipped, 0
failed** each run (~185s / ~189s wall clock).
