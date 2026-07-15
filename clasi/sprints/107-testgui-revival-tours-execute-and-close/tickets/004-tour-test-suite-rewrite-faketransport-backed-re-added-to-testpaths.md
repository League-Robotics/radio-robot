---
id: "004"
title: "Tour test suite rewrite: FakeTransport-backed, re-added to testpaths"
status: open
use-cases: [SUC-035]
depends-on: ["002", "003"]
github-issue: ""
issue: ""
# completes_issue: Controls whether linked issues are archived when this ticket
# is moved to done. Default: true (archive when all referencing tickets are done).
# Set to false (scalar) to suppress archival for ALL linked issues on this ticket.
# Set to a mapping {filename.md: false} to suppress archival per issue filename.
# Use false for tickets that partially address a multi-sprint umbrella issue.
completes_issue: true
# exception: Written by a lower agent when it cannot proceed (see architecture §exception-protocol).
# exception:
#   thrown_by: "programmer"          # "programmer" | "sprint-planner"
#   thrown_at: "2026-05-07T14:23:00Z"
#   attempted: |
#     Description of what was attempted before giving up.
#   conflict: "architecture-update.md §3 — reason the agent is blocked"
#   surface: "internal"              # "user-visible" | "internal"
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

- [ ] `test_tour1_geometry.py` (or its rewritten equivalent) passes under
      `uv run python -m pytest`, using a `FakeTransport`/double instead of
      the deleted `tests/_infra/sim` ctypes library — no skip, an actual
      pass.
- [ ] `test_tour_stop.py` (or its rewritten equivalent) passes the same
      way, confirming Stop Tour re-enables buttons synchronously (ticket
      003's own regression-tested contract) against the new tour driver.
- [ ] `test_tour_idle_detection.py` is deleted (its own subject —
      `_wait_for_idle()` — no longer exists per ticket 003).
- [ ] `pyproject.toml`'s `testpaths` gains the rewritten `tests/testgui/`
      subset (or the whole directory, implementer's call, as long as every
      test that runs actually passes — no newly-collected-but-skipped or
      newly-collected-but-failing file).
- [ ] Full suite (`uv run python -m pytest`) stays green with the new
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
