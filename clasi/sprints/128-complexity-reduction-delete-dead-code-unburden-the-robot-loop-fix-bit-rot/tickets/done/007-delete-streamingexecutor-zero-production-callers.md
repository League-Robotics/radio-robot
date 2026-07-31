---
id: '007'
title: Delete StreamingExecutor (zero production callers)
status: done
use-cases:
- SUC-006
depends-on: []
github-issue: ''
issue: streaming-executor-delete-or-adopt-for-pathplan.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Delete StreamingExecutor (zero production callers)

**Worktree note**: implement in the git worktree checked out for
`sprint/128-complexity-reduction-delete-dead-code-unburden-the-robot-loop-fix-bit-rot`.
All paths below are relative to that worktree.

**Decision (settled, verified this sprint)**: **delete**, not adopt. The
source issue asked whoever built sprint 127 ticket 006's `pathplan`
planner loop to check whether `StreamingExecutor`'s shape fit before
choosing. Verified directly: `grep -rn "StreamingExecutor(" src/`
(construction, not docstring mentions) returns zero results anywhere in
the tree, including `src/tests/`. `pathplan/planner.py` built its own
`gotoWorld`/`followPath` loop from scratch — `StreamingExecutor` was
never adopted. This ticket is a straight deletion, not a build-vs-adopt
decision point.

## Description

`planner/executor.py:94-101`'s `TwistTransport` Protocol docstring claims
"a real `NezhaProtocol` instance already satisfies this Protocol as-is" —
false: only `move_twist` exists, not `.twist(...)`. `planner/tour.py` (the
live tour runner) imports only the `RunOutcome`/`RunState`/`TickResult`
enum/dataclass shells from `executor.py`, running its own `Move`-based
loop against a `MoveTransport` Protocol whose "already satisfies" claim is
actually true.

## Acceptance Criteria

- [x] `planner/executor.py` (`StreamingExecutor` and its untrue
      `TwistTransport` Protocol claim) is deleted.
- [x] The `RunOutcome`/`RunState`/`TickResult` shells `tour.py` imports
      are kept — either left in a slimmed `executor.py` or moved into
      `tour.py` directly (programmer's call; moving into `tour.py` is
      the cleaner outcome if `executor.py` would otherwise be nearly
      empty).
- [x] `grep -rn "StreamingExecutor" src/` shows no construction site
      anywhere (docstring/comment mentions of the historical name in
      other files, e.g. `robot_radio/DESIGN.md`'s own narrative, may
      remain as history — only the class and its construction must be
      gone).
- [x] `tour.py` still passes its existing test suite unchanged.
- [x] No Protocol in `planner/`/`pathplan/` carries an untested "a real
      X satisfies this" claim after this ticket (the one this issue
      found is deleted with its host).

## Testing

- **Existing tests to run**: `uv run python -m pytest`, with particular
  attention to `planner/tour.py`'s own test suite (`test_tour1_geometry.py`,
  `test_sim_transport_tour1.py`, `test_tour_closure_gate.py`) — these
  must pass unchanged since `tour.py` never routed through
  `StreamingExecutor`.
- **New tests to write**: none required for a pure deletion.
- **Verification command**: `uv run python -m pytest src/tests/testgui -q`
  (tour-related suites) plus the full `uv run python -m pytest`.

## Implementation Notes

- **Approach**: delete `executor.py`'s `StreamingExecutor` class and its
  false-claim Protocol; relocate the three shells `tour.py` imports
  rather than deleting them.
- **Files to modify**: `src/host/robot_radio/planner/executor.py`
  (delete or slim to shells only), `src/host/robot_radio/planner/tour.py`
  (update import if the shells move).
- **Documentation updates**: `robot_radio/DESIGN.md`'s `planner/` row —
  drop the `executor.py`/`StreamingExecutor` dormant-half description
  (coordinate with ticket 009's doc-rot sweep if it also touches this
  row).
