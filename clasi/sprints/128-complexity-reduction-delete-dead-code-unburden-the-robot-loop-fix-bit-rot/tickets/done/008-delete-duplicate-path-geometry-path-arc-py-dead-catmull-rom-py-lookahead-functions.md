---
id: 008
title: Delete duplicate path geometry (path/arc.py, dead catmull_rom.py lookahead
  functions)
status: done
use-cases:
- SUC-006
depends-on: []
github-issue: ''
issue: path-geometry-is-already-solved-reuse-it-for-ticket-008.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Delete duplicate path geometry (path/arc.py, dead catmull_rom.py lookahead functions)

**Worktree note**: implement in the git worktree checked out for
`sprint/128-complexity-reduction-delete-dead-code-unburden-the-robot-loop-fix-bit-rot`.
All paths below are relative to that worktree.

**Decision (settled, verified this sprint)**: **pure deletion**, not
adaptation. The source issue (written before sprint 127 ticket 008
landed) recommended adapting `path/catmull_rom.py`'s
`find_lookahead_target`/`circle_intersections` into `pathplan/` for
ticket 008's lookahead target-picker. Verified directly: ticket 008 is
DONE (sprint 127 closed 2026-07-31) and shipped its own
`solver.pursuitTarget()` (lookahead-circle pure pursuit, committed
`8e38b2b6`, 64 unit tests, sim-validated) independently of
`path/catmull_rom.py`'s functions. `grep` confirms zero callers of
`find_lookahead_target`/`circle_intersections` outside their own
definition file, and zero callers of `path/arc.py`'s `compute_arc`
(superseded by ticket 005's `solver.solveArcToPoint`). There is nothing
left to adapt — only a now-confirmed-dead duplicate to delete.

## Description

`path/catmull_rom.py:63-86` (`find_lookahead_target` +
`circle_intersections`) and `path/arc.py:13-76` (`compute_arc`) each
solve a tangent-circle geometry problem that `pathplan/solver.py` already
solves independently, via `pursuitTarget()` and `solveArcToPoint()`
respectively. Neither `path/` implementation has any caller anywhere in
the tree.

## Acceptance Criteria

- [x] `path/arc.py` is deleted outright.
- [x] `path/catmull_rom.py`'s `find_lookahead_target` and
      `circle_intersections` functions are deleted (the rest of
      `catmull_rom.py` — spline construction/sampling — is unaffected
      and stays).
- [x] `pathplan/solver.py`'s module header gains a one-line note that
      `path/arc.py`'s per-wheel formulation and `path/catmull_rom.py`'s
      lookahead functions existed and were superseded by
      `solveArcToPoint()`/`pursuitTarget()` respectively — acknowledging
      the duplication as recorded history, not silently erasing it.
- [x] `grep -rn "circle_intersections\|find_lookahead_target\|compute_arc" src/host`
      returns nothing.
- [x] Exactly one circle-intersection/tangent-arc implementation remains
      in the tree (`pathplan/solver.py`), with live callers and tests
      (including the `da == db` degenerate-case guard, already covered
      by ticket 008's own 64 unit tests from sprint 127 — confirm they
      still pass, do not re-derive).

## Testing

- **Existing tests to run**: `uv run python -m pytest`, in particular
  `pathplan`'s own solver test suite (sprint 127's 64 unit tests covering
  `pursuitTarget`/`solveArcToPoint`) — must be unaffected by this
  deletion since they never depended on `path/`'s duplicates.
- **New tests to write**: none required for a pure deletion; if any test
  imports `path.arc` or the deleted `catmull_rom` functions, remove or
  update it.
- **Verification command**: `uv run python -m pytest src/tests -k "pathplan or solver" -q`,
  then the full `uv run python -m pytest`.

## Implementation Notes

- **Approach**: delete `path/arc.py` outright; remove the two named
  functions from `catmull_rom.py` (not the whole file — its spline
  construction/sampling code is live-adjacent and stays); add the
  acknowledgment note to `solver.py`'s header.
- **Files to modify**: delete
  `src/host/robot_radio/path/arc.py`; edit
  `src/host/robot_radio/path/catmull_rom.py` (remove two functions),
  `src/host/robot_radio/pathplan/solver.py` (header note).
- **Documentation updates**: none required beyond the `solver.py` header
  pointer named above — `robot_radio/DESIGN.md`'s `path/` row already
  correctly marks the package dormant/orphaned; no row change needed for
  removing two functions from within it.
