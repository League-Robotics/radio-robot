---
status: in-progress
sprint: '128'
tickets:
- 128-008
---

# The lookahead/arc geometry sprint 127 needs already exists in path/ — reuse it, then delete the leftovers

**Source:** code review 2026-07-30, `04-host-planning.md` MAJOR §3, §4.
**Priority:** P1, and **time-sensitive**: sprint 127 ticket 008
(path-following lookahead target picker) is currently in `exception`;
whoever recovers it should read this FIRST, before writing geometry.
**Goal served:** the review found the same tangent-circle problem solved
twice, unknowingly, four modules apart. Solved-twice geometry is a bug
multiplier — a fix lands in one copy and not the other.

## The two finds

1. `path/catmull_rom.py:63-86` — `find_lookahead_target` +
   `circle_intersections` already implement exactly ticket 008's Main Flow
   ("walk forward from `start_idx`, return the first exit from the lookahead
   circle"), including the `da == db` degenerate case
   (`t = 0.5 if abs(da - db) < 1e-9 else da / (da - db)` — no
   division-by-zero). Zero callers, not exported.
2. `path/arc.py:13-76` — `compute_arc` solves the identical tangent-circle
   problem ticket 005's `pathplan/solver.py::solveArcToPoint` re-derived
   from scratch (different output shape: per-wheel arc distances vs. a
   twist, same hard geometry). Zero callers, not exported.

## What to do

1. **Ticket 008:** adapt `find_lookahead_target`/`circle_intersections` to
   walk a `SampledPath` (`points`/`headings`) instead of a bare tuple list —
   move them into `pathplan/` (the designated owner) or export them from
   `path/` with `pathplan` as the caller. Keep the degenerate-case guard;
   add the unit test that pins it.

```python
# pathplan/lookahead.py -- adapted, not re-derived. The circle-intersection
# core is path/catmull_rom.py's (reviewed correct 2026-07-30); only the
# container walked changed (SampledPath instead of list[tuple]).
def findLookaheadTarget(path: SampledPath, start_idx: int,
                        center: tuple[float, float],
                        radius: float) -> tuple[int, tuple[float, float]]:
    ...
```

2. **`path/arc.py`:** delete it (dead, unexported, superseded by
   `solveArcToPoint`), and note in `pathplan/solver.py`'s header that
   `path/arc.py`'s per-wheel formulation existed and where it went — so the
   duplication is acknowledged history, not an accident waiting to recur.

3. Whichever module keeps the geometry, there is exactly ONE
   circle-intersection implementation in the tree afterwards.

## Acceptance

- `grep -rn "circle_intersections\|find_lookahead_target\|compute_arc" src/host`
  shows one implementation, in the owning package, with live callers and
  tests (including the `da == db` degenerate case).
- `path/arc.py` is gone; `pathplan/solver.py`'s header carries the pointer.
- Ticket 008's implementation imports the shared primitive rather than
  containing its own intersection math.
