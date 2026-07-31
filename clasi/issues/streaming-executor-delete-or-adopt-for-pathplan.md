---
status: pending
---

# StreamingExecutor: delete it, or formally adopt it as pathplan's transport loop — not both, not neither

**Source:** code review 2026-07-30, `04-host-planning.md` MAJOR §2.
**Priority:** P1 — ~300 doc-heavy lines with zero production callers and a
false "no adapter needed in production" claim, sitting exactly where sprint
127's ticket 006 is about to build the same shape from scratch.
**Goal served:** either outcome removes a fork: one continuously-replacing
executor loop in the tree, or zero — never a dead one beside a new one.

## What is wrong

- `planner/executor.py:94-101` — `TwistTransport` Protocol requires
  `.twist(...)`; its docstring claims "a real `NezhaProtocol` instance
  already satisfies this Protocol as-is". False: only `move_twist` exists.
- `grep -rn "StreamingExecutor("` across `src/host` + `src/tests`: zero
  production call sites. `planner/tour.py` — the live tour runner — imports
  only the enum/dataclass shells (`RunOutcome`/`RunState`/`TickResult`) and
  runs its own `Move`-based loop against a `MoveTransport` Protocol whose
  "already satisfies" claim is actually TRUE.

## What to do — one of:

**Adopt (check first):** ticket 006's `pathplan` planner loop is a
continuously-replacing outer loop — exactly the shape `StreamingExecutor`
was built and unit-tested for. If it fits, adopt it explicitly: fix
`TwistTransport` to the real surface (`.move_twist(v_x, v_y, omega, *,
stop_time, timeout)` or wrap it), state the adoption in sprint 127's
architecture notes, and make ticket 006 its first production caller.

**Delete:** if ticket 006's design genuinely wants its own loop (its
description sketches one), delete `StreamingExecutor` and its
binding-requirements docstring claims, keeping the `RunOutcome`/`RunState`/
`TickResult` shells where `tour.py` imports them (or move those into
`tour.py`).

Whoever recovers ticket 006/008 should read this issue and
`path-geometry-is-already-solved-reuse-it-for-ticket-008.md` BEFORE writing
new loop/geometry code.

## Acceptance

- Adopt: `StreamingExecutor` has a production caller and its Protocol claim
  is true (unit test constructs it over a real `NezhaProtocol` mock).
- Delete: `grep -rn "StreamingExecutor" src/` returns nothing; `tour.py`
  still passes its suite.
- Either way, no Protocol in `planner/`/`pathplan/` carries an untested
  "a real X satisfies this" claim — each such claim gets a one-line test.
