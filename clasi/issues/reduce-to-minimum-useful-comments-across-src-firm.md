---
status: pending
---

# Reduce to minimum useful comments across src/firm

Stakeholder request (2026-07-31, mid-sprint-128): "reduce to minimum
useful comments across src/firm."

The firmware base has accumulated comment mass across many sprints:
narrative change-log comments ("moved in sprint NNN", "was previously
X"), comments that restate what the next line does, stale references to
deleted mechanisms, and reviewer-facing justification prose that stopped
being useful the moment its sprint closed. The project's own standard
(CLAUDE.md communication rules) is that a comment exists only to state a
constraint the code itself cannot show — units tags (`// [unit]`),
load-bearing ordering constraints, hardware errata, wire-format
invariants.

## Scope

One sweep across `src/firm` (base tree only; `src/motion` can follow in
a later pass):

- Delete comments that narrate history, restate the adjacent code,
  or justify a change to a long-gone reviewer.
- Keep (and sharpen) the load-bearing minimum: `// [unit]` tags,
  documented ordering constraints (e.g. the LOAD-BEARING ORDER notes in
  `robot_loop.cpp`), hardware errata notes (e.g. the I2C guard-removal
  note from 128-013), boundary/writer documentation (e.g.
  `robot_state.h`'s `cmdVelocity`/`pose` field docs), and API doc
  comments on public surfaces.
- DESIGN.md files are documentation, not code comments — out of scope
  here; their history-paragraph convention stays.

## Acceptance sketch

- Diff is comment-only (zero object-code change; verify with a clean
  build producing identical behavior / passing the same tests).
- Each kept comment states something the code cannot: a unit, an
  ordering constraint, an invariant, an erratum, or a public-API
  contract.
- No `// [unit]` tag, LOAD-BEARING note, or errata note is lost.
