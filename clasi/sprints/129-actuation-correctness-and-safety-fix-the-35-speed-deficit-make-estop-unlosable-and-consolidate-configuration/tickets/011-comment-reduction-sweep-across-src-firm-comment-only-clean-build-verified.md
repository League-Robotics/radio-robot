---
id: '011'
title: Comment reduction sweep across src/firm (comment-only, clean-build verified)
status: open
use-cases: [SUC-009]
depends-on: ['001', '002', '003', '006', '007', '009', '010']
github-issue: ''
issue: 01-reduce-to-minimum-useful-comments-across-src-firm.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Comment reduction sweep across src/firm (comment-only, clean-build verified)

## Description

Last ticket in the sprint, by design. Depends on every other
`src/firm`-touching ticket (001, 002, 003, 006, 007, 009, 010 — not 008,
which is host-only) so this sweep judges "is this comment still
load-bearing" against the tree's final state for the sprint, not a
mid-sprint snapshot that would need a second pass.

Stakeholder request (2026-07-31, mid-sprint-128): "reduce to minimum
useful comments across src/firm." The firmware base has accumulated
comment mass across many sprints: narrative change-log comments ("moved
in sprint NNN", "was previously X"), comments that restate what the next
line does, stale references to deleted mechanisms, and reviewer-facing
justification prose that stopped being useful the moment its sprint
closed. The project's own standard is that a comment exists only to state
a constraint the code itself cannot show — unit tags (`// [unit]`),
load-bearing ordering constraints, hardware errata, wire-format
invariants.

One sweep across `src/firm` (base tree only; `src/motion` can follow in a
later pass, per the issue's own scope statement):

- Delete comments that narrate history, restate the adjacent code, or
  justify a change to a long-gone reviewer.
- Keep (and sharpen) the load-bearing minimum: `[unit]` tags, documented
  ordering constraints (e.g. the LOAD-BEARING ORDER notes in
  `robot_loop.cpp`), hardware errata notes (e.g. the I2C guard-removal
  note from 128-013), boundary/writer documentation (e.g.
  `robot_state.h`'s `cmdVelocity`/`pose` field docs), and API doc
  comments on public surfaces. This explicitly includes every comment
  this sprint's own tickets just wrote as load-bearing (the stop-confirm
  rationale in ticket 001, the timescale-separation rationale in ticket
  007, etc.) — the sweep should confirm those are kept, not accidentally
  caught by the deletion pass.
- DESIGN.md files are documentation, not code comments — out of scope
  here; their history-paragraph convention stays.

## Acceptance Criteria

- [ ] Diff is comment-only (zero object-code change) — verify with a
      clean build producing identical behavior / passing the same tests.
- [ ] Each kept comment states something the code cannot: a unit, an
      ordering constraint, an invariant, an erratum, or a public-API
      contract.
- [ ] No `[unit]` tag, LOAD-BEARING note, or errata note is lost.
- [ ] This sprint's own newly-written load-bearing comments (tickets 001,
      002, 003, 006, 007, 009) survive the sweep intact.

## Testing

- **Existing tests to run**: full clean build (`just build-clean`) +
  `motion_tests` + planner `ctest` + firmware pytest tiers — comment-only
  changes must produce byte-identical object code, so a full pass is the
  actual verification, not a spot check.
- **New tests to write**: none — this ticket adds no behavior; the
  "verification" is the clean build + full suite passing identically to
  pre-sweep.
- **Verification command**: `just build-clean && uv run pytest` (plus the
  project's standard firmware test invocation).

## Implementation Plan

- **Approach**: read each file in `src/firm` (base tree), classify every
  comment against the load-bearing test (states a unit, an ordering
  constraint, an invariant, an erratum, or a public-API contract — or
  not), delete the "or not" comments, sharpen the kept ones if they're
  verbose without being more informative.
- **Files to modify**: all of `src/firm` (comment-only diffs).
- **Documentation updates**: none beyond the comments themselves — this
  ticket IS the documentation update.
