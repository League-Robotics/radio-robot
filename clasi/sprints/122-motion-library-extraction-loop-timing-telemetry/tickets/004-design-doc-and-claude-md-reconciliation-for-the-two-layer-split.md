---
id: '004'
title: Design-doc and CLAUDE.md reconciliation for the two-layer split
status: open
use-cases: [SUC-004]
depends-on: ["003"]
github-issue: ''
issue:
- extract-motion-library-to-src-motion.md
- telemetry-report-loop-cycle-duration.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Design-doc and CLAUDE.md reconciliation for the two-layer split

## Description

Bring the design-doc set and `CLAUDE.md` into agreement with the code
after tickets 001-003 land. This is required, not optional bookkeeping:
`close_sprint`'s overlay-apply step runs full canonical `validate_design`
and fails closed if a declared root (`src/firm`, per
`.clasi/config.yaml`'s `sources:`) has a one-level-down child directory
with no `DESIGN.md`, or a `DESIGN.md` whose subsystem directory no longer
exists. This sprint's extraction empties `move_queue`/`state_estimator`/
`odometry` out of `src/firm/app/` (app/ itself still exists and keeps its
own DESIGN.md, just narrower) and empties `src/firm/motion/` and
`src/firm/kinematics/` entirely (their only contents move to
`src/motion`) — those two directories becoming empty (or disappearing) is
exactly the case sprint.md's Open Question 4 flags: decide per what the
validator actually requires, don't leave a dangling requirement.

`src/motion/DESIGN.md` (stubbed in ticket 001) gets its real content
here: full module description, the boundary interface, and an explicit
statement that it sits outside `.clasi/config.yaml`'s validated `sources:`
(stakeholder-locked scope decision — do not add `src/motion` to that
list).

## Acceptance Criteria

- [ ] `docs/design/design.md` §2 (Subsystem Map) and §5 (dependency
      diagram / firmware-tree overview) updated: `src/firm/motion` and
      `src/firm/kinematics` no longer describe the moved modules; a new
      row or section points to `src/motion` as the (unvalidated, per §4's
      existing rationale) motion library, alongside `src/sim`/
      `src/protos`/etc. in the "Other source trees" table.
- [ ] `src/firm/app/DESIGN.md` updated: no longer describes
      `MoveQueue`/`StateEstimator`/`Odometry` as `app/`'s own modules;
      describes `Drive` as the narrowed wheel-target sink implementing
      the boundary interface; documents the two new telemetry fields
      (`cycle_busy`/`cycle_period`) if ticket 003 didn't already.
- [ ] `src/firm/motion/DESIGN.md` and `src/firm/kinematics/DESIGN.md`:
      either deleted (if their directories are now empty/removed and the
      validator requires no doc for a nonexistent subsystem) or rewritten
      as a short redirect ("this subsystem moved to src/motion, sprint
      122 — see src/motion/DESIGN.md") — whichever
      `clasi.design.validator`'s actual behavior requires. Verify by
      running `validate_design` (or `close_sprint`'s dry-run path if
      available) against the post-move tree before considering this
      criterion met, not by inspection alone.
- [ ] `src/motion/DESIGN.md` has full content: purpose, module list
      (MoveQueue, StateEstimator, Odometry, BodyKinematics, StopCondition,
      VelocityShaper, twist decomposition), the boundary interface's
      In/Out shape, the `motion_tests` build target, and an explicit note
      that this directory is real/current documentation but sits OUTSIDE
      `.clasi/config.yaml`'s validated `sources:` (mirrors the existing
      rationale `docs/design/design.md` §4 already gives for `src/sim`/
      `src/protos`/`src/scripts`).
- [ ] `.clasi/config.yaml`'s `sources:` list is UNCHANGED
      (`[src/firm, src/host]`) — explicitly verify this file was not
      touched; this is a locked scope decision, not an oversight to fix.
- [ ] `CLAUDE.md` updated to name the two layers (firmware base vs.
      motion library) and the boundary between them, at whatever altitude
      the rest of `CLAUDE.md` already uses for architecture pointers (a
      short paragraph + a pointer to `docs/design/design.md` and
      `src/motion/DESIGN.md`, not a full restatement).
- [ ] `validate_design` (or the equivalent check `close_sprint` runs)
      passes clean against the final tree — this is the concrete,
      checkable definition of "done" for this ticket, not "looks right on
      read-through."

## Testing

- **Existing tests to run**: none code-level (this is a documentation
  ticket) — but DO run whatever CLASI-side design validation is available
  standalone (`validate_design` MCP tool, if callable outside a full
  `close_sprint`) to confirm the fix before the sprint actually closes,
  rather than discovering a validator failure only at close time.
- **New tests to write**: none.
- **Verification command**: `validate_design` (CLASI MCP tool) if
  available as a standalone check; otherwise defer final confirmation to
  `close_sprint`'s own validation step, called out explicitly as a risk
  in this ticket if no standalone check exists.

## Implementation Plan

**Approach**: read the final post-ticket-002/003 tree state directly
(don't reconstruct it from memory of the architecture doc — verify
against actual file layout), update each doc in turn, then validate.

**Files to modify**:
- `docs/design/design.md` (§2, §5, and any other section referencing the
  old `motion/`/`kinematics/` locations or the pre-split dependency
  diagram)
- `src/firm/app/DESIGN.md`
- `src/firm/motion/DESIGN.md`, `src/firm/kinematics/DESIGN.md` (delete or
  redirect, per validator behavior)
- `CLAUDE.md`

**Files to create**:
- `src/motion/DESIGN.md` (full content; ticket 001 only stubbed it)

**Testing plan**: run the design validator against the final tree;
fix any reported gap before considering this ticket done.

**Documentation updates**: this ticket IS the documentation update — no
further downstream doc work expected after it, beyond whatever the sprint
retrospective / consolidate-architecture pass does at a later sprint's
own discretion.
