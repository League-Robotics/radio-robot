---
id: '003'
title: Adopt cycle-order variant B (drive_.tick() after both motor ticks)
status: open
use-cases:
- SUC-115-005
depends-on:
- '002'
github-issue: ''
issue:
- cycle-order-ab-verdict-e7fb9be2-is-worst-recommend-b.md
- cycle-order-reorder-experiment-ab-before-hardware.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Adopt cycle-order variant B (drive_.tick() after both motor ticks)

## STAKEHOLDER APPROVAL — CONDITION SATISFIED, EXECUTE THIS TICKET

This ticket was planned as conditional on explicit stakeholder approval of
cycle-order variant B (see sprint.md SUC-115-005 and this sprint's
Architecture Open Questions). **That condition is now satisfied: Eric
approved the sprint 115 plan, including adoption of variant B, at sprint
review on 2026-07-21** (recorded by team-lead in the sprint's
`stakeholder_approval` gate notes: *"Eric approved the sprint 115 plan
2026-07-21 (\"okay, do it\") after reviewing the detail plan: 10-ticket
dependency-ordered plan incl. the conditional cycle-order variant B
adoption (ticket 003) — approval read as covering B, flagged to
stakeholder for interrupt if not intended."*). Do not re-ask the
stakeholder or treat this as still open — execute the code change below
as designed. If Eric interrupts to say the approval did not cover this,
that supersedes this note; absent such an interrupt, proceed.

## Description

`RobotLoop::cycle()`'s tick order (where `drive_.tick()` sits relative to
the two motor ticks and `pilot_.tick()`) was A/B/C-measured in sim
(`clasi/issues/cycle-order-ab-verdict-e7fb9be2-is-worst-recommend-b.md`):

| Variant | `drive_.tick()` placement | worst turn err |
|---|---|---|
| A (committed HEAD, pre-ticket) | top of cycle, ABOVE the motor ticks | ~1.1-1.5° |
| **B (adopt this ticket)** | top of cycle, AFTER both motor ticks | **~0.2-0.7°** |
| C (e7fb9be2 "original", rejected) | end of R-settle block | ~2.1-2.3° |

Root cause of A's inferiority: `motorL_.tick()` writes L's duty at the TOP
of the cycle (last cycle's target) while `motorR_.tick()` (if `drive_.tick()`
sits above both) would get THIS cycle's target-write timing skewed a full
cycle apart from L, biasing every turn. Variant B eliminates the L/R timing
asymmetry with a single-line move. The companion issue
(`cycle-order-reorder-experiment-ab-before-hardware.md`) is the
tracking/process record of the live experiment this verdict resolves —
its own "keep the reorder, A/B compare before hardware" instruction is
superseded by this ticket's execution.

Note the scope boundary from the same issue: cycle order is **not** the
tour's hard blocker (turn non-termination and the terminal wedge happen
under every order — those are sprint 119's job). This ticket is purely
about turn-accuracy, independent of termination behavior.

## Implementation Plan

- **Approach**: move the `drive_.tick()` call in
  `App::RobotLoop::cycle()` (`src/firm/app/robot_loop.cpp`) to
  immediately after BOTH motor ticks (`motorL_.tick()`/`motorR_.tick()`)
  have run this cycle — a single-line relocation, not a restructure of
  the surrounding schedule blocks. Do not touch `kSettle`/`kClear`/
  `kCycle`/`kPace` (ticket 002's concern, independent).
- **Comment/doc updates**: `App::Pilot::tick()`'s own comment currently
  claims it runs "BEFORE `drive_.tick()`" — this is already false today
  (per the reorder-experiment issue) and must now be made true and
  accurate for the shipped order; update it to describe the real,
  now-permanent placement. `src/firm/DESIGN.md`'s cycle-placement table
  must be updated to match. Retire any remaining "this is an experiment"
  framing in both files — this is now the shipped, permanent order, not
  a live A/B trial.
- **Files to modify**: `src/firm/app/robot_loop.cpp` (`cycle()`'s
  `drive_.tick()` call site + `App::Pilot::tick()`'s comment, if that
  comment lives in this file — confirm actual location before editing),
  `src/firm/DESIGN.md` (cycle-placement table).
- **Verification**: re-run the same sim acceptance traces the issue's own
  verdict was measured against (D700 straight, 360° pivot,
  `tovez_nocal`, deterministic, TOUR_1+TOUR_2, ideal + realistic) and
  record the resulting worst-case turn error — it should land in the
  ~0.2-0.7° range the issue reports for variant B, not regress toward A
  or C's numbers.

## Acceptance Criteria

- [ ] `drive_.tick()` is called immediately after both `motorL_.tick()`
      and `motorR_.tick()` have run, every cycle.
- [ ] `App::Pilot::tick()`'s own comment accurately describes its
      placement relative to `drive_.tick()` under the new (shipped)
      order — no stale "BEFORE drive_.tick()" claim, no "experiment"
      framing.
- [ ] `src/firm/DESIGN.md`'s cycle-placement table matches the shipped
      order.
- [ ] Sim acceptance traces (D700 straight, 360° pivot) are re-run and
      the resulting worst-case turn error is recorded in this ticket's
      completion notes, confirming it matches (or improves on) the
      issue's own ~0.2-0.7° figure for variant B.
- [ ] `test_behavior_lock::test_pivot_terminal_bounds` and
      `test_deadband_terminal_correction` (the two sim tests the issue's
      own evidence section names as sensitive to cycle order) still pass.
- [ ] No change to `kSettle`/`kClear`/`kCycle`/`kPace` or any other
      schedule-block timing (ticket 002's independent concern).

## Testing

- **Existing tests to run**: full sim suite, with particular attention to
  `test_behavior_lock::test_pivot_terminal_bounds` and
  `test_deadband_terminal_correction`; `just build-clean`.
- **New tests to write**: none required — the existing D700-straight/
  360°-pivot acceptance traces are the verification instrument; record
  their numbers rather than adding new test files.
- **Verification command**: `uv run pytest`
