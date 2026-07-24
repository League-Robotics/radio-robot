---
id: '002'
title: 'Same-axis carry-through: conditional completing-axis reset'
status: open
use-cases:
- SUC-076
depends-on:
- '001'
github-issue: ''
issue: chain-advance-reset-defeats-same-axis-compatible-leg-continuity.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Same-axis carry-through: conditional completing-axis reset

## Description

Make `MoveQueue::tick()`'s completing-axis reset conditional on the existing
`sameAxisCompatible(pending_[0])` predicate (121-003 scaffolding, unchanged)
— skip the reset when true (carry the shaper's running state through the
boundary), keep it when false (orthogonal boundary, or drain-to-stop —
unchanged from today). Re-verify against ticket 001's NEW analytic
completion predicate and the tour-closure gate — NOT a re-sweep of any
deleted margin constant; `chain-advance-completion-margin-narrow-pocket.md`'s
own narrow-pocket finding is superseded by ticket 001's predicate, not
silently reproduced.

**Read the Open Decision block at the top of `sprint.md` before starting.**
If re-verification finds this conditional reset regresses chain-turn
accuracy — the way the structurally-similar `pendingCount()`-gated variant
already did once (118-003's own record: best worst-case 2.932° vs. the
shipped 2.323° at the time, reverted) — this is the trigger for that
surfaced stakeholder decision, not a signal to re-tune or invent a new
predicate. Report which outcome actually occurred, with measured numbers,
regardless of which branch fires.

Full design context: `sprint.md`'s Architecture Decision 4 (why this ticket
is sequenced after, not merged with, ticket 001) and the Open Decision block.

## Approach

1. Read the Open Decision block in `sprint.md`.
2. Add the `sameAxisCompatible()` gate to `tick()`'s completing-axis reset
   (skip when true, keep when false). No new predicate invented.
3. Run `test_two_compatible_distance_legs_carry_velocity_through_the_
   boundary_at_tour_level` against the new (ticket 001) predicate.
4. If it passes with the existing 90%-of-`v_max` no-dip floor: remove its
   `xfail` marker, done.
5. If it regresses (dip below floor, or chain-turn accuracy regresses on
   TOUR_1/TOUR_2 — verify these explicitly, don't assume they're unaffected
   just because their boundaries are orthogonal): invoke the Open Decision —
   report the regression with numbers; implement the stakeholder-accepted
   bounded-recovery-time alternative if/when the team-lead resolves it, and
   rewrite the test's assertion to that stated bound (cycles/ms).
6. Update `tick()`'s own comment to clearly distinguish this
   `sameAxisCompatible()`-gated variant from the already-rejected
   `pendingCount()==0`-gated variant (118-003) — do not let them read as the
   same experiment.
7. Update the sprint's `design/DESIGN.md` overlay if the reset's contract
   description in §4 needs it.

## Acceptance Criteria

- [ ] The completing-axis reset (`shaperOmega_.reset()`/`shaperVX_.reset()`)
      is skipped exactly when `sameAxisCompatible()` is true; kept
      otherwise. No new predicate invented.
- [ ] `test_two_compatible_distance_legs_carry_velocity_through_the_
      boundary_at_tour_level` passes with its existing 90%-of-`v_max`
      no-dip floor intact, `xfail` removed — **OR**, if regression is
      found, the stakeholder-accepted bounded-recovery-time alternative
      (Open Decision) is implemented instead, with the accepted window
      stated in cycles/ms and the test's assertion rewritten to it. Either
      outcome is reported here with measured numbers.
- [ ] TOUR_1/TOUR_2 (100% orthogonal boundaries by construction) are
      verified unaffected by this change, not assumed.
- [ ] The updated `tick()` comment clearly distinguishes this
      `sameAxisCompatible()`-gated variant from the already-rejected
      `pendingCount()==0`-gated variant (118-003).
- [ ] No new constant.
- [ ] STANDING VERIFICATION GATE (`.claude/rules/hardware-bench-testing.md`):
      built + flashed to the robot on the stand and exercised on real
      hardware — sensors alive and changing, wheels drive both directions
      with encoders incrementing, and a same-axis chained motion (or a tour
      containing one) driven and observed over the real link — NOT tests
      alone. Record the bench results in this ticket.
      (pending team-lead bench run on the stand)

## Files to modify

- `src/firm/app/move_queue.cpp` — `tick()`'s reset block + its own comment.
- `src/tests/testgui/test_tour_closure_gate.py` — resolve the
  `test_two_compatible_distance_legs_carry_velocity_through_the_boundary_at_tour_level`
  `xfail` one way or the other.
- `src/tests/sim/unit/test_app_move_queue.py` — confirm/add the same-axis
  carry-through-reset unit scenario (121-003 added scenario 19 for the
  unchanged-behavior side; this ticket needs the CHANGED-behavior side).
- `clasi/sprints/122-same-axis-carry-through-chain-margin-cleanup/design/DESIGN.md`
  — overlay edit if the reset's contract description needs it.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/testgui/test_tour_closure_gate.py -v`,
  `src/tests/sim/unit/test_app_move_queue.py`, full `uv run python -m pytest`.
- **New tests to write**: a same-axis-carry-through-reset unit scenario in
  the `MoveQueue` harness if not already covered.
- **Verification command**: `uv run python -m pytest src/tests/testgui/test_tour_closure_gate.py -v`
  for the sim gate, THEN a bench run on the stand (bench gate) with results
  recorded here.
