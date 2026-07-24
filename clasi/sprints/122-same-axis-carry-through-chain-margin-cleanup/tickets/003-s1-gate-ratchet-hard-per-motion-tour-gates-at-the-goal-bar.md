---
id: '003'
title: 'S1 gate ratchet: hard per-motion + tour gates at the goal bar'
status: open
use-cases:
- SUC-077
depends-on:
- '001'
- '002'
github-issue: ''
issue: s1-gate-ratchet-harden-ideal-chip-gates-at-goal-bars.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# S1 gate ratchet: hard per-motion + tour gates at the goal bar

## Description

Convert `test_tour_1_ideal_chip_turns_are_exact`/`test_tour_2_ideal_chip_
turns_are_exact` (currently `xfail(strict=False)` at
`_TURN_TOLERANCE_IDEAL_DEG=0.05`) from aspirational xfail to hard asserts at
the S1 bar (`docs/design/goal-exact-tours.md`: per-motion heading ≤0.1°,
position ≤1mm; tour net heading ≤0.5°, closure ≤5mm, per-leg straight
heading gain ≤0.1°) — using tickets 001/002's ACTUAL achieved numbers, not a
fresh guess. Add new per-motion gates: an isolated 90° turn, an isolated
360° turn, and an isolated 700mm straight (deterministic sim, ideal chip),
each asserted against sim ground truth at the S1 per-motion bar.

If, after tickets 001/002 land, any S1 number is genuinely NOT met, this
ticket does NOT loosen a tolerance to close — it NAMES the physical floor
(the specific mechanism, its magnitude, why it is irreducible) as the new
hard gate, records the derivation in this ticket AND in `move_queue.cpp`'s
comment, and flags it explicitly for STAKEHOLDER adjudication (per the
ratchet issue's own "stakeholder adjudicates any proposed floor" clause) —
never silently accepted. State the gate file's own ratchet rule in its
header: tolerances here only ever tighten.

Realistic-error-profile (S2) gates (`_TURN_TOLERANCE_REALISTIC_DEG=1.0`
etc.) are explicitly OUT OF SCOPE — S2 is gated on OTOS fusion (sprint 126).
This ticket does not touch or promote them, and states so.

Full design context: `sprint.md`'s Use Cases section (SUC-077) and the
Architecture section's sizing/ratchet discussion.

## Approach

1. After tickets 001/002 land, re-run the full closure-gate suite and
   record the actual achieved numbers (per-motion and per-tour, ideal chip).
2. Compare row-by-row against the S1 bar. For each number: hard-assert at
   the bar (with real margin stated) if met; otherwise name and record a
   floor per the Description above, flagged for stakeholder sign-off.
3. Add the three new per-motion gate tests (90° turn, 360° turn, 700mm
   straight) — reuse `test_gui_button_acceptance.py`'s existing managed-turn
   presets as the drive mechanism where suitable. The "straight-with-
   injected-entry-error" per-motion gate mentioned in the source issue is
   explicitly deferred to sprint 123 (needs heading-hold) — NOT this
   ticket's scope; state this deferral.
4. Remove now-dead xfail machinery for the ideal-chip half only
   (`_XFAIL_REASON_IDEAL`); leave the realistic-profile xfail machinery
   untouched (S2, out of scope).
5. Rewrite the gate file's header to state the ratchet rule explicitly.
6. Update `docs/design/goal-exact-tours.md`'s "Current position vs the
   bars" S1 row directly (not overlaid — outside `src/firm`/`src/host`) to
   read "met" or "met at floor X."

## Acceptance Criteria

- [ ] `pytest` (no `--runxfail`) fails on any S1 regression; zero
      `xfail`/`skip` markers remain on ideal-chip accuracy assertions in
      `test_tour_closure_gate.py`.
- [ ] New per-motion gates (isolated 90° turn, 360° turn, 700mm straight)
      exist and assert the S1 per-motion bar against sim ground truth.
- [ ] TOUR_1/TOUR_2 ideal-chip hard gates assert the S1 tour bar (net
      ≤0.5°, closure ≤5mm, per-leg straight heading gain ≤0.1°), replacing
      the current real (non-xfail) `_TURN_TOLERANCE_SHAPED_DEG=2.5`/
      `_CRUISE_HEADING_TOLERANCE_IDEAL_DEG=5.5` gate once superseded.
- [ ] If the bar cannot be met, a named, measured floor is recorded
      (mechanism + magnitude + why irreducible) in this ticket and in
      `move_queue.cpp`'s comment, explicitly flagged for stakeholder
      sign-off — never silently accepted.
- [ ] The gate file's header states the ratchet rule: numbers here only
      ever tighten.
- [ ] Realistic-error-profile gates are explicitly untouched; this ticket
      states so.
- [ ] `docs/design/goal-exact-tours.md`'s "Current position vs the bars" S1
      row is updated to reflect "met" or "met at floor X."

## Files to modify

- `src/tests/testgui/test_tour_closure_gate.py` (primary).
- Possibly a new `src/tests/testgui/test_per_motion_accuracy_gate.py` if the
  per-motion assertions don't fit cleanly into the existing file.
- `docs/design/goal-exact-tours.md` (direct edit, S1 row only).

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/testgui/test_tour_closure_gate.py -v`,
  full `uv run python -m pytest`.
- **New tests to write**: the three per-motion gate tests (90° turn, 360°
  turn, 700mm straight).
- **Verification command**: `uv run python -m pytest src/tests/testgui/test_tour_closure_gate.py -v --runxfail`
  (confirm `--runxfail` has zero effect — nothing should remain marked
  xfail for ideal-chip accuracy).
