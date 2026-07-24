---
status: pending
filed: 2026-07-23
filed_by: team-lead (replan Step 0; review §8 item G3)
related:
- replan-sprints-122-plus-to-close-goal-exact-tours.md
tickets: []
sprint: '122'
---

# S1 gate ratchet: convert the ideal-chip accuracy gates to permanent hard asserts at the goal bars

## Description

`docs/design/goal-exact-tours.md` S1 bar: per-motion heading ≤ 0.1°, position
≤ 1 mm; tour net heading ≤ 0.5°, closure ≤ 5 mm, per-leg straight heading gain
≤ 0.1° — deterministic sim, ideal chip. Today the "exact" gates are
aspirational `xfail`s (`test_tour_closure_gate.py`) with looser shipped bands
(2.5° shaped band). Once sprint 122 lands the analytic completion, the S1
numbers must become HARD, non-xfail asserts that run in the default suite and
never loosen.

## What to do

- Replace the aspirational-xfail pair with hard gates at the S1 bar, per-leg
  assertions included (TRUE-heading delta per leg, not endpoint-only — the
  crab lesson).
- Add the per-motion gates: single 90°/360° turn, single 700 mm straight,
  straight-with-injected-entry-error (after 123).
- If a bar cannot be met, the sprint closes by NAMING the physical floor with
  a measurement (mechanism + magnitude + why it is irreducible), never by
  loosening the number. Stakeholder adjudicates any proposed floor.
- Record the ratchet rule in the gate file's header: tolerances only ever
  tighten.

## Acceptance

- `pytest` default run fails if any S1 number regresses; no xfail/skip paths
  remain for ideal-chip accuracy.
- TOUR_1/TOUR_2 pass the S1 tour bar on this checkout, or the named-floor
  record exists and the stakeholder has signed it.
