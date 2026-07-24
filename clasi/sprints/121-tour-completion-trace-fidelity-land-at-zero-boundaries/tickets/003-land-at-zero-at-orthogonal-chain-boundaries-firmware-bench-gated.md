---
id: '003'
title: Land at zero at orthogonal chain boundaries (firmware; bench-gated)
status: open
use-cases:
- SUC-074
depends-on: []
github-issue: ''
issue: land-at-zero-at-orthogonal-chain-boundaries.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Land at zero at orthogonal chain boundaries (firmware; bench-gated)

## Description

The meaty firmware control change of this sprint, implementing a recorded
STAKEHOLDER DECISION (read the "Stakeholder decision" block in
`land-at-zero-at-orthogonal-chain-boundaries.md`).

With the 119-005 crab fix landed, TOUR_1 per-leg TRUE heading deltas isolate
all remaining tour error to chain-advance boundaries (deterministic sim, ideal
chip, 2026-07-23): leg 1 (straight from rest) is exactly +0.00 deg; every
straight FOLLOWING a turn gains +1.34..+4.24 deg (mean ~+2.9 deg/boundary — the
turn completes on the loosened chain margin `kStoppingMarginFactorChain` with
residual omega ~0.5-0.9 rad/s that decays into the next `Move` (omega
uncommanded there), arcing the straight's entry); turns scatter
-2.20..+2.06 deg. Tour total +17.9 deg over 540 commanded, all boundary
residue. A FINAL (unchained) move under land-at-zero measures -0.3 deg on a
360 deg turn — when crossing speed to zero, the bias to zero with no
compensation.

## Stakeholder decision (implement exactly this split)

Do NOT keep tuning the chain margin. Split boundary semantics by axis
relationship, in `MoveQueue::landAtZero()` / `MoveQueue::tick()`
(`src/firm/app/move_queue.cpp`):

- **Orthogonal boundary** (the ending `Move`'s stop-condition axis is NOT
  commanded by the incoming chained `Move` — turn->straight, straight->turn):
  the ending axis must LAND AT ZERO with the SAME predicate as a FINAL `Move`
  (physical stopping-distance form, final-move margin, NO chain discretization
  term). There is no velocity worth carrying across an orthogonal boundary;
  trading a beat of corner dwell for exactness is accepted.
- **Same-axis compatible boundary** (next `Move` commands the same axis, same
  sign — e.g. two Distance legs): KEEP the velocity carry. That case is owned
  by sprint 122 (`chain-advance-reset-defeats-same-axis-compatible-leg-
  continuity.md`) and MUST NOT regress (SUC-003 no-dip property).

## Approach

- **Detect the boundary kind.** On completion, when `pendingCount_ > 0`,
  inspect the incoming pending `Move` (`pending_[0]`): does it command the
  ending `Move`'s own stop-kind axis (and compatible sign)? If NOT (orthogonal),
  select the FINAL-move completion predicate so the ending axis lands at zero.
  If YES (same-axis compatible), keep the chain (carry) predicate — the
  deferred-to-122 path, unchanged here. When `pendingCount_ == 0` the final
  predicate already applies (queue drain), unchanged.
- **Predicate selection today** in `landAtZero()` is
  `pendingCount_ > 0 ? kStoppingMarginFactorChain : kStoppingMarginFactorFinal`
  and `discretizationCycles = pendingCount_ > 0 ? kDiscretizationCyclesChain :
  0.0f`. Generalize to: use the FINAL branch (`kStoppingMarginFactorFinal`, no
  discretization term) for BOTH the drain case AND an orthogonal chain
  boundary; use the CHAIN branch only for a same-axis compatible chain
  boundary.
- **Margin value — reconcile the issue's "1.00" phrasing with the code.** The
  issue says orthogonal boundaries land "with marginFactor=1.00 physical form,"
  but the shipped `kStoppingMarginFactorFinal` is 0.92 (re-swept 119-005). Reuse
  the FINAL-move predicate (i.e. `kStoppingMarginFactorFinal`); DO NOT hardcode
  1.00 — the "1.00" phrasing predates the 0.92 re-sweep. Verify the chosen
  value against the closure gate; if the orthogonal case genuinely wants a
  distinct value, derive/verify it, do not assume. (sprint.md Architecture Open
  Question 1.)
- **No fitted constant.** If a residual remains after landing at zero,
  compensate ANALYTICALLY with the deterministic overshoot of crossing a
  threshold at rate omega with cycle `T` and plant constant `tau`:
  `remaining <= |omegaMeasured| * (kCycle/2 + tauPlant)`, every term named and
  calibrated, direction-independent, self-scaling — the derived version of the
  deleted `stop_lead_ms`. Expectation (from the issue): after land-at-zero at
  orthogonal boundaries the residual will not justify it.
- **Decoupling note for sprint 122.** After this change,
  `kStoppingMarginFactorChain` / `kDiscretizationCyclesChain` govern ONLY
  same-axis boundaries; land-at-zero replaces their orthogonal-boundary use.
  Update `move_queue.cpp`'s own anonymous-namespace comments to say so (they
  currently describe the chain constants as governing every chain-advance
  turn). Do NOT re-sweep the chain constants here — that is 122's scope.

Coding standards: Google C++ condensed
(`docs/reference/google-cppguide-condensed.md`) with project overrides —
UpperCamelCase types/namespaces, lowerCamelCase functions/variables (never
PascalCase functions), NO units in identifiers (units in a leading `// [unit]`
comment tag, e.g. `// [rad/s]`, `// [s]`), members trailing-underscore. Name
the quantity, not the unit (`tauPlant`, not `tau_ms`).

## Files to modify

- `src/firm/app/move_queue.cpp` — `landAtZero()` boundary-kind selection and
  the anonymous-namespace comments about the chain constants' scope; possibly
  `tick()` where the completion path reads `pendingCount_`. `move_queue.h` if a
  helper predicate/signature is added.
- `src/tests/testgui/test_tour_closure_gate.py` — ADD a per-leg TRUE-heading-
  delta assertion (endpoint checks are blind to intra-leg drift — the crab
  lesson). The existing `TurnCheck` (per-turn achieved-vs-commanded) and
  `StraightLegCruiseCheck` (per-straight max |heading delta| during the leg)
  are the hooks; tighten/assert the straight-following-a-turn gain to the
  acceptance band below and state the achieved band honestly per the gate's own
  convention.
- `clasi/sprints/121-.../design/DESIGN.md` (the OVERLAY copy of
  `src/firm/app/DESIGN.md`) — edit in place: the land-at-zero completion
  paragraph in §4 (orthogonal boundaries land at zero with the final predicate;
  the chain constants now govern only same-axis boundaries). Diff via
  `clasi.design.overlay.generate_diffs` and validate via
  `clasi design validate --overlay` before close.

## Acceptance Criteria

- [ ] Orthogonal chain boundaries (turn->straight, straight->turn) complete
      with the ending axis landing at zero via the FINAL-move predicate;
      same-axis compatible boundaries keep the chain (carry) predicate unchanged.
- [ ] Straights following turns gain <= 0.3 deg each (from ~+2.9 deg); turn legs
      |error| <= ~0.5 deg; TOUR_1 net heading 540 deg +- ~1 deg — measured
      against sim ground truth (ideal chip), the 2.5 deg shaped-band gate
      tightened accordingly, achieved band stated honestly.
- [ ] `test_two_compatible_distance_legs_carry_velocity_through_the_boundary_at_
      tour_level` still passes (same-axis carry preserved; the deferred-to-122
      half is not regressed).
- [ ] A per-leg TRUE-heading-delta assertion is added to the closure gate.
- [ ] No fitted constant; any residual compensation uses the analytic
      `|omega| * (kCycle/2 + tauPlant)` form with every term named/calibrated.
- [ ] `move_queue.cpp`'s comments and the overlaid `app/DESIGN.md` §4 state that
      `kStoppingMarginFactorChain`/`kDiscretizationCyclesChain` now govern only
      same-axis boundaries (decoupling note for sprint 122).
- [ ] STANDING VERIFICATION GATE (`.claude/rules/hardware-bench-testing.md`):
      built + flashed to the robot on the stand and exercised on real hardware —
      sensors alive and changing, wheels drive both directions with encoders
      incrementing, and a tour (or the relevant managed turn->straight sequence)
      driven and observed over the real link — NOT tests alone. Record the
      bench results in this ticket.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/testgui/test_tour_closure_gate.py`
  (TOUR_1/TOUR_2 x ideal/realistic), `test_gui_button_acceptance.py`'s
  managed-turn presets (the settle-based +-90 deg checks that 119-005's
  final-margin re-sweep protected), plus the `App::MoveQueue` sim unit harness
  (`src/tests/sim/unit/test_app_move_queue*` if present) and the full
  `uv run python -m pytest` gate.
- **New tests to write**: the per-leg TRUE-heading-delta assertion in the
  closure gate; an orthogonal-boundary land-at-zero unit assertion in the
  MoveQueue harness (a turn->straight boundary lands omega at ~0); confirm the
  same-axis-carry test is exercised and green.
- **Verification command**: `uv run python -m pytest src/tests/testgui/test_tour_closure_gate.py`
  for the sim gate, THEN `mbdeploy deploy --build` + a bench tour/turn run over
  `/dev/cu.usbmodem2121102` on the stand (bench gate) with results recorded here.
