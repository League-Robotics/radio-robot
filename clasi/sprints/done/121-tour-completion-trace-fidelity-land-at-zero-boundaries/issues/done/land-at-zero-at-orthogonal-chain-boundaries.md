---
status: done
filed: 2026-07-23
filed_by: team-lead (stakeholder-directed; stakeholder decision recorded below)
related:
- chain-advance-completion-margin-narrow-pocket.md
- chain-advance-reset-defeats-same-axis-compatible-leg-continuity.md
tickets:
- 121-003
sprint: '121'
---

# Chain-advance turns: land at zero at ORTHOGONAL boundaries (stakeholder decision — stop sweeping the chain margin)

## Description

With 119-005 (crab fix) landed, TOUR_1 per-leg TRUE heading deltas isolate all
remaining tour error to chain-advance boundaries (deterministic sim, ideal
chip, measured 2026-07-23):

- Leg 1 (straight, from rest): **exactly +0.00°** — the plant executes an
  unchained straight perfectly.
- Every straight FOLLOWING a turn: **+1.34 to +4.24°** gained (mean
  ~+2.9°/boundary) — the turn completes on the loosened chain margin
  (`kStoppingMarginFactorChain=0.60`) with residual ω ≈ 0.5-0.9 rad/s, which
  decays INTO the next Move (omega uncommanded there), arcing the straight's
  entry.
- Turn legs themselves: −2.20 to +2.06° scatter — dominated by crossing the
  angle threshold at speed (per-cycle quantization at 40 ms is 2.3° at
  ω=2 rad/s) plus the swept-margin pocket.
- Contrast: a FINAL (unchained) move under land-at-zero measures −0.3° on a
  360° turn — when crossing speed → 0, the bias → 0 with no compensation.

Tour total: +17.9° of unwanted rotation over 540° commanded, all boundary
residue. The 118-003 resolution already established there is NO broad margin
plateau to sweep to (`chain-advance-completion-margin-narrow-pocket.md`).

## Stakeholder decision (2026-07-23)

Do not keep tuning the chain margin. Split boundary semantics by axis
relationship:

- **Orthogonal boundary** (ending Move's stop-condition axis is NOT commanded
  by the next Move — e.g. turn→straight, straight→turn): the ending axis must
  **land at zero** with the SAME predicate as a final Move
  (`marginFactor=1.00` physical form). There is no velocity worth carrying
  across an orthogonal boundary; trading a beat of corner dwell for exactness
  is accepted.
- **Same-axis compatible boundary** (next Move commands the same axis, same
  sign — e.g. two distance legs): keep the velocity carry; that case is owned
  by `chain-advance-reset-defeats-same-axis-compatible-leg-continuity.md` and
  must not regress (SUC-003 no-dip property).

## Expected result / acceptance

- Straights following turns: heading gain ≤ 0.3° each (from ~+2.9°).
- Turn legs: |error| ≤ ~0.5° (crossing at ω≈0 collapses the quantization
  cost); the 2.5° shaped-band gate tightens accordingly — state the achieved
  band honestly, per the existing gate's own convention.
- TOUR_1 net heading: 540° ± ~1° (from +17.9° over).
- `test_two_compatible_distance_legs_carry_velocity_through_the_boundary_at_
  tour_level` still passes (same-axis carry preserved).
- Per-leg TRUE-heading-delta assertion added to the closure gate (endpoint
  checks are blind to intra-leg drift — the crab lesson).

## If a residual remains after this (and only then)

Compensate analytically, never with a fitted constant: the deterministic
overshoot of crossing a threshold at rate ω with cycle T and plant constant τ
is |ω|·(T/2 + τ). The stop test may use
`remaining <= |omega_measured| * (kCycle/2 + tau_plant)` — every term named and
calibrated, direction-independent, self-scaling. This is the derived version of
what the deleted `stop_lead_ms` approximated by sweep. Expectation: after
land-at-zero at orthogonal boundaries the residual will not justify it.

## Related

- Measurement table: `chain-advance-completion-margin-narrow-pocket.md`
  ("Post-119-005 per-leg measurement" section) and
  `docs/code_review/2026-07-22-turn-execution-review.md`.
- Hand-off contract: sprint 119 `specify-and-assert-the-leg-handoff-contract.md`
  (done) — this decision refines the orthogonal-boundary clause of that
  contract.
