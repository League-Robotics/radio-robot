---
status: pending
filed: 2026-07-23
filed_by: team-lead (stakeholder-directed)
related:
- land-at-zero-at-orthogonal-chain-boundaries.md
tickets: []
---

# Hold heading during Distance moves (close the loop on the uncommanded omega axis)

## Description

A Distance-kind TWIST Move is heading-open-loop today: `omega` is commanded to
a constant 0 and nothing ever corrects the body's heading during the leg. Any
heading error present at leg entry (chain hand-off residue, and on hardware:
asymmetric friction, deadband, load) persists for the whole leg and translates
directly into lateral path error — the robot drives a straight line in the
wrong direction and never squares up. Measured post-119-005 (sim, ideal chip):
a straight from rest holds heading exactly (leg 1: +0.00°), so the plant needs
no help when entered clean — but every straight entered with residual ω arcs
and then keeps the acquired error forever.

Land-at-zero at orthogonal boundaries (related issue) removes the sim's entry
error at the source; heading hold is the complement that makes straights
self-squaring against whatever error remains — and on hardware, against
disturbances the sim doesn't model. Both are wanted: one removes the known
cause, the other bounds all causes.

## Proposed design

In `MoveQueue::shapeAndStage()` (`src/firm/app/move_queue.cpp`), for a
Distance-kind TWIST Move whose commanded omega is 0, drive the omega axis with
a small proportional correction toward the ACTIVATION heading instead of a
constant 0:

    omegaCorrection = -headingGain * (theta - active_.activationTheta);  // [rad/s]

- `theta` is the SAME same-cycle odometry reading `tick()` already passes in
  (fresh since 118-002 moved the stop decision after `odom_.integrate()`).
- Route the correction through the existing `shaperOmega_` (cruise target =
  omegaCorrection) so it stays slew/jerk-limited — no new actuation path, no
  step commands.
- Clamp the correction magnitude (a config ceiling, e.g. 0.3 rad/s) so a large
  entry error produces a gentle re-square, not a swerve.
- Reference is the Move's own activation heading — each leg squares to its own
  entry line; no global heading state, no cross-Move memory.
- Gain source: resurrect the orphaned `control.heading_kp` robot-JSON key as
  this loop's gain (it currently has no consumer — flagged in the 2026-07-22
  review §6 as config-attic; this gives it a real meaning again). `0` disables
  the hold (byte-identical to today's behavior), the same 0-==-off contract
  ShaperLimits uses.

Scope guards:
- Distance-kind TWIST Moves with `cruiseOmega == 0` ONLY. An arc (Distance +
  nonzero omega) is deliberately untouched — its heading is supposed to
  change. WHEELS Moves untouched (no body-frame intent to hold). Angle-kind
  Moves untouched (omega is the commanded axis).
- This is a ~10-line addition plus config plumbing; no new module, no new
  wire message, no host involvement.

## Acceptance

- Sim: inject a deliberate entry heading error (e.g. 3°) at the start of a
  700 mm Distance Move; the leg ends with |heading − activation heading| ≤
  0.3° and lateral drift materially reduced vs today (today: error persists
  100%).
- Leg-1 exactness preserved: a straight from rest with zero entry error stays
  +0.00° (the correction term is zero on a clean entry; it must not inject
  wobble — assert no omega command exceeds the clamp during a clean leg).
- TOUR_1/TOUR_2 closure gates unchanged or improved; no oscillation at any
  `heading_kp` in the shipped range (state the stable gain range measured).
- Naming/units per `.claude/rules/coding-standards.md` (gain is dimensionless
  1/s — tag accordingly; no units in identifiers).

## Related

- `land-at-zero-at-orthogonal-chain-boundaries.md` — removes the dominant
  entry-error source; do that first so this loop's acceptance numbers measure
  the hold itself, not the boundary bug.
- `docs/code_review/2026-07-22-turn-execution-review.md` §6 item 3 (orphaned
  `control.*` keys) — `heading_kp` moves from attic to consumer instead of
  deletion.
