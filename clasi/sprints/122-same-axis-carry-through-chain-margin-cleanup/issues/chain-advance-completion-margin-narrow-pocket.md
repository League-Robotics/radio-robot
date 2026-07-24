---
status: in-progress
filed: 2026-07-23
filed_by: team-lead (118-003 resolution escalation, commit b736a4ab)
related:
- specify-and-assert-the-leg-handoff-contract.md
tickets:
- 119-002
- 122-001
sprint: '122'
---

# Chain-advance completion margin is a narrow pocket, not a plateau

## Description

The land-at-zero completion predicate (`MoveQueue::landAtZero()`,
`move_queue.cpp`) is robust for FINAL moves (`marginFactor=1.00`, physical
stopping-distance form, cadence-transferable via the measured-dt
discretization term). But the CHAIN-advance case
(`kStoppingMarginFactorChain=0.60` + `kDiscretizationCyclesChain=0.53`,
swept at 40 ms parity in 118-003's resolution) sits in a narrow pocket:
closure-gate worst 2.323° at the shipped point, 3.7-4.5° at neighbors
0.02-0.03 away. A ~90-build sweep (1-D, 2-D joint, plus a tested-and-
reverted conditional-reset structural variant) found NO broad plateau under
the 2.5° band — TOUR_1/TOUR_2's varied turn angles (90/124/146/215/217°,
both directions) cross zero-error at different coefficients.

Root cause (from the resolution report): tours alternate Distance/Angle
legs, so every chain-advance turn hands its axis to a Move that doesn't
command it; completion is scored at the ack instant while the post-handoff
coast is only partially visible — an ack-timing/quantization artifact, not
a control error. The final-move case has no such sensitivity.

## What to do (future sprint, likely with the handoff contract)

Revisit with a different mechanism rather than more sweeping. Candidates:
score/complete chain turns on the same settle-consistent basis as final
moves; have the handoff contract (related issue) define the axis-drop coast
explicitly so the predicate can subtract it; or complete on measured (not
commanded) axis speed for the chain case. Phase-B bench data should say
whether the pocket is even observable on hardware before investing.

Not urgent while the closure gate is green; becomes urgent the first time
the gate flakes on an unrelated change.

## Post-119-005 per-leg measurement (2026-07-23, team-lead session, deterministic sim, ideal chip)

With the straight-leg crab fixed, TOUR_1 per-leg TRUE heading deltas isolate the
remaining error entirely to chain boundaries: leg 1 (straight, from rest) is exactly
+0.00°; every straight FOLLOWING a turn gains +1.34 to +4.24° (mean ~+2.9°/boundary —
the turn's residual omega handed off into a Move that commands omega=0, decaying while
driving); turns scatter -2.20 to +2.06°. Tour total +17.9° over 540 commanded, all of
it boundary residue. Supports the "complete chain turns on the same settle-consistent
(land-at-zero) basis as final moves" candidate: at near-zero crossing speed the
per-cycle quantization cost also collapses, so this addresses BOTH the straight-leg
skew and most of the turn scatter. Note the trade to decide explicitly: orthogonal
(turn->straight) boundaries carry no useful velocity anyway; same-axis carry is the
separate reset-defeats issue.
