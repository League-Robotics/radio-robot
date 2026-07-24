---
source_file: DESIGN.md
source_hash: 4ff0474f6e1d8ae51af06b9e15046a862f32c747c47cc76b4fdf879fed110df9
---
# Diff: DESIGN.md

Comparison of the sprint overlay copy of `DESIGN.md` against its pristine (seed-commit) canonical version.

```diff
--- DESIGN.md (pristine)
+++ DESIGN.md (current)
@@ -231,6 +231,80 @@
 planned consumer for future fake-OTOS/fusion bench work, per the same
 "greenfield, not yet wired to motion" posture the 117 note above already
 established for the estimator as a whole.
+
+**121 ticket 003 (land at zero at orthogonal chain boundaries — landed;
+stakeholder decision, 2026-07-23) — splits the land-at-zero completion
+predicate above by AXIS RELATIONSHIP at a chain boundary, not merely by
+whether one is imminent.** With the 119-005 crab fix landed, TOUR_1/TOUR_2
+per-leg TRUE-heading measurement isolated the remaining tour error to
+chain-advance boundaries: every straight FOLLOWING a turn gained
++1.34–4.24° (mean ~+2.9°/boundary) because the ending turn completed on
+the CHAIN margin with residual ω that decays into the next `Move` (which
+does not command ω), arcing the straight's entry. `MoveQueue::
+landAtZero()`/`tick()` now classify every chain-advance boundary
+(`pendingCount_ > 0`) into one of two kinds via a new pure predicate,
+`MoveQueue::sameAxisCompatible(next)`: a **same-axis compatible** boundary
+(the incoming pending `Move` continues the ending `Move`'s own stop-kind
+axis — `v_x` for Distance, `ω` for Angle — in the SAME direction, e.g. two
+Distance legs both forward) keeps the existing CHAIN margin
+(`kStoppingMarginFactorChain`/`kDiscretizationCyclesChain`, UNCHANGED,
+sprint 122's own deferred velocity-carry scope); an **orthogonal**
+boundary (turn→straight, straight→turn — the incoming `Move` does NOT
+continue this axis) now lands the ending axis at zero via a THIRD,
+dedicated constant, `kStoppingMarginFactorOrthogonal`, structurally
+shaped like the drain-case FINAL branch (no discretization term) but NOT
+numerically equal to it: the plan's own default proposal — reuse
+`kStoppingMarginFactorFinal` (0.92) verbatim — was verified against the
+closure gate, not assumed, and measurably FAILED (worst |turn error|
+8.043°/7.863° ideal/realistic, against the shaped-band gate's 2.5°) —
+TOUR_1/TOUR_2 alternate Distance/Angle unconditionally, so EVERY boundary
+in both tours is orthogonal, and reusing the settle-based drain margin for
+an ack-instant, never-settles chain-advance recreates the exact
+measurement-convention mismatch that originally justified
+`kStoppingMarginFactorChain`'s own existence, one level down. A dedicated
+1-D sweep of `kStoppingMarginFactorOrthogonal` against the SAME gate found
+a genuinely broad plateau at [0.665, 0.674]; 0.67 (mid-plateau) ships.
+`kStoppingMarginFactorChain`/`kDiscretizationCyclesChain` now govern
+ONLY a same-axis-compatible chain boundary — decoupled from orthogonal
+accuracy, their remaining tuning is sprint 122's concern.
+
+**Honest residual (the issue's own "if a residual remains" clause).**
+Even at the shipped 0.67, the sprint's own aspirational SUC-074 targets —
+straight-following-turn gain ≤0.3°, turn |error| ≤0.5°, TOUR_1 net
+heading 540°±1° — are NOT met. Measured against `test_tour_closure_gate.py`
+at 0.67: turn |error| 2.314° (ideal, TOUR_1)/2.100° (realistic, TOUR_2);
+straight-leg cruise |delta| 4.104° (ideal)/9.852° (realistic); TOUR_1/ideal
+net heading closure residual ~+21° over the 540° commanded. This is
+COMPARABLE TO, not dramatically better than, the pre-ticket baseline
+(chain margin applied uniformly, since 100% of TOUR_1/TOUR_2 boundaries
+are orthogonal): turn 2.195°/2.218°, cruise 4.254°/9.307°, net closure
+~+17.9°/+34.2° — this ticket's margin-only mechanism avoids the
+disastrous naive-reuse regression and keeps every EXISTING hard gate
+(shaped-band 2.5°, cruise 5.5°/10.5°) passing with real margin, but does
+NOT deliver the hoped-for cruise/closure improvement. Root cause, and why
+no further margin sweep can fix it: the residual ω/`v_x` that "decays
+into the next Move" is the REAL PLANT's own post-reset momentum
+(`tick()`'s own unconditional `shaperOmega_.reset()`/`shaperVX_.reset()`
+zeroes the KINEMATIC shaper target instantly, but the physical
+wheel/velocity-PID plant that had been tracking the PREVIOUS nonzero
+target does not stop instantly) — a separate physical effect from "how
+much of the taper's own v²/(2·a) remaining distance is left," which is
+all a `marginFactor` scale on that formula can ever adjust. The full
+[0.00, 1.00] sweep confirms this structurally: low margins minimize
+cruise-leak but blow turn-error up via raw-backstop-driven OVERSHOOT;
+high margins minimize net-heading closure (crossing ~0 around 0.85–0.90)
+but blow turn-error up via a large systematic UNDERSHOOT that a following
+straight leg's own compensating overshoot happens to cancel — a
+two-wrongs-cancel artifact, rejected on that basis, not a genuine fix.
+Closing this residual properly needs the issue's own analytic
+`remaining ≤ |ω_measured|·(kCycle/2 + τ_plant)` form using an ACTUALLY
+MEASURED velocity (not this predicate's own kinematic `cmd`) and an
+independently-characterized `τ_plant` (measured via an isolated
+step-response test, not fitted against this same closure gate) — both
+are new capability beyond this ticket's authorized scope and are flagged
+for a follow-up ticket rather than rushed into a second fitted constant.
+Full sweep table and derivation in `move_queue.cpp`'s own
+anonymous-namespace comment.
 
 **120 (bench tour bring-up: ack ring + build-selectable fake OTOS + I2C
 safety-net diagnosis — all three tickets LANDED).** Three independent,
```
