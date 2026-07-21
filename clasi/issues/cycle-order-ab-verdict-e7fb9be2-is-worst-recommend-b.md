---
status: in-progress
tickets:
- 115-003
---

# Cycle-order A/B verdict: the e7fb9be2 "original" is the WORST; recommend the symmetric "drive-after-both-motors" order (B)

## Context

`RobotLoop::cycle()`'s tick order (where `drive_.tick()` sits relative to the
motor ticks and `pilot_.tick()`) was a live experiment. The stakeholder asked
to revert the main loop "back to its shape" (commit **e7fb9be2**) and A/B it
in sim before hardware. This issue records the verdict so the decision isn't
lost. **The revert direction turned out to be wrong.**

## Three variants measured (sim, tovez_nocal, deterministic, TOUR_1+TOUR_2,
ideal + realistic; worst |turn error| vs SimPlant ground truth)

| Variant | drive_.tick() placement | worst turn err |
|---|---|---|
| **A** (committed HEAD) | top of cycle, ABOVE the motor ticks | ~1.1–1.5° |
| **B** (best) | top of cycle, AFTER both motor ticks | **~0.2–0.7°** |
| **C** = e7fb9be2 "original" | end of R-settle block, feeds motorR_.tick() same cycle | ~2.1–2.3° |

All three complete the tours in sim. **C is the worst.**

## Why C is worst — L/R timing asymmetry

In the e7fb9be2 schedule, `motorL_.tick()` writes L's duty at the TOP of the
cycle (last cycle's target) but `motorR_.tick()` writes R's duty right after
`drive_.tick()` (THIS cycle's target). So left and right wheels get their
targets **one cycle apart** → a systematic over-rotation bias on every turn.
The "reorder experiment" the stakeholder suspected of hurting things was
actually *removing* that asymmetry.

## Corroborating evidence against C

- Sim suite on C: `test_behavior_lock::test_pivot_terminal_bounds` and
  `test_deadband_terminal_correction` FAIL — C changes terminal/pivot behavior.
- Hardware C + calibrated: the 90° turn spins forever (see the turn
  non-termination issue) — though A also spins, so the turn bug is separate.

## Recommendation

- **Do NOT adopt the e7fb9be2 (C) order.** Keep the committed order (A) or,
  better, adopt **B** (move `drive_.tick()` to just after both motor ticks) —
  best sim turn accuracy, symmetric L/R timing, single-line change.
- The cycle order is NOT the tour blocker: turn non-termination and the
  straight wedge happen on every order. Decide cycle order for accuracy, then
  fix the two real blockers separately.
