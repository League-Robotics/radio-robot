---
status: done
tickets:
- NONE
---

# Bench OTOS integrates commanded wheel speeds, not measured — blind to coast, tours cannot close

## Problem

`NezhaHAL::tick(now, cmds)` feeds the bench plant the **commanded**
targets `cmds.tgtSpeed[]`
([NezhaHAL.cpp:100](source/robot/NezhaHAL.cpp#L100)). The real wheels do
not execute exactly the command: they lag on ramp-up and, critically,
**coast past the cutoff** after a stop condition zeroes the targets. The
bench "perfect world" pose therefore systematically disagrees with what
the encoders measured — and the EKF, which trusts the OTOS heading
heavily, follows the bench frame.

This is exactly the sim/bench asymmetry: in sim the tours close because
`SimOdometer` **samples plant ground truth** (ticket 066-001) — the same
truth the sim encoders measure, so the two observation streams cannot
disagree. On hardware, "plant truth" of the wheels IS the encoders, but
the bench sensor ignores them and re-integrates the command stream.

## Hardware evidence (2026-07-03, tovez on stand, fw 0.20260703.19, bench mode)

With `SET rotSlip=0` (nocal parity) and zero bench noise, four
consecutive `RT 9000`:

| turn | fused/bench heading gain | encoder differential (geometric, tw=128) |
|------|--------------------------|------------------------------------------|
| 1    | +82.3°                   | ~+91.8°                                   |
| 2    | +82.2°                   | ~+91.5°                                   |
| 3    | +82.0°                   | ~+91.8°                                   |
| 4    | +81.6°                   | ~+92.5°                                   |

The encoder-arc stop is excellent (~92° per 90° command, ~2° coast), but
the bench heading counts only ~82° — the ~10 mm/wheel of coast after
`tgtSpeed` drops to zero is invisible to a commanded-speed integrator.
Net: the fused heading loses ~8°/turn; over Tour 1's six turns that is
~46°, and the tour geometrically cannot return to its start. Straight
`D` legs show the same effect smaller (D 345 → bench 352–353 mm, ~+2%).

With the baked Tovez calibration active (rotSlip=0.92) the mismatch is
larger still: RT 9000 → bench +112–114°/turn, encoders ~124°/turn
(slip-inflated arc executed free-of-scrub on the stand).

## Proposed fix (parity-correct option)

Feed the bench plant **measured** wheel velocities (encoder-derived
`inputs.vel[]`, already computed each tick) instead of `cmds.tgtSpeed[]`
— i.e., make the bench OTOS an errored copy of encoder truth, the same
relationship SimOdometer has to plant truth. The noise/drift error model
stays unchanged. `SimHardware::advance`'s bench branch
([SimHardware.cpp:90](source/hal/sim/SimHardware.cpp#L90)) should get the
same source so sim-bench mode stays behaviour-identical.

Alternative (if commanded-integration is a hard design requirement):
model lag+coast in the bench plant — considerably more work and still an
approximation of what the encoders already measure.

Also worth fixing while in there: `NezhaHAL::_trackwidth` is cached at
construction ([NezhaHAL.cpp:30](source/robot/NezhaHAL.cpp#L30)); a
runtime `SET tw=…` (pushed by the TestGUI on robot select) never reaches
the bench plant. Read the live config instead.
