---
status: done
tickets:
- NONE
---

# D/T/TURN terminal reverse-creep: stakeholder decision needed — bounded-correction approach proven infeasible

## Status: needs a stakeholder (Eric) decision before any D/T motion change ships

Sprint 092 ticket 001 attempted the bounded stop-decel seed correction
(option (a) from [[d-t-terminal-reverse-persists-decel-reseed-from-plan-velocity]])
to fix the terminal reverse-creep sprint 089 left on hardware (11–21 mm on
`D`, 19–23 mm on `T`, measured at bench in 089-007). The approach is now
**proven infeasible in sim** and was NOT shipped — nothing merged to master;
the robot's motion behavior is unchanged. This issue carries the decision
forward. The original in-sprint issue archives with sprint 092; THIS pool
issue is the live one.

## What was tried and why it failed (sim-proven)

A new `JerkTrajectory::solveToVelocityWithSeedCorrection(target, max,
measured, cap)` nudged the stop-decel seed velocity toward the measured
wheel speed, bounded by a cap, once per stop-arm event (at
`armDistanceStopDecel()`/`armVelocityStopDecel()`/`armRotationalStopDecel()`).
The 9 new monotonic-convergence sim tests (D/T/RT synthetic-observation +
unit-level cap/rotational proofs) all pass in isolation. **But the full
`tests/sim` gate regressed**: 3 failed / 308 passed / 2 xfailed —
`test_motion_overshoot_regression.py` D-200-200-500 (087-009 baseline
502.27 mm/+0.45% → 516.29 mm/+3.26%, over bar) and two TURN absolute-heading
tests (~98° vs 90°, over the 6° bar).

Root cause (instrumented):
1. **No workable cap exists.** `armStopDecel()` fires — by 087-009 /
   Decision-10 design — very close to the plan's own natural convergence, so
   `lastVelocity_` is already low at that instant and the realistic sim PID's
   higher instantaneous read looks like ordinary end-of-decel noise, not
   sustained divergence. A cap sweep (20/40/60/80/100 mm/s) showed D passes
   only at **cap ≤ 40 mm/s** but the confirmed hardware divergence needing
   correction is **50–110 mm/s**. No single cap both fixes the real bug and
   avoids over-correcting the normal case into overshoot.
2. **TURN has a separate blocker.** `rotationalArcScale_` defaults to **1.0**
   (a 089-005 position-domain placeholder, not a physical trackwidth/2
   conversion), so dividing a measured wheel-velocity differential by it
   inflates the "rad/s" wildly and always saturates the cap — an
   unconditional bias, not a measurement-informed correction. RT (a real
   trackwidth/2 conversion) was unaffected and stayed green.

## The decision (options, refined)

- **(a) A reworked bounded correction.** The scoped version is dead, but a
  variant might work if the arm-timing / divergence-detection is reworked so
  the correction only fires on *sustained* divergence, AND
  `rotationalArcScale_` is first made a real trackwidth/2 conversion. Higher
  complexity; still needs bench proof. Reopens some of the 087-009 tuning.
- **(b) Retune the velocity PID** so the real wheel tracks the plan (removes
  the root divergence rather than patching the seed). Touches the
  sprint-077-tuned bench defaults (`boot_config.cpp` kp/ki/kff); needs bench
  time and risks its own regressions.
- **(c) Accept a terminal-tolerance bar.** Declare ~11–21 mm terminal reverse
  within spec and close the issue — no code change. Cheapest; a product call.

## Preserved work

Branch `spike/092-001-infeasible-bounded-seed-correction` (commit
`3559d28e`) holds the full approach plus the 9 passing monotonic-convergence
sim tests — reusable if option (a)-variant or the `rotationalArcScale_` fix
is pursued. Not for merge.

## Recommendation

Bring this to Eric. Given his sprint-089 stance on the seeding contract and
that this is a control-safety change, do not pick (a)/(b)/(c) autonomously.
The `rotationalArcScale_` placeholder (item 2) is a real latent defect worth
fixing regardless of which option is chosen for the linear channel.

## Closed 2026-07-09 — obsoleted by the sprint 094 motion-stack replacement (stakeholder triage)

The a/b/c decision is framed entirely against the old `Subsystems::Planner`
stop-decel machinery, which no longer exists on the live build: sprint 094
parked the Planner (`source_parked/094/subsystems/planner.{h,cpp}`), deleted
`Motion::VelocityRamp`, and replaced the motion path with the Drivetrain's
`Motion::SegmentExecutor`. Both technical blockers documented above are moot
in the new executor:

1. The bounded-seed-correction code path (option (a)'s target) was never
   merged and its host machinery is gone.
2. The `rotationalArcScale_` placeholder defect is fixed by construction —
   `SegmentExecutor::arcScale_` is exactly `trackwidth_/2` (a pivot phase's
   arc is *defined* as `|targetAngle| * trackwidth/2` at phase start; see
   `source/motion/segment_executor.h`).

Whether terminal reverse-creep persists on the NEW segment path is an
empirical question covered by sprint 094's standing bench gate ("no terminal
reverse-creep — regression check vs 093"). If it shows up there, file a
fresh issue against `SegmentExecutor` — the sequencing remains per the
stakeholder's 093 direction: fix the actuation-latency gap first
([[motor-actuation-latency-flipflop-coupling]]), then residual reversals.
The spike branch `spike/092-001-infeasible-bounded-seed-correction`
(commit `3559d28e`) stays preserved for reference.
