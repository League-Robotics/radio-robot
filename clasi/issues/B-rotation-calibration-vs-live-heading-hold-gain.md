---
status: pending
priority: medium
---

# Rotation-gain/offset calibration conflicts with the now-live heading-hold gain

Found during sprint 130 ticket 002 (unify-sim-and-robot-composition-roots.md)
while triaging sim test fallout from composition-root unification.

`composeRobot()` now boots `Motion::Planner`'s REAL `headingHoldGain` (2.0,
baked from the robot JSON) live in sim for the first time -- previously it
was always 0 (effectively off) under `TestSim::SimHarness`'s own pre-130-002
`simPlannerLimits()` literals. `headingHoldGain` is a genuine CLOSED-LOOP
heading corrector that runs DURING a move. `data/robots/tovez_nocal.json`'s
`rotation_gain`/`rotation_offset_deg` calibration (from issue 125-007) is a
separate, OPEN-LOOP host-side command pre-scaling correction, fit against the
OLD sim dynamics where heading-hold was off and the plant's coast-down
overshoot (~13 deg at a 90 deg ANGLE-stop, omega=2.0 rad/s) was uncorrected
by anything else.

With both mechanisms live simultaneously,
`src/tests/sim/system/test_angle_stop_rotation_calibration.py`'s two tests
regress:

- `test_angle_stop_lands_close_to_target_with_tovez_nocal_calibration`: now
  UNDERSHOOTS (measured -10.86 deg vs the +/-5 deg bound) -- the move is
  double-corrected (open-loop `rotation_gain` PLUS closed-loop heading-hold).
- `test_angle_stop_overshoots_without_rotation_calibration` (negative
  control): now lands within the bound on its own (+1.31 deg overshoot,
  expected >5 deg) -- heading-hold alone, with NO `rotation_gain`/offset
  calibration at all, already corrects most of the old coast-down overshoot.

Both tests are marked `xfail` (`strict=False`) with this finding referenced,
so ticket 002 does not silently paper over the regression, and does not
invent a recalibration number without real measurement.

## Needed follow-up (a measurement task, not a composition-root change)

1. Determine whether `rotation_gain`/`rotation_offset_deg` calibration is
   still needed at all now that `headingHoldGain` is genuinely live in sim
   (and has presumably always been live on real hardware, per `main.cpp`'s
   own pre-130-002 boot sequence -- meaning REAL hardware angle-stop moves
   may have been double-corrected in exactly this way all along, an angle
   worth checking against bench data too).
2. If still needed, refit `tovez_nocal.json`'s `rotation_gain`/
   `rotation_offset_deg` against the NEW closed-loop (heading-hold-on)
   dynamics, using the same omega=2.0 rad/s measurement protocol
   `test_angle_stop_rotation_calibration.py`'s own docstring documents.
3. Remove the `xfail` markers from both tests once resolved.

This is adjacent to (but distinct from) sprint 130 ticket 010's own
`sim-tour-turn-shaping-undershoots-90-degree-turns.md` investigation -- both
concern turn/angle dynamics newly exposed by composition-root unification
turning on real shaping/heading-hold for the first time, but this issue is
specifically about the `rotation_gain`/offset calibration's interaction with
heading-hold, not the turn-shaping profile's own undershoot.

Priority: medium (bounded by explicit `xfail`, not silently broken; but
affects turn-accuracy correctness on both sim and potentially real
hardware).
