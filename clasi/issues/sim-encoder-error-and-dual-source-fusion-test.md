---
status: pending
---

# Sim: encoder error injection + dual-noisy-source EKF fusion test

## Context

Phase 2 (sprint 057) added the ground-truth → sensor-error model and an
`test_ekf_fusion_beats_noise` test. But that test injects error **only into the
OTOS/optical path** — the encoder reads ground truth *perfectly* (0.00 mm error).
So it proves "the EKF discards a bad OTOS and trusts a clean encoder," not genuine
**fusion of two imperfect sources**.

The stakeholder explicitly asked for **error versions of BOTH the encoders AND the
optical flow** ("produce error versions of those for the encoders and optical flow,
the simulated errors for optical flow"). This issue closes that gap so the
drivetrain simulation realistically exercises the EKF the way the real robot does.

## What to build

1. **Encoder error injection in the sim** (`source/hal/sim/`): add per-wheel
   encoder error knobs on the encoder/`SimMotor` path — wheel **slip** (fraction of
   commanded motion not registered), **scale error** (mm-per-tick mismatch), and
   **quantization** — derived from the ground-truth wheel motion, mirroring the
   existing `SimOdometer` OTOS error knobs added in 057-005
   (`setDriftPerTick*`/`set*ScaleError`). Default zero ⇒ no behavior change.
   Add a C-ABI shim to configure them (beside `drive2_api_enable_otos_sim_model`).

2. **A dual-noisy-source fusion test** (extend `test_drive2_subsystem.py` or a new
   `test_ekf_dual_source.py`): inject **both** encoder error (slip/scale) **and**
   OTOS error (drift/scale/noise), command motion, tick ~50–100×, then assert:
   - `fused_err` < each of `encoder_only_err` and `optical_only_err` (the fused
     estimate beats **both** raw sources individually — the real test of fusion),
     within a sensible absolute bound.
   - Choose injection magnitudes so neither raw source is trivially perfect and the
     EKF has to genuinely blend them. If the EKF cannot beat both raws, that is a
     real finding about the fusion/tuning — report it rather than weakening the test.

3. Keep a scenario (or sub-case) where the encoder is good and OTOS is bad (the
   existing 057-005 behavior) so both regimes are covered.

## Verification

- `python build.py --clean` → zero errors.
- `uv run python -m pytest` (use `python -m pytest`, NOT bare `uv run pytest`) →
  baseline "2377 passed, 2 failed" (the 2 pre-existing `tag_offset_mm.z` failures)
  PLUS the new test(s) passing.

## Notes

Small, additive, sim-only. Does not touch the live `loopTickOnce`/`Drive::periodic`
wiring. Builds directly on `subsystems::Drive2` and the 057-005 sim error model.
Relates to [[message-based-subsystem-architecture]] (the simulation requirement).
</content>
