---
id: '001'
title: Sim OTOS ground-truth sampling and lever-arm compensation
status: done
use-cases:
- SUC-001
- SUC-002
depends-on: []
github-issue: ''
issue: sim-otos-fidelity-ground-truth-and-lever-arm.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sim OTOS ground-truth sampling and lever-arm compensation

## Description

`SimOdometer` currently re-integrates the plant's *true* (pre-slip) wheel
velocities with the same midpoint-arc kinematics `PhysicsWorld::update()`
already runs, so with zero configured noise the sim OTOS can never disagree
with the encoders, and `readTransformed()` applies no mounting-offset
(lever-arm) compensation at all — the exact host-side compensation code a
past hardware regression (`db11b7c`, 433 mm phantom translation on a pure
spin) broke has zero sim reachability (CR-07, CR-08,
`clasi/issues/sim-otos-fidelity-ground-truth-and-lever-arm.md`).

This ticket makes `SimOdometer` a genuine ground-truth-sampling sensor with a
modeled lever arm, sharing the *same* compensation math the real `OtosSensor`
driver uses (extracted to a new header,
`source/hal/capability/OtosLeverArm.h`), so a future regression in that
math — hardware or sim — fails identically in both. See
`architecture-update.md` §"Sim OTOS ground truth + lever arm (CR-07/CR-08)"
and Design Rationale Decisions 1-2 for the full design.

This ticket also resolves CR-15 item 1 (`PhysicsWorld._truePoseH` wrapping),
since it becomes load-bearing here: `SimOdometer` starts reading
`_truePoseH` directly instead of maintaining its own independently-wrapped
copy.

## Acceptance Criteria

- [x] `source/hal/capability/OtosLeverArm.h` exists with `sensorToCentre()`
      and `centreToSensor()` pure functions, extracted verbatim from
      `OtosSensor::readTransformed()`/`OtosSensor::setWorldPose()`'s existing
      formulas (no behavior change to those two functions beyond calling the
      extracted helpers).
- [x] `PhysicsWorld::update()` wraps `_truePoseH` to `(-π, π]` after each
      accumulation.
- [x] `SimOdometer`'s constructor takes `const RobotConfig&` in addition to
      `const PhysicsWorld&`; `SimHardware`'s constructor threads its existing
      `cfg` member through.
- [x] `SimOdometer::tick()` no longer takes wheel velocities — it samples
      `PhysicsWorld::truePoseX/Y/H()`, computes the delta since the previous
      sample, and applies the existing noise/drift/scale-error knobs to that
      delta (same public setter API: `setLinearNoiseSigma`,
      `setYawNoiseSigma`, `setDriftPerTickMm`, `setLinearScaleError`,
      `setAngularScaleError` — unchanged).
- [x] `SimOdometer::readTransformed()` projects the accumulated centre
      estimate through `centreToSensor()` then `sensorToCentre()` (both from
      `OtosLeverArm.h`) before returning — the same round-trip
      `db11b7c` broke on hardware.
- [x] New sim test: pure spin with a nonzero `odomOffX`/`odomOffY` configured
      → OTOS-derived centre translation stays ≈ 0.
- [x] New sim test: turn with chassis-truth slip configured
      (`sim_set_motor_slip` in the effective `[0.5, 1.0]` range, see
      `Odometry.h::effectiveSlip`) → encoder pose and OTOS pose disagree the
      way they do on hardware (OTOS ≈ plant truth, encoder does not); with
      fusion enabled the fused estimate tracks OTOS.
- [x] `test_perfect_otos_tracks_truth_on_straight_drive`,
      `test_perfect_otos_tracks_turn` (`test_observation_models.py`) and
      every other existing sim-OTOS test pass unmodified (verified, not
      assumed — see architecture-update.md's explicit existing-test-impact
      analysis; no test in the current suite configures chassis-truth slip
      while asserting OTOS/encoder agreement).
- [x] Full default test suite green (`uv run --with pytest python -m pytest
      -q`).
- [x] ARM firmware builds clean (`--clean` build per
      `stale-incremental-build-on-volumes.md`).

## Implementation Plan

**Approach:**
1. Add `source/hal/capability/OtosLeverArm.h` with the two pure transforms,
   copied from `OtosSensor.cpp`'s existing `readTransformed()` (sensor→centre)
   and `setWorldPose()` (centre→sensor) bodies.
2. Refactor `OtosSensor::readTransformed()`/`setWorldPose()` to call the
   extracted functions. Run existing OTOS-driver tests to confirm
   behavior-preservation.
3. Fix `PhysicsWorld::update()`'s heading wrap. Run `test_physics_world_basic.py`
   to confirm no regression to the golden-TLM sub-step A/A′ paths (unaffected
   — the wrap only touches sub-step B's `_truePoseH`).
4. Change `SimOdometer`'s constructor and `tick()` signature; update
   `SimHardware`'s constructor and its one `_odom.tick(...)` call site in
   `advance()`.
5. Rework `SimOdometer::readTransformed()` for the sample-then-project design
   (architecture-update.md Decision 2: accumulator stays centre-frame; the
   lever-arm round-trip happens only inside `readTransformed()`).
6. Write the two new sim tests.
7. Run the full existing sim-OTOS test set explicitly (see below) to confirm
   the architecture doc's existing-test-impact analysis holds in practice,
   not just on paper.

**Files to create:**
- `source/hal/capability/OtosLeverArm.h`
- `tests/simulation/unit/test_sim_otos_lever_arm.py` (or extend
  `test_observation_models.py` — programmer's call, whichever keeps the file
  cohesive)

**Files to modify:**
- `source/hal/sim/SimOdometer.h`, `source/hal/sim/SimOdometer.cpp`
- `source/hal/sim/SimHardware.h`, `source/hal/sim/SimHardware.cpp`
- `source/hal/sim/PhysicsWorld.cpp`
- `source/hal/real/OtosSensor.cpp`

**Testing plan:**
- Existing tests to run: `tests/simulation/unit/test_observation_models.py`,
  `test_sim_otos_heading_reset.py`, `test_otos_warn_persistence.py`,
  `test_estimator_isolation.py`, `test_estimator_command_paths.py`,
  `test_fusion_validation.py`, `test_ekf_dual_source.py`,
  `test_physics_world_basic.py`,
  `tests/simulation/system/test_ekf_odometry_commands_coverage.py`,
  `test_goto_bounds.py`, `test_golden_tlm.py` (confirms sub-step A/A′
  untouched), full default suite.
- New tests: pure-spin lever-arm test; turn-with-slip OTOS/encoder
  divergence test (both described in Acceptance Criteria).
- Verification command: `uv run --with pytest python -m pytest -q` (full
  default suite), plus `--clean` ARM build.

**Documentation updates:** None beyond this ticket and
`architecture-update.md` (already written this planning pass).
