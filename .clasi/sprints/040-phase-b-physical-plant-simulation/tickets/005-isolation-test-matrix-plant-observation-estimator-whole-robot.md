---
id: "005"
title: "Isolation test matrix — plant, observation, estimator, whole-robot"
status: open
use-cases: [SUC-001, SUC-002, SUC-003]
depends-on: ["040-004"]
github-issue: ""
issue: "migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md"
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Isolation test matrix — plant, observation, estimator, whole-robot

## Description

Add the four test categories from the §7 verification matrix using the new
`sim_set_true_*`, `sim_get_true_pose_*`, and `estimation_error()` ABI (from T3).
These tests go under `tests/simulation/unit/` and `tests/simulation/system/`.

This ticket is pure test authoring — no source code changes.

### Test 1: plant-only — true pose reaches target under setActuators+update

File: `tests/simulation/unit/test_plant_correctness.py`

The goal is to verify `PhysicsWorld::update` produces the expected kinematic
trajectory without any sensor error involved.

Key scenarios (using `sim_set_true_pose`, `sim_get_true_pose_*`, zero PWM for idle):
- **Straight drive:** Set `pwmL = pwmR = 50` for 1000 ms; assert `true_pose_x > 0`,
  `true_pose_y ≈ 0`, `true_pose_h ≈ 0`. Expected: `x ≈ (50/100)*400*1.0 = 200 mm`.
- **Spot turn:** Set `pwmL = -50, pwmR = 50` for 500 ms; assert `true_pose_h ≈ π/2`
  (quarter turn). Expected arc: `(100 mm/s * 0.5 s) / (trackwidth/2)`.
- **Truth injection:** Call `sim_set_true_pose(0, 0, 0)` mid-sim; verify `sim_get_true_pose_*`
  returns the injected values immediately; next `sim_tick` with zero PWM leaves
  pose unchanged (no velocity).
- **sim_set_enc fix:** Call `sim_set_enc_l(h, 500.0)`; run 5 ticks at PWM=0;
  assert `sim_get_enc_l(h) ≈ 500.0` (not reset to 0). This is the regression test
  for the lying `sim_set_enc_l/r` bug.

Use the `sim` fixture (no `sim_field_profile` — test clean plant with no slip).

### Test 2: observation-only — setTruePose → sensor returns truth ± error

File: `tests/simulation/unit/test_observation_models.py`

The goal is to verify that observation models correctly reflect injected errors
without changing plant truth.

Scenarios:
- **Perfect odometer (default):** `sim_set_true_pose(100, 200, 0.5)`;
  call `sim_set_otos_fusion(h, 1)` and run one tick; assert that `sim_get_otos_x/y/h`
  returns values close to `(100, 200, 0.5)` — odometer reads truth when no error.
- **Read failure dropout:** `sim_set_otos_read_failure(h, 1)`; run one tick;
  assert EKF position does not change drastically (fusion skipped). Reset failure;
  EKF resumes fusing on the next tick.
- **Frozen encoder:** Use `sim_set_perfect()` to reset, then inject a scenario
  where the encoder is frozen (simulated by holding PWM=0 while issuing a motion
  command — or, if `SimMotor::setFrozen` is exposed via ABI, use it). Verify
  that `sim_get_enc_l/r` stays constant while the plant advances.

  NOTE: `SimMotor::setFrozen` is not yet in the C ABI. The programmer may either:
  (a) Add `sim_set_motor_frozen(void* h, int side, int frozen)` to `sim_api.cpp`
  in this ticket (minor ABI addition), or (b) test frozen behavior via the
  existing watchdog/wedge path (`test_033_005_wedge_hardening.py`). Option (b)
  avoids an ABI addition. Programmer decides.

- **Yaw drift (if `SimOdometer::setYawDriftRadsPerSec` is exposed via ABI):**
  Set drift = 0.1 rad/s; run 1000 ms; assert `sim_get_otos_h` drifts by ~0.1 rad
  from plant truth. If not yet in ABI, skip this test in T5; note as deferred.

Use the `sim` fixture with `sim_set_otos_fusion(h, 1)` where needed.

### Test 3: estimator-only — estimation_error() < TOL; bad OTOS rejected

File: `tests/simulation/unit/test_estimator_isolation.py`

The goal is to verify the EKF estimate tracks plant truth and rejects bad OTOS.

Scenarios:
- **Straight drive, no OTOS:** Drive straight for 2 s; assert
  `sim.estimation_error()[0] < 5.0` (EKF tracks dead-reckoning within 5 mm).
  Heading error < 0.05 rad.
- **OTOS fusion:** Enable OTOS model + fusion; drive 2 s; assert estimation error
  < 3 mm (fusion improves estimate).
- **Bad OTOS injection:** `sim_set_otos_pose(h, 9999, 9999, 99)` for 3 ticks;
  assert Mahalanobis gate rejects the bad measurement (EKF reject count increases
  via `sim_get_ekf_rej_count`); true pose from `sim_get_true_pose_*` remains correct.
- **Recovery:** Stop injecting bad OTOS; assert `estimation_error()[0]` returns
  within tolerance after the EKF recovers (P-inflation mechanism).

Use `sim_field_profile` for the field-conditions scenarios.

### Test 4: whole-robot — final TRUE pose within tolerance of D/G/TURN plan

File: `tests/simulation/system/test_whole_robot_plant_correctness.py`

The goal is to verify end-to-end robot motion: from command to true chassis pose.

Scenarios:
- **D command (distance drive):** `D mm=200`; after `EVT done D`, assert
  `sim_get_true_pose_x > 150` (true position, not EKF). Tolerance: 20 mm.
- **G command (go-to):** `G x=200 y=0`; after `EVT done G`, assert
  `sim_get_true_pose_x ≈ 200` within 25 mm and `sim_get_true_pose_y ≈ 0` within 25 mm.
- **TURN command:** `TURN deg=90`; after `EVT done TURN`, assert
  `sim_get_true_pose_h ≈ π/2` within 0.1 rad.
- **estimation_error gate:** After each command, assert `sim.estimation_error()[0] < 20`
  (EKF tracks true pose within 20 mm).

Use `sim_field_profile` fixture for realistic slip conditions.

## Acceptance Criteria

- [ ] `test_plant_correctness.py` exists and passes all four scenarios.
- [ ] `test_observation_models.py` exists and passes all implemented scenarios
      (frozen-encoder and yaw-drift deferred if ABI not added).
- [ ] `test_estimator_isolation.py` exists and passes all four scenarios.
- [ ] `test_whole_robot_plant_correctness.py` exists and passes all three
      command scenarios plus estimation error gate.
- [ ] `uv run --with pytest python -m pytest -q` ≥ 1957 + new tests passed, 0 errors.
      (The floor grows from 1957 by however many new test cases are added.)
- [ ] All existing canaries still green.
- [ ] No source code changes (test files only).

## Implementation Plan

### Approach

Pure test authoring. Use the `sim` and `sim_field_profile` fixtures from `conftest.py`
and the new `get_true_pose()`, `estimation_error()`, `set_true_pose()`,
`set_perfect()` methods added to `Sim` in T3.

### Files to Create

- `tests/simulation/unit/test_plant_correctness.py`
- `tests/simulation/unit/test_observation_models.py`
- `tests/simulation/unit/test_estimator_isolation.py`
- `tests/simulation/system/test_whole_robot_plant_correctness.py`

### Files to Modify

- `tests/_infra/sim/sim_api.cpp` — optional: add `sim_set_motor_frozen` if
  programmer chooses option (a) for the frozen-encoder test.
- `tests/_infra/sim/firmware.py` — optional: corresponding Python wrapper.

### Testing Plan

```
uv run --with pytest python -m pytest \
    tests/simulation/unit/test_plant_correctness.py \
    tests/simulation/unit/test_observation_models.py \
    tests/simulation/unit/test_estimator_isolation.py \
    tests/simulation/system/test_whole_robot_plant_correctness.py \
    -v
uv run --with pytest python -m pytest -q
```

### Documentation Updates

None required — tests are self-documenting.
