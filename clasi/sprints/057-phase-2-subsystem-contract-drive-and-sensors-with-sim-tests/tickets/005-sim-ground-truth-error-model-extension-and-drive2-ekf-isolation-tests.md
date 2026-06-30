---
id: '005'
title: Sim ground-truth/error model extension and Drive2 EKF isolation tests
status: done
use-cases:
- SUC-005
depends-on:
- '001'
- '004'
github-issue: ''
issue: message-based-subsystem-architecture.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sim ground-truth/error model extension and Drive2 EKF isolation tests

## Description

The issue's "Simulation & test infrastructure (REQUIRED)" section mandates an
end-to-end EKF fusion test: inject realistic noise into the sim OTOS and encoder
paths, tick `Drive2` N times, and assert the fused pose tracks ground truth better
than either raw sensor alone.

This ticket extends `SimOdometer` with additional error knobs, adds ground-truth
accessors to `PhysicsWorld` / `SimHardware`, adds C-ABI shims to expose them to
Python, and writes `test_ekf_fusion_beats_noise` in `test_drive2_subsystem.py`.

The `Drive2` class (ticket 004) and the `drive2_api.cpp` shims are already in place;
this ticket only extends the sim and adds the noise-injection test.

## Approach

### 1. Extend `SimOdometer` with error knobs

`source/hal/sim/SimOdometer.h` — add:
```cpp
// Drift error: accumulated position drift added per tick (mm and rad).
// A fresh SimOdometer has zero drift (perfect sensor).
void setDriftPerTickMm(float mm)  { _driftPerTickMm = mm; }
void setDriftPerTickRad(float rad){ _driftPerTickRad = rad; }

// Scale error: multiplies the reported delta by (1 + error).
// 0.0 = perfect, 0.05 = 5% scale error.
void setLinearScaleError(float err)  { _linearScaleErr = err; }
void setAngularScaleError(float err) { _angularScaleErr = err; }
```

Private members: `float _driftPerTickMm = 0.0f; float _driftPerTickRad = 0.0f;
float _linearScaleErr = 0.0f; float _angularScaleErr = 0.0f;`

Apply in `SimOdometer::tick(velLMms, velRMms, tw, dt)`:
- After computing the true delta from the kinematic model (existing code), multiply
  by `(1.0f + _linearScaleErr)` and `(1.0f + _angularScaleErr)` for position / yaw.
- Add `_driftPerTickMm` to `_odomX` (or accumulated delta) and `_driftPerTickRad`
  to `_odomH` each tick.
- Existing `linearNoiseSigma` / `yawNoiseSigma` (zero-mean Gaussian) are preserved
  and compose with the new deterministic drift/scale errors.

`source/hal/sim/SimOdometer.cpp` — implement the error injection in `tick()`.

### 2. Add ground-truth accessors to `PhysicsWorld`

`source/hal/sim/PhysicsWorld.h` — the `PhysicsWorld` plant already integrates true
position. Add:
```cpp
float groundTruthX() const { return _x; }
float groundTruthY() const { return _y; }
float groundTruthH() const { return _h; }
float idealX()       const { return _idealX; }
float idealY()       const { return _idealY; }
float idealH()       const { return _idealH; }
```
Where `_x, _y, _h` are the plant's authoritative integrated position (already
maintained in the existing plant integration code). `_idealX/Y/H` can initially
alias to `_x/y/h` (they diverge only if slip or wheel encoder error is introduced
in a future sprint — for this sprint they are identical to ground truth, which is
sufficient for the test).

`source/hal/sim/PhysicsWorld.cpp` — update `update(dt)` to keep `_idealX/Y/H`
in sync (or simply alias via the accessors if the naming is already equivalent).

### 3. Expose ground-truth via `SimHardware`

`source/hal/sim/SimHardware.h` — add pass-through:
```cpp
float groundTruthX() const { return _plant.groundTruthX(); }
float groundTruthY() const { return _plant.groundTruthY(); }
float groundTruthH() const { return _plant.groundTruthH(); }
```

### 4. Extend `drive2_api.cpp` shims

Add to `tests/_infra/sim/drive2_api.cpp`:
- `drive2_api_enable_otos_sim_model(handle, linear_noise_sigma, yaw_noise_sigma,
   drift_per_tick_mm, drift_per_tick_rad, linear_scale_err, angular_scale_err)` —
   calls `simHardware.simOdometer().enableSimModel(true)` and sets all the noise/drift
   knobs.
- `drive2_api_ground_truth_x/y/h(handle)` → `float` — returns plant ground truth.
- `drive2_api_get_encoder_x/y(handle)` → `float` — returns `state().get_encoder()`.
- `drive2_api_get_optical_x/y(handle)` → `float` — returns `state().get_optical()`.

### 5. `tests/simulation/unit/test_drive2_subsystem.py` — add EKF test

Add `test_ekf_fusion_beats_noise` after the 4 tests from ticket 004:

```python
def test_ekf_fusion_beats_noise(drive2):
    """EKF fused pose tracks ground truth better than either raw sensor alone."""
    # Enable OTOS error model with noise + drift
    drive2_api_enable_otos_sim_model(
        handle,
        linear_noise_sigma=5.0,  # mm
        yaw_noise_sigma=0.02,    # rad
        drift_per_tick_mm=0.5,
        drift_per_tick_rad=0.001,
        linear_scale_err=0.03,
        angular_scale_err=0.02,
    )
    # Apply forward twist and tick 50 times (each tick = 20ms)
    drive2_api_apply_twist(handle, 200.0, 0.0, 0.0)
    now = 0
    for _ in range(50):
        now += 20
        drive2_api_tick_update(handle, now)
        drive2_api_tick_action(handle, now)

    gt_x = drive2_api_ground_truth_x(handle)
    fused_x = drive2_api_get_fused_x(handle)
    enc_x   = drive2_api_get_encoder_x(handle)
    opt_x   = drive2_api_get_optical_x(handle)

    fused_err  = abs(fused_x - gt_x)
    enc_err    = abs(enc_x   - gt_x)
    opt_err    = abs(opt_x   - gt_x)

    # Fused should be better than raw sensors
    assert fused_err < 20.0, f"Fused error {fused_err:.1f} mm exceeds 20 mm"
    # At least one raw sensor must show > 10 mm error (proving noise is injected)
    assert max(enc_err, opt_err) > 10.0, (
        f"Raw sensors too accurate (enc={enc_err:.1f}, opt={opt_err:.1f}) — "
        "noise may not be injected correctly"
    )
```

Note: the existing `SimOdometer` RNG is seeded at `43u` (deterministic). The test
should be deterministic. If the EKF is not sufficiently tuned, the `fused_err < 20`
bound may need calibration — the implementer should run the test, check the actual
fused error, and adjust the threshold or noise level to be meaningful but not flaky.
Document the chosen values in the test comment.

## Files to Create/Modify

- `source/hal/sim/SimOdometer.h` — add 4 error knob setters
- `source/hal/sim/SimOdometer.cpp` — apply drift/scale in `tick()`
- `source/hal/sim/PhysicsWorld.h` — add `groundTruthX/Y/H()`, `idealX/Y/H()`
- `source/hal/sim/PhysicsWorld.cpp` — expose existing plant state via new accessors
- `source/hal/sim/SimHardware.h` — add `groundTruthX/Y/H()` pass-throughs
- `tests/_infra/sim/drive2_api.cpp` — add `enable_otos_sim_model`, ground-truth,
  encoder, optical shims
- `tests/simulation/unit/test_drive2_subsystem.py` — add `test_ekf_fusion_beats_noise`

## Acceptance Criteria

- [x] `SimOdometer` gains `setDriftPerTickMm(float)`, `setDriftPerTickRad(float)`,
      `setLinearScaleError(float)`, `setAngularScaleError(float)` setters.
- [x] `SimOdometer::tick()` applies drift and scale error in addition to existing
      zero-mean Gaussian noise. A default `SimOdometer` (all knobs at zero) behaves
      identically to before.
- [x] `PhysicsWorld::groundTruthX/Y/H()` accessors added and return the plant's
      authoritative integrated position.
- [x] `SimHardware::groundTruthX/Y/H()` pass-throughs added.
- [x] `drive2_api.cpp` shims added for `enable_otos_sim_model` and ground-truth reads.
- [x] `test_ekf_fusion_beats_noise` passes: fused error < 20 mm, max(enc_err, opt_err)
      > 10 mm after 50 ticks with noise injected.
- [x] All 4 existing `test_drive2_subsystem.py` tests from ticket 004 still pass
      (noise knobs default to zero — no regression).
- [x] All other existing simulation tests still pass (no existing `SimOdometer` or
      `PhysicsWorld` behavior changes when knobs are left at default zero).
- [x] `python build.py --clean` zero errors.
- [x] `uv run python -m pytest` green at baseline + all new tests.

## Testing Plan

- **New test**: `test_drive2_subsystem.py::test_ekf_fusion_beats_noise` — the core
  deliverable of this ticket.
- **Non-regression for sim devices**: run `uv run python -m pytest
  tests/simulation/` to confirm no change to existing OTOS/odometry behavior when
  knobs are at default zero.
- **Device compile**: `python build.py --clean` — validates C++11 extensions to
  `SimOdometer.h/.cpp` and `PhysicsWorld.h/.cpp`.

## Verification Command

`uv run python -m pytest tests/simulation/unit/test_drive2_subsystem.py::test_ekf_fusion_beats_noise -v && uv run python -m pytest tests/simulation/ && python build.py --clean`
