---
id: '001'
title: Encoder error injection + dual-source EKF fusion test
status: done
use-cases:
- SUC-001
- SUC-002
depends-on: []
github-issue: ''
issue: sim-encoder-error-and-dual-source-fusion-test.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Encoder error injection + dual-source EKF fusion test

## Description

Sprint 057's `test_ekf_fusion_beats_noise` injects error only into the OTOS
path, proving the EKF can discard a noisy optical sensor when the encoder is
clean. It does not prove genuine dual-source fusion. This ticket adds encoder
error injection to the sim and a new test that exercises the EKF with both
sensors imperfect simultaneously.

The work is purely additive and sim-only. It builds directly on the
`subsystems::Drive2` and the 057-005 sim error model.

## Acceptance Criteria

- [x] `python build.py --clean` completes with zero errors.
- [x] `uv run python -m pytest` passes the existing baseline ("2377 passed,
      2 failed" — the 2 pre-existing `tag_offset_mm.z` failures) AND all new
      tests in this ticket.
- [x] `drive2_api_enable_encoder_sim_model` exists in `drive2_api.cpp` with
      the correct C-ABI signature and works (configures both `SimMotor` instances).
- [x] After calling only the encoder shim (no OTOS error), the encoder-only
      pose error after 50+ forward ticks exceeds 5 mm — confirming injection
      is active.
- [x] `test_ekf_dual_source_fusion` asserts `fused_err < encoder_only_err`
      AND `fused_err < optical_only_err`, with both raw errors exceeding 5 mm
      (non-trivial magnitudes so the assertion is not vacuous).
- [x] `test_otos_bad_encoder_good` (or equivalent) preserves the 057-005
      single-bad-sensor scenario and continues to pass.
- [x] All new error parameters default to zero — a freshly constructed
      `Drive2Handle` with no shim calls behaves identically to before.

## Implementation Plan

### Approach

Three-file change: `PhysicsWorld` (error fields + update logic), `SimMotor`
(setters forwarding to `PhysicsWorld`), `drive2_api.cpp` (C-ABI shim). Then
a new Python test file.

### Files to Modify

**`source/hal/sim/PhysicsWorld.h`**
- Add private fields `_encScaleErrL`, `_encScaleErrR`, `_encSlipL`, `_encSlipR`
  (all `float`, default `0.0f`).
- Add public setters:
  ```cpp
  void setEncoderScaleError(int side, float err);  // 0=left,1=right,2=both
  void setEncoderSlip(int side, float fraction);   // 0=left,1=right,2=both
  ```

**`source/hal/sim/PhysicsWorld.cpp`**
- In `update()`, within the reported-encoder accumulation block (sub-step A),
  apply the scale error and slip to the per-tick delta before accumulation:
  ```cpp
  // delta is the per-tick motion for this side (same as before).
  float deltaL = /* existing expression */;
  // Apply scale error: reported delta grows or shrinks by (1 + scaleErr).
  deltaL *= (1.0f + _encScaleErrL);
  // Apply slip: a fraction of motion is not registered.
  deltaL *= (1.0f - _encSlipL);
  _reportedEncLMm += deltaL;
  // Symmetric for R side.
  ```
  This must not touch `_trueEncLMm` / `_truePoseX/Y/H`.
  The golden-TLM path (zero scale error, zero slip, zero noise) must remain
  bit-identical to the pre-058 output.

**`source/hal/sim/SimMotor.h`**
- Add two public setters (inline, forwarding to `PhysicsWorld`):
  ```cpp
  void setScaleError(float err);    // fractional; 0 = perfect
  void setSlip(float fraction);     // fractional; 0 = perfect
  ```

**`source/hal/sim/SimMotor.cpp`**
- Implement `setScaleError` and `setSlip`, forwarding to the new
  `PhysicsWorld::setEncoderScaleError(sideIdx(), err)` and
  `PhysicsWorld::setEncoderSlip(sideIdx(), fraction)` methods.

**`tests/_infra/sim/drive2_api.cpp`**
- Add one new `extern "C"` function after `drive2_api_enable_otos_sim_model`:
  ```cpp
  // Configure per-wheel encoder error on both SimMotors.
  // slip_l/r: fraction of motion not registered (0 = perfect, 0.05 = 5% slip).
  // scale_err_l/r: fractional over/under-report (0 = perfect, 0.05 = 5% over).
  void drive2_api_enable_encoder_sim_model(void* h,
                                           float slip_l,
                                           float slip_r,
                                           float scale_err_l,
                                           float scale_err_r)
  {
      Drive2Handle* d = static_cast<Drive2Handle*>(h);
      d->hal.simMotorL().setSlip(slip_l);
      d->hal.simMotorR().setSlip(slip_r);
      d->hal.simMotorL().setScaleError(scale_err_l);
      d->hal.simMotorR().setScaleError(scale_err_r);
  }
  ```

### Files to Create

**`tests/simulation/unit/test_ekf_dual_source.py`**

Three tests:

1. **`test_encoder_error_injection_only`** — verify the encoder shim works in
   isolation.
   - Create a `Drive2Ctx`.
   - Call `drive2_api_enable_encoder_sim_model(h, slip_l=0.05, slip_r=0.05, scale_err_l=0.05, scale_err_r=0.05)`.
   - Do NOT enable OTOS error (OTOS stays perfect).
   - Apply `vx=200` twist; tick 60 times at 20 ms each.
   - Assert `abs(encoder_x - ground_truth_x) > 5.0` — injection is working.
   - Assert `abs(fused_x - ground_truth_x) < abs(encoder_x - ground_truth_x)` —
     EKF leans on clean OTOS to correct the noisy encoder.

2. **`test_ekf_dual_source_fusion`** — the headline deliverable (SUC-002).
   - Create a `Drive2Ctx`.
   - Enable encoder error: `slip_l=slip_r=0.04`, `scale_err_l=scale_err_r=0.05`.
   - Enable OTOS error: `linear_noise_sigma=4.0`, `yaw_noise_sigma=0.015`,
     `drift_per_tick_mm=0.4`, `drift_per_tick_rad=0.0008`, `linear_scale_err=0.03`,
     `angular_scale_err=0.02`.
   - Apply `vx=200` twist; tick 75 times at 20 ms each (re-apply twist each tick).
   - Read `gt_x`, `fused_x`, `enc_x`, `opt_x`.
   - Compute `fused_err`, `enc_err`, `opt_err`.
   - Assert `enc_err > 5.0` and `opt_err > 5.0` (both non-trivially wrong).
   - Assert `fused_err < enc_err` (fused beats encoder).
   - Assert `fused_err < opt_err` (fused beats optical).
   - If either final assertion fails, the failure message must print all four
     position values and the three error values so a real EKF tuning finding
     is surfaced clearly, not silently swallowed.

3. **`test_otos_bad_encoder_good`** — regression of the 057-005 scenario.
   - Mirrors `test_ekf_fusion_beats_noise` in `test_drive2_subsystem.py` but
     lives here to confirm both regimes pass in the same test run.
   - Enable OTOS error only (same parameters as the existing 057-005 test).
   - Tick 50 times; assert `fused_err < 20.0` AND `max(enc_err, opt_err) > 10.0`.

The file loads `drive2_api_enable_encoder_sim_model` via ctypes after importing
`_load_lib` from `test_drive2_subsystem` (both are in `tests/simulation/unit/`
and importable as siblings). Add the new shim's ctypes signature at the top of
the test file alongside the import.

### Testing Plan

- Build: `python build.py --clean` from the repo root (verifies the C++ changes compile).
- Full suite: `uv run python -m pytest` — must show baseline 2377 + 2 failed
  unchanged, plus 3 new tests collected and passing.
- If pytest is not installed in the uv venv: `uv pip install pytest` first.

### Documentation Updates

No public docs to update. The `SimMotor.h` and `drive2_api.cpp` docstrings
should be updated inline to mention the new error knobs (mirror the style of
the 057-005 `setDriftPerTickMm` / `drive2_api_enable_otos_sim_model` comments).
