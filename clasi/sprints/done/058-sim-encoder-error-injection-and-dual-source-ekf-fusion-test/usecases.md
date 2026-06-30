---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 058 Use Cases

## SUC-001: Inject deterministic encoder error into the drive simulation

- **Actor**: Test author / CI system
- **Preconditions**: A `Drive2Handle` has been created via `drive2_api_create`.
  The `SimMotor` observed encoder path (`reportedEncMm`) is perfect by default.
- **Main Flow**:
  1. Call `drive2_api_enable_encoder_sim_model(h, slip_l, slip_r, scale_err_l, scale_err_r)`
     to configure per-wheel slip fraction and scale error on the left and right
     reported-encoder accumulators.
  2. Tick the drive loop; `PhysicsWorld::update()` applies the error on each
     tick's delta before adding it to `_reportedEncLMm` / `_reportedEncRMm`.
  3. `SimMotor::positionMm()` returns the errored reported value; the EKF
     encoder-predict path receives a biased position signal.
- **Postconditions**: The encoder dead-reckoning pose (`state().encoder`) diverges
  from plant ground truth by an amount proportional to the injected slip/scale.
- **Acceptance Criteria**:
  - [ ] `drive2_api_enable_encoder_sim_model` exists in `drive2_api.cpp` with the
        correct C-ABI signature.
  - [ ] After calling the shim and ticking 50+ times with a forward twist, the
        encoder-only pose error exceeds a meaningful threshold (e.g., > 5 mm).
  - [ ] Default (no shim call) behaviour is unchanged: encoder reads ground truth.

## SUC-002: Assert EKF fused pose beats both noisy raw sources

- **Actor**: Test author / CI system
- **Preconditions**: Both `drive2_api_enable_encoder_sim_model` and
  `drive2_api_enable_otos_sim_model` have been called with non-trivial error.
  A `Drive2Handle` has been created.
- **Main Flow**:
  1. Enable encoder error (e.g., 5% scale error, 3% slip on both wheels).
  2. Enable OTOS error (e.g., 3 mm/tick drift, 5 mm noise sigma, 3% scale error).
  3. Apply a forward twist command and tick 50–100 times.
  4. Read `ground_truth_x`, `fused_x`, `encoder_x`, `optical_x`.
  5. Compute `fused_err`, `encoder_only_err`, `optical_only_err` vs. ground truth.
  6. Assert `fused_err < encoder_only_err` AND `fused_err < optical_only_err`.
- **Postconditions**: The test passes, demonstrating the EKF fuses two imperfect
  sources into an estimate better than either alone. If the assertion fails, it
  surfaces a real gap in EKF tuning — the finding is reported, not suppressed.
- **Acceptance Criteria**:
  - [ ] Test exists in `tests/simulation/unit/` (new file or extension of
        `test_drive2_subsystem.py`).
  - [ ] Injection magnitudes are non-trivial: both `encoder_only_err` and
        `optical_only_err` exceed a lower bound (e.g., > 5 mm) so the assertion
        is not vacuously true.
  - [ ] `fused_err` is strictly less than both raw errors.
  - [ ] A second scenario (encoder good, OTOS bad) mirrors the 057-005 behaviour
        and continues to pass.
