---
id: '004'
title: EKF gating, Odometry wedge-suppress, PhysicsWorld slip paths, SystemCommands
  and OtosCommands coverage
status: done
use-cases:
- SUC-005
depends-on:
- '001'
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 045-004: EKF gating, Odometry wedge-suppress, PhysicsWorld slip paths, SystemCommands and OtosCommands coverage

## Description

**EKF (56 uncovered / 80%):**
`source/state/EKF.cpp` correction/gating branches are partly uncovered. The existing
`test_ekf.py` is a pure-Python mirror (zero C++ binary coverage). Specific uncovered
paths include:
- `update_position` Mahalanobis gating reject branch: inject an OTOS reading far
  enough outside the current estimate to trigger the gate rejection (d² > 5.99).
- `update_velocity` gating branches.
- `update_heading` reject branch.
- P-inflation re-baseline recovery triggered after 10 consecutive gate rejections.

**Odometry (23 uncovered / 75%):**
`source/control/Odometry.cpp` uncovered paths include the wedge-suppression predict
branch: when `wheelWedgedL` or `wheelWedgedR` is true, `predict()` suppresses the
phantom dTheta contribution from the stalled wheel. This path requires setting up
the `wheelWedgedL/R` flags in the Robot state, which happens after `EVT enc_wedged`
fires in `MotorController`.

**PhysicsWorld (20 uncovered / 56%):**
`source/io/sim/PhysicsWorld.cpp` slip/dynamics-error paths. `setSlip` exists
(`sim_set_motor_slip` in sim_api.cpp). Uncovered: the `effectiveSlip()` branches
for non-trivial slip factor, the turn-extra component, and possibly error-handling
in the dynamics step when velocities diverge.

**SystemCommands (252 uncovered / 56%):**
Most uncovered lines are inside `#ifndef HOST_BUILD` blocks (RESET, hardware-query
commands). The simulatable subset includes:
- `SNAP` — snapshot telemetry; sends TLM frame.
- `ZERO` — zero encoder accumulators.
- `HALT` — register a halt condition.
- `GET VEL` — get current velocity.
- `GET <key>` and `SET <key>=<value>` — config access (many already covered by
  `test_config_set.py` and `test_config_registry.py`; programmer checks which ones
  remain uncovered).
- `HELP`, `VER`, `ECHO` — simple reply commands.
- `SAFE`, `SI` — safe mode and sensor interval commands.
- The `+` command (alias).
- Error branches: malformed GET/SET args.

**OtosCommands (18 uncovered / 86%):**
`source/app/OtosCommands.cpp` near-covered; small number of uncovered error/edge
branches (likely malformed OTOS command args or out-of-range values).

## Acceptance Criteria

- [x] New file `tests/simulation/system/test_ekf_odometry_commands_coverage.py` created (system tier — estimator + sensor-injection scenarios).
- [x] EKF gate-reject path: `test_ekf_gate_rejects_far_otos` injects a far OTOS teleport and asserts `sim_get_ekf_rej_count` increases while the estimate barely moves (updatePosition/updateHeading reject branches).
- [x] EKF P-inflation recovery: `test_ekf_p_inflation_recovers_after_10_rejections` injects 15 consecutive rejections (crossing the 10-streak), asserting both position and heading P-inflation re-baseline fire (estimate snaps to the held reading). `test_ekf_recovers_to_truth_after_bad_burst` confirms reconvergence to truth.
- [x] Odometry wedge-suppress: `test_odometry_wedge_suppresses_heading_drift` freezes R mid-drive (T002 technique), waits for `get_odometry_wedge_active()`, and asserts heading holds while the left wheel advances — the `if (_wedgeActive) dTheta=0` branch in Odometry::predict.
- [x] PhysicsWorld slip: `test_physics_slip_reduces_reported_travel` sets `sim_set_motor_slip(straight=0.3)` and confirms reported travel < true travel; `test_physics_encoder_noise_path` exercises the HOST_BUILD pwGaussianNoise draw via `sim_set_encoder_noise`.
- [x] `SNAP`: `test_snap_emits_tlm_frame` asserts a TLM frame.
- [x] `ZERO`: `test_zero_enc_resets_accumulators` (enc near 0 after ZERO enc) + `test_zero_pose_resets_pose`.
- [~] `HALT`: HALT TIME/DIST/POS/COLOR/LINE firing is already covered by `test_halt_controller.py` and (COLOR/LINE evaluate) by 045-003's `test_stop_condition_coverage.py`; not duplicated here.
- [x] `VER`: `test_ver_reports_version` (OK + version digits).
- [x] `ECHO hello`: `test_echo_returns_tokens`.
- [x] `HELP`: `test_help_multiline_no_crash`.
- [x] Also covered: `SI` (→ Odometry/EKF::setPose), `SAFE`, the `+` quiet keepalive, and GET/SET (round-trip + unknown-key ERR).
- [x] OtosCommands: `test_otos_verbs_nodev_when_not_initialized` (all O-verb `nodev` ERR branches), `test_otos_verbs_ok_when_initialized` (OI/OZ/OR/OV/OL/OA OK branches after begin()), `test_otos_op_reports_pose_no_init_required` (OP), `test_otos_ov_bad_args_errors` (parseOV badarg).
- [x] All existing tests still pass: 2076 passed (was 2055; +21).
- [x] Golden-TLM, field-pin, vendor grep gates all green.

## Implementation Plan

### Approach

**EKF gate-reject:**
`sim_get_ekf_rej_count` already exists in `sim_api.cpp` (confirmed). Pattern:
1. Start sim, let it settle (OTOS fused).
2. Teleport OTOS reading far from current pose via `sim_set_otos_pose(x=5000, y=5000)`.
3. Tick a few times; check `sim_get_ekf_rej_count` > 0.

For P-inflation recovery: after 10+ consecutive rejections, reset OTOS to true pose
via `sim_set_otos_pose(true_x, true_y)`; tick until estimate converges (EKF rej
count stops growing). Use `sim_get_estimation_error_xy` to confirm convergence.

**Odometry wedge-suppress:**
Combine with T002's wedge technique: after `EVT enc_wedged` fires, check that
pose heading is stable (not drifting in the direction a phantom wheel rotation
would induce). This exercises the `wheelWedgedL` flag path in `Odometry::predict`.

**PhysicsWorld slip:**
```python
sim._lib.sim_set_motor_slip(sim._h, ctypes.c_int(2), ctypes.c_float(0.3), ctypes.c_float(0.0))
# side=2 = both wheels
sim.send_command("D dist=500")
sim.tick_for(3000)
enc_l = sim._lib.sim_get_true_enc_l(sim._h)
assert enc_l < 450  # slip means actual travel < commanded
```

**SystemCommands:**
Simple send_command calls for each uncovered non-CODAL command. Check which are
already covered by existing tests before writing new ones.

**OtosCommands:**
Read `source/app/OtosCommands.cpp` to find uncovered branches; write minimal tests
that trigger each error/edge case.

### Files to create

- `tests/simulation/unit/test_ekf_odometry_commands_coverage.py`

### Files to read (for implementation)

- `source/state/EKF.cpp` and `EKF.h` — gating thresholds, rejection counter
- `source/control/Odometry.cpp` — wedge-suppression predict path
- `source/io/sim/PhysicsWorld.cpp` — slip dynamics implementation
- `source/app/SystemCommands.cpp` — which paths are `#ifndef HOST_BUILD`
- `source/app/OtosCommands.cpp` — uncovered branches
- `tests/_infra/sim/sim_api.cpp` — `sim_get_ekf_rej_count`, `sim_get_estimation_error_xy`, `sim_set_motor_slip`

### Testing plan

- Run full simulation tier after each test group.
- For convergence tests: use tick_for(N) with generous N to allow EKF to converge.
- Do not hardcode tight numerical tolerances — use relative bounds (e.g., error < 50mm).

### Documentation updates

- None required; existing architecture documents the EKF gating and wedge-suppress paths.
