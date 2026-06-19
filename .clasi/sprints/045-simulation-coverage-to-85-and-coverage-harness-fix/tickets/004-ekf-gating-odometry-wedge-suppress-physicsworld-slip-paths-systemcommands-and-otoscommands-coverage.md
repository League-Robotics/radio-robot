---
id: '004'
title: EKF gating, Odometry wedge-suppress, PhysicsWorld slip paths, SystemCommands
  and OtosCommands coverage
status: in-progress
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

- [ ] New file `tests/simulation/unit/test_ekf_odometry_commands_coverage.py` created.
- [ ] EKF gate-reject path: inject an OTOS reading far from the current pose (e.g., 500mm teleport while robot is at origin with tight P); assert that `sim_get_ekf_rej_count` increases.
- [ ] EKF P-inflation recovery: inject 10+ consecutive rejected OTOS readings; assert EKF eventually re-acquires (estimate converges back after P inflation).
- [ ] Odometry wedge-suppress: run a motion command, trigger wedge on one wheel (via T002's technique), then confirm `predict()` is called with wedge flag set. Proxy: the pose heading does not drift in the wedged direction (observable via `sim_get_pose_h`).
- [ ] PhysicsWorld slip: call `sim_set_motor_slip` with non-trivial values (`straight=0.2, turn_extra=0.1`) and run a straight-line drive; confirm encoder/pose shows expected slip behavior (actual travel < commanded travel).
- [ ] `SNAP` command: `sim.send_command("SNAP")` produces a telemetry-format reply or OK.
- [ ] `ZERO` command: `sim.send_command("ZERO")` resets encoder accumulators; `sim_get_enc_l` returns 0 (or near 0) after ZERO.
- [ ] `HALT` command: `sim.send_command("HALT TIME ms=500")` registers a halt condition; motion stops after 500ms.
- [ ] `VER` command: produces a version string reply.
- [ ] `ECHO hello` command: produces `hello` in the reply.
- [ ] `HELP` command: produces a multi-line reply without crashing.
- [ ] OtosCommands: `sim.send_command("OTOS GET")` or equivalent produces a reply; error branches produce ERR.
- [ ] All existing tests still pass.
- [ ] Golden-TLM, field-pin, vendor grep gates all green.

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
