---
id: '002'
title: "Phase B \u2014 Odometry encoder accumulator and three-estimate wiring"
status: done
use-cases:
- SUC-047-001
- SUC-047-002
- SUC-047-003
depends-on:
- '001'
github-issue: ''
issue: robot-state-object-proposed-structure-for-review.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Phase B — Odometry encoder accumulator and three-estimate wiring

## Description

Wire the behavioral change that retains three pose estimates side by side.
`Odometry` gains a private encoder-only dead-reckoning accumulator
(`_encPoseX/Y/H`, `_encVx/Vy/Omega`) that fusion never touches. `predict()`
dual-writes both the encoder estimate and the EKF-fused estimate into
`ActualState`. `correctEKF()` captures raw OTOS into `actual.optical` before
fusing. `setPose()`/`zero()` reset the encoder accumulator too.

`PhysicalStateEstimate` is updated to forward `ActualState&` and gains three
new read-only forwarders. `sim_api.cpp` C-ABI function bodies are re-pointed
to the new struct paths so the existing simulation suite stays green.

Legacy mirror-writes (e.g. writing `s.poseX = actual.fused.pose.x` after the
EKF update) are kept during this phase so all consumers that have not yet
migrated in Phase C continue to read correct values through the shims.

## Files to Modify

- `source/control/Odometry.h` — add private `_encPoseX`, `_encPoseY`, `_encPoseH`, `_encVx`, `_encVy`, `_encOmega` fields; change `predict()`, `correctEKF()`, `setPose()`, `zero()` signatures to take `ActualState&` instead of `HardwareState&`.
- `source/control/Odometry.cpp` — implement dual-write in `predict()`: arc-integrate into `_encPose*`, write `actual.encoder.{pose,twist,stamp}`, then run EKF and write `actual.fused.{pose,twist,stamp}` plus legacy mirror (`s.poseX = actual.fused.pose.x`, `s.fusedV = actual.fused.twist.vx_mmps`). In `correctEKF()`: before EKF update, persist raw OTOS into `actual.optical.{pose,twist,stamp}`; after EKF update, write `actual.fused.*` plus legacy mirrors. In `setPose()`: reset `_encPoseX/Y/H` and `_encVx/Vy/Omega`. In `zero()`: same via `setPose`.
- `source/state/PhysicalStateEstimate.h` — change `addOdometryObservation()` and `addOtosObservation()` signatures to take `ActualState&`; add `encoderEstimate()`, `opticalEstimate()`, `fusedEstimate()` const forwarders.
- `source/state/PhysicalStateEstimate.cpp` — implement updated method bodies forwarding to `_odometry`.
- `tests/_infra/sim/sim_api.cpp` — re-point all existing C-ABI body reads/writes to the new struct paths:
  - `sim_get_pose_x/y/h` → `robot.state.actual.fused.pose.x/y/h`
  - `sim_get_enc_l/r` → `robot.state.actual.encMm[1/0]`
  - `sim_get_vel_l/r` → `robot.state.actual.velMms[1/0]`
  - `sim_get_pwm_l/r` → `robot.state.actual.outputs.pwm[1/0]`  (NOTE: `outputs` is a top-level field of `RobotStateContainer`, not nested under `actual` — correct path is `robot.state.outputs.pwm[1/0]`)
  - `sim_get_fused_v` → `robot.state.actual.fused.twist.vx_mmps`
  - `sim_get_fused_omega` → `robot.state.actual.fused.twist.omega_rads`
  - `sim_set_pose` → write `robot.state.actual.fused.pose.x/y/h`
  - `sim_get_enc_l/r` injection sites → `robot.state.actual.encMm[1/0]`
  - `WorldView` constructor binding: change from `robot.state.inputs` to pass through `robot.state.actual` (update `WorldView` constructor to take `const ActualState&`; update `WorldView.h`, `WorldView.cpp`, and the `SimHandle` initializer list).
  - Add new ABI functions:
    ```c
    float sim_get_enc_pose_x(void* h);
    float sim_get_enc_pose_y(void* h);
    float sim_get_enc_pose_h(void* h);
    float sim_get_otos_pose_x(void* h);
    float sim_get_otos_pose_y(void* h);
    float sim_get_otos_pose_h(void* h);
    float sim_get_fused_pose_x(void* h);
    float sim_get_fused_pose_y(void* h);
    float sim_get_fused_pose_h(void* h);
    ```
- `source/io/sim/WorldView.h` — change constructor parameter from `const HardwareState& inputs` to `const ActualState& actual`; update `estimationErrorXY()` / `estimationErrorH()` to read `_actual.fused.pose.x/y/h`.
- `source/io/sim/WorldView.cpp` — update implementation to read from `_actual.fused.pose`.

## Key Implementation Notes

- **Mirror-write ordering**: in `predict()`, write `_encPose*` first, then run EKF, then write `actual.fused.*`, then write legacy mirrors (`s.poseX`, `s.fusedV`, etc.). Do NOT remove legacy mirrors yet — Phase C consumers still use them via shims.
- **`correctEKF()` now_ms**: the OTOS observation timestamp must be captured before the EKF update to populate `actual.optical.stamp.lastUpdMs`. The current `correctEKF()` does not take `now_ms`; either add it as a parameter or read it from `_lastPredictMs` (which is updated in `predict()`). Simplest: add `uint32_t now_ms` parameter to `correctEKF()` — check all call sites (Robot.cpp `estimate.addOtosObservation()`).
- **Encoder accumulator reset in `setPose()`**: must also rebaseline `_prevEncL/_prevEncR` (already done today); add reset of `_encPoseX/Y/H` to the same value as the new pose, and zero `_encVx/Vy/Omega`.
- **mecanum `_fusedVy`**: continues to write `actual.fused.twist.vy_mmps` in `correctEKF()` under `#ifdef ROBOT_DRIVETRAIN_MECANUM`. Differential build writes 0 (field always exists in `BodyTwist3`).
- **`hal.tick(now_ms, state.commands)` in sim_api.cpp**: `state.commands` no longer exists after Phase A restructure — this call must be updated to `hal.tick(now_ms, state.outputs)` in Phase B (the `MotorCommands`-typed parameter in `SimHardware::tick()` must also update to `OutputState` or be adapter-shimmed). Check `source/io/sim/SimHardware.h` and `SimHardware.cpp` for the `tick(uint32_t, const MotorCommands&)` overload — update to `tick(uint32_t, const OutputState&)` or provide an adapter.

## Acceptance Criteria

- [x] `Odometry` has private `_encPoseX`, `_encPoseY`, `_encPoseH`, `_encVx`, `_encVy`, `_encOmega` fields, initialized to 0 in the constructor.
- [x] `predict()` writes `actual.encoder.pose`, `actual.encoder.twist`, `actual.encoder.stamp` (never touched by EKF).
- [x] `predict()` writes `actual.fused.pose`, `actual.fused.twist`, `actual.fused.stamp` (from EKF output).
- [x] `correctEKF()` writes `actual.optical.pose`, `actual.optical.twist`, `actual.optical.stamp` before fusing.
- [x] `correctEKF()` writes `actual.fused.*` after EKF update.
- [x] `setPose()`/`zero()` reset `_encPoseX/Y/H` and `_encVx/Vy/Omega`.
- [x] Legacy mirror-writes remain active (e.g. `s.poseX = actual.fused.pose.x`) — no consumer outside this ticket's scope should break.
- [x] `PhysicalStateEstimate::encoderEstimate()`, `opticalEstimate()`, `fusedEstimate()` return const refs to the correct `PoseEstimate` members.
- [x] `WorldView` constructor updated to `const ActualState&`; `estimationErrorXY/H()` read `actual.fused.pose`.
- [x] All existing `sim_api.cpp` ABI function bodies re-pointed to new struct paths.
- [x] New `sim_get_enc_pose_*`, `sim_get_otos_pose_*`, `sim_get_fused_pose_*` ABI functions added.
- [x] **Differential build compiles clean** (`python build.py --clean`).
- [x] **Mecanum build compiles clean**.
- [x] **Sim unit suite green**: `uv run --with pytest python -m pytest tests/simulation/ -q` — no Python test edits required.

## Implementation Plan

1. Update `Odometry.h`: add six private float fields; update method signatures.
2. Update `Odometry.cpp`: implement `predict()` dual-write, `correctEKF()` optical capture, `setPose()`/`zero()` accumulator reset.
3. Update `PhysicalStateEstimate.h`/`.cpp`: updated forwarding methods + three new forwarder accessors.
4. Update `WorldView.h`/`.cpp`: change constructor to `const ActualState&`.
5. Update `SimHardware.h`/`.cpp`: update `tick(uint32_t, const MotorCommands&)` to `tick(uint32_t, const OutputState&)` (or provide an `OutputState`-typed overload and alias).
6. Update `sim_api.cpp`: re-point all ABI bodies; update `_worldView` constructor call; add new ABI functions.
7. Build both variants; run sim suite.

## Testing Plan

- **Sim suite**: `uv run --with pytest python -m pytest tests/simulation/ -q` — must pass with no Python test edits.
- **Build test**: `python build.py --clean` (differential); mecanum if available.
- **Manual inspection**: after building sim, call `sim_get_enc_pose_x(h)` and `sim_get_fused_pose_x(h)` from a quick Python snippet to confirm they differ after injecting an OTOS offset (preview of the fusion-validation test in ticket 005).

## Documentation Updates

Architecture update section B covers this ticket. No additional docs.
