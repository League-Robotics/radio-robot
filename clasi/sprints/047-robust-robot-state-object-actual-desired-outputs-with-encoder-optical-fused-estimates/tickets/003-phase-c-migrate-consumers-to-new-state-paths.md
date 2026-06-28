---
id: '003'
title: "Phase C — Migrate consumers to new state paths"
status: open
use-cases: [SUC-047-002, SUC-047-003, SUC-047-006]
depends-on: ['002']
github-issue: ''
issue: robot-state-object-proposed-structure-for-review.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Phase C — Migrate consumers to new state paths

## Description

Migrate all consumer files off the legacy shim names to direct new-path
accesses. Wire `BodyVelocityController::setStateRef()` and call it from
`Robot`. After this ticket, no consumer reads `state.inputs.*` or
`state.commands.*` or `state.target.*` directly — all use `state.actual.*`,
`state.desired.*`, or `state.outputs.*`.

The legacy mirror-writes in `Odometry` (from Phase B) remain active until
Phase D. This ticket removes the consumer use of shim names; it does NOT
yet drop the shim definitions or mirror-writes.

## Files to Modify

**Control / Robot layer:**
- `source/control/BodyVelocityController.h` — add `void setStateRef(DesiredState* ds)`; add private `DesiredState* _ds = nullptr`; add write-back call at end of `advance()`.
- `source/control/BodyVelocityController.cpp` — implement `setStateRef()`; at end of `advance()`: `if (_ds) { _ds->bodyTwist = {_v, _vy, _omega}; _ds->bodyTwistRaw = {_vTgt, _vyTgt, _omegaTgt}; }` (differential build: `_vy`/`_vyTgt` are 0 via always-present `BodyTwist3.vy_mmps` field).
- `source/robot/Robot.h` / `source/robot/Robot.cpp` — call `bvc.setStateRef(&state.desired)` in constructor or `init()`. Migrate all `state.inputs.*` / `state.commands.*` / `state.target.*` references to direct new paths.

**Application layer:**
- `source/app/commands/MotionCommand.cpp` — migrate `state.target.*` reads/writes to `state.desired.*`.
- `source/app/commands/MotionCommandHandlers.cpp` — migrate to `state.desired.*` for mode/target/reply fields.
- `source/app/commands/SystemCommands.cpp` — migrate `state.inputs.encLMm/encRMm` to `state.actual.encMm[1/0]`; `estimate.zero(state.inputs)` to `estimate.zero(state.actual)`; `estimate.resetPose(state.inputs,...)` to `estimate.resetPose(state.actual,...)`.
- `source/app/commands/ConfigCommands.cpp` — migrate any `state.inputs.*` reads.
- `source/app/commands/DebugCommandable.cpp` — migrate all state field accesses.
- `source/app/LoopTickOnce.cpp` — migrate all field accesses.
- `source/robot/RobotTelemetry.cpp` — migrate to `state.actual.fused.*`, `state.actual.enc.*`, `state.desired.*`, `state.outputs.*`.

**Control layer:**
- `source/control/StopCondition.cpp` — migrate `inputs.poseX/Y` to `actual.fused.pose.x/y`; `inputs.line[]` to `actual.line[]`; `inputs.colorR/G/B/C` to `actual.colorR/G/B/C`; `inputs.digitalIn[]/analogIn[]` to `actual.digitalIn[]/analogIn[]`.
- `source/superstructure/MotionController.cpp` — migrate `inputs.encLMm/R` to `actual.encMm[1/0]`.
- `source/control/MotorController.cpp` — migrate `_cmds->tgtLMms/tgtRMms` to pointer/ref into `desired.wheelMms[1/0]`; `_cmds->pwmL/pwmR` to `outputs.pwm[1/0]`; dirty flags to `outputs.digitalDirty`/`outputs.analogDirty`.
- `source/control/MotionControllerBegin.cpp` — migrate any `state.inputs.*` / `state.target.*` accesses.

**Subsystems:**
- `source/subsystems/drive/Drive.cpp` — migrate `_commands.tgtLMms/tgtRMms` to `desired.wheelMms[1/0]`; `_inputs.encLMm/encRMm` to `actual.encMm[1/0]`.

**HAL:**
- `source/io/real/NezhaHAL.cpp` — migrate any `MotorCommands`-typed references.
- `source/io/real/MecanumHAL.cpp` — same.

**Sim infrastructure:**
- `source/io/sim/SimHardware.cpp` — migrate any remaining `HardwareState`/`MotorCommands` field accesses.
- `source/io/sim/SimMotor.cpp` — same.
- `source/app/WedgeTest.cpp` — migrate `encLMm/encRMm` to `actual.encMm[1/0]`.

## Acceptance Criteria

- [ ] `BodyVelocityController` has `setStateRef(DesiredState*)` and writes `desired.bodyTwist` / `desired.bodyTwistRaw` at the end of each `advance()` call.
- [ ] `Robot` calls `bvc.setStateRef(&state.desired)` during init/construction.
- [ ] No consumer file reads `state.inputs.*`, `state.commands.*`, or `state.target.*` directly.
- [ ] `desired.bodyTwist.vx_mmps` equals BVC `currentV()` after each `advance()` tick.
- [ ] `desired.bodyTwistRaw` equals BVC `targetV()`/`targetOmega()` targets.
- [ ] BVC `currentV()`, `currentOmega()`, `currentVy()`, `targetV()`, `targetOmega()` accessors still compile and return correct values (back-compat).
- [ ] **Differential build compiles clean** (`python build.py --clean`): zero errors.
- [ ] **Mecanum build compiles clean**: zero errors.
- [ ] **Sim unit suite green**: `uv run --with pytest python -m pytest tests/simulation/ -q` — no Python test edits required.

## Implementation Plan

1. Add `setStateRef()` to BVC; implement write-back in `advance()`. Wire in `Robot`.
2. Migrate `Robot.cpp` field accesses (largest file — ~35 references to old fields).
3. Migrate `MotionCommand.cpp` and `MotionCommandHandlers.cpp`. Run sim suite.
4. Migrate `SystemCommands.cpp`.
5. Migrate `StopCondition.cpp`.
6. Migrate `MotorController.cpp` and `Drive.cpp`.
7. Migrate `MotionController.cpp` and `MotionControllerBegin.cpp`.
8. Migrate `RobotTelemetry.cpp`. Run sim suite.
9. Migrate `DebugCommandable.cpp`, `ConfigCommands.cpp`, `LoopTickOnce.cpp`.
10. Migrate `WedgeTest.cpp`, `NezhaHAL.cpp`, `MecanumHAL.cpp`, `SimHardware.cpp`, `SimMotor.cpp`.
11. Build both variants; run sim suite.

## Testing Plan

- **Sim suite**: `uv run --with pytest python -m pytest tests/simulation/ -q` — run after every 2-3 file migrations to catch regressions early.
- **Build test**: `python build.py --clean` after all migrations.
- **No new tests required**: behavioral correctness validated by existing sim suite; BVC write-back verified in the fusion-validation test (ticket 005).

## Documentation Updates

Architecture update sections D (BVC) and E (consumer table) cover this ticket.
