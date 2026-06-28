---
id: '047'
title: 'Robust robot state object: actual/desired/outputs with encoder/optical/fused
  estimates'
status: planning-docs
branch: sprint/047-robust-robot-state-object-actual-desired-outputs-with-encoder-optical-fused-estimates
use-cases: [SUC-047-001, SUC-047-002, SUC-047-003, SUC-047-004, SUC-047-005, SUC-047-006]
issues:
- robot-state-object-proposed-structure-for-review.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 047: Robust robot state object: actual/desired/outputs with encoder/optical/fused estimates

## Goals

Restructure `RobotStateContainer` from the current flat `{commands, inputs, target}` layout
into a clean three-group model: `{actual, desired, outputs}`. Crucially, retain the
**encoder-only dead-reckoned pose** separately from the **OTOS optical estimate** and the
**EKF-fused estimate** so all three can be dumped side by side to validate whether fusion
is working.

## Problem

The current `HardwareState` is a single flat blob that:
- Conflates the EKF-fused pose with the pre-fusion encoder pose (encoder pose is discarded).
- Mixes measured state (`HardwareState`) with commanded state (`MotorCommands`) and goal state (`TargetState`).
- Makes it impossible to compare encoder vs optical vs fused estimates at runtime.
- Uses misleading field names (`poseX` is actually the EKF output, not raw dead-reckoning).

## Solution

1. **Phase A** (ticket 001): Add five new POD headers (`PoseEstimate`, `ActualState`, `DesiredState`,
   `OutputState`, `EstimateDump`) and restructure `RobotStateContainer`. Provide inline
   shim functions for all legacy field names — no behavior change, both builds compile.

2. **Phase B** (ticket 002): Add a private encoder-only accumulator to `Odometry`; `predict()` dual-writes
   encoder and fused estimates; `correctEKF()` captures raw OTOS into `actual.optical` before
   fusing. Update `sim_api.cpp` ABI bodies and add new `sim_get_enc_pose_*` /
   `sim_get_otos_pose_*` ABI functions.

3. **Phase C** (ticket 003): Migrate all consumers off shim names to direct new-path accesses. Wire
   `BodyVelocityController::setStateRef()` to publish the profiled body twist into
   `desired.bodyTwist` each `advance()`.

4. **Phase D** (ticket 004): Drop shims and mirror-writes; implement `dumpEstimates()` and add `DBG EST`
   telemetry command emitting all three `EST enc/otos/fuse` lines.

5. **Ticket 005**: New fusion-validation pytest proves encoder estimate is never overwritten
   by EKF fusion.

## Success Criteria

- `RobotStateContainer` has exactly three fields: `actual`, `desired`, `outputs`.
- All three estimates (encoder, optical, fused) are retained side by side and independently readable.
- `EST enc/otos/fuse` telemetry lines appear with pose, twist, age, and validity.
- Both differential and mecanum builds compile clean with no `#ifdef` inside the new struct bodies.
- Sim unit suite passes green with no Python test edits.
- New fusion-validation test proves encoder and fused diverge under injected OTOS offset.

## Scope

### In Scope

- New `source/state/` headers: `PoseEstimate.h`, `ActualState.h`, `DesiredState.h`,
  `OutputState.h`, `EstimateDump.h`, `StateShims.h` (temporary, dropped in ticket 004).
- `source/types/Inputs.h` restructure; removal of `HardwareState`, `MotorCommands`,
  `TargetState` by end of sprint.
- `Odometry.cpp`/`PhysicalStateEstimate.cpp` encoder accumulator and dual-write.
- `BodyVelocityController` `setStateRef()` and `desired.bodyTwist` publish.
- Consumer migration: all `source/` files using old state paths.
- `sim_api.cpp` ABI body re-pointing and new `sim_get_enc_pose_*` / `sim_get_otos_pose_*` ABI.
- `WorldView` constructor type update (`HardwareState&` → `ActualState&`).
- `Hardware::tick(uint32_t, const OutputState&)` virtual signature update.
- New pytest `test_fusion_validation.py`.
- `DBG EST` command and `dumpEstimates()` implementation.

### Out of Scope

- cmon-pid / TinyEKF consolidation (separate issue; this sprint is interface-compatible with it).
- 6-state holonomic EKF extension (deferred per DR-2 from sprint 046).
- Distance-bounded lateral mecanum commands.
- Architecture consolidation document (done after sprint close).

## Test Strategy

- `python build.py --clean` for both differential and mecanum builds at every ticket boundary.
- `uv run --with pytest python -m pytest tests/simulation/ -q` after every ticket — must remain green throughout.
- New `tests/simulation/unit/test_fusion_validation.py` added in ticket 005.
- Manual `DBG EST` command check after ticket 004.

## Architecture Notes

- `BodyTwist3` used uniformly; `vy` always present (0 on differential). No `#ifdef` inside struct bodies.
- Array sizing via `Kinematics::kWheelCount` (2 differential, 4 mecanum).
- Inline shim functions (not reference members) for legacy names — reference members break `= {}` aggregate init.
- `Odometry` private encoder accumulator is never touched by EKF; `setPose()`/`zero()` reset it.
- BVC remains the single source of truth for profiler dynamics; publishes a read-model copy into `desired.bodyTwist` each `advance()`.
- Migration is incremental (A→B→C→D): each phase is independently compilable and testable.

## GitHub Issues

(None opened for this sprint.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed (APPROVE WITH CHANGES — 2026-06-28)
- [x] Stakeholder has approved the sprint plan (Q1–Q5 resolved 2026-06-27)

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Phase A — New state type headers and RobotStateContainer restructure | — |
| 002 | Phase B — Odometry encoder accumulator and three-estimate wiring | 001 |
| 003 | Phase C — Migrate consumers to new state paths | 002 |
| 004 | Phase D — Drop shims and mirror-writes, add EST dump telemetry | 003 |
| 005 | Fusion-validation test: assert encoder and fused estimates diverge under OTOS offset | 004 |

Tickets execute serially in the order listed.
