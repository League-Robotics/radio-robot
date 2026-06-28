---
id: '001'
title: "Phase A \u2014 New state type headers and RobotStateContainer restructure"
status: done
use-cases:
- SUC-047-002
- SUC-047-003
depends-on: []
github-issue: ''
issue: robot-state-object-proposed-structure-for-review.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Phase A — New state type headers and RobotStateContainer restructure

## Description

Add the five new POD state-type headers and restructure `RobotStateContainer`
from `{commands, inputs, target}` to `{actual, desired, outputs}`. Provide
inline shim functions so all existing field names continue to resolve — this
phase has **no behavior change**. Both differential and mecanum builds must
compile cleanly with zero changes to any existing consumer.

This is the foundation for Phases B–D. Nothing outside of `source/types/Inputs.h`
and the new `source/state/` headers changes in this ticket.

## Files to Create

- `source/state/PoseEstimate.h` — POD `struct PoseEstimate { Pose2D pose; BodyTwist3 twist; ValueSet stamp; }`
- `source/state/ActualState.h` — POD `struct ActualState` with three `PoseEstimate` members, `encMm[kWheelCount]`, `velMms[kWheelCount]`, raw sensors
- `source/state/DesiredState.h` — POD `struct DesiredState` absorbing all `TargetState` fields + BVC-published body twist + `wheelMms[kWheelCount]` + port outputs
- `source/state/OutputState.h` — POD `struct OutputState { int16_t pwm[kWheelCount]; bool digitalDirty[4]; bool analogDirty[4]; }`
- `source/state/EstimateDump.h` — `struct EstimateDump` + `dumpEstimates()` function declaration (implementation may be inline or in a companion `.cpp`)
- `source/state/StateShims.h` — inline free functions returning references to old-name fields via new paths

## Files to Modify

- `source/types/Inputs.h` — replace `RobotStateContainer { MotorCommands commands; HardwareState inputs; TargetState target; }` with `{ ActualState actual; DesiredState desired; OutputState outputs; }`. Update `defaultInputs()` to seed `ValueSet::lagMs` via the new paths (`actual.enc.lagMs`, `actual.lineVS.lagMs`, etc.). Keep `HardwareState`, `MotorCommands`, and `TargetState` struct definitions in place (they are still compiled; shims reference them if needed, and consumers still include this header).

## Key Implementation Notes

- Array sizing: use `Kinematics::kWheelCount` (requires `#include "kinematics/IKinematics.h"`). No `#ifdef` inside any struct body.
- `BodyTwist3` used uniformly in `PoseEstimate` and `DesiredState`; `vy` is always present (0 on differential).
- All shims are inline free functions, NOT reference members (reference members break `= {}` aggregate init — existing `Inputs.h` comment documents this).
- Shim examples:
  ```cpp
  inline float& poseX(RobotStateContainer& s)    { return s.actual.fused.pose.x; }
  inline float& encLMm(RobotStateContainer& s)   { return s.actual.encMm[1]; }   // FL=index 1
  inline int16_t& pwmL(RobotStateContainer& s)   { return s.outputs.pwm[1]; }    // FL=index 1
  inline float& tgtLMms(RobotStateContainer& s)  { return s.desired.wheelMms[1]; }
  inline DriveMode& mode(RobotStateContainer& s) { return s.desired.mode; }
  ```
- `WorldView` constructor takes `const HardwareState&` today; during Phase A it still binds `robot.state.inputs` — leave this unchanged. Phase D migrates it.
- `hal.tick(now_ms, state.commands)` call sites: still compile because `MotorCommands` struct still exists. Phase D updates these.
- `EstimateDump.h` may declare `dumpEstimates()` as inline or forward-declare it with an implementation in `source/state/EstimateDump.cpp` — choose whichever fits the no-heap embedded constraint. The function body only touches `const ActualState&` inputs and a fixed-size `EstimateDump[3]` output array.

## Acceptance Criteria

- [x] `source/state/PoseEstimate.h`, `ActualState.h`, `DesiredState.h`, `OutputState.h`, `EstimateDump.h`, `StateShims.h` all exist and are syntactically correct C++.
- [x] `RobotStateContainer` has exactly three top-level fields: `actual`, `desired`, `outputs`.
- [x] `defaultInputs()` correctly seeds `lagMs` in `actual.otos`, `actual.lineVS`, `actual.colorVS`, `actual.portsVS` (enc lag is 0 — synchronous, per inline comment).
- [x] No `#ifdef ROBOT_DRIVETRAIN_MECANUM` inside any of the new struct bodies.
- [x] **Differential build compiles clean** (`python build.py --clean`): zero errors, zero new warnings.
- [x] **Mecanum build compiles clean**: zero errors, zero new warnings.
- [x] **Sim unit suite green**: `uv run --with pytest python -m pytest tests/simulation/ -q` — 2228 passed, 2 pre-existing failures (unrelated schema validation issue). `test_estimate_dependency_rule` now passes with ALLOWED_HEADERS updated.
- [x] All existing field names (`poseX`, `encLMm`, `pwmL`, `tgtLMms`, `mode`, `replyFn`, etc.) resolve via shim functions without any consumer file being edited.

## Implementation Plan

1. Create `source/state/` directory structure (directory already exists with `EKF.h` etc.).
2. Write `PoseEstimate.h` — include `"io/capability/Pose2D.h"` for `BodyTwist3`, include `"types/Inputs.h"` for `ValueSet` (or forward-declare and use a separate include ordering).
3. Write `ActualState.h` — include `PoseEstimate.h` and `"kinematics/IKinematics.h"` for `kWheelCount`.
4. Write `DesiredState.h` — include `"types/Config.h"` for `DriveMode`, `"types/Protocol.h"` for `ReplyFn`, `"types/MotionEventSink.h"` for `MotionEventSink`.
5. Write `OutputState.h` — include `"kinematics/IKinematics.h"`.
6. Write `EstimateDump.h` — include `ActualState.h`.
7. Update `source/types/Inputs.h` — add includes for `ActualState.h`, `DesiredState.h`, `OutputState.h`; replace `RobotStateContainer` body; update `defaultInputs()`.
8. Write `source/state/StateShims.h` — one inline function per legacy name, covering at minimum: `poseX`, `poseY`, `poseHrad`, `encLMm`, `encRMm`, `velLMms`, `velRMms`, `fusedV`, `fusedOmega`, `otosX`, `otosY`, `otosH`, `otosAccelX`, `otosAccelY`, `pwmL`, `pwmR`, `tgtLMms`, `tgtRMms`, `mode`, `replyFn`, `replyCtx`, `corrId`, `deadlineMs`, `targetXWorld`, `targetYWorld`, `targetSpeedMms`, `distanceTargetMm`.
9. Run both builds; fix any include-order or forward-declaration issues.
10. Run `uv run --with pytest python -m pytest tests/simulation/ -q` and confirm green.

## Testing Plan

- **Build test**: `python build.py --clean` (differential). Mecanum build if toolchain available.
- **Sim suite**: `uv run --with pytest python -m pytest tests/simulation/ -q` — must pass unchanged.
- **No new tests required** for this ticket: it is a pure type-addition + shim ticket with no behavior change.

## Documentation Updates

Architecture update already captures this ticket in section A. No additional docs.
