---
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 041 Use Cases

## SUC-001: Estimator belief encapsulated in one object

- **Actor**: Robot firmware / cooperative loop
- **Preconditions**: `Odometry` and `EKF` exist as separate objects; callers reach
  into both independently via `Robot`.
- **Main Flow**:
  1. `PhysicalStateEstimate` is created wrapping `Odometry` and an (owned or wrapped)
     `EKF` by composition.
  2. The cooperative loop calls one method per observation type instead of calling
     `Odometry::predict`, `Odometry::correctEKF`, and `Odometry::setPose` directly.
  3. The fused pose and velocity are readable from a single `getPose()` / `getVelocity()`
     belief-out API.
- **Postconditions**: One named object is the single source of fused belief;
  callers no longer reach into two separate estimator objects.
- **Acceptance Criteria**:
  - [ ] `source/state/PhysicalStateEstimate.{h,cpp}` exists and compiles.
  - [ ] `EKF.{h,cpp}` is located under `source/state/`.
  - [ ] `addOdometryObservation`, `addOtosObservation`, `addCameraObservation` /
        `resetPose`, `getPose()`, `getVelocity()` all exist on `PhysicalStateEstimate`.
  - [ ] `Robot` owns a `PhysicalStateEstimate` value member; `odometry` member retired.

## SUC-002: Estimator dependency rule enforced — no firmware types

- **Actor**: Build system / vendor-confinement grep gate
- **Preconditions**: `PhysicalStateEstimate` is created; it must not import CODAL,
  device handles, `Protocol.h`, or `Commandable`.
- **Main Flow**:
  1. Programmer adds `PhysicalStateEstimate.h` to the vendor-confinement grep scope.
  2. CI gate runs and checks that `source/state/PhysicalStateEstimate.h` /
     `PhysicalStateEstimate.cpp` include only `<stdint.h>`, `<math.h>`, `EKF.h`,
     and plain POD observation-struct headers.
  3. Gate passes with zero forbidden-token hits in `source/state/`.
- **Postconditions**: The estimator is unit-testable without any device mock;
  observation structs are plain PODs that carry no device reference.
- **Acceptance Criteria**:
  - [ ] `PhysicalStateEstimate` headers include no `MicroBit.h`, `I2CBus`, `Protocol.h`,
        or `Commandable`.
  - [ ] Observation structs (`OdometryObservation`, `OtosObservation`,
        `CameraObservation`) are plain PODs.
  - [ ] Vendor-confinement grep gate passes (baseline updated to cover `source/state/`).

## SUC-003: OTOS-tuning verbs handled at app layer, not estimator

- **Actor**: Radio / serial command dispatcher
- **Preconditions**: `Odometry` implements `Commandable`; seven OTOS-tuning verbs
  (`OI/OZ/OR/OV/OL/OA/OP`) are registered from inside the estimator.
- **Main Flow**:
  1. `Commandable` is stripped from `PhysicalStateEstimate`; it no longer calls
     `getCommands()`.
  2. `source/app/OtosCommands.{h,cpp}` is created as an app-layer handler set that
     implements the same seven verbs by calling through to the `IOdometer&` device ref.
  3. `Robot::buildCommandTable` registers `OtosCommands::getCommands()` in place of
     `odometry.getCommands()`.
- **Postconditions**: The seven verbs behave identically; the estimator layer has
  no dependency on `CommandTypes.h` or `Protocol.h`.
- **Acceptance Criteria**:
  - [ ] `OI`, `OZ`, `OR`, `OP`, `OV`, `OL`, `OA` all respond correctly to commands.
  - [ ] `PhysicalStateEstimate` has no `#include "CommandTypes.h"` or `Commandable`
        inheritance.
  - [ ] `source/app/OtosCommands.{h,cpp}` owns the handler set; `buildCommandTable`
        aggregates it.

## SUC-004: Three observation call-sites repointed to estimate API

- **Actor**: Robot firmware loop (`loopTickOnce`, `Robot::otosCorrect`, `handleSI`)
- **Preconditions**: `PhysicalStateEstimate` exists with the observations-in API;
  three call sites still invoke the old `Odometry` methods directly.
- **Main Flow**:
  1. `loopTickOnce` replaces `robot.odometry.predict(...)` with
     `robot.estimate.addOdometryObservation(...)`.
  2. `Robot::otosCorrect` replaces `odometry.correctEKF(...)` with
     `estimate.addOtosObservation(...)`.
  3. `handleSI` (in `SystemCommands.cpp`) replaces `robot->odometry.setPose(...)` with
     `robot->estimate.resetPose(...)`.
- **Postconditions**: All three observation paths flow through `PhysicalStateEstimate`;
  `Odometry` is no longer called directly from outside the estimate object.
- **Acceptance Criteria**:
  - [ ] Zero direct calls to `odometry.predict`, `odometry.correctEKF`, or
        `odometry.setPose` outside `PhysicalStateEstimate`.
  - [ ] `robot.estimate.addOdometryObservation`, `addOtosObservation`, and `resetPose`
        are the sole external observation entry points.
  - [ ] Golden-TLM canary byte-exact; `test_ekf*.py` and `test_otos_fusion.py` pass
        unchanged.

## SUC-005: Back-compat pose mirroring into HardwareState preserves existing readers

- **Actor**: `buildTlmFrame`, `MotionController::getPoseFloat`
- **Preconditions**: `PhysicalStateEstimate` produces the fused pose; existing readers
  still read from `HardwareState.poseX/Y/poseHrad` and `fusedV/fusedOmega`.
- **Main Flow**:
  1. Each observation method in `PhysicalStateEstimate` continues to write
     `s.poseX`, `s.poseY`, `s.poseHrad`, `s.fusedV`, `s.fusedOmega` into the
     passed `HardwareState&` exactly as `Odometry::predict` and `correctEKF` did.
  2. `buildTlmFrame` and `getPoseFloat` require zero changes — they read the same
     `HardwareState` fields as before.
- **Postconditions**: TLM frame is byte-identical; `getPoseFloat` returns the same
  values as before; no Phase F reader repointing is needed this sprint.
- **Acceptance Criteria**:
  - [ ] `defaultRobotConfig()` field-pin diff is empty.
  - [ ] Golden-TLM canary unchanged.
  - [ ] `buildTlmFrame` and `MotionController::getPoseFloat` require no edits.
