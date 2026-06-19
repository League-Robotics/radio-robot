---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 040 Use Cases

Sprint 040 targets developers writing or debugging tests. All use cases are
sim-layer internal; no robot firmware behavior changes.

---

## SUC-001: Isolate plant dynamics from sensor errors in a simulation test

- **Actor**: Test author
- **Preconditions**: Host sim build available; `PhysicsWorld` and `SimHardware` exist.
- **Main Flow**:
  1. Test calls `sim_set_true_pose(x, y, h)` to place the robot at a known pose.
  2. Test calls `sim_tick()` with `setActuators(pwmL, pwmR)` driving the plant forward.
  3. Test reads true chassis pose from `sim_get_true_pose_*()`.
  4. Test verifies that true pose reached the expected target within tolerance,
     without any sensor error model involved.
- **Postconditions**: True pose matches kinematic prediction; no encoder noise,
  odometer drift, or slip is present (all error setters at default no-op).
- **Acceptance Criteria**:
  - [ ] `sim_set_true_pose` / `sim_get_true_pose_*` functions exist in the ABI.
  - [ ] Fresh `PhysicsWorld` has all error setters at no-op (perfect fidelity).
  - [ ] Plant-only test: true pose after N ticks of constant-PWM equals the
        kinematic prediction within float tolerance.

---

## SUC-002: Verify observation model error in isolation

- **Actor**: Test author
- **Preconditions**: `PhysicsWorld` with settable truth; observation models
  (`SimMotor`, `SimOdometer`, etc.) compiled into the sim lib.
- **Main Flow**:
  1. Test calls `sim_set_true_pose(known)` to fix the ground truth.
  2. Test configures error parameters on a specific observation model
     (e.g., yaw drift on `SimOdometer`).
  3. Test reads the sensor output via the capability interface.
  4. Test verifies sensor output differs from ground truth by the expected
     error magnitude.
- **Postconditions**: Observation model reflects injected error; plant truth unchanged.
- **Acceptance Criteria**:
  - [ ] Observation-only test: `SimOdometer` with yaw drift returns heading that
        diverges from `PhysicsWorld` truth at the configured rate.
  - [ ] Dropout (frozen encoder, LIFT status) causes `SimMotor` / `SimOdometer`
        to return stale / invalid data.
  - [ ] `sim_set_perfect()` resets all error layers to no-op in one call.

---

## SUC-003: Assert estimate vs. true pose in a whole-sim run

- **Actor**: Test author
- **Preconditions**: Sim running with `SimHardware` (plant + observation + firmware EKF).
- **Main Flow**:
  1. Test drives the robot through a D/G/TURN command.
  2. After the command completes, test calls `sim.estimation_error()`.
  3. Test asserts `estimation_error() < tolerance`.
- **Postconditions**: EKF estimate is within tolerance of the plant's true pose.
- **Acceptance Criteria**:
  - [ ] `sim.estimation_error()` (Python) / `sim_get_estimation_error_*()`
        (C ABI) returns the Euclidean distance between EKF pose and true pose.
  - [ ] Estimator-only test: bad OTOS injection triggers Mahalanobis gate
        rejection; true pose remains correct; EKF recovers.
  - [ ] Whole-robot test: final true pose is within 20 mm / 5° of the D/G/TURN
        commanded target.

---

## SUC-004: Preserve behavior of existing sim-driven tests during migration

- **Actor**: Regression suite
- **Preconditions**: ~25 existing sim-driven Python tests use `MockMotor`,
  `MockOtosSensor`, `ExactPoseTracker` through the `sim_*` ABI.
- **Main Flow**:
  1. The sim lib is rebuilt against `SimHardware` + `PhysicsWorld`.
  2. All existing `sim_*` ABI entry points continue to work (aliased or re-routed
     to the plant).
  3. Existing tests run without modification.
- **Postconditions**: All ~25 sim-driven tests pass; no test source changes required
  for behavior-preservation fence tests.
- **Acceptance Criteria**:
  - [ ] `sim_set_enc_l/r` sets true wheel travel in `PhysicsWorld` (bug fixed).
  - [ ] `sim_get_exact_pose_*` aliased to `sim_get_true_pose_*`; same values returned.
  - [ ] `sim_field_profile` slip behavior preserved (field-024 fixture passes).
  - [ ] All canaries green: golden-TLM byte-exact, field-pin unchanged,
        vendor grep gate passes.

---

## SUC-005: Consolidate midpoint-arc integration to a single canonical formula

- **Actor**: Developer (maintenance / correctness)
- **Preconditions**: Three copies of the midpoint-arc integration exist
  (`MockMotor::integrate`, `ExactPoseTracker::update`, `BenchOtosSensor::tick`).
- **Main Flow**:
  1. Developer moves the integration into `PhysicsWorld::update(dt)`.
  2. All three former owners are retired or re-routed to read from `PhysicsWorld`.
  3. `Odometry::predict` (the firmware formula) remains separate and untouched.
- **Postconditions**: Exactly one midpoint-arc integration formula in `source/io/sim/`.
- **Acceptance Criteria**:
  - [ ] `MockMotor::integrate`, `ExactPoseTracker::update`, and
        `BenchOtosSensor::tick` chassis-integration branches are removed or
        reduced to reads from `PhysicsWorld`.
  - [ ] The golden-TLM canary passes bit-exactly after consolidation.
  - [ ] `sim_field_profile` slip behavior passes numerically against the
        field-024 regression baseline.

---

## SUC-006: Slip model moves to the plant chassis-integration step

- **Actor**: Developer (architecture)
- **Preconditions**: Slip currently applied in `MockMotor::integrate` (encoder
  reading step), not the chassis-integration step.
- **Main Flow**:
  1. Developer moves slip from the encoder accumulation into `PhysicsWorld::update`
     (reduces body rotation by `effectiveSlip`; wheel travel remains real).
  2. Slip behavior validated numerically against the field-024 fixture.
- **Postconditions**: Slip is applied at the chassis body-rotation step.
  Encoder readings reflect true wheel travel (not slip-reduced).
- **Acceptance Criteria**:
  - [ ] `sim_field_profile` with `slip_turn_extra=0.26` produces the same
        end-pose within float tolerance as before the move.
  - [ ] `sim_set_enc_l/r` sets true travel (encoder = physical wheel distance).

---

## SUC-007: Retire obsolete mock objects after migration

- **Actor**: Developer (cleanup)
- **Preconditions**: `PhysicsWorld`, `Sim*` observation models, and `SimHardware`
  in place; all existing tests passing through the new layer.
- **Main Flow**:
  1. Developer verifies all callers of `MockMotor`, `ExactPoseTracker`, and the
     `BenchOtosSensor` dual-accumulator are re-routed.
  2. Developer deletes or stubs out the obsolete files.
- **Postconditions**: `source/io/sim/` contains only the new clean split;
  no dead code remains.
- **Acceptance Criteria**:
  - [ ] `MockMotor`, `ExactPoseTracker` removed from `source/io/sim/`
        (or reduced to thin type-alias stubs if the ABI references them by name).
  - [ ] `BenchOtosSensor` retained in `source/io/real/` (gated by
        `BENCH_OTOS_ENABLED`); its dual-accumulator is the *bench sensor* that
        integrates commanded velocity — this stays; only its role in the mock-sim
        is retired (it is no longer the odometry truth source in SIM mode).
  - [ ] Simulation tier ≥ 1957 passed, 0 errors.
