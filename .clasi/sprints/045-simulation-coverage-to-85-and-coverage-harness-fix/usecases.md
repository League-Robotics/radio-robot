---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 045 Use Cases

## SUC-001: Run coverage harness and get a valid report
Parent: (engineering infrastructure)

- **Actor**: Developer / CI system
- **Preconditions**: Repo is on the sprint branch. `cmake`, `uv`, `gcovr` are available.
- **Main Flow**:
  1. Developer runs `bash tests/_infra/coverage.sh`.
  2. Script builds a coverage-instrumented `libfirmware_host` in a fresh build directory.
  3. Script runs the full simulation pytest tier against the instrumented library.
  4. Script invokes `gcovr` with `--root .`, `--filter 'source/'`, `--gcov-ignore-errors=source_not_found`.
  5. Script prints overall `source/` line coverage percentage and a per-file table.
  6. Script prints a second "simulatable-code" coverage percentage excluding the documented CODAL-only file set.
  7. If `--fail-under N` is passed, script exits non-zero when overall coverage is below N%.
- **Postconditions**: A valid coverage report is produced; exit code is 0 (or non-zero if `--fail-under` threshold not met).
- **Acceptance Criteria**:
  - [ ] `coverage.sh` runs to completion with no gcovr errors or stale-path warnings.
  - [ ] Overall `source/` line percentage is printed.
  - [ ] Per-file table is printed.
  - [ ] Simulatable-code percentage (with exclusion list applied) is printed.
  - [ ] `--fail-under 85` exits non-zero below 85% and zero at or above 85%.

## SUC-002: MotorController control-path coverage
Parent: (firmware quality)

- **Actor**: Test suite
- **Preconditions**: Sim library is built with coverage instrumentation.
- **Main Flow**:
  1. Test drives the robot via `S` / `D` / `T` commands with encoder values controlled via sim setters.
  2. Test exercises the wedge-detector (stuck encoder) path: hold encoder constant while commanding motion until `EVT enc_wedged` is emitted.
  3. Test exercises the per-wheel ZOH velocity differentiation and PI+FF inner loop with both wheels.
  4. Test exercises `startDriveClean`, `startDrive`, `stop`, `resetIntegrators`, `updateVelGains`.
- **Postconditions**: MotorController line coverage is materially higher (target: ≥85% of its own lines).
- **Acceptance Criteria**:
  - [ ] `EVT enc_wedged` is observed after holding encoder constant for `kWedgeThreshold` ticks.
  - [ ] Per-wheel velocity differentiation is exercised (refreshedWheel = 1 and 2 paths).
  - [ ] `startDriveClean` and `startDrive` paths are exercised.
  - [ ] Suite remains green.

## SUC-003: StopCondition C++ binary path coverage
Parent: (firmware quality)

- **Actor**: Test suite
- **Preconditions**: Sim library is built.
- **Main Flow**:
  1. Test sends motion commands with ROTATION, COLOR, LINE_ANY, POSITION, and SENSOR stop conditions using the wire protocol.
  2. Test manipulates sim sensor values (line, color RGBC, analogIn) to trigger each condition.
  3. Test confirms the command stops when the condition fires.
- **Postconditions**: All StopCondition Kind branches in the C++ binary are exercised.
- **Acceptance Criteria**:
  - [ ] ROTATION, COLOR, LINE_ANY, POSITION, SENSOR (GE and LE) conditions each trigger a motion stop in simulation.
  - [ ] HEADING and DISTANCE stop-condition paths are exercised via the wire (not just the pure-Python mirror).
  - [ ] Suite remains green.

## SUC-004: MotionCommandHandlers edge-path coverage
Parent: (firmware quality)

- **Actor**: Test suite
- **Preconditions**: Sim library is built.
- **Main Flow**:
  1. Test sends malformed or boundary-case motion verbs (missing args, out-of-range values, sensor= parse failures) and verifies ERR replies.
  2. Test exercises the `ctx->queue == nullptr` direct-call fallback path in motion handlers.
  3. Test exercises the D-command, T-command, G-command, R-command, TURN, RT, and ARC verb error branches.
- **Postconditions**: Error-handling and edge branches in MotionCommandHandlers are exercised.
- **Acceptance Criteria**:
  - [ ] ERR replies are received for malformed motion verb inputs.
  - [ ] Direct-call fallback path is reachable and produces correct behavior.
  - [ ] Suite remains green.

## SUC-005: EKF, Odometry, PhysicsWorld, and subsystem command coverage
Parent: (firmware quality)

- **Actor**: Test suite
- **Preconditions**: Sim library is built.
- **Main Flow**:
  1. Test exercises EKF `update_position` / `update_heading` gating branches: inject OTOS readings that are within and outside the Mahalanobis gate.
  2. Test exercises Odometry prediction with wedge suppression active (both `wheelWedgedL` and `wheelWedgedR` true).
  3. Test exercises PhysicsWorld dynamics-error and slip paths via sim API.
  4. Test sends testable `SystemCommands` (SNAP, ZERO, HALT, GET VEL, GET, SET, HELP, VER, ECHO) and `OtosCommands` (OTOS GET, OTOS SET) via the sim command interface.
- **Postconditions**: EKF correction, Odometry wedge, PhysicsWorld slip, and non-CODAL system command branches are covered.
- **Acceptance Criteria**:
  - [ ] EKF update gating: a reading outside the gate is rejected; one inside is accepted.
  - [ ] Odometry wedge suppression path is exercised (predict runs with wedge flag set).
  - [ ] Testable SystemCommands return expected OK/ERR/value responses.
  - [ ] OtosCommands paths respond correctly.
  - [ ] Suite remains green.

## SUC-006: Simulatable-code coverage reaches ≥85%
Parent: (firmware quality)

- **Actor**: Developer / CI system
- **Preconditions**: Tickets T1–T4 are complete and suite is green.
- **Main Flow**:
  1. Developer runs `bash tests/_infra/coverage.sh --fail-under 85`.
  2. Coverage report shows simulatable-code coverage ≥85%.
- **Postconditions**: Coverage gate is met; exclusion set is documented.
- **Acceptance Criteria**:
  - [ ] Overall `source/` line coverage ≥85%, OR simulatable-code coverage ≥85% with documented CODAL-only exclusion set.
  - [ ] No existing assertions weakened or deleted.
  - [ ] `--fail-under 85` exits zero.
