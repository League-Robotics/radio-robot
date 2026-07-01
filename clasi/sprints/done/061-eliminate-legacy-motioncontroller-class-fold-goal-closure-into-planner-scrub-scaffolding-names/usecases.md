---
status: final
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 061 Use Cases

## SUC-001: Planner exposes goal-start and mode-query API to call sites
Issue: internalize-legacy-motioncontroller-into-planner.md

- **Actor**: Firmware subsystems (Superstructure, Robot, command handlers, telemetry)
- **Preconditions**: Sprint 060 is complete; `Planner` wraps `MotionController` by
  reference; `MotionController` is a public Robot member.
- **Main Flow**:
  1. A call site that previously called `motionController.beginX(...)`,
     `motionController.mode()`, `motionController.disableSafetyOneShot()`, or
     similar now calls the equivalent method on `planner`.
  2. `Planner` satisfies the call with the same behavior and semantics as
     `MotionController` previously provided.
  3. No behavioral difference is visible to the caller.
- **Postconditions**: All call sites reach goal-closure through `Planner` with no
  direct `MotionController` access.
- **Acceptance Criteria**:
  - [ ] `Planner` exposes `mode()`, `beginStream`, `beginVelocity`, `beginTimed`,
        `beginDistance`, `beginGoTo`, `beginTurn`, `beginRotation`, `stop`,
        `cancel`, `softStop`, `beginRawVelocity`, `disableSafetyOneShot`,
        `hasActiveCommand`, `emitToActiveChannel`, `setHardwareState`,
        `setRobotCtx`, `setBvcStateRef`.
  - [ ] Each method produces behavior byte-identical to the former `MotionController`
        counterpart (golden-TLM canary stays green).

## SUC-002: Superstructure dispatches goals through Planner
Issue: internalize-legacy-motioncontroller-into-planner.md

- **Actor**: `Superstructure` class, goal-dispatch path
- **Preconditions**: `Planner` exposes the full goal-start API (SUC-001 complete).
- **Main Flow**:
  1. A command handler calls `superstructure.requestGoal(gr)`.
  2. `Superstructure` dispatches `gr.goal` to the appropriate `planner.beginX()`
     call.
  3. EVT completion events are emitted through the correct reply sink.
- **Postconditions**: `Superstructure` holds `Planner&` (not `MotionController&`);
  `Superstructure.mc()` returns a `Planner&`.
- **Acceptance Criteria**:
  - [ ] `Superstructure` constructor takes `Planner&` (not `MotionController&`).
  - [ ] `requestGoal` switch dispatches to `planner.beginX()` for all Goal kinds.
  - [ ] `superstructure.mc()` removed or updated to return `Planner&`.
  - [ ] All tests that exercise goal-dispatch continue to pass.

## SUC-003: Command handlers and telemetry reference Planner directly
Issue: internalize-legacy-motioncontroller-into-planner.md

- **Actor**: `SystemCommands`, `MotionCommands`, `RobotTelemetry`, `Robot.cpp`
  orchestration methods
- **Preconditions**: SUC-001 and SUC-002 complete.
- **Main Flow**:
  1. `MotionCtx::mc` carries a `Planner*` (not `MotionController*`).
  2. `SystemCommands.cpp` calls `robot->planner.disableSafetyOneShot()`.
  3. `RobotTelemetry.cpp` calls `planner.mode()` instead of
     `motionController.mode()`.
  4. `Robot.cpp` `otosCorrect` and `distanceDrive` call through `planner`.
- **Postconditions**: No call site outside `Planner.*` / `PlannerBegin.cpp`
  references `MotionController` by name.
- **Acceptance Criteria**:
  - [ ] `MotionCtx::mc` is `Planner*`.
  - [ ] `RobotTelemetry.cpp` mode-char switch reads `planner.mode()`.
  - [ ] `Robot.cpp::otosCorrect` emits via `planner.emitToActiveChannel`.
  - [ ] `Robot.cpp::distanceDrive` calls `planner.beginDistance(...)`.
  - [ ] `SystemCommands.cpp` calls `planner.disableSafetyOneShot()`.

## SUC-004: Planner owns the MotionController implementation directly
Issue: internalize-legacy-motioncontroller-into-planner.md

- **Actor**: `Planner` class internals; `Robot` struct
- **Preconditions**: All call sites rerouted (SUC-001 through SUC-003).
- **Main Flow**:
  1. `Planner` is reconstructed as a self-contained class owning `_bvc`,
     `_activeCmd`, `_safetyEnabled`, and all the begin*() / driveAdvance() logic.
  2. `Robot` drops the `motionController` value member.
  3. `Planner` constructor signature changes to take `MotorController&`,
     `Odometry&`, and `const subsystems::Drive&` directly.
  4. `Robot.h` declaration order is updated accordingly.
- **Postconditions**: `MotionController.h/.cpp` and `MotionControllerBegin.cpp`
  are no longer referenced or compiled. `Robot` has no `motionController` member.
- **Acceptance Criteria**:
  - [ ] `Planner` compiles and passes all tests without `MotionController.h`.
  - [ ] `Robot.h` no longer declares `MotionController motionController`.
  - [ ] `planner_api.cpp` `PlannerHandle` constructs `Planner` directly.

## SUC-005: Legacy MotionController source files deleted; codebase clean
Issue: internalize-legacy-motioncontroller-into-planner.md

- **Actor**: Firmware build system; codebase grep
- **Preconditions**: SUC-004 complete; all references to `MotionController` removed.
- **Main Flow**:
  1. `source/superstructure/MotionController.h` and `.cpp` are deleted.
  2. `source/control/MotionControllerBegin.cpp` is deleted.
  3. All `#include "superstructure/MotionController.h"` directives removed.
  4. `grep -rIn "MotionController\b" source/` returns zero hits (excluding provenance
     comments that may be updated/removed).
- **Postconditions**: Three legacy files gone; clean host and firmware build.
- **Acceptance Criteria**:
  - [ ] Host `cmake --build build_sim` succeeds with zero errors.
  - [ ] `grep -rIn "MotionController\b" source/` returns nothing meaningful.
  - [ ] Golden-TLM, ordered-tick parity, and planner-subsystem tests still green.

## SUC-006: Test-infra scaffolding names scrubbed to canonical
Issue: internalize-legacy-motioncontroller-into-planner.md

- **Actor**: Test infrastructure; Python ctypes call sites
- **Preconditions**: SUC-005 complete.
- **Main Flow**:
  1. `tests/_infra/sim/drive2_api.cpp` is renamed `drive_api.cpp`; all C-ABI
     function symbols renamed from `drive2_api_*` to `drive_api_*`.
  2. `tests/_infra/sim/bus_drain_api.cpp` symbols renamed from
     `bus_drain_api_drive2_*` to `bus_drain_api_drive_*`.
  3. `tests/simulation/unit/test_drive2_subsystem.py` renamed
     `test_drive_subsystem.py`; Python ctypes bindings updated.
  4. `tests/simulation/unit/test_motioncontroller2_smoke.py` renamed
     `test_planner_subsystem_smoke.py` (if not already canonical).
  5. `Drive2Ctx` Python struct renamed `DriveCtx`.
  6. C++ symbol renames and Python renames done atomically in one ticket.
- **Postconditions**: No `drive2`/`mc2`/`Drive2` scaffolding names in test
  infra; Python tests load the renamed symbols and pass.
- **Acceptance Criteria**:
  - [ ] `grep -rIn "drive2_api\|bus_drain_api_drive2\|Drive2Ctx\|mc2" tests/`
        returns nothing.
  - [ ] Full suite green (except 2 baseline) after rename.

## SUC-007: Bench-validated build artifact produced for tovez
Issue: internalize-legacy-motioncontroller-into-planner.md

- **Actor**: Stakeholder (bench operator); firmware build pipeline
- **Preconditions**: SUC-005 and SUC-006 complete; full test suite green.
- **Main Flow**:
  1. Programmer runs `build.py --clean` in the firmware directory.
  2. Firmware compiles without errors.
  3. `MICROBIT.hex` is produced and its embedded build banner decoded to confirm
     the clean binary (no stale incremental artifact).
  4. Bench checklist (`tests/bench/061_bench_checklist.md`) is created with the
     VW/TURN/GOTO/DISTANCE sequences and expected EVT completions.
  5. Stakeholder flashes firmware to tovez and runs the checklist manually.
- **Postconditions**: Sprint branch is ready for stakeholder bench validation;
  sprint is left open until validation is complete.
- **Acceptance Criteria**:
  - [ ] `build.py --clean` exits zero.
  - [ ] `MICROBIT.hex` build banner verified (not a stale incremental build).
  - [ ] `tests/bench/061_bench_checklist.md` created with VW/TURN/GOTO/DISTANCE
        sequences and expected EVT completions for tovez.
  - [ ] Full host suite run twice; both runs green except 2 baseline failures.
