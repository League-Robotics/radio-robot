---
status: approved
sprint: 039
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 039 Use Cases

## SUC-039-001 — Capability interfaces replace device-named interfaces

The codebase provides `IVelocityMotor`, `IPositionMotor`, `IOdometer`, `IBusDiagnostics`
(and retains `ILineSensor`, `IColorSensor`, `IPortIO`) in `source/io/capability/`. Each
interface carries only neutral types: SI units (mm, mm/s, rad, rad/s), plain enums,
no vendor/CODAL/transport types in any method signature. All existing consumers (control
layer, Robot, tests) continue to compile unchanged because alias shims bridge the old
names to the new ones.

- **Actor**: Firmware control layer (MotorController, Robot, Odometry).
- **Preconditions**: Sprint 038 (Phase 0) canaries are green; `source/io/capability/` does not yet exist.
- **Main Flow**:
  1. Developer adds `source/io/capability/IVelocityMotor.h`, `IPositionMotor.h`, `IOdometer.h`, `IBusDiagnostics.h`, `ILineSensor.h`, `IColorSensor.h`, `IPortIO.h`.
  2. Each new header defines the capability interface in SI units.
  3. Alias shims (`using IMotor = IVelocityMotor;` etc.) in the old `hal/` headers let all existing callers compile without changes.
  4. The vendor-confinement grep gate passes; simulation tier stays green.
- **Postconditions**: Seven capability headers exist; host build compiles; canaries green.
- **Acceptance Criteria**:
  - [ ] `source/io/capability/` contains all 7 headers.
  - [ ] No vendor/CODAL type appears in any capability interface signature.
  - [ ] Simulation tier green; vendor-confinement gate passes.

---

## SUC-039-002 — Vendor types sealed out of the control layer

`MotorController.h` no longer includes `MicroBit.h` and no longer names `I2CBus`
directly. Bus diagnostics (error count, reentry violations, last error) are accessible
via an `IBusDiagnostics` reference, implemented by a thin `MotorBusDiagnostics` adapter
owned by `NezhaHAL`. `DebugCommandable.cpp` no longer includes `I2CBus.h` in a
file-global scope — it reaches diagnostics via `IBusDiagnostics`. The vendor-confinement
grep gate baseline is ratcheted to remove all `MotorController`-level entries.

- **Actor**: `MotorController::controlTick`, `DebugCommandable`.
- **Preconditions**: `IBusDiagnostics` interface exists (SUC-039-001 done); `MotorBusDiagnostics` adapter implemented.
- **Main Flow**:
  1. `MotorController` stores an `IBusDiagnostics*` instead of `I2CBus*`.
  2. `MotorBusDiagnostics` wraps the real `I2CBus` at addr 0x10, owned by `NezhaHAL`.
  3. `NezhaHAL` exposes `IBusDiagnostics& busDiagnostics()`.
  4. `main.cpp` binds `motorController.setBusDiagnostics(&hal.busDiagnostics())`.
  5. `DebugCommandable.cpp` reaches `I2CBus` only through the `NezhaHAL*` it already holds (firmware-only path).
  6. `MotorController.h`'s `#include "MicroBit.h"` guard and `I2CBus` forward declaration are removed.
  7. Vendor-confinement baseline updated (smaller).
- **Postconditions**: `MotorController.h` contains no `MicroBit.h` include and no `I2CBus` declaration.
- **Acceptance Criteria**:
  - [ ] `MotorController.h` compiles in `HOST_BUILD` without `MicroBit.h`.
  - [ ] `MotorController.h` contains no `I2CBus` forward declaration.
  - [ ] Vendor-confinement baseline entry count for `MotorController.*` is zero.
  - [ ] `IBusDiagnostics` interface exists with `errorCount()`, `reentryViolations()`, `lastError()`.
  - [ ] Simulation tier green.

---

## SUC-039-003 — Nezha split-phase state machine lives in the Motor impl

The request-then-collect I2C split-phase encoder sequence is owned by `Motor`, not by
`Robot::controlCollectSplitPhase`. `Hardware::tick(now_ms)` drives the Motor impl's
internal request/collect cycle each cooperative-loop iteration. `positionMm()` and
`velocityMmps()` become cheap accessors on `IVelocityMotor`. The outlier filter and
wedge-detector logic moves into the Motor impl. I2C bytes on the wire are unchanged.
`Robot::controlCollectSplitPhase` is removed.

- **Actor**: `Motor` impl, `NezhaHAL::tick`, cooperative loop.
- **Preconditions**: `IVelocityMotor` interface defines `positionMm()` and `velocityMmps()` (SUC-039-001 done).
- **Main Flow**:
  1. `Motor` gains internal state: `_lastPositionMm`, `_lastVelocityMmps`, `_splitPhaseState`.
  2. `NezhaHAL::tick(now_ms)` calls `motorL.tick(now_ms)` and `motorR.tick(now_ms)` — each motor alternates request/collect internally.
  3. `Motor::tick()` encapsulates the outlier filter and wedge-detect logic.
  4. `MotorController::controlTick()` reads `IVelocityMotor::positionMm()` / `velocityMmps()` instead of calling `readEncoderMmFSettle()`.
  5. `Robot::controlCollectSplitPhase` is removed; its call site in `sim_api.cpp` is removed or replaced by `hal.tick(now_ms)`.
  6. The golden-TLM canary verifies byte-exact output.
- **Postconditions**: `Robot.cpp` contains no `controlCollectSplitPhase`; `Motor` owns the state machine.
- **Acceptance Criteria**:
  - [ ] `Robot::controlCollectSplitPhase` does not exist.
  - [ ] `Motor::tick(now_ms)` (or equivalent) advances the split-phase state machine.
  - [ ] `positionMm()` and `velocityMmps()` return last-collected values.
  - [ ] Golden-TLM canary unchanged (byte-exact).
  - [ ] Simulation tier green.

---

## SUC-039-004 — IMotor split into IVelocityMotor and IPositionMotor; IServo folded in

`IMotor` is replaced by `IVelocityMotor` (drive wheel) and `IPositionMotor`
(on-chip position control). The `Motor` impl exposes both via an RTTI-free
`asPositionMotor()` accessor. `IServo` is folded into `IPositionMotor`; `Servo`
implements `IPositionMotor` only. Alias shims keep callers compiling.

- **Actor**: `MotorController`, `ServoController`, `Robot` constructor, `Hardware`.
- **Preconditions**: `IVelocityMotor` and `IPositionMotor` interfaces exist; alias shims in place.
- **Main Flow**:
  1. `Hardware::motorL/R()` return `IVelocityMotor&`.
  2. `Hardware::gripper()` returns `IPositionMotor&`.
  3. `Motor` implements both `IVelocityMotor` and `IPositionMotor`; `asPositionMotor()` returns `this`.
  4. `Servo` implements `IPositionMotor` only.
  5. Alias `using IMotor = IVelocityMotor;` in the old `IMotor.h`; `using IServo = IPositionMotor;` in the old `IServo.h`.
  6. All consumers compile without modification.
- **Postconditions**: `IMotor` and `IServo` are aliases only; `IVelocityMotor` and `IPositionMotor` are canonical.
- **Acceptance Criteria**:
  - [ ] `IVelocityMotor` and `IPositionMotor` at `source/io/capability/`.
  - [ ] `asPositionMotor()` virtual accessor on `IVelocityMotor`; default returns `nullptr`.
  - [ ] `Motor` returns non-null from `asPositionMotor()`.
  - [ ] `Servo` derives from `IPositionMotor`; `ServoController` uses `IPositionMotor&`.
  - [ ] Simulation tier green.

---

## SUC-039-005 — IOtosSensor renamed IOdometer; LSB and cfg leaks sealed

`IOtosSensor` is renamed `IOdometer`. Public methods carry `Pose2D` / `BodyTwist` /
`BodyAccel` (SI units) instead of `OtosPose` / `OtosVelocity` / `OtosAccel` with
`RobotConfig&` parameters. The concrete `OtosSensor` impl retains its internal LSB
math and `RobotConfig` member. Aliases (`using OtosPose = Pose2D;` etc.) and
`using IOtosSensor = IOdometer;` prevent TLM and test churn.

- **Actor**: `Odometry::correctEKF`, `Robot::otosCorrect`, OTOS command handlers.
- **Preconditions**: `IOdometer` defined in `source/io/capability/`; `Pose2D` / `BodyTwist` types defined.
- **Main Flow**:
  1. `source/io/capability/IOdometer.h` defines `Pose2D`, `BodyTwist`, `BodyAccel` structs and the `IOdometer` interface.
  2. `IOdometer::readTransformed()` returns `Pose2D` (or writes to it) with no `RobotConfig&` parameter.
  3. `OtosSensor` stores `RobotConfig _cfg` as a constructor-injected member.
  4. `using OtosPose = Pose2D;` and `using OtosVelocity = BodyTwist;` added in old header or alias header.
  5. `using IOtosSensor = IOdometer;` added in old `IOtosSensor.h` for backward compat.
  6. All callers of `readTransformed` compile without modification.
- **Postconditions**: `IOdometer` is canonical; no `RobotConfig&` in any public read signature.
- **Acceptance Criteria**:
  - [ ] `IOdometer` at `source/io/capability/IOdometer.h`.
  - [ ] `readTransformed` signature has no `RobotConfig&` parameter.
  - [ ] `Pose2D`, `BodyTwist`, `BodyAccel` structs defined.
  - [ ] `using OtosPose = Pose2D;` and `using OtosVelocity = BodyTwist;` aliases present.
  - [ ] All existing callers compile; TLM tests pass; simulation tier green.

---

## SUC-039-006 — hal/ renamed io/; ROBOT_RUN_MODE drives the build

`source/hal/` becomes `source/io/` with `io/capability/`, `io/real/`, and `io/sim/`
subdirectories (the last scaffolded empty for Phase B). The CMake host-build replaces
`list(FILTER … EXCLUDE REGEX ".*/hal/mock/.*")` with a `ROBOT_RUN_MODE` variable
(`REAL` | `SIM` | `REPLAY`). The firmware `build.py` passes `ROBOT_RUN_MODE=REAL`;
the sim `CMakeLists.txt` uses `ROBOT_RUN_MODE=SIM`. `BENCH_OTOS_ENABLED` and
`PRODUCTION_BUILD` remain orthogonal. A `ReplayHAL` stub (`RobotMode::REPLAY` with
empty feed impls) is added.

- **Actor**: Build system (CMake), developer.
- **Preconditions**: All interface moves from prior use cases complete; `source/io/capability/`, `io/real/`, `io/sim/` dirs ready.
- **Main Flow**:
  1. `git mv source/hal/ source/io/` (real files move to `io/real/`; mock files to `io/sim/`).
  2. All `#include "hal/…"` → `#include "io/…"` (or the include paths updated in CMake).
  3. `CMakeLists.txt` (sim): replace `MOCK_SOURCES` glob over `hal/mock/` with `SIM_SOURCES` glob over `io/sim/`.
  4. `ROBOT_RUN_MODE` variable replaces the `list(FILTER EXCLUDE REGEX)` idiom.
  5. `ReplayHAL.cpp` stub added at `source/io/ReplayHAL.cpp`.
  6. Vendor-confinement gate's scope comment updated to "above `source/io/`".
- **Postconditions**: `source/hal/` does not exist; `source/io/` layout established; both builds green.
- **Acceptance Criteria**:
  - [ ] `source/hal/` directory does not exist.
  - [ ] `source/io/real/` and `source/io/sim/` and `source/io/capability/` exist.
  - [ ] Host build uses `ROBOT_RUN_MODE=SIM`; `list(FILTER EXCLUDE REGEX … hal/mock …)` removed from CMakeLists.
  - [ ] Vendor-confinement gate scope is "above `source/io/`".
  - [ ] Simulation tier green; `defaultRobotConfig()` field-pin unchanged.
