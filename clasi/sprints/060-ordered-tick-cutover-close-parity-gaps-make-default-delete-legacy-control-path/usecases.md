---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 060 Use Cases

## SUC-001: Telemetry accurately reflects Drive2/Sensors internal state
Parent: (message-based architecture migration)

- **Actor**: Robot firmware (ordered-tick path)
- **Preconditions**: Drive2 and Sensors subsystems are running as the live tick path.
- **Main Flow**:
  1. `buildTlmFrame` reads encoder, pose, velocity, twist, OTOS, line, and color
     fields from `drive2.state()` and `sensors.state()` instead of `robot.state.actual`.
  2. The TLM frame is emitted over the STREAM channel.
  3. The legacy `drive.periodic()` crutch in the ordered-tick path is removed.
- **Postconditions**: TLM fields are sourced exclusively from the message-contract
  subsystems. The golden-TLM capture is regenerated and accepted by the stakeholder.
- **Acceptance Criteria**:
  - [ ] `buildTlmFrame` no longer reads `robot.state.actual` for encoder/pose/vel/twist/otos/line/color fields.
  - [ ] `drive.periodic()` is not called from the ordered-tick branch.
  - [ ] `tests/_infra/golden_tlm_capture.json` is regenerated; diff is reviewed and accepted.
  - [ ] `test_golden_tlm.py` passes with the new capture.

## SUC-002: Motor output has a single authoritative source
Parent: (message-based architecture migration)

- **Actor**: Drive2 subsystem / Robot constructor
- **Preconditions**: Drive2's `tickAction` is the motor-command authority in the ordered-tick path.
- **Main Flow**:
  1. Drive2's internal `_outputs` buffer is the sole target of `MotorController::setCommandsRef`.
  2. The Robot constructor no longer overrides `setCommandsRef` with `&state.outputs`.
  3. `drive2.tickAction` writes motor commands to `Drive2::_outputs`; the HAL tick reads from there.
- **Postconditions**: Motor output flows through exactly one buffer without a subsequent override.
- **Acceptance Criteria**:
  - [ ] `motorController.setCommandsRef(&state.outputs)` is removed or conditioned to not override Drive2's binding.
  - [ ] HAL tick `robot.hal.tick(now, ...)` is passed the motor commands from the authoritative source.
  - [ ] Host tests remain green.

## SUC-003: Sensor reads fire exactly once per due interval
Parent: (message-based architecture migration)

- **Actor**: `sensors.tick()` / loopTickOnce ordered-tick path
- **Preconditions**: `sensors.tick()` owns its own lag timers (`_lastLineTick`, `_lastColorTick`).
- **Main Flow**:
  1. `sensors.tick(now)` fires line and color reads when their respective lag gates are due.
  2. Legacy `robot.lineSensor.periodic(ts, now)` and `robot.colorSensor_.periodic(ts, now)` are not called from the ordered-tick path.
  3. `LoopTickState.lastLine` / `.lastColor` are not consulted in the ordered-tick path.
- **Postconditions**: Sensor lag schedule is driven exclusively by `sensors.tick()`.
- **Acceptance Criteria**:
  - [ ] Legacy `lineSensor.periodic` and `colorSensor_.periodic` calls are absent from the ordered-tick branch.
  - [ ] Sensor reads occur at the correct lag-gated interval under `sensors.tick()`.
  - [ ] Host tests remain green.

## SUC-004: Ordered-tick path is the sole live control path
Parent: (message-based architecture migration)

- **Actor**: Build system / firmware developer
- **Preconditions**: All 3 parity gaps are closed.
- **Main Flow**:
  1. `USE_ORDERED_TICK` is defined by default (e.g., via `CMakeLists.txt` or a top-level `#define`).
  2. The full host test suite runs against the ordered-tick path and passes.
  3. The legacy `#ifndef USE_ORDERED_TICK` branch is no longer the production path.
- **Postconditions**: Any firmware build without explicit flag manipulation uses the ordered-tick path.
- **Acceptance Criteria**:
  - [ ] The sim build compiles with `USE_ORDERED_TICK` active without any extra flags.
  - [ ] Full host suite passes (excluding 2 known-baseline config-golden failures).
  - [ ] `test_059_ordered_tick_parity.py` passes.

## SUC-005: Legacy control path code is removed from the codebase
Parent: (message-based architecture migration)

- **Actor**: Firmware developer / codebase
- **Preconditions**: `USE_ORDERED_TICK` is the default and tests are green.
- **Main Flow**:
  1. The `#ifndef USE_ORDERED_TICK` block in `LoopTickOnce.cpp` (lines 57-159) is deleted.
  2. The `#ifdef`/`#else`/`#endif` scaffolding around the two paths is removed.
  3. Dead Robot members (`subsystems::Drive drive`, legacy lineSensor/colorSensor_ periodic
     usage, `_tlmBoundFn`/`drive.periodic` TLM crutch) are removed from `Robot.h/.cpp`.
  4. `grep -r USE_ORDERED_TICK source/ tests/` returns nothing.
- **Postconditions**: No legacy control path code exists in the codebase.
- **Acceptance Criteria**:
  - [ ] `LoopTickOnce.cpp` contains only the ordered-tick body (no `#ifdef USE_ORDERED_TICK` guards).
  - [ ] Dead Robot members removed; codebase compiles cleanly.
  - [ ] `grep -r USE_ORDERED_TICK source/ tests/` returns nothing.
  - [ ] Full host suite passes.

## SUC-006: Subsystem names reflect their permanent role (no `2` suffix)
Parent: (message-based architecture migration)

- **Actor**: Firmware developer / codebase
- **Preconditions**: Legacy Drive, legacy MotionController, and bvc (old) are deleted.
- **Main Flow**:
  1. `bvc2` is renamed to `bvc`.
  2. `subsystems::Drive2` class and files are renamed to `subsystems::Drive`.
  3. `MotionController2` class and files are renamed to `MotionController`.
  4. Robot.h/cpp references and `#include` paths are updated throughout.
  5. The `planner` member retains its name (it is already the clean name for the Planner role).
- **Postconditions**: All subsystem names are their final canonical names; no `2`-suffixed identifiers remain.
- **Acceptance Criteria**:
  - [ ] `grep -r "Drive2\|bvc2\|MotionController2" source/` returns nothing.
  - [ ] Codebase compiles without warnings related to renamed identifiers.
  - [ ] Full host suite passes.
  - [ ] Robot.h declaration order preserved (bvc before drive before sensors before planner).

## SUC-007: Bench parity confirmed on tovez (human-operated)
Parent: (message-based architecture migration)

- **Actor**: Stakeholder / team-lead (human), running the bench with the new firmware
- **Preconditions**: Sprint 060 firmware is built with the fully-cutover ordered-tick path and renamed subsystems.
- **Main Flow**:
  1. The sprint produces a firmware build artifact and a documented bench checklist.
  2. The stakeholder flashes the firmware to the tovez robot.
  3. The stakeholder runs VW (body-velocity), TURN, and a goto/turn/distance sequence.
  4. Behavior is compared to the pre-cutover reference build.
- **Postconditions**: The stakeholder confirms parity or files a bug.
- **Acceptance Criteria**:
  - [ ] Firmware build succeeds with the fully-cutover ordered-tick path.
  - [ ] Bench checklist is produced documenting the VW/TURN/GOTO sequences and expected outcomes.
  - [ ] Physical bench run is executed by the stakeholder (not autonomous).
