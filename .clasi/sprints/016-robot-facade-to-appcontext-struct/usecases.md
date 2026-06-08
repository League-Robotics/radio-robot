---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 016 Use Cases

This sprint is a **pure structural refactor**. The use cases below are
regression-guard use cases: they describe behaviors that must be preserved
identically through and after the refactor. No new user-visible behavior is
introduced.

---

## SUC-001: Drive verb execution (S/T/D/G/VW/STOP)

- **Actor**: Python host (via serial or radio)
- **Preconditions**: Robot is powered on, firmware flashed, PING confirms
  liveness. `AppContext` is wired in `main()`.
- **Main Flow**:
  1. Host sends `S <l> <r>`, `T <l> <r> <ms>`, `D <l> <r> <mm>`,
     `G <x> <y> <speed>`, `VW <v> <omega>`, or `STOP`.
  2. `CommandProcessor` dispatches to `robot.driveController.beginX()` directly
     (no facade delegation).
  3. Motor wheels move at the commanded setpoints.
  4. Drive completions (`EVT done T/D/G`, `EVT safety_stop`) are emitted over
     the originating channel.
- **Postconditions**: Robot is driving or stopped per the command. EVT is
  emitted on completion for T/D/G.
- **Acceptance Criteria**:
  - [ ] `S 200 200` starts both wheels at 200 mm/s.
  - [ ] `T 200 200 2000` drives for 2 s then emits `EVT done T`.
  - [ ] `D 200 200 300` drives 300 mm then emits `EVT done D`. Encoder-reset
        workaround preserved: encLMm/encRMm zeroed after `beginDistance` so the
        outlier filter does not spasm.
  - [ ] `G <x> <y> <speed>` navigates to the target and emits `EVT done G`.
  - [ ] `VW 200 0` sets body velocity; watchdog fires `EVT safety_stop` on
        keepalive timeout.
  - [ ] `STOP` halts motors immediately.

---

## SUC-002: Telemetry stream and SNAP

- **Actor**: Python host
- **Preconditions**: Robot is connected. `AppContext.buildTlmFrame` assembles
  frames from `state.inputs`.
- **Main Flow**:
  1. Host sends `STREAM <ms>` to start periodic TLM frames.
  2. `LoopScheduler::run_blocks` calls `robot.telemetryEmit(...)` at the
     configured period while driving (plus grace period after stop).
  3. Host sends `SNAP` for a synchronous single-frame reply at any time.
- **Postconditions**: TLM frames match the pre-refactor format exactly.
- **Acceptance Criteria**:
  - [ ] `STREAM 40` produces periodic `TLM t=... mode=... enc=...` lines
        while driving.
  - [ ] Stream goes silent after the grace period when motors are idle.
  - [ ] `SNAP` returns one frame synchronously at any time (stopped or driving).
  - [ ] `STREAM fields=enc,pose` bitmask still filters fields correctly.

---

## SUC-003: Sensor capability advertisement and reads (OTOS/line/color/port)

- **Actor**: Python host
- **Preconditions**: Sensors initialized via `begin()` in `main()`.
  `is_initialized()` used directly at call sites (no nullable pointer accessors).
- **Main Flow**:
  1. Host sends `ID` to query which sensors are present.
  2. Host sends OTOS verbs (`OI`, `OZ`, `OR`, `OP`, `OV`, `OL`, `OA`) or port
     commands (`P`, `PA`).
  3. `CommandProcessor` checks `robot.otos.is_initialized()` directly and calls
     methods on the sensor object.
- **Postconditions**: Correct `caps=` field in `ID` response; sensor commands
  return data or `ERR nodev` if absent.
- **Acceptance Criteria**:
  - [ ] `ID` response includes `caps=otos,line,color,gripper,portio` when all
        devices are present.
  - [ ] `OI`/`OZ`/`OR`/`OP`/`OV`/`OL`/`OA` succeed when OTOS is present;
        return `ERR nodev` when absent.
  - [ ] `P` and `PA` read/write GPIO ports correctly.
  - [ ] `OTOS` readings appear in TLM `pose=` field after `otosCorrect` runs.

---

## SUC-004: Gripper angle control and query (GRIP)

- **Actor**: Python host
- **Preconditions**: Servo wired and initialized. `Servo::setAngle` records the
  clamped value; `Servo::currentAngle()` returns it.
- **Main Flow**:
  1. Host sends `GRIP <deg>` to set the gripper angle.
  2. `CommandProcessor` calls `robot.gripper.setAngle(clamped)` directly.
  3. Host sends `GRIP` (no arg) to query the current angle.
  4. `CommandProcessor` calls `robot.gripper.currentAngle()`.
- **Postconditions**: Servo moves to the requested angle; query returns the
  actual clamped angle.
- **Acceptance Criteria**:
  - [ ] `GRIP 90` moves the servo to 90°; `GRIP` returns `deg=90`.
  - [ ] `GRIP 200` is clamped to 180°; `GRIP` returns `deg=180`.
  - [ ] Angle is tracked in `Servo` (not in `AppContext` or `CommandProcessor`).

---

## SUC-005: Encoder zeroing and odometry zeroing (ZERO)

- **Actor**: Python host
- **Preconditions**: Robot constructed. `AppContext` exposes `motorController`
  and `odometry` directly.
- **Main Flow**:
  1. Host sends `ZERO enc` — `CommandProcessor` calls
     `robot.motorController.resetEncoderAccumulators()` directly.
  2. Host sends `ZERO pose` — `CommandProcessor` calls
     `robot.odometry.zero(robot.state.inputs)` directly.
- **Postconditions**: Encoder accumulators reset to 0; odometry pose zeroed.
- **Acceptance Criteria**:
  - [ ] `ZERO enc` resets encoder accumulators; subsequent `GET VEL` shows 0.
  - [ ] `ZERO pose` zeros the odometry pose; TLM `pose=0,0,0` on next frame.
  - [ ] `ZERO enc pose` does both.

---

## SUC-006: Cooperative main loop integrity (encoder/PID/sensor cadence)

- **Actor**: Firmware loop (internal)
- **Preconditions**: `AppContext` constructed; `LoopScheduler::run_blocks`
  called.
- **Main Flow**:
  1. Every loop iteration: `controlCollectSplitPhase` reads both encoders
     (M1 first), runs velocity PID, writes PWM.
  2. Timed sensor tasks (`otosCorrect`, `lineRead`, `colorRead`, `portsRead`,
     `telemetryEmit`) fire at their configured periods.
  3. All task entry points now go through `AppContext` member functions (same
     signatures, same behavior as the pre-refactor `Robot` methods).
- **Postconditions**: Loop runs at `controlPeriodMs`; sensor reads do not
  wedge; encoder reads are outlier-filtered.
- **Acceptance Criteria**:
  - [ ] `DBG LOOP` reports control loop still running at configured period.
  - [ ] Encoder outlier-filter logic (scaled `kMaxDeltaMm`, retries) preserved
        verbatim in `AppContext::controlCollectSplitPhase`.
  - [ ] OTOS correction fires at `lagOtosMs` cadence.
  - [ ] No encoder wedge during a 10-minute bench soak at driving speed.

---

## SUC-007: Configuration (SET/GET) and runtime config mutation

- **Actor**: Python host
- **Preconditions**: `RobotConfig` is an owned member of `AppContext`.
  `CommandProcessor` holds `AppContext&`.
- **Main Flow**:
  1. Host sends `SET ml=0.487` — `CommandProcessor` writes through to
     `robot.config` directly.
  2. Host sends `GET ml` — `CommandProcessor` reads from `robot.config`.
  3. SET of `vel.*` or `pid.*` keys triggers `mc.updateVelGains()` /
     `mc.updatePidGains()`.
- **Postconditions**: Config values updated; PID/velocity gains pushed into
  live controllers.
- **Acceptance Criteria**:
  - [ ] All existing SET/GET keys work identically after the refactor.
  - [ ] `SET sTimeout=2000` changes the streaming watchdog timeout live.
  - [ ] `GET` (no args) dumps all registry keys.
