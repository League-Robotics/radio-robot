---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 059 Use Cases

---

## SUC-001: Planner accepts a goal command and emits a twist setpoint

- **Actor**: Robot firmware (message-driven path)
- **Preconditions**: `MotionController2` is constructed; `Drive2::state()` provides
  a valid `DrivetrainState` with a pose estimate.
- **Main Flow**:
  1. Caller calls `planner.apply(PlannerCommand{timed/turn/distance/velocity})`.
  2. Caller calls `planner.tick(now)`.
  3. Planner reads `drive2.state().fused` (pose + twist).
  4. Planner advances its trapezoid/heading/stop-condition logic.
  5. `tick()` returns a `CommandBatch` containing a `DrivetrainCommand{twist}`.
- **Postconditions**: The returned `CommandBatch` holds a non-null `DrivetrainCommand`
  whose `twist` fields encode the velocity setpoint for this tick (vx, omega).
- **Acceptance Criteria**:
  - [ ] `apply()` stages the command without touching hardware.
  - [ ] `tick()` returns a `CommandBatch` with a `DrivetrainCommand{twist}` variant.
  - [ ] The twist sequence for a timed goal matches the expected trapezoidal profile
        (ramp-up, cruise, ramp-down) when `tick()` is called repeatedly in sim.
  - [ ] The twist sequence for a turn goal converges heading error to zero.
  - [ ] When the goal is complete, `planner.state().active` becomes `false`.

---

## SUC-002: Planner can be tested in isolation (no robot, no comms)

- **Actor**: Test engineer
- **Preconditions**: `MotionController2` is constructed with an injected pose
  (from `Drive2` backed by `SimHardware`).
- **Main Flow**:
  1. Test constructs `Drive2` on `SimHardware`.
  2. Test constructs `MotionController2` with a ref to `drive2`.
  3. Test feeds a `PlannerCommand` via `apply()`.
  4. Test calls `tick()` N times.
  5. Test asserts the RETURNED `CommandBatch` of `DrivetrainCommand{twist}`s.
- **Postconditions**: Planner behavior is verified entirely from function return
  values — no mock sink, no bus, no comms stack.
- **Acceptance Criteria**:
  - [ ] `test_planner_subsystem.py` covers timed, turn, and distance goals.
  - [ ] Assertions are on the returned `CommandBatch` twist fields directly.
  - [ ] No full-robot fixture needed; test file is self-contained.
  - [ ] Test runs via `uv run python -m pytest` without side effects.

---

## SUC-003: Command-queue bus drains and routes subsystem CommandBatches

- **Actor**: Robot firmware scheduler (inside `loopTickOnce`)
- **Preconditions**: A subsystem `tick()` has returned a non-empty `CommandBatch`.
- **Main Flow**:
  1. Scheduler calls `drainBus(batch, cmd_processor, drive2, planner, ...)`.
  2. For each `OutCommand` in the batch:
     a. If `priority == true`, route to `CommandQueue::push_front`.
     b. Otherwise route to `CommandProcessor::dequeueOne` or dispatch directly.
  3. A bounded cascade guard (`max_iters=8`) prevents infinite routing loops.
  4. If `max_iters` is exceeded, scheduler emits `EVT bus_overflow` and stops.
- **Postconditions**: All routable commands are dispatched; safety/STOP commands
  arrive at the front of the queue; the tick budget is respected.
- **Acceptance Criteria**:
  - [ ] A `DrivetrainCommand{twist}` returned by `Planner::tick()` is routed to
        `drive2.apply()` in the same tick.
  - [ ] An `OutCommand` with `priority=true` is routed via `push_front`.
  - [ ] Cascade depth exceeding `max_iters` emits an EVT and terminates gracefully.
  - [ ] Existing suite passes (golden-TLM canary unaffected).

---

## SUC-004: Each subsystem is configured at construction time from typed projections

- **Actor**: Robot firmware init sequence (in `main.cpp` / `Robot` constructor)
- **Preconditions**: `RobotConfig` is loaded; subsystems are constructed.
- **Main Flow**:
  1. `Drive2::configure(toDriveConfig(cfg))` is called after `Drive2` is constructed.
  2. `Sensors::configure(toSensorsConfig(cfg).line, toSensorsConfig(cfg).color)` is
     called after `Sensors` is constructed.
  3. `MotionController2::configure(toPlannerConfig(cfg))` is called after the
     Planner is constructed.
- **Postconditions**: Each subsystem uses its typed config slice; parameters are
  consistent with `RobotConfig`; no field is silently ignored.
- **Acceptance Criteria**:
  - [ ] `toPlannerConfig(RobotConfig)` maps `aMax`, `vBodyMax`, `yawRateMax`, and
        the motion-limits subset correctly.
  - [ ] `configure()` is called on each subsystem in the init path before the loop.
  - [ ] `python build.py --clean` zero errors.

---

## SUC-005: Live SET command routes to the owning subsystem

- **Actor**: Operator (via host SET command)
- **Preconditions**: Robot is running the ordered tick; `subsystem:` annotation
  is present on `robot_config.schema.json` fields.
- **Main Flow**:
  1. Operator sends `SET vel.kP 1.2` over the wire.
  2. `handleSet` parses the key, finds `vel.kP` annotated with `subsystem: drive`.
  3. `handleSet` calls `drive2.configure(delta)` with the changed field.
  4. Drive2 applies the delta on the next tick without requiring a robot restart.
- **Postconditions**: The changed field is live in the subsystem; the old
  `MotorController::updateVelGains` path is generalized, not forked.
- **Acceptance Criteria**:
  - [ ] Setting `vel.kP` via SET routes to `drive2.configure()`.
  - [ ] Setting a planner motion limit via SET routes to `planner.configure()`.
  - [ ] Setting a sensor lag via SET routes to `sensors.configure()`.
  - [ ] Unknown subsystem annotation falls back gracefully (no crash).

---

## SUC-006: Live loop runs the ordered tick with sense-before-actuate preserved

- **Actor**: Robot firmware main loop
- **Preconditions**: Phase 2 subsystems (`Drive2`, `Sensors`) and Planner
  (`MotionController2`) are constructed and configured; bus drain is wired.
- **Main Flow**:
  1. `loopTickOnce` calls comms drain (serial/radio → parse → enqueue).
  2. `Drive2::tickUpdate(now)` — sense: encoders, OTOS, fusion.
  3. Bus drain+route: user motion verbs → `planner.apply()`; emitted
     `DrivetrainCommand`s → `drive2.apply()`.
  4. `Planner::tick(now)` — advance goal, return `CommandBatch`.
  5. Bus drain: route planner's batch → `drive2.apply()`.
  6. `Drive2::tickAction(now)` — kinematics → wheel PID → motor output.
  7. `Sensors::tick(now)` — line/color reads.
  8. Telemetry emit from subsystem state snapshots.
- **Postconditions**: Sense-before-actuate order is preserved; split-phase encoder
  M1-before-M2 order is preserved; safety `push_front` is honored; the loop is
  bounded per tick.
- **Acceptance Criteria**:
  - [ ] VW command walks end-to-end through the new tick with byte-plausible parity.
  - [ ] TURN command walks end-to-end through the new tick with byte-plausible parity.
  - [ ] `tickUpdate` is called before `tickAction` every tick.
  - [ ] Bench smoke on tovez confirms telemetry + on-stand spin parity.
