---
status: in-progress
sprint: '016'
tickets:
- 016-001
- 016-006
---

# Replace the Robot facade class with an open AppContext struct

## Problem / motivation

`Robot` (`source/robot/Robot.h` / `.cpp`) is a leaky facade. Of its ~38 methods,
~43% are pure passthroughs to `MotorController`/`Odometry`/`DriveController`/sensors,
~28% are trivial getters, and only ~29% carry real logic. Every subsystem it
"owns" is a singleton — there is exactly one robot — so the encapsulation buys
nothing and forces callers (`CommandProcessor`, `LoopScheduler`, `WedgeTest`)
through delegation methods and nullable-pointer accessors instead of touching the
subsystems directly.

Replace `Robot` with an **open `struct AppContext`** (single global instance
`robot`) whose members are public. Delete pure passthroughs so callers reach
subsystems directly (`robot.driveController.stop(...)`). Keep genuine cross-cutting
orchestration as plain `AppContext` member functions. Push device-specific logic
down into the HAL class that owns it. Outcome: less indirection, no facade to
maintain, subsystems directly reachable.

## Scope decisions (confirmed with stakeholder)

- **Wiring = AppContext constructor (lower risk).** `MotorController`, `Odometry`,
  `DriveController` keep their existing constructors and reference members
  untouched. `AppContext` gets a constructor whose initializer-list binds deps in
  dependency order (the same wiring Robot does today, relocated).
- **Move device logic into subsystems.** OTOS raw-read + LSB→mm transform → new
  `OtosSensor::readTransformed(const RobotConfig&)`; gripper angle clamp/tracking →
  `Servo` (`setAngle` records the clamped value, add `currentAngle()`).
- **Drop `Communicator` from the struct** — `robot.comm()` has zero callers
  (LoopScheduler/CommandProcessor hold their own `Communicator&`). `comm` stays a
  `main()` static passed to those two as today.
- HAL devices (`Motor`×2, `OtosSensor`, `LineSensor`, `ColorSensor`, `Servo`,
  `PortIO`) stay external statics in `main.cpp` bound to `AppContext` reference
  members — they need `motorId`/`fwdSign`/pins and cannot self-construct.

## The struct

New files `source/robot/AppContext.h` / `AppContext.cpp` (replacing `Robot.h`/`.cpp`).
Member declaration order is load-bearing (must match init order; the sketch order
already satisfies it):

```cpp
struct AppContext {
    RobotConfig         config;            // owned copy; runtime SET mutates it
    RobotStateContainer state;             // = defaultInputs(config)
    MotorController      motorController;   // (motorL, motorR, config)
    Odometry             odometry;          // default ctor
    DriveController      driveController;   // (motorController, odometry, config)
    Motor&      motorL;  Motor&      motorR;
    OtosSensor& otos;    LineSensor& line;   ColorSensor& color;
    Servo&      gripper; PortIO&     portio;

    AppContext(Motor& mL, Motor& mR, OtosSensor& o, LineSensor& l,
               ColorSensor& c, Servo& g, PortIO& p);

    // Kept orchestration (genuine cross-cutting logic):
    void controlCollectSplitPhase(uint32_t now_ms, int pendingWheel);
    void otosCorrect(uint32_t now_ms);
    void lineRead();  void colorRead();  void portsRead();
    void distanceDrive(int32_t l, int32_t r, int32_t targetMm,
                       ReplyFn fn, void* ctx, const char* corr_id);
    int  buildTlmFrame(char* buf, int len);
    void telemetryEmit(uint32_t now_ms, ReplyFn fn, void* ctx);
    uint32_t systemTime() const;

    // Telemetry/control gating state that pairs with the kept logic:
    uint32_t _lastTlmMs = 0, _lastActiveMs = 0, _lastControlMs = 0;
    bool     _prevDriving = false;
};
```

Constructor body does the two binds Robot does today:
`driveController.setHardwareState(&state.inputs); motorController.setCommandsRef(&state.commands);`

The instance is a `main()`-local `static AppContext robot{...}` constructed after
the HAL statics; passed by reference everywhere (no global pointer needed).

## Method disposition

**KEEP as `AppContext` member functions** (cross-cutting orchestration over
multiple subsystems + state): `controlCollectSplitPhase`, `otosCorrect` (now thin),
`lineRead`, `colorRead`, `portsRead`, `distanceDrive` (retains the encoder-reset
workaround `state.inputs.encLMm/encRMm = 0` after `beginDistance`), `buildTlmFrame`,
`telemetryEmit`, `systemTime`.

**MOVE into subsystems:**
- `OtosSensor` gains `OtosPose readTransformed(const RobotConfig&) const` holding
  the LSB→mm/rad constants, upside-down flip, and mounting-offset rotation
  (currently inline in `Robot::otosCorrect`). `AppContext::otosCorrect` shrinks to:
  presence check → `readTransformed` → stamp `state.inputs.otos*` → `odometry.correct(...)`.
- `Servo` gains angle tracking: `setAngle()` records the clamped value; add
  `int16_t currentAngle() const`. Absorbs `Robot::setGripperAngle`/`gripperAngle`
  and the `_currentGripperAngle` field. `_gripperPresent` (hard-coded true) dropped.

**INLINE-DELETE** (delete method; caller hits the subsystem directly):
`stop`, `streamDrive`, `velocityDrive`, `timedDrive`, `goTo` → `robot.driveController.beginX(...)`;
`odometryPredict` → `robot.odometry.predict(robot.state.inputs, robot.config.trackwidthMm)`;
`driveAdvance` → `robot.driveController.driveAdvance(robot.state.inputs, robot.state.commands, robot.state.target, now)`;
`controlFireRequest` → `robot.motorL/R.requestEncoder()`;
`zeroEncoders` → `robot.motorController.resetEncoderAccumulators()`;
`zeroOdometry` → `robot.odometry.zero(robot.state.inputs)`;
`getEncoders` → `robot.motorController.getEncoderPositions(l, r)` (drop `EncoderReading`);
all accessors (`state/stateMut/config/comm/motor/driveController/odometry/portIO`) → direct member access.
Nullable device accessors (`otos()/lineSensor()/colorSensor()/servo()`) → replace with
`robot.<dev>.is_initialized()` checks at the call site.

**DELETE outright (dead code):** `controlCollect` (synchronous stub, no callers),
`noteActivity` (no callers), `setPose`/`getPose` (no callers in tree — verify with
grep before removing), `_lastOtosMs`/`kOtosSlowMs` (redundant cadence gate;
`run_blocks` already gates OTOS via `cfg.lagOtosMs`).

## Files to modify

- **`source/robot/AppContext.h` + `.cpp`** (new, replacing `Robot.h`/`.cpp`): struct,
  constructor, kept orchestration functions.
- **`source/hal/OtosSensor.h` + `.cpp`**: add `OtosPose readTransformed(const RobotConfig&)`.
- **`source/hal/Servo.h` + `.cpp`**: record clamped angle in `setAngle`, add `currentAngle()`.
- **`source/main.cpp`**: construct `static AppContext robot{motorL, motorR, otos, line,
  color, gripper, portio};` after the HAL statics; keep existing ordering (uBit.init →
  cfg → bus + 100kHz → HAL devices → comm.begin → 2.5s settle → sensor begin()s →
  robot → cmd → sched → `setI2CBus`/`setEvtSink` post-wiring → `run_blocks`).
- **`source/app/CommandProcessor.h` + `.cpp`** (~55–60 sites): `class Robot;` →
  `struct AppContext;`; `Robot& _robot` → `AppContext& _robot`; rewrite drive verbs
  (S/T/D/G/VW/STOP), OTOS verbs (OI/OZ/OR/OP/OV/OL/OA) to use `is_initialized()` +
  direct device calls, GRIP→`robot.gripper`, ZERO→`robot.motorController/odometry`,
  accessor swaps.
- **`source/control/LoopScheduler.h` + `.cpp`** (~18 sites): type/forward-decl swap;
  task wrappers + `run_blocks` call kept functions directly or inline the deleted ones.
- **`source/app/WedgeTest.h` + `.cpp`** (~5 sites): forward-decl/`AppContext*` swap;
  `motor()`→`motorController`, `getEncoders()`→`getEncoderPositions(l,r)`.

## Suggested sequencing (lowest risk first)

1. Pure additions: `OtosSensor::readTransformed`, `Servo::currentAngle`/`setAngle` tracking.
2. Add `AppContext.h/.cpp` with members + kept functions; wire in `main.cpp` preserving init order + the two binds.
3. Migrate `LoopScheduler` (smallest caller, exercises kept task entries).
4. Migrate `WedgeTest` (5 sites).
5. Migrate `CommandProcessor` verb-by-verb (largest).
6. Delete `Robot.h`/`.cpp` and dead methods once nothing references them.

## Acceptance / verification

1. **Clean build** (stale incremental builds flash broken binaries): `python3 build.py --clean <target>`.
2. **Unit tests**: `uv run --with pytest python -m pytest`.
3. **Bench (robot on stand, safe to drive)** via `uv run rogo ...`:
   - PING/ID liveness preflight; confirm `caps=` still lists otos/line/color/gripper correctly (the `is_initialized()` rewrite).
   - Drive verbs: S, T, D (D must reach distance without spasm — encoder-reset workaround), G, VW, STOP.
   - GRIP set + query (angle tracking moved to Servo).
   - ZERO (encoders + odometry), OTOS verbs (OR/OP/OV/OL/OA), PORT (P/PA).
   - Telemetry: `STREAM` then confirm a clean TLM stream, and `SNAP`.
4. Confirm encoders + wheels + sensors all read on the bench (standing acceptance gate).
