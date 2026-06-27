---
id: '002'
title: Add AppContext struct and wire in main.cpp
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
- SUC-007
depends-on:
- '001'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Add AppContext struct and wire in main.cpp

## Description

Create `source/robot/AppContext.h` and `source/robot/AppContext.cpp` as new
files alongside the existing `Robot.h`/`Robot.cpp` (which remain untouched
until Ticket 006). Wire `AppContext` into `main.cpp` as a second parallel
construction to confirm the struct compiles and the two required binds work,
without yet removing `Robot` or changing any callers.

The approach is deliberately incremental: `main.cpp` constructs both `robot`
(Robot) and a temporary `AppContext appCtx(...)` just to prove it builds.
The `appCtx` is unused at this stage. Callers still go through the `Robot`
path. This validates the struct definition, constructor, and init order before
any caller migration begins.

At the end of this ticket: the firmware builds and the robot runs identically
to before; `AppContext` exists but is dead code in the binary.

### AppContext.h

New file: `source/robot/AppContext.h`

Required includes: `Config.h`, `Motor.h`, `OtosSensor.h`, `LineSensor.h`,
`ColorSensor.h`, `Servo.h`, `PortIO.h`, `MotorController.h`, `Odometry.h`,
`DriveController.h`, `RobotState.h`, `Protocol.h`.

Member declaration order (load-bearing for C++ init order):

```cpp
struct AppContext {
    RobotConfig         config;
    RobotStateContainer state;
    Motor&              motorL;
    Motor&              motorR;
    OtosSensor&         otos;
    LineSensor&         line;
    ColorSensor&        color;
    Servo&              gripper;
    PortIO&             portio;
    MotorController     motorController;
    Odometry            odometry;
    DriveController     driveController;

    AppContext(Motor& mL, Motor& mR, OtosSensor& o, LineSensor& l,
               ColorSensor& c, Servo& g, PortIO& p,
               const RobotConfig& cfg);

    void controlCollectSplitPhase(uint32_t now_ms, int pendingWheel);
    void otosCorrect(uint32_t now_ms);
    void lineRead();
    void colorRead();
    void portsRead();
    void distanceDrive(int32_t l, int32_t r, int32_t targetMm,
                       ReplyFn fn, void* ctx, const char* corr_id = nullptr);
    int  buildTlmFrame(char* buf, int len);
    void telemetryEmit(uint32_t now_ms, ReplyFn fn, void* ctx);
    uint32_t systemTime() const;

    uint32_t _lastTlmMs     = 0;
    uint32_t _lastActiveMs  = 0;
    uint32_t _lastControlMs = 0;
    bool     _prevDriving   = false;
};
```

If the compiler warns about `color` shadowing a macro, rename to `colorSensor`
throughout AppContext.h and AppContext.cpp.

### AppContext.cpp

New file: `source/robot/AppContext.cpp`

Constructor initializer list must match declaration order:

```cpp
AppContext::AppContext(Motor& mL, Motor& mR, OtosSensor& o, LineSensor& l,
                       ColorSensor& c, Servo& g, PortIO& p,
                       const RobotConfig& cfg)
    : config(cfg),
      state(defaultInputs(cfg)),
      motorL(mL), motorR(mR),
      otos(o), line(l), color(c), gripper(g), portio(p),
      motorController(motorL, motorR, config),
      odometry(),
      driveController(motorController, odometry, config)
{
    driveController.setHardwareState(&state.inputs);
    motorController.setCommandsRef(&state.commands);
}
```

Kept method bodies: copy verbatim from `Robot.cpp` with mechanical member-name
substitutions (`_otos` -> `otos`, `_mc` -> `motorController`, `_dc` ->
`driveController`, `_odo` -> `odometry`, `_state` -> `state`, `_config` ->
`config`, etc.).

`AppContext::otosCorrect` is the exception: shrink it to use
`otos.readTransformed(config)` from Ticket 001 instead of the inlined LSB
math, and drop the `kOtosSlowMs` cadence gate (run_blocks handles that):

```cpp
void AppContext::otosCorrect(uint32_t now_ms) {
    if (!otos.is_initialized()) return;
    OtosPose p = otos.readTransformed(config);
    state.inputs.otosX = p.x;
    state.inputs.otosY = p.y;
    state.inputs.otosH = p.h;
    state.inputs.otos.lastUpdMs = now_ms;
    state.inputs.otos.valid     = true;
    odometry.correct(state.inputs, p.x, p.y, p.h,
                     config.alphaPos, config.alphaYaw, config.otosGate);
}
```

`AppContext::distanceDrive` must preserve the encoder-reset workaround:
```cpp
void AppContext::distanceDrive(int32_t l, int32_t r, int32_t targetMm,
                                ReplyFn fn, void* ctx, const char* corr_id) {
    driveController.beginDistance((float)l, (float)r, targetMm,
                                   systemTime(), state.target, fn, ctx, corr_id);
    state.inputs.encLMm = 0.0f;
    state.inputs.encRMm = 0.0f;
}
```

### main.cpp change

Add a temporary AppContext construction immediately after the Robot
construction:

```cpp
static Robot     robot(motorL, motorR, otos, line, color, gripper, portio, comm, cfg);
// T002: AppContext validation — unused until T003-T005 migrate callers.
static AppContext appCtx(motorL, motorR, otos, line, color, gripper, portio, cfg);
(void)appCtx;  // suppress unused-variable warning
```

No callers are changed. LoopScheduler, CommandProcessor, and all task lambdas
still use `robot` (the Robot type).

## Acceptance Criteria

- [x] `source/robot/AppContext.h` and `source/robot/AppContext.cpp` exist.
- [x] AppContext compiles cleanly alongside Robot (both present in the build).
- [x] Constructor initializer-list order matches member declaration order with
      no compiler warnings about initialization order.
- [x] `AppContext::otosCorrect` uses `otos.readTransformed(config)` from T001.
- [x] `AppContext::distanceDrive` contains the encoder-reset workaround
      (`state.inputs.encLMm = 0; state.inputs.encRMm = 0;` after `beginDistance`).
- [x] `main.cpp` constructs both `robot` (Robot) and `appCtx` (AppContext).
- [x] Clean build: `python3 build.py` passes with no new errors.
- [x] Host unit tests pass: `uv run --with pytest python -m pytest`.
- [x] Robot behavior unchanged — all callers still go through Robot.
      (STAKEHOLDER-DEFERRED: on-robot PING verification — bench flash not performed per ticket instructions)

## Implementation Plan

**Approach**: Create new files; do not delete or modify Robot.h/Robot.cpp.
Add a temporary `(void)appCtx` in main.cpp for build validation only.

**Files to create**:
- `source/robot/AppContext.h`
- `source/robot/AppContext.cpp`

**Files to modify**:
- `source/main.cpp` — add temporary `static AppContext appCtx(...)` after robot

**Files NOT to touch**: `Robot.h`, `Robot.cpp`, `CommandProcessor`,
`LoopScheduler`, `WedgeTest`.

**Testing plan**:
- `python3 build.py` — clean build with both Robot and AppContext present.
- `uv run --with pytest python -m pytest` — no regressions.
- Optionally flash to robot; confirm `PING` responds (Robot path unchanged).

**Documentation updates**: None required.
