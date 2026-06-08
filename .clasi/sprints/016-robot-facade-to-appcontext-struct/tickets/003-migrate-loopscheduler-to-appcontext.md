---
id: '003'
title: Migrate LoopScheduler to AppContext
status: open
use-cases:
  - SUC-001
  - SUC-002
  - SUC-006
depends-on:
  - '002'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Migrate LoopScheduler to AppContext

## Description

Migrate `LoopScheduler` from `Robot&` to `AppContext&`. This is the smallest
caller (~18 sites) and exercises all eight task entry points, making it a
good early validation of the AppContext path before migrating the larger
`CommandProcessor`.

After this ticket, `LoopScheduler` and `run_blocks` drive the firmware through
`AppContext` instead of `Robot`. The `robot` variable in `main.cpp` is switched
from `Robot` to `AppContext`. `CommandProcessor` still holds a `Robot&` (it has
not been migrated yet), but it is no longer the main execution path.

**Critical ordering concern**: `CommandProcessor` holds a `Robot& _robot`. If
`main.cpp` removes the `Robot` instance before `CommandProcessor` is migrated,
the linker will break. The approach in this ticket: keep the old `Robot robot`
AND the new `AppContext appCtx` both alive in `main.cpp`, switch the
`LoopScheduler` and the `setI2CBus`/`setEvtSink` post-wiring calls to use
`appCtx`, but pass the old `robot` to `CommandProcessor` unchanged. This
avoids a flag-day switch. (The temporary `appCtx` from T002 is promoted here
to the primary `LoopScheduler` input.)

### LoopScheduler.h changes

1. Replace `class Robot;` forward declaration with `struct AppContext;`.
2. Replace `Robot& _robot` with `AppContext& _robot`.
3. Replace `Robot& robot, CommandProcessor& cmd, ...` constructor parameters:
   `LoopScheduler(AppContext& robot, CommandProcessor& cmd, Communicator& comm, MicroBit& uBit)`.
4. Replace `Robot& robot()` accessor return type with `AppContext& robot()`.

### LoopScheduler.cpp changes

The include changes from `#include "Robot.h"` to `#include "AppContext.h"`.

Task function substitutions (all eight `run*` static functions and
`controlCollect`, `run_tasks`, `run_all`, `run_blocks`):

| Old call | New call |
|----------|----------|
| `sched.robot().driveAdvance(now)` | `{ AppContext& r = sched.robot(); r.driveController.driveAdvance(r.state.inputs, r.state.commands, r.state.target, now); }` |
| `sched.robot().odometryPredict()` | `{ AppContext& r = sched.robot(); r.odometry.predict(r.state.inputs, r.config.trackwidthMm); }` |
| `sched.robot().otosCorrect(now)` | `sched.robot().otosCorrect(now)` (kept method — unchanged) |
| `sched.robot().lineRead()` | `sched.robot().lineRead()` (kept method) |
| `sched.robot().colorRead()` | `sched.robot().colorRead()` (kept method) |
| `sched.robot().portsRead()` | `sched.robot().portsRead()` (kept method) |
| `sched.robot().telemetryEmit(now, fn, ctx)` | `sched.robot().telemetryEmit(now, fn, ctx)` (kept method) |
| `_robot.config()` | `_robot.config` |
| `_robot.config().controlPeriodMs` | `_robot.config.controlPeriodMs` |
| `_robot.controlCollectSplitPhase(now, wheel)` | `_robot.controlCollectSplitPhase(now, wheel)` (kept method — unchanged) |

In `run_blocks` the config period sync block:
```cpp
// Old:
const RobotConfig& cfg = _robot.config();
_table[3].periodMs = cfg.lagOtosMs; ...
// New:
const RobotConfig& cfg = _robot.config;
_table[3].periodMs = cfg.lagOtosMs; ...
```

### main.cpp changes

Promote `appCtx` to be the primary scheduler input. Remove the `(void)appCtx`
suppression from T002. Change the LoopScheduler and post-wiring calls:

```cpp
// T002 temporary appCtx is now the primary AppContext — remove the (void) cast:
static AppContext appCtx(motorL, motorR, otos, line, color, gripper, portio, cfg);

// LoopScheduler now takes AppContext:
static LoopScheduler sched(appCtx, cmd, comm, uBit);
cmd.setScheduler(&sched);
cmd.setI2CBus(&bus);

// Post-wiring: motorController is now a direct member:
appCtx.motorController.setI2CBus(&bus);
appCtx.motorController.setEvtSink(&sched.activeFn, &sched.activeCtx);
```

The old `Robot robot(...)` and its `cmd(robot)` wiring remain until T005
(CommandProcessor migration). The `cmd` still holds `Robot& _robot`.

## Acceptance Criteria

- [ ] `LoopScheduler.h` uses `AppContext&` for the robot reference; no
      `class Robot;` forward declaration remains.
- [ ] `LoopScheduler.cpp` includes `AppContext.h` (not `Robot.h`).
- [ ] All eight task functions (`runCommsIn`, `runDriveAdvance`,
      `runOdometryPredict`, `runOtosCorrect`, `runLineRead`, `runColorRead`,
      `runPortsRead`, `runTelemetryEmit`) compile against `AppContext`.
- [ ] `run_blocks` in `LoopScheduler.cpp` drives through AppContext for all
      control/sensor/telemetry tasks.
- [ ] `main.cpp` passes `appCtx` to `LoopScheduler`; post-wiring uses
      `appCtx.motorController` directly.
- [ ] `Robot robot(...)` still exists in `main.cpp` and is passed to
      `CommandProcessor cmd(robot)` — CommandProcessor not yet migrated.
- [ ] Clean build: `python3 build.py` passes.
- [ ] Host unit tests pass: `uv run --with pytest python -m pytest`.
- [ ] On-robot smoke test: `PING` responds; `S 200 200` drives (via AppContext
      control path); `STOP` halts. Telemetry stream works (`STREAM 40`).

## Implementation Plan

**Approach**: Change `LoopScheduler`'s held reference type; update all
call sites inside its task functions. Promote the T002 `appCtx` to primary.

**Files to modify**:
- `source/control/LoopScheduler.h` — type swap, constructor signature
- `source/control/LoopScheduler.cpp` — include swap, ~18 call sites
- `source/main.cpp` — promote appCtx, update sched + post-wiring

**Files NOT to touch**: `Robot.h`, `Robot.cpp`, `CommandProcessor.h/.cpp`,
`WedgeTest.h/.cpp`, `AppContext.h/.cpp` (already correct from T002).

**Testing plan**:
- `python3 build.py` — clean build.
- `uv run --with pytest python -m pytest` — no regressions.
- Flash to robot (`python3 build.py --clean <target>`); PING liveness check;
  `S 200 200` drives; `STOP` halts; `STREAM 40` produces TLM frames.

**Documentation updates**: None required.
