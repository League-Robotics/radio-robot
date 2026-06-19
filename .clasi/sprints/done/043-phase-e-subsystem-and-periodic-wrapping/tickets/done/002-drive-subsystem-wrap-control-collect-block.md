---
id: '002'
title: 'Drive subsystem: wrap CONTROL COLLECT block'
status: done
use-cases:
- SUC-002
- SUC-004
depends-on:
- 043-001
github-issue: ''
issue: migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 043-002: Drive subsystem: wrap CONTROL COLLECT block

## Description

Create `source/subsystems/drive/Drive.{h,cpp}` and move the CONTROL COLLECT block from
`loopTickOnce` into `Drive::periodic()`, producing a single one-liner call in the loop.

`Drive` wraps the per-wheel velocity control + encoder-filter concern:
- `periodic()` — contains the outlier-filter pass, `motorController.controlTick()` call,
  and wedge-push into `PhysicalStateEstimate`, verbatim from the CONTROL COLLECT block.
- `updateInputs()` — called internally at the same positions the encoder writes
  (`state.inputs.encLMm`, `encRMm`) occur in the existing block.

The five filter-streak members currently on `Robot` (`_filterRejectStreakL/R`,
`_prevDriving`, `_prevAnyWedged`, `_lastControlMs`) move to `Drive` value members.
`Robot.h` gains a `Drive drive` value member declared after `motorController` and
`estimate` (which `Drive` holds references to).

Depends on 043-001 because the sensor subsystem tickets establish the `source/subsystems/`
directory and CMakeLists wiring; Drive can build on that foundation.

## Acceptance Criteria

- [x] `source/subsystems/drive/Drive.{h,cpp}` exist and compile.
- [x] `Drive` has `periodic()` and `updateInputs()`.
- [x] `Drive` holds: `MotorController&`, `PhysicalStateEstimate&`, `HardwareState&`,
      `const RobotConfig&`, plus the five filter-streak value members.
      (Per architecture-update.md verbatim-move requirement, Drive ALSO holds
      `IMotor& motorL`, `IMotor& motorR`, and `MotorCommands&` — the exact lvalues
      the inline block reached via `r.motorL`/`r.motorR`/`r.state.commands`; these
      are required for byte-exactness and are not behavior changes. Annotated honestly.)
- [x] `loopTickOnce` CONTROL COLLECT block (~lines 26-129) replaced by
      `robot.drive.periodic(now, robot._tlmBoundFn, robot._tlmBoundCtx);` — one
      call, in the SAME position (before `cmd.dequeueOne(queue)`). The fn/ctx
      params thread the Robot TLM sink so the EVT enc_filter_hold emission is
      byte-identical (architecture-update.md OQ-2).
- [x] `Robot.h` adds `Drive drive` value member, declared AFTER `motorController` and `estimate`.
- [x] `Robot.h` removes `_filterRejectStreakL`, `_filterRejectStreakR`, `_prevDriving`,
      `_prevAnyWedged`, `_lastControlMs` members (grep confirmed no external accesses — OQ-1).
- [x] `Robot.cpp` constructor init-list wires `drive` with the appropriate refs.
- [x] No CODAL/MicroBit/I2CBus types in `source/subsystems/drive/`.
- [x] No `printf` / `telemetryEmit` calls inside `Drive` methods. (The EVT path uses
      `snprintf` into a local buffer then calls the bound `fn` — verbatim from the
      original block; neither `printf` nor `telemetryEmit`.)
- [x] Simulation tier green: `uv run --with pytest python -m pytest -q` -> 2001 passed, 0 errors.
- [x] Golden-TLM canary byte-exact (`test_golden_tlm.py` passed).
- [x] `defaultRobotConfig()` field-pin diff empty (`test_default_config_pin.py` passed).
- [x] ARM firmware build gate: `python3 build.py --fw-only` -> 0 errors, `MICROBIT.hex`
      produced; then `git checkout -- source/robot/DefaultConfig.cpp` (restored).
- [x] Behavior-preservation fences green:
      `test_033_005_wedge_hardening.py`, `test_incident_scenarios.py`,
      `test_goto_bounds.py`, `test_watchdog_exemption.py`,
      `test_ekf*.py`, `test_otos_fusion.py`,
      Phase-B estimator/plant tests, motion/VW tests. (175 passed in the
      wedge/incident/goto/watchdog/ekf/otos/filter/streak fence selection.)

## Implementation Plan

### Approach

Verbatim move of the CONTROL COLLECT block (~100 lines, `loopTickOnce` lines 26-129)
into `Drive::periodic()`. The five filter-streak members move from `Robot` to `Drive`.
No numeric or ordering changes.

### Files to Create

**`source/subsystems/drive/Drive.h`**

```cpp
#pragma once
#include "MotorController.h"        // source/control/ or source/superstructure/ via shim
#include "PhysicalStateEstimate.h"  // source/state/
#include "RobotState.h"             // HardwareState
#include "Config.h"                 // RobotConfig
#include <stdint.h>

class Drive {
public:
    Drive(MotorController& mc, PhysicalStateEstimate& est,
          HardwareState& inputs, const RobotConfig& cfg);

    void updateInputs();   // writes encLMm/R; called internally from periodic()
    void periodic();       // runs outlier filter + controlTick + wedge push

    // Expose streak counters for EVT emission (currently accessed from loopTickOnce context).
    // After the move, the EVT emission stays inside periodic() — these are not needed externally.

private:
    MotorController&       _mc;
    PhysicalStateEstimate& _est;
    HardwareState&         _inputs;
    const RobotConfig&     _cfg;

    // Filter-streak state (moved from Robot)
    uint8_t  _filterRejectStreakL;
    uint8_t  _filterRejectStreakR;
    bool     _prevDriving;
    bool     _prevAnyWedged;
    uint32_t _lastControlMs;

    // TLM bound fn/ctx for EVT enc_filter_hold emission — pointer to Robot members.
    // Drive needs access to _tlmBoundFn/_tlmBoundCtx (still on Robot) for the EVT.
    // Option: pass ReplyFn** and void*** in constructor, or make them parameters to periodic().
    // See OQ-1 in architecture-update.md — resolved by passing the Robot TLM sink pointers.
    ReplyFn*  _tlmFn;
    void*     _tlmCtx;
};
```

Note on TLM sink: The CONTROL COLLECT block emits `EVT enc_filter_hold` via
`r._tlmBoundFn` / `r._tlmBoundCtx`. These live on `Robot`. `Drive` should receive them
at construction time (or as parameters to `periodic()` — simpler). The programmer should
choose the approach that requires the fewest changes: passing `ReplyFn* fn, void* ctx`
as parameters to `periodic()` is simplest and avoids storing live pointers.

**`source/subsystems/drive/Drive.cpp`**
- `periodic()`: verbatim CONTROL COLLECT block body from `loopTickOnce` lines 26-129,
  with `r.` replaced by `_` members and `r._tlmBoundFn`/`r._tlmBoundCtx` passed as
  parameters (or pulled from stored refs).
- `updateInputs()`: the encoder write lines inside the outlier filter (these are
  already inlined into the block; `updateInputs` can be a no-op distinct method or
  simply be the conceptual seam documented for Phase F — the writes already live inside
  `periodic()`).

### Files to Modify

**`source/control/LoopTickOnce.cpp`**
- Replace the entire CONTROL COLLECT block (lines 26-129 plus the surrounding braces)
  with a single call: `robot.drive.periodic();`
  (Pass `robot._tlmBoundFn, robot._tlmBoundCtx` if those are parameters to `periodic()`.)

**`source/robot/Robot.h`**
- Add `#include "subsystems/drive/Drive.h"`.
- Add `Drive drive;` value member — must appear AFTER `motorController` and `estimate`
  (both are refs that `Drive` holds).
- Remove `_filterRejectStreakL`, `_filterRejectStreakR`, `_prevDriving`,
  `_prevAnyWedged`, `_lastControlMs` member declarations.
  Grep first: `grep -rn "_filterRejectStreak\|_prevDriving\|_prevAnyWedged\|_lastControlMs" source/ tests/`

**`source/robot/Robot.cpp`**
- Add `drive(motorController, estimate, state.inputs, config)` to constructor init-list.
- Remove the former filter-streak member initializations from the constructor (if any
  explicit initializations exist outside the CONTROL COLLECT block).

### Testing Plan

1. Build after creating `Drive.h` / `Drive.cpp` before touching `loopTickOnce`:
   `python3 build.py --fw-only` — expect linker success.
2. After `loopTickOnce` repoint: run full simulation suite:
   `uv run --with pytest python -m pytest -q`
3. Run wedge-hardening fence specifically:
   `uv run --with pytest python -m pytest tests/simulation/ -k "wedge" -v`
4. Run golden-TLM canary. This is the highest-signal check — any ordering change
   shows up immediately as a frame mismatch.
5. Run field-pin check.
6. Run full behavior fence suite:
   `uv run --with pytest python -m pytest tests/simulation/ -k "wedge or incident or goto_bounds or watchdog or ekf or otos_fusion" -v`

### Documentation Updates

`architecture-update.md` already documents this change. No additional doc updates.
