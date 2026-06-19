---
id: '001'
title: 'Sensor subsystems: LineSensor, ColorSensor, Ports'
status: done
use-cases:
- SUC-001
- SUC-004
depends-on: []
github-issue: ''
issue: migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 043-001: Sensor subsystems: LineSensor, ColorSensor, Ports

## Description

Create the three sensor subsystem classes under `source/subsystems/sensors/` —
`LineSensor`, `ColorSensor`, and `Ports` — and repoint `loopTickOnce` to call each
subsystem's `periodic()` instead of the inline timed blocks that currently call
`robot.lineRead()` / `robot.colorRead()` / `robot.portsRead()`.

Each subsystem:
- holds references to its HAL device interface, `HardwareState`, and `const RobotConfig&`
- exposes `updateInputs(uint32_t now)` — the verbatim body of the former `Robot::lineRead()`
  / `colorRead()` / `portsRead()`, with `systemTime()` replaced by the `now` parameter
- exposes `periodic(LoopTickState& ts, uint32_t now)` — the verbatim lag-gate + timer-update
  wrapper from the inline timed block in `loopTickOnce`

After this ticket, the LINE/COLOUR/PORTS sections of `loopTickOnce` are three one-liners.
`Robot::lineRead()` / `colorRead()` / `portsRead()` declarations are deleted (grep first to
confirm no other callers). The `Robot` struct gains three value members. This ticket does NOT
touch the CONTROL COLLECT block (Drive, ticket 002) or the OTOS block.

## Acceptance Criteria

- [x] `source/subsystems/sensors/LineSensor.{h,cpp}` exist and compile.
- [x] `source/subsystems/sensors/ColorSensor.{h,cpp}` exist and compile.
- [x] `source/subsystems/sensors/Ports.{h,cpp}` exist and compile.
- [x] Each class has `periodic(LoopTickState& ts, uint32_t now)` and `updateInputs(uint32_t now)`.
- [x] `loopTickOnce` LINE/COLOUR/PORTS inline blocks replaced by one-liner subsystem calls
      in the SAME ORDER as today (LINE before COLOUR before PORTS). Verified: OTOS → LINE →
      COLOUR → PORTS → TLM, unchanged position.
- [x] `Robot.h` adds `subsystems::LineSensor lineSensor`, `subsystems::ColorSensor colorSensor_`,
      `subsystems::Ports ports` value members declared AFTER the `line`, `colorSensor`, `portio`
      device-interface refs they bind. ANNOTATION: (1) the three subsystem classes are placed in
      a `subsystems` namespace because `LineSensor`/`ColorSensor` are ALREADY global class names
      (the io/real device drivers, source/io/real/{LineSensor,ColorSensor}.h, reachable in the
      firmware build via NezhaHAL) — without the namespace the ARM build fails with ODR
      redefinition. (2) the ColorSensor subsystem member is named `colorSensor_` (trailing
      underscore) because the existing `IColorSensor& colorSensor` device ref already owns the
      `colorSensor` name (kept to avoid macro collisions; used by SystemCommands caps). Both are
      internal naming only — no behavior/TLM change. loopTickOnce calls `robot.colorSensor_.periodic`.
- [x] `Robot.cpp` constructor init-list wires each subsystem with device ref, `state.inputs`, `config`.
- [x] `Robot::lineRead()`, `colorRead()`, `portsRead()` declarations and bodies removed
      (grep confirmed only callers were the loopTickOnce inline blocks; io/sim comments updated to
      reference the new subsystem path are left as-is since they are descriptive comments).
- [x] `source/subsystems/` added to `tests/_infra/sim/CMakeLists.txt` source glob
      (GLOB_RECURSE SUBSYSTEM_SOURCES + include dir + add_library entry).
- [x] INSPECT_DIRS updated to include `subsystems`. ANNOTATION: INSPECT_DIRS lives in
      `tests/simulation/unit/test_vendor_confinement.py` (not `vendor_baseline.txt`, which is the
      known-hits baseline output). `subsystems` added to the INSPECT_DIRS list there; the baseline
      needs no new entries because source/subsystems/ is vendor-clean (zero hits).
- [x] Vendor-confinement grep: `grep -rn "MicroBit\|I2CBus\|microbit_random" source/subsystems/`
      returns zero hits.
- [x] No `printf` / `telemetryEmit` calls inside any subsystem method body (only in header comments).
- [x] Simulation tier green: `uv run --with pytest python -m pytest -q` -> 2001 passed, 0 errors.
- [x] Golden-TLM canary byte-exact (test_golden_tlm_unchanged PASSED).
- [x] `defaultRobotConfig()` field-pin diff empty (test_default_config_pin green).
- [x] ARM firmware build gate: `python3 build.py --fw-only` -> 0 errors, MICROBIT.hex built; then
      `git checkout -- source/robot/DefaultConfig.cpp` (restored).
- [x] Behavior-preservation fences green: `test_incident_scenarios.py`, `test_goto_bounds.py`,
      `test_watchdog_exemption.py`.

## Implementation Plan

### Approach

Verbatim move of three inline timed blocks from `loopTickOnce` and three method bodies from
`Robot.cpp` into new subsystem classes. The only non-verbatim change: `systemTime()` calls
in `lineRead`/`colorRead` are replaced by the `now` parameter (same value, already in scope
at the `loopTickOnce` call site). No logic changes.

### Files to Create

**`source/subsystems/sensors/LineSensor.h`**

```cpp
#pragma once
#include "ILineSensor.h"
#include "RobotState.h"
#include "Config.h"
#include "LoopTickOnce.h"
#include <stdint.h>

class LineSensor {
public:
    LineSensor(ILineSensor& line, HardwareState& inputs, const RobotConfig& cfg);
    void updateInputs(uint32_t now);
    void periodic(LoopTickState& ts, uint32_t now);
private:
    ILineSensor&       _line;
    HardwareState&     _inputs;
    const RobotConfig& _cfg;
};
```

**`source/subsystems/sensors/LineSensor.cpp`**
- `updateInputs(now)`: verbatim `Robot::lineRead()` body (`is_initialized()` check,
  `_line.readValues(_inputs.line)`, set `_inputs.lineVS.lastUpdMs = now`, set `valid = true`).
- `periodic(ts, now)`: verbatim LINE timed-block logic from `loopTickOnce`:
  check `_cfg.lagLineMs > 0 && (int32_t)(now - ts.lastLine) >= (int32_t)_cfg.lagLineMs`,
  call `updateInputs(now)`, set `ts.lastLine = now`.

**`source/subsystems/sensors/ColorSensor.h` / `.cpp`**
Same pattern. `updateInputs(now)`: verbatim `Robot::colorRead()` body (poll RGBC, set
`colorVS.lastUpdMs = now`). `periodic()`: `lagColorMs` gate, `ts.lastColor`.

**`source/subsystems/sensors/Ports.h` / `.cpp`**
Same pattern. `updateInputs(now)`: verbatim `Robot::portsRead()` body (4-port loop,
set `portsVS.lastUpdMs = now`). `periodic()`: `lagPortsMs` gate, `ts.lastPorts`.

### Files to Modify

**`source/control/LoopTickOnce.cpp`**
- Replace the LINE timed block (currently ~lines 177-181) with:
  `robot.lineSensor.periodic(ts, now);`
- Replace the COLOUR timed block (~lines 183-187) with:
  `robot.colorSensor.periodic(ts, now);`
- Replace the PORTS timed block (~lines 189-193) with:
  `robot.ports.periodic(ts, now);`
- Add includes as needed (they will flow via Robot.h / LoopTickOnce.h includes).

**`source/robot/Robot.h`**
- Add `#include` directives for the three subsystem headers.
- Add value members after existing `portController` / `servoController`:
  `LineSensor lineSensor;`, `ColorSensor colorSensor;`, `Ports ports;`
- Remove `void lineRead();`, `void colorRead();`, `void portsRead();` declarations.

**`source/robot/Robot.cpp`**
- In the constructor init-list, add:
  `lineSensor(line, state.inputs, config),`
  `colorSensor(colorSensor, state.inputs, config),`  (note: member name vs ref name collision — use `colorSensor` field; verify naming)
  `ports(portio, state.inputs, config),`
- Delete `Robot::lineRead()`, `Robot::colorRead()`, `Robot::portsRead()` method bodies.
  (Grep for callers first: `grep -rn "\.lineRead\(\)\|\.colorRead\(\)\|\.portsRead\(\)" source/ tests/`)

**`tests/_infra/sim/CMakeLists.txt`**
- Add `source/subsystems/` to the glob pattern for simulation sources.

**`tests/_infra/vendor_baseline.txt`**
- Add `source/subsystems/` to the INSPECT_DIRS list.

### Testing Plan

1. After each subsystem header/cpp pair is created, run `python3 build.py --fw-only` to
   confirm incremental ARM build stays green.
2. After all three subsystems + `loopTickOnce` + `Robot` changes: run full suite:
   `uv run --with pytest python -m pytest -q`
3. Verify golden-TLM canary (run the canary command from `tests/_infra/`).
4. Run field-pin check.
5. Run vendor-confinement grep over `source/subsystems/`.
6. Run behavior fences:
   `uv run --with pytest python -m pytest tests/simulation/ -k "incident or goto_bounds or watchdog" -v`

### Documentation Updates

`architecture-update.md` already documents this change. No additional doc updates.
