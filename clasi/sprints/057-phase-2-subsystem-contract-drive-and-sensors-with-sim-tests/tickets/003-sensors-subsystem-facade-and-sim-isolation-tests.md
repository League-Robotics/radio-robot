---
id: '003'
title: Sensors subsystem facade and sim isolation tests
status: open
use-cases:
- SUC-003
depends-on:
- '001'
- '002'
github-issue: ''
issue: message-based-subsystem-architecture.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sensors subsystem facade and sim isolation tests

## Description

The issue defines `Sensors` as a pure-observation subsystem (empty command axis)
aggregating `LineSensor` and `ColorSensor` behind a single facade. The existing
`subsystems::LineSensor` and `subsystems::ColorSensor` classes call `periodic()` on
the raw `ILineSensor` / `IColorSensor` devices — this ticket wraps them behind the
message contract: `tick(now)` / `state() -> const SensorsState&` / `configure(...)`.

The `SensorsState` aggregate type already exists from ticket 002. This ticket
implements the `Sensors` class, the `toSensorsConfig()` projection, C-ABI shims,
and the simulation isolation tests.

## Approach

1. Create `source/subsystems/sensors/Sensors.h`:
   ```cpp
   #pragma once
   #include "subsystems/sensors/SensorsState.h"
   #include "subsystems/sensors/LineSensor.h"
   #include "subsystems/sensors/ColorSensor.h"
   #include "messages/sensors.h"
   namespace subsystems {
   class Sensors {
   public:
       Sensors(LineSensor& line, ColorSensor& color);
       void tick(uint32_t now);
       const SensorsState& state() const { return _state; }
       void configure(const msg::LineSensorConfig& lc, const msg::ColorSensorConfig& cc);
       // newConfig() fluent builder: per-sensor sub-builders not needed for the
       // empty-command-axis case; configure() takes two typed args directly.
   private:
       LineSensor&    _line;
       ColorSensor&   _color;
       SensorsState   _state = {};
       msg::LineSensorConfig  _lineCfg  = {};
       msg::ColorSensorConfig _colorCfg = {};
       // Internal loop-tick state (lag timers — mirrors LoopTickState.lastLine/Color)
       uint32_t _lastLineTick  = 0;
       uint32_t _lastColorTick = 0;
   };
   }
   ```

2. Create `source/subsystems/sensors/Sensors.cpp`:
   - `tick(now)` calls `_line.updateInputs(now)` (or `periodic()` with a local
     `LoopTickState`) when the lag gate fires, then copies `HardwareState`
     line/color fields into `_state.line` and `_state.color`. Use the lag values from
     `_lineCfg.get_lag_line_ms()` and `_colorCfg.get_lag_color_ms()`.
   - `configure(lc, cc)` stores the configs; next `tick()` picks them up.
   - Note on `LoopTickState` dependency: `LineSensor::periodic()` takes a
     `LoopTickState&`. The `Sensors` wrapper owns a minimal local `LoopTickState`
     (just the `lastLine`/`lastColor` fields used inside) OR calls `updateInputs()`
     directly and manages the lag gate itself. Prefer the `updateInputs()` direct
     call to avoid dragging in `LoopTickOnce.h` as a dependency of `Sensors`.

3. Create `source/subsystems/sensors/SensorsConfig.cpp` with:
   ```cpp
   #include "messages/sensors.h"
   #include "types/Config.h"
   namespace subsystems {
   msg::LineSensorConfig  toLineSensorConfig(const RobotConfig& rc);
   msg::ColorSensorConfig toColorSensorConfig(const RobotConfig& rc);
   // Or: struct SensorsConfigSlice { ... }; SensorsConfigSlice toSensorsConfig(rc);
   }
   ```
   Map `RobotConfig::lagLineMs` → `LineSensorConfig::lag_line_ms`, threshold, norm
   min/max, channel map; similarly for color lag, integration, gain, calibration.
   Trace against `source/types/Config.h` and `source/state/ActualState.h` for
   exact field names.

4. Create C-ABI shim `tests/_infra/sim/sensors_api.cpp`:
   - `sensors_api_create(RobotConfig*)` — constructs `SimHardware`, `LineSensor`,
     `ColorSensor`, `Sensors`, returns opaque handle.
   - `sensors_api_tick(handle, now_ms)` — calls `sensors.tick(now)`.
   - `sensors_api_line_connected(handle)` → `int` (0/1).
   - `sensors_api_color_connected(handle)` → `int` (0/1).
   - `sensors_api_line_normalized(handle, idx)` → `uint32_t`.
   - `sensors_api_color_r/g/b/c(handle)` → `uint32_t`.
   - `sensors_api_destroy(handle)` — tears down.

5. Update `tests/_infra/sim/CMakeLists.txt` to add `sensors_api.cpp` to the
   `firmware_host` source list.

6. Create `tests/simulation/unit/test_sensors_subsystem.py`:
   - Load `firmware_host` via `ctypes`, configure shim signatures.
   - Test `test_sensors_connected`: construct sensors; tick 5 times; assert
     `line_connected == 1` and `color_connected == 1` (sim devices always return
     connected in sim mode).
   - Test `test_sensors_line_normalized_range`: tick 10 times with default sim line
     sensor; read normalized[0..3]; assert values in `[0, 1023]` range (or whatever
     the sim sensor's default output range is — check `SimLineSensor.cpp`).
   - Test `test_sensors_color_reads`: tick 5 times; assert `color_r >= 0` (smoke test
     that the color path executes without crash).
   - Test `test_sensors_configure_lag`: call `sensors_api_configure_lag(handle, 5)`
     to set `lagLineMs=5`; tick once with `now=10`; assert the line sensor was read
     (state updated).

## Files to Create/Modify

- `source/subsystems/sensors/Sensors.h` — NEW
- `source/subsystems/sensors/Sensors.cpp` — NEW
- `source/subsystems/sensors/SensorsConfig.cpp` — NEW (projection functions)
- `tests/_infra/sim/sensors_api.cpp` — NEW (C-ABI shims)
- `tests/_infra/sim/CMakeLists.txt` — add `sensors_api.cpp`
- `tests/simulation/unit/test_sensors_subsystem.py` — NEW

## Acceptance Criteria

- [ ] `Sensors::tick(now)` drives both `LineSensor` and `ColorSensor` reads when
      the lag gate fires; skips them when lag has not elapsed.
- [ ] `Sensors::state()` returns a `const SensorsState&` (no copy, no heap).
- [ ] `Sensors::configure(lc, cc)` stores configs; next tick uses updated lag values.
- [ ] `toLineSensorConfig()` / `toColorSensorConfig()` map `RobotConfig` fields to
      generated message config types correctly (spot-check at least `lag_line_ms`,
      `threshold`, `lag_color_ms`).
- [ ] `test_sensors_subsystem.py` passes: connected, range, color reads, configure-lag.
- [ ] No virtual dispatch introduced. No heap allocation.
- [ ] `python build.py --clean` zero errors.
- [ ] `uv run python -m pytest` green (no regressions to existing suite).

## Testing Plan

- **New tests**: `tests/simulation/unit/test_sensors_subsystem.py` — 4 tests described.
- **Regression**: full `uv run python -m pytest` must pass at baseline + new tests.
- **Device compile**: `python build.py --clean` zero errors (validates C++11 compliance
  of `Sensors.h/.cpp` and `SensorsConfig.cpp`).

## Verification Command

`uv run python -m pytest tests/simulation/unit/test_sensors_subsystem.py -v && python build.py --clean`
