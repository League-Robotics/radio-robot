---
id: '003'
title: StopCondition C++ binary paths and MotionCommandHandlers edge/fallback branch
  coverage
status: in-progress
use-cases:
- SUC-003
- SUC-004
depends-on:
- '001'
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 045-003: StopCondition C++ binary paths and MotionCommandHandlers edge/fallback branch coverage

## Description

**StopCondition (63 uncovered / 25% covered):**
The existing `test_stop_condition.py` is a pure-Python mirror — it does not call the
C++ binary and contributes zero gcovr coverage. The C++ `StopCondition::evaluate()`
has six Kinds with non-trivial branches, most of which have never been exercised in
the C++ binary via the sim:

- `Kind::ROTATION` — arc/spin stop condition; uses `(encRMm - encLMm) - base.encDiff0Mm`.
- `Kind::COLOR` — HSV distance; requires `rgbToHSV`, `hueDistance`, `sqrtf`.
- `Kind::LINE_ANY` — OR-across-4-channels short-circuit.
- `Kind::POSITION` — Euclidean distance to target; already exercised by `test_goto_bounds.py`?
  Programmer checks; if covered, skip.
- `Kind::SENSOR` — both GE and LE branches for non-line channels (colorR, analogIn).
- `getSensorValue` — channels 4–11 (colorR/G/B/C and analogIn[0..3]).

To inject line/color/analogIn sensor values into the sim, `PhysicsWorld` has
`setTrueLineRaw(uint16_t[4])` and `setTrueColorRGBC(uint16_t r,g,b,c)` but these
may not be exposed in `sim_api.cpp`. Programmer checks: if `sim_set_line_values` and
`sim_set_color_rgbc` wrappers do not exist in `sim_api.cpp`, add them. Adding these
wrappers is a `tests/_infra/sim/sim_api.cpp` change (test infrastructure, not
production source).

**MotionCommandHandlers (144 uncovered / 79% covered):**
Uncovered paths include:
- Verb error branches: missing required args, out-of-range values, sensor= parse
  failures in `mc_parseSensorToken`.
- The `ctx->queue != nullptr` vs `ctx->queue == nullptr` fallback paths. In the
  normal sim fixture the queue is always wired, so the `queue == nullptr` else-branch
  (e.g., `beginStream()` direct call in `handleS`) is never taken. Programmer
  decides: write a test that constructs a `MotionCtx` with `queue = nullptr` (bypassing
  the sim fixture and calling the handler directly via the sim's C API), OR note the
  branch as dead-in-practice-with-queue-wired and exclude from coverage expectations.
- `mc_parseSensorToken` failure paths (bad channel name, bad op string).
- The `packSensorArg` / `packKVArg` helper edge cases.
- `parseVW` edge cases with too few / malformed arguments.
- HALT verb handlers and their parsing edge cases.

## Acceptance Criteria

- [x] New file `tests/simulation/system/test_stop_condition_coverage.py` created (system tier — whole-robot drive+sensor+halt scenarios).
- [x] ROTATION stop: `test_rotation_stop_terminates_spin` sends `RT 9000` (registers a `makeRotationStop`) and confirms the spin self-terminates (EVT / PWM zero), firing `Kind::ROTATION::evaluate`.
- [x] COLOR stop: `test_color_stop_fires_on_match` / `..._does_not_fire_on_mismatch` inject RGBC via the new `sim_set_color_rgbc` wrapper and confirm `HALT COLOR` fires/does-not-fire — exercising `Kind::COLOR` (rgbToHSV, hueDistance, HSV distance, both fire and non-fire paths).
- [x] LINE_ANY stop: `test_line_any_stop_fires_ge` / `..._le` inject line values via `sim_set_line_values` and confirm `HALT LINE ANY GE/LE` fires — exercising both `Kind::LINE_ANY` comparison branches and the OR short-circuit.
- [~] SENSOR stop: `Kind::SENSOR` / `getSensorValue` are UNREACHABLE through the sim. The `sensor=` token attaches a SENSOR stop only on the direct (queue==null) path; on the QUEUE path (the sim's only mode) the stop is silently dropped — `packSensorArg` stores the value WITHOUT the `sensor=` prefix that handleVW's forwarding loop (`strncmp(...,"sensor=",7)`) requires. Covering it would need a production `source/` change (out of scope, test-additive only). Documented + regression-pinned by `test_sensor_stop_dropped_on_queue_path_documented`.
- [x] New file `tests/simulation/unit/test_motion_handlers_coverage.py` created (26 tests).
- [x] ERR on malformed VW arg: `test_vw_no_args_errors` / `..._one_arg_errors` confirm ERR; plus range-ERR for v/omega.
- [x] ERR on bad sensor= token: `test_t_bad_sensor_channel_errors`, `..._bad_sensor_op_errors`, `..._malformed_sensor_no_colon_errors`, `test_d_bad_sensor_channel_errors`, `test_turn_bad_sensor_errors` all confirm ERR (mc_parseSensorToken failure paths on the queue path).
- [~] MotionCommandHandlers queue-null fallback (OQ-3): DOCUMENTED dead-in-practice. The sim always wires the queue (SimHandle ctor), so the `else { direct begin*() }` branches are unreachable via `sim.send_command`; no C API exposes a `MotionCtx{queue=nullptr}` and adding one to cover a host-only fallback (firmware always wires the queue) is not warranted. Note in `test_motion_handlers_coverage.py` docstring.
- [x] `sim_api.cpp` new wrappers `sim_set_line_values` / `sim_set_color_rgbc` added (OQ-2 — Sim sensors read a schedule table, not the plant, so these install a single-row schedule) and exposed in `firmware.py` (`set_line_values` / `set_color_rgbc`).
- [x] All existing tests still pass: 2055 passed (was 2023; +32).
- [x] Golden-TLM, field-pin, vendor grep gates all green.

## Implementation Plan

### Approach

**Step 1: Check sim_api.cpp for line/color setters.**
Search for `sim_set_line` and `sim_set_color` in `tests/_infra/sim/sim_api.cpp`.
`PhysicsWorld` already has `setTrueLineRaw` and `setTrueColorRGBC` (confirmed in
`source/io/sim/PhysicsWorld.h`). If no sim_api wrapper exists, add:
```cpp
void sim_set_line_values(void* h, uint16_t l0, uint16_t l1, uint16_t l2, uint16_t l3) {
    uint16_t line[4] = {l0, l1, l2, l3};
    static_cast<SimHandle*>(h)->hal.physics().setTrueLineRaw(line);
}
void sim_set_color_rgbc(void* h, uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
    static_cast<SimHandle*>(h)->hal.physics().setTrueColorRGBC(r, g, b, c);
}
```
Also expose these in `tests/_infra/sim/firmware.py`'s ctypes bindings.

**Step 2: Write stop-condition binary tests.**
Sensor-injection tests pattern:
1. Start a long-duration motion command with a sensor/color/line stop condition.
2. Inject the sensor value via the new sim API.
3. Tick until the command completes or a timeout.
4. Assert the sim stopped (e.g., PWM drops to 0, or an OK reply is in the async buffer).

For ROTATION: send a spin (T command or VW with omega only) with a small ROTATION
threshold. After enough ticks, the encoder differential should cross the threshold.

**Step 3: Write MotionCommandHandlers error-path tests.**
Send malformed commands via `sim.send_command(...)` and assert ERR in the reply.
Examples:
- `sim.send_command("VW")` with no args → ERR.
- `sim.send_command("D dist=200 sensor=nosuchchan:ge:500")` → ERR or graceful ignore.

### Files to create

- `tests/simulation/unit/test_stop_condition_coverage.py`
- `tests/simulation/unit/test_motion_handlers_coverage.py`

### Files to modify (if wrappers missing)

- `tests/_infra/sim/sim_api.cpp` — add `sim_set_line_values`, `sim_set_color_rgbc`
- `tests/_infra/sim/firmware.py` — expose new ctypes bindings

### Testing plan

- Run full simulation tier after each new test file is added.
- Confirm stop-condition tests actually terminate (don't spin forever) by using short
  timeouts and asserting the stop within N ticks.

### Documentation updates

- If queue-null fallback branch is determined to be dead-in-practice with a wired
  queue, note this in a comment at the top of `test_motion_handlers_coverage.py`.
