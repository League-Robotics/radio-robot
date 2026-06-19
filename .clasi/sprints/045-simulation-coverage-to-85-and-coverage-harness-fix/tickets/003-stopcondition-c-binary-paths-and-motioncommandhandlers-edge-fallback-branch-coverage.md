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

- [ ] New file `tests/simulation/unit/test_stop_condition_coverage.py` created.
- [ ] ROTATION stop: test sends a spin command (e.g., `T` with rotation stop) and confirms the command stops when the rotation threshold is reached.
- [ ] COLOR stop: if `sim_set_color_rgbc` wrapper exists or is added, test injects an RGBC value that matches a COLOR stop condition hue/sat/val target and confirms the command stops.
- [ ] LINE_ANY stop: if `sim_set_line_values` wrapper exists or is added, test injects a line sensor value above threshold on one channel and confirms the command stops via `sensor=line0:ge:500` (or similar).
- [ ] SENSOR stop (colorR channel 4, analogIn channel 8): test injects color/analog values and confirms stop.
- [ ] New file `tests/simulation/unit/test_motion_handlers_coverage.py` (or `test_motion_command_handlers_coverage.py`) created.
- [ ] ERR on malformed VW arg (missing required argument): send `VW` with no arguments; confirm ERR reply.
- [ ] ERR on bad sensor= token: send a motion command with `sensor=badchan:ge:100`; confirm ERR or graceful failure.
- [ ] MotionCommandHandlers queue-null fallback: programmer's decision documented (either a test covering the path, or a comment explaining it's dead-in-practice).
- [ ] If `sim_api.cpp` needed new wrappers for line/color injection: the wrappers are added and exposed from `tests/_infra/sim/firmware.py` (or tested via ctypes directly).
- [ ] All existing tests still pass.
- [ ] Golden-TLM, field-pin, vendor grep gates all green.

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
