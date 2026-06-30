---
id: '002'
title: 'Close gap 2: single motor-output authority (setCommandsRef)'
status: done
use-cases:
- SUC-002
depends-on:
- '001'
github-issue: ''
issue: make-ordered-tick-the-default-close-parity-gaps.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Close gap 2: single motor-output authority (setCommandsRef)

## Description

Parity gap #2: `Drive2`'s constructor calls `_mc.setCommandsRef(&_outputs)` to bind
the `MotorController`'s motor-command output to Drive2's own `_outputs` buffer.
However, the `Robot` constructor body at `Robot.cpp:121` then calls:

```cpp
motorController.setCommandsRef(&state.outputs);
```

This overrides Drive2's binding, so `MotorController::controlTick()` writes motor
commands into `robot.state.outputs` rather than `drive2._outputs`. This means
`drive2.outputs()` (which returns `_outputs`) is never actually populated by the
motor controller — defeating Drive2's ownership model.

Additionally, `robot.hal.tick(now, robot.state.outputs)` in the ordered-tick branch
(step 6b of `LoopTickOnce.cpp`, around line 250) passes `state.outputs` to the HAL
plant. After fixing the `setCommandsRef` override, the HAL tick must pass the buffer
that the motor controller actually writes to: `drive2.outputs()`.

This ticket:
1. Removes `motorController.setCommandsRef(&state.outputs)` from `Robot.cpp` (the
   override). Since the legacy path (`#ifndef USE_ORDERED_TICK`) is still compiled
   at this point, the implementer must verify whether removing this line leaves the
   legacy path's `hal.tick(now, state.outputs)` correct. The legacy `MotorController`
   wrote to `state.outputs` via this binding — if Drive2's ctor binding now wins,
   the legacy `hal.tick(now, state.outputs)` would get a stale buffer. Safest approach:
   keep the call in Robot.cpp but also guard the ordered-tick hal.tick to use
   `drive2.outputs()`. Alternatively, verify the legacy path behavior in tests before
   removing. The implementer decides the lowest-risk approach.
2. Updates the `robot.hal.tick(now, ...)` call in the ordered-tick branch (step 6b)
   to pass `robot.drive2.outputs()`.
3. Verifies `MockHAL::tick` accepts the updated call.

After this ticket: in the ordered-tick path, `MotorController` writes to
`drive2._outputs`, and the HAL plant reads from `drive2.outputs()`.

## Acceptance Criteria

- [x] The ordered-tick branch step 6b calls `robot.hal.tick(now, robot.drive2.outputs())` (not `robot.state.outputs`).
- [x] Drive2's `_mc.setCommandsRef(&_outputs)` binding is not overridden for the ordered-tick path.
- [x] `MockHAL::tick` compiles and runs correctly with the updated argument.
- [x] `uv run python -m pytest` — green except the 2 known-baseline failures.
- [x] `test_golden_tlm.py` remains green.
- [x] `test_059_ordered_tick_parity.py` remains green.

## Implementation Plan

### Approach

Surgical change: two lines in `LoopTickOnce.cpp` (HAL tick argument) and one decision
in `Robot.cpp` (whether to remove or conditionally guard the setCommandsRef override).

Read `source/robot/Robot.cpp:110-145` to understand the constructor wiring.
Read `source/robot/LoopTickOnce.cpp:240-260` for step 6b.
Read `source/subsystems/drive/Drive2.h:78-83` for `Drive2::outputs()` return type.
Check MockHAL or sim_api for the `hal.tick` signature in tests.

### Files to modify

- `source/robot/Robot.cpp` — remove or guard `motorController.setCommandsRef(&state.outputs)` at ~line 121.
- `source/robot/LoopTickOnce.cpp` — update step 6b `hal.tick` argument in the `#else` ordered-tick branch.

### Testing plan

1. `uv run python -m pytest tests/simulation/unit/test_golden_tlm.py -v`
2. `uv run python -m pytest tests/simulation/unit/test_059_ordered_tick_parity.py -v`
3. `uv run python -m pytest` — green except 2 known-baseline failures.

### Documentation updates

None. Architecture-update.md for this sprint documents the fix.
