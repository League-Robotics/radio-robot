---
id: '003'
title: Refactor VelocityController to compose cmon-pid backcalculation core
status: done
use-cases:
- SUC-002
depends-on:
- '002'
github-issue: ''
issue: consolidate-control-code-onto-vendored-libraries-cmon-pid-tinyekf.md
completes_issue: false
---

# Refactor VelocityController to compose cmon-pid backcalculation core

## Description

Replace the hand-rolled integral/anti-windup arithmetic in `VelocityController`
with a composed `cmon_pid::backcalculation_t<cmon_pid::pid_bwe>` instance from
the vendored cmon-pid header. The thin wrapper retains: feed-forward
(`kFF * |setpoint|`), deadband gate (`|setpoint| < minWheelMms` suppresses the
integrator), output sign (`sign(setpoint)`), and PWM clamp (`[-100, +100]`).

The public API — constructor signature, `update()`, `reset()`, the six public
fields `kFF`, `kP`, `kI`, `iMax`, `minWheelMms`, `kAw` — must remain unchanged.
`MotorController` calls `VelocityController::update()` and sets the public fields
via `updateVelGains()`; both must continue to work without modification.

Gain mapping:
- `kP`  -> cmon-pid Kp
- `kI`  -> cmon-pid Ki
- `kAw` -> cmon-pid back-calculation coefficient (Kaw)
- `iMax` -> cmon-pid integrator clamp (the `clamp` parameter to `backcalculation_t`)
- `kFF` and `minWheelMms` remain in the wrapper only

## Acceptance Criteria

- [x] `VelocityController.cpp` includes `cmon-pid.h` and holds a
      `cmon_pid::backcalculation_t<cmon_pid::pid_bwe>` member.
- [x] The hand-rolled `integral +=` and freeze-or-bleed integrator code is
      removed from `VelocityController::update()`.
- [x] The public fields `kFF`, `kP`, `kI`, `iMax`, `minWheelMms`, `kAw`,
      and `integral` remain on the class with the same names and types.
      (`integral` may delegate to cmon-pid's internal state accessor for
      inspection; it must read the current integrator value.)
- [x] `VelocityController::reset()` resets the cmon-pid internal state.
- [x] Constructor signature is unchanged: `VelocityController(float kFF, float kP,
      float kI, float iMax, float minWheelMms, float kAw = 0.0f)`.
- [x] `MotorController::updateVelGains()` pushes updated gains into the
      cmon-pid instance when called (the six public fields are updated AND the
      cmon-pid instance is reconfigured).
- [x] `uv run --with pytest python -m pytest tests/simulation -q` shows no new
      failures beyond the 2 pre-existing baseline failures.
- [x] `test_velocity_controller.py` is fully green.
- [x] `test_motor_controller.py` is fully green.
- [x] `test_body_velocity_controller.py` is fully green.

## Implementation Plan

### Approach

**VelocityController.h** changes:
1. Add `#include "cmon-pid.h"` (angle-bracket or quoted depending on build).
2. Add a private member: `cmon_pid::backcalculation_t<cmon_pid::pid_bwe> _pid;`
3. The public `integral` field changes from a standalone `float` to a computed
   property or a cached float that mirrors the cmon-pid internal state. Simplest
   approach: keep `float integral` as a public data member and sync it in
   `update()` after the cmon-pid call for inspection. Alternatively, make it a
   method that reads from cmon-pid. The test suite reads `integral` directly, so
   a public `float integral` member (possibly set to the cmon-pid state each tick)
   is the safest approach.

**VelocityController.cpp** changes:
1. In the constructor: initialise `_pid` using `cmon_pid::ParallelPid(dt_hint,
   kP, kI, 0.0f, 0.0f)` wrapped in `cmon_pid::backcalculation_t<...>(pid, kAw,
   iMax)` — or whatever the cmon-pid construction API is. Read the header to
   confirm the exact API.
2. In `update()`: remove the hand-rolled `integral += ...` and clamp block.
   Instead:
   - Compute `err = setpoint - measured`.
   - Compute `ff = kFF * spAbs` and `spSign`.
   - If not in deadband: call `_pid.Update(err, dt_s)` to advance the integrator.
   - Read `_pid.Output()` (or equivalent) for the PI contribution.
   - `rawPwm = spSign * ff + piOutput`.
   - `output = clamp(rawPwm, -100, 100)`.
   - Sync `integral = _pid.Integrator()` (or however cmon-pid exposes the
     integrator state) for public inspection.
3. In `reset()`: call `_pid.Reset()` (or equivalent cmon-pid reset method).

**MotorController** — the `updateVelGains()` method must reconfigure the cmon-pid
instance after updating the public fields. Confirm the cmon-pid API for runtime
gain changes and implement accordingly. If cmon-pid has no in-place reconfigure
method, reconstruct `_pid` in `updateVelGains()`.

### Files to modify

- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/source/control/VelocityController.h`
  (add `_pid` member, keep public API unchanged)
- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/source/control/VelocityController.cpp`
  (compose cmon-pid; remove hand-rolled integrator math)

### Files that may need minor updates

- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/source/control/MotorController.h`
  or its `.cpp` if `updateVelGains` needs to reconfigure the cmon-pid instance
  (MotorController calls `_vcL.kP = ...; _vcR.kI = ...;` — verify and update
  if cmon-pid needs an explicit reconfigure call after field assignment).

### Testing plan

IMPORTANT: Use the canonical test command, NOT bare `uv run pytest`:

```
uv run --with pytest python -m pytest tests/simulation -q
```

The expected state after this ticket:
- `test_velocity_controller.py` — all tests pass (this is the primary gate).
- `test_motor_controller.py` — all tests pass.
- `test_body_velocity_controller.py` — all tests pass.
- `test_vendor_confinement.py` — still passes (no CODAL tokens introduced).
- Total failure count: exactly 2 (the pre-existing config-schema failures).

If `test_velocity_controller.py` has numeric-precision tests that compare
integrator state exactly, they may need tolerance adjustments if cmon-pid's
accumulation order differs slightly. Document any such adjustment with a comment.

### Documentation

Update the control-law comment block at the top of `VelocityController.h` to
reference cmon-pid for the integral/anti-windup terms. The existing doc-comment
for the constructor parameter `kAw` remains valid.
