---
id: '001'
title: 'HAL additions: OtosSensor::readTransformed and Servo angle tracking'
status: done
use-cases:
- SUC-003
- SUC-004
- SUC-006
depends-on: []
github-issue: ''
issue: replace-robot-facade-with-appcontext-struct.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# HAL additions: OtosSensor::readTransformed and Servo angle tracking

## Description

Pure additions to two HAL classes. No callers are modified in this ticket;
it only adds new API. This is the lowest-risk starting point: it can be
built and tested in complete isolation, and the subsequent tickets depend on
these additions being present.

**Also perform dead-code grep verification** before later tickets remove
methods:
- `grep -r setPose source/` — confirm zero callers
- `grep -r getPose source/` — confirm zero callers
- `grep -r noteActivity source/` — confirm zero callers
- `grep -r controlCollect source/` (the synchronous stub, not
  `controlCollectSplitPhase`) — confirm `LoopScheduler.cpp` does NOT call it
  in `run_blocks`

Document grep results in a brief code comment or in this ticket's notes.
These verifications unblock the delete step in Ticket 006.

### OtosSensor::readTransformed

Add a new method to `source/hal/OtosSensor.h` and `source/hal/OtosSensor.cpp`.

New type in `OtosSensor.h`:
```cpp
struct OtosPose { float x, y, h; };
```

New method signature:
```cpp
OtosPose readTransformed(const RobotConfig& cfg) const;
```

Method body: extract verbatim from `Robot::otosCorrect` in `Robot.cpp` (lines
267–296). Constants stay as local `constexpr` inside the method body:
```cpp
constexpr float kPosMmPerLsb  = 0.305f;
constexpr float kHdgRadPerLsb = 0.00549f * (3.14159265f / 180.0f);
```
The method does NOT update `_state.inputs` or call `odometry.correct` — it
only reads, converts, and returns the `OtosPose`. The caller (`AppContext::otosCorrect`)
retains those steps.

`RobotConfig` is already forward-declared in `OtosSensor.h`; the method
parameter adds a `const RobotConfig&` dependency that is safe because
`OtosSensor` already holds a `const RobotConfig& _cfg` (used in `begin()` for
scalars). The method can use `cfg` (parameter) instead of `_cfg` to keep the
signature consistent with how `AppContext` will call it.

### Servo angle tracking

Modify `source/hal/Servo.h` and `source/hal/Servo.cpp`.

Add to `Servo` private section:
```cpp
int16_t _currentAngle = 0;
```

Modify `Servo::setAngle(uint8_t degrees)` to record the clamped value:
```cpp
void Servo::setAngle(uint8_t degrees) {
    uint8_t clamped = (degrees > _maxDegrees) ? (uint8_t)_maxDegrees : degrees;
    _pin.setServoValue(clamped);
    _currentAngle = (int16_t)clamped;
}
```

Add new accessor:
```cpp
int16_t currentAngle() const { return _currentAngle; }
```

## Acceptance Criteria

- [x] `OtosSensor.h` declares `struct OtosPose { float x, y, h; }` and
      `OtosPose readTransformed(const RobotConfig& cfg) const`.
- [x] `OtosSensor::readTransformed` produces identical numerical results to
      the corresponding block in `Robot::otosCorrect` for the same raw sensor
      values (verified by inspection: same constants, same flip, same rotation
      formula, same offset subtraction — method body is a direct extract).
- [x] `Servo.h` declares `int16_t currentAngle() const`.
- [x] `Servo::setAngle` records the clamped angle in `_currentAngle`.
- [x] `Servo::currentAngle()` returns the last clamped angle set (default 0
      before any `setAngle` call).
- [x] Clean build: `python3 build.py` passes with no new errors or warnings.
      (Pre-existing library warnings present; no new warnings introduced.)
- [x] Host unit tests pass: `uv run --with pytest python -m pytest`.
      (1035 pass; 8 pre-existing failures in test_push_calibration /
      test_robot_config / test_sensors_v2 unrelated to this ticket —
      confirmed identical failures before and after these changes.)
- [x] Grep verification complete:
      - `Robot::setPose` / `Robot::getPose`: zero callers outside the class
        (`grep -rn "\.setPose\|->setPose" source/` and `->getPose` hit only
        `_odo.setPose(...)` inside Robot.cpp itself and Odometry.cpp
        implementations — no external callers). Safe to delete in T006.
      - `Robot::noteActivity`: defined only in Robot.h (inline); zero callers
        in source/ (`grep -rn "noteActivity" source/` returns only Robot.h).
        Safe to omit from AppContext in T002.
      - `Robot::controlCollect` (synchronous stub): zero callers in source/
        outside the class itself. `LoopScheduler::controlCollect` is a private
        LoopScheduler method that calls `controlCollectSplitPhase`, NOT the
        Robot sync stub. Safe to drop from AppContext in T002.

## Implementation Plan

**Approach**: Pure additions. Do not modify any callers. Do not modify `Robot.cpp`
yet (that is Ticket 006).

**Files to modify**:
- `source/hal/OtosSensor.h` — add `OtosPose` struct + method declaration
- `source/hal/OtosSensor.cpp` — add `readTransformed` implementation (extract
  from `Robot::otosCorrect`, leave `Robot::otosCorrect` intact for now)
- `source/hal/Servo.h` — add `_currentAngle` field + `currentAngle()` accessor
- `source/hal/Servo.cpp` — modify `setAngle` to record clamped value

**Files NOT to touch**: `Robot.h`, `Robot.cpp`, `AppContext.h/.cpp` (does not
exist yet), `CommandProcessor`, `LoopScheduler`, `main.cpp`.

**Testing plan**:
- `python3 build.py` — verify clean build.
- `uv run --with pytest python -m pytest` — verify no regressions.
- Inspect `readTransformed` output against `Robot::otosCorrect` math by
  reading the code: same constants, same flip, same rotation formula.

**Documentation updates**: None required for this ticket.
