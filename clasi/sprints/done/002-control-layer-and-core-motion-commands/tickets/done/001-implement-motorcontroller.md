---
id: '001'
title: Implement MotorController
status: done
use-cases: []
depends-on: []
github-issue: ''
issue: plan-c-port-of-radio-robot-firmware
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Implement MotorController

## Description

Create `source/control/MotorController.h` and `source/control/MotorController.cpp`.

MotorController wraps `NezhaV2` and provides a PI + feed-forward control loop for both
drive wheels. It has no knowledge of commands or drive modes — that responsibility belongs
to `CommandProcessor` (ticket 003). The public interface is designed so Sprint 5 can
replace the `tick()` body with a ratio PID without changing any callers.

The implementation is a direct C++ port of the per-tick logic described in `nezha.ts`
(`driveTick`) simplified to the simple PI+FF model used in this sprint.

## Header — `source/control/MotorController.h`

```cpp
#pragma once
#include "MicroBit.h"
#include "NezhaV2.h"
#include "Config.h"

/**
 * MotorController — PI + feed-forward wheel speed control.
 *
 * Owns two independent PI integrators (left, right) and a ratio
 * cross-coupling correction. Sprint 5 replaces the tick() body with
 * a ratio PID; callers in CommandProcessor are unchanged.
 *
 * Thread safety: single-threaded tick loop only.
 */
class MotorController {
public:
    explicit MotorController(NezhaV2& motor, const CalibParams& cal);

    // Gains — public so CommandProcessor can update via K-commands.
    // Defaults: kFF=0.15, kP=0.05, kI=0.20, iClamp=60, kRatio=0.01
    struct Gains {
        float kFF;      // feed-forward coefficient
        float kP;       // proportional gain
        float kI;       // integral gain
        float iClamp;   // integral windup clamp (PWM units, ±)
        float kRatio;   // ratio cross-coupling gain (sprint 2 stub, small)
    } gains;

    // Set speed targets in mm/s. Zero both to coast (not brake).
    void setTarget(float leftMms, float rightMms);

    // Stop: zero targets and reset integrators.
    void stop();

    // Reset integrators only (called by CommandProcessor on mode change,
    // NOT on S-command watchdog refresh — integrators survive keepalives).
    void resetIntegrators();

    // Run one control tick. dt_s is elapsed seconds since last tick.
    // Reads encoders, runs PI+FF+ratio, clamps output, calls NezhaV2::setPwm().
    // Sprint 5 replaces this body only.
    void tick(float dt_s);

    // Read actual wheel velocities in mm/s (encoder delta since last tick).
    void getActualVelocity(float& leftMms, float& rightMms) const;

    // Read cumulative encoder positions in mm (sum since last resetEncoderAccumulators()).
    void getEncoderPositions(int32_t& leftMm, int32_t& rightMm) const;

    // Zero encoder accumulators — delegates to NezhaV2::resetEncoders().
    void resetEncoderAccumulators();

private:
    NezhaV2&          _motor;
    const CalibParams& _cal;

    float _targetL;   // mm/s
    float _targetR;   // mm/s
    float _integralL; // PI integral accumulator, left wheel
    float _integralR; // PI integral accumulator, right wheel

    // Cached encoder readings from the most recent tick() call.
    // Used to compute velocity and expose via getActualVelocity().
    mutable int32_t _prevEncL; // mm at start of last tick
    mutable int32_t _prevEncR;
    mutable float   _actualVelL; // mm/s computed in tick()
    mutable float   _actualVelR;

    // clamp helper
    static float clamp(float v, float lo, float hi);
};
```

## Implementation — `source/control/MotorController.cpp`

### Constructor

Initialize `gains` with defaults from the sprint spec:
- `gains.kFF = 0.15f`
- `gains.kP  = 0.05f`
- `gains.kI  = 0.20f`
- `gains.iClamp = 60.0f`
- `gains.kRatio = 0.01f`

Zero all private state (`_targetL`, `_targetR`, `_integralL`, `_integralR`,
`_prevEncL`, `_prevEncR`, `_actualVelL`, `_actualVelR`).

### `setTarget(float leftMms, float rightMms)`

Store into `_targetL` and `_targetR`. Do NOT reset integrators — that is the
caller's (CommandProcessor's) responsibility on mode change.

### `stop()`

Set `_targetL = _targetR = 0.0f`. Call `resetIntegrators()`. Call
`_motor.setPwm(0, 0)` to immediately brake.

### `resetIntegrators()`

Set `_integralL = _integralR = 0.0f`.

### `tick(float dt_s)`

Guard: if `dt_s <= 0.0f`, return immediately.

Read current encoder positions:
```
int32_t encL = _motor.readEncoder(true,  _cal);
int32_t encR = _motor.readEncoder(false, _cal);
```

Compute actual velocities in mm/s from encoder delta:
```
_actualVelL = (encL - _prevEncL) / dt_s;
_actualVelR = (encR - _prevEncR) / dt_s;
_prevEncL   = encL;
_prevEncR   = encR;
```

For each wheel (left then right), run PI + FF:
```
errorL = _targetL - _actualVelL
_integralL += errorL * dt_s
_integralL = clamp(_integralL, -gains.iClamp, gains.iClamp)

// FF term: proportional to |target|, signed by target direction.
// maxSpeed is an implicit 1000 mm/s — the kFF coefficient already
// scales the output to roughly 0-100 PWM range.
ffL = gains.kFF * _targetL   // direct: kFF * target (signed)
outputL = ffL + gains.kP * errorL + gains.kI * _integralL
pwmL = clamp(outputL, -100.0f, 100.0f)
```

Ratio cross-coupling (applies when both wheels have non-zero same-sign targets):
```
if (_targetL != 0 && _targetR != 0 &&
    ((_targetL > 0) == (_targetR > 0))) {
    float ratio = _targetR / _targetL;          // desired ratio
    float actualRatio = _actualVelL != 0.0f
        ? _actualVelR / _actualVelL : ratio;
    float ratioErr = ratio - actualRatio;
    float correction = gains.kRatio * ratioErr;
    pwmL -= correction * 0.5f;
    pwmR += correction * 0.5f;
    // Re-clamp after correction
    pwmL = clamp(pwmL, -100.0f, 100.0f);
    pwmR = clamp(pwmR, -100.0f, 100.0f);
}
```

Apply: `_motor.setPwm(static_cast<int8_t>(pwmL), static_cast<int8_t>(pwmR))`

### `getActualVelocity(float& l, float& r) const`

Assign `l = _actualVelL; r = _actualVelR;`

### `getEncoderPositions(int32_t& l, int32_t& r) const`

```
l = _motor.readEncoder(true,  _cal);
r = _motor.readEncoder(false, _cal);
```

### `resetEncoderAccumulators()`

Call `_motor.resetEncoders()`. Also zero `_prevEncL = _prevEncR = 0`.

### `clamp(float v, float lo, float hi)` (static private)

Standard three-way clamp.

## Files to Create

- `source/control/MotorController.h`
- `source/control/MotorController.cpp`

## Files to Read First

- `source/hal/NezhaV2.h` — understand `setPwm()` and `readEncoder()` signatures
- `source/types/Config.h` — `CalibParams` and `MotorGains` structs
- `/Volumes/Proj/proj/league-projects/scratch/radio-robot/src/nezha.ts` — reference tick logic (lines 357–387)

## Acceptance Criteria

- [x] `source/control/MotorController.h` exists with the exact public interface above
- [x] `source/control/MotorController.cpp` exists and implements all methods
- [x] Default gains match spec: kFF=0.15, kP=0.05, kI=0.20, iClamp=60, kRatio=0.01
- [x] `stop()` zeroes targets, resets integrators, and calls `setPwm(0,0)`
- [x] `resetIntegrators()` does NOT reset encoder state or targets
- [x] `tick()` returns early if dt_s <= 0
- [x] `resetEncoderAccumulators()` calls `NezhaV2::resetEncoders()` and zeroes `_prevEnc*`
- [ ] `python build.py` compiles without errors after this ticket

## Testing

- **Hardware-in-the-loop only** — CODAL does not support off-device unit tests
- **Build verification**: `python build.py` must succeed with zero errors
- **Smoke test** (deferred to ticket 004): connect serial, send `S+100+100` after
  CommandProcessor wired in — verify motors spin
- **Verification command**: `python build.py`
