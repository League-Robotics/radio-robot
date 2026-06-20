---
id: "003"
title: "HAL: MecanumHAL, NoopDevices refactor, Hardware Noop additions, main.cpp select"
status: open
use-cases:
  - SUC-001
  - SUC-002
depends-on:
  - "046-002"
github-issue: ""
issue: ""
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 046-003: HAL: MecanumHAL, NoopDevices refactor, Hardware Noop additions, main.cpp select

## Description

Create `MecanumHAL` (4-motor sibling of `NezhaHAL`), refactor `NoopVelocityMotor`
out of `ReplayHAL.h` into a shared `NoopDevices.h`, add default-Noop virtual
accessors to `Hardware.h` for the two new motor slots, and add the single
`#ifdef` in `main.cpp` that selects between `NezhaHAL` and `MecanumHAL`.

After this ticket both build variants compile cleanly. The differential build
is byte-identical. The mecanum build compiles and links but does not yet have
functional control logic (that comes in T5).

## Approach

### 1. source/io/NoopDevices.h — new shared header

Refactor `NoopVelocityMotor` from `source/io/ReplayHAL.h` into
`source/io/NoopDevices.h`. Add `#include "NoopDevices.h"` to `ReplayHAL.h` to
restore its existing behaviour (no callers should need to change their includes).

```cpp
// source/io/NoopDevices.h
#pragma once
#include "io/capability/IVelocityMotor.h"

// NoopVelocityMotor — do-nothing motor for stubs and default HAL slots.
class NoopVelocityMotor : public IVelocityMotor {
public:
    void setSpeed(int8_t) override {}
    float readEncoderMmFAtomic(const RobotConfig&) override { return 0.0f; }
    float readEncoderMmFSettle(const RobotConfig&) override { return 0.0f; }
    void resetEncoder() override {}
};
```

Adjust the class definition to match the actual `IVelocityMotor` interface
(read the current `IVelocityMotor.h` for the exact virtual method set).

### 2. source/io/Hardware.h — additive default-Noop methods

Add at the bottom of the `Hardware` class body (before the closing `}`):

```cpp
// Default Noop accessors for rear motors (mecanum build overrides in MecanumHAL).
// All existing Hardware subclasses (MockHAL, ReplayHAL, NezhaHAL) inherit these
// defaults and require no modification.
virtual IVelocityMotor& motorBR() { return _noopMotor; }
virtual IVelocityMotor& motorBL() { return _noopMotor; }
virtual int motorCount() const { return 2; }

private:
    NoopVelocityMotor _noopMotor;
```

Add `#include "io/NoopDevices.h"` to `Hardware.h`.

Note: the `_noopMotor` must be declared before it is used in the virtual
method bodies. If `Hardware.h` already has a `private:` section, add the member
there. The existing pure-virtual methods (`motorL`, `motorR`) do not change.

### 3. source/io/real/MecanumHAL.h + MecanumHAL.cpp

Model after `NezhaHAL.{h,cpp}`. Key differences:

- Four `Motor` members: `_motorFR(_bus, 1, cfg.fwdSignFR)`,
  `_motorFL(_bus, 2, cfg.fwdSignFL)`, `_motorBR(_bus, 3, cfg.fwdSignBR)`,
  `_motorBL(_bus, 4, cfg.fwdSignBL)`.
- `motorL()` returns `_motorFL` (front-left, semantic "left").
- `motorR()` returns `_motorFR` (front-right, semantic "right").
- `motorBL()` returns `_motorBL`; `motorBR()` returns `_motorBR`.
- `motorCount()` returns 4.
- All other devices (OTOS, color sensor, portIO, gripper, bus diagnostics,
  raw bus access, `LineSensor`) are identical to `NezhaHAL`.
- `tick(now_ms)`: drives the split-phase encoder read for all four motors.
  Use a right-first ordering: FR(port 1), BR(port 3), FL(port 2), BL(port 4)
  (preserves the NezhaHAL convention "right before left" for the front pair;
  rear pair follows). This affects the `refreshedWheel` value passed to
  `MotorController::controlTick` — coordinate with T5 implementer.
- `tick(now_ms, cmds)`: integrates all four commanded velocities into
  `BenchOtosSensor` when bench mode is on. For the mecanum build the
  integration uses `MecanumKinematics::forward` (available after T2).
  If that's not yet available at the time of implementation, use a zero-vy
  placeholder: integrate `(cmds.tgtMms[0] + cmds.tgtMms[1]) / 2` as vx
  and `(cmds.tgtMms[0] - cmds.tgtMms[1]) / trackwidthMm` as omega (front
  pair only) — same as NezhaHAL — with a TODO comment for T5.
- `BENCH_OTOS_ENABLED` guards: same pattern as `NezhaHAL`.

The `MecanumHAL` header must NOT be included by host-build TUs (same restriction
as `NezhaHAL.h`).

### 4. source/main/main.cpp — #ifdef select

Add `#include "io/real/MecanumHAL.h"` under `#ifdef ROBOT_DRIVETRAIN_MECANUM`
alongside the existing `#include "io/real/NezhaHAL.h"`. Add the select:

```cpp
#ifdef ROBOT_DRIVETRAIN_MECANUM
    MecanumHAL hw(uBit.i2c, uBit.io, cfg);
#else
    NezhaHAL   hw(uBit.i2c, uBit.io, cfg);
#endif
```

The `Robot` binding receives `Hardware& hw` — no change needed there.

## Files to Create

- `source/io/NoopDevices.h`
- `source/io/real/MecanumHAL.h`
- `source/io/real/MecanumHAL.cpp`

## Files to Modify

- `source/io/ReplayHAL.h` (include NoopDevices.h; remove inline NoopVelocityMotor)
- `source/io/Hardware.h` (add motorBR, motorBL, motorCount, _noopMotor)
- `source/main/main.cpp` (add #ifdef drivetrain select)

## Acceptance Criteria

- [ ] `python build.py` (differential / tovez) exits 0; `MICROBIT.hex` produced;
      `ROBOT_DRIVETRAIN_MECANUM` not defined.
- [ ] `python build.py` (mecanum robot JSON) exits 0; `MICROBIT.hex` produced with
      `ROBOT_DRIVETRAIN_MECANUM` defined; `MecanumHAL` is the active HAL.
- [ ] `uv run --with pytest python -m pytest tests/simulation -q` reports `2093 passed`.
- [ ] Differential build: `NezhaHAL.cpp` compiled in; `MecanumHAL.cpp` excluded.
- [ ] Mecanum build: `MecanumHAL.cpp` compiled in; `NezhaHAL.cpp` excluded.
- [ ] All existing `Hardware` subclasses (MockHAL, ReplayHAL/sim build) compile
      without modification — they inherit the default Noop `motorBR`/`motorBL`/
      `motorCount` from `Hardware.h`.
- [ ] `NoopVelocityMotor` still accessible from `ReplayHAL.h` (via `#include
      "NoopDevices.h"`) — no existing include paths broken.
- [ ] `MecanumHAL` constructs and calls `begin()` in the mecanum build without
      crashing or wedging the I2C bus (confirmed on first-flash in T4).

## Testing

- **Regression gate**: `uv run --with pytest python -m pytest tests/simulation -q`
- **Build verification**: compile both variants and inspect the symbol table
  (`nm MICROBIT.hex` or check link map) to confirm the correct HAL is linked.
- **New tests**: none required — HAL correctness is validated on hardware in T4.
- **Verification command**: `uv run --with pytest python -m pytest tests/simulation -q`
