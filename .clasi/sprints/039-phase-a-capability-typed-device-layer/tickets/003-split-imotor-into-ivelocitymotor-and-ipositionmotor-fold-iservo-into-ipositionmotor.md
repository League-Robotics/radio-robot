---
id: "003"
title: "Split IMotor into IVelocityMotor and IPositionMotor; fold IServo into IPositionMotor"
status: open
use-cases:
  - SUC-039-004
depends-on:
  - "039-001"
  - "039-002"
github-issue: ""
issue: ""
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# T3 — Split IMotor into IVelocityMotor and IPositionMotor; fold IServo into IPositionMotor

## Description

Wire up the `IVelocityMotor` / `IPositionMotor` split that was introduced as empty
capability headers in T1. In T1, `IMotor.h` became a shim (`using IMotor = IVelocityMotor;`)
and `IServo.h` became a shim (`using IServo = IPositionMotor;`). In T2, `Motor` was
updated to implement `IVelocityMotor`. This ticket completes the split:

1. `Motor` implements `asPositionMotor()` returning a non-null `IPositionMotor*` for
   the Nezha position-move operations (`timedMove`, `moveToAngle`).
2. `Servo` base class changes from `IServo` to `IPositionMotor` (since `IServo` is
   now an alias, this may already work; confirm and make canonical).
3. `MockServo` changes from `IServo` to `IPositionMotor`.
4. `Hardware::gripper()` return type changes from `IServo&` to `IPositionMotor&`.
5. `ServoController` stores `IPositionMotor&` instead of `IServo&`.
6. `Hardware::motorL/R()` return types are confirmed as `IVelocityMotor&`.

After T1's alias shims, most of this may compile unchanged. This ticket makes the
types CANONICAL (the concrete classes derive from the capability types, not the aliases),
removes any redundant shim indirection in the impl headers, and verifies everything
compiles with the capability types used directly.

**Host-verifiable:** Yes — host build includes all of Servo.h, MockServo.h,
ServoController.h.
**ARM files touched:** `Servo.h/.cpp` (base class canonicalization),
`NezhaHAL.h/.cpp` (gripper return type).

## Approach

### Step 1 — Confirm Motor implements IVelocityMotor directly

After T2, `Motor` should already declare `class Motor : public IVelocityMotor`. Confirm
this. If T2 left `class Motor : public IMotor` (using the alias), update to use
`IVelocityMotor` directly (includes `source/io/capability/IVelocityMotor.h`).

### Step 2 — Add IPositionMotor implementation to Motor

`Motor` exposes the Nezha position-move operations (0x70 timedMove, 0x5D moveToAngle)
via an inner `MotorPositionImpl` member or by deriving from `IPositionMotor` directly.

The simpler approach is direct derivation:
```cpp
class Motor : public IVelocityMotor, public IPositionMotor {
public:
    IPositionMotor* asPositionMotor() override { return this; }
    void     setAngleDeg(uint16_t deg, uint8_t mode) override; // calls moveToAngle()
    uint16_t currentAngleDeg() const override;                  // returns last commanded angle
};
```
The existing `timedMove(dir, value, mode)` and `moveToAngle(angle, mode)` methods
become the implementation body of `setAngleDeg`.

Alternatively (lower-risk, avoids C++ multiple inheritance diamond issues):
```cpp
class Motor : public IVelocityMotor {
    IPositionMotor* asPositionMotor() override { return &_posImpl; }
    class PosImpl : public IPositionMotor {
        Motor& _outer;
    public:
        PosImpl(Motor& m) : _outer(m) {}
        void setAngleDeg(uint16_t deg, uint8_t mode) override { _outer.moveToAngle(deg, mode); }
        uint16_t currentAngleDeg() const override { return _outer._lastAngle; }
    } _posImpl;
};
```
Programmer's choice. The inner-impl approach avoids multiple inheritance and is safer
given firmware's `-fno-rtti`.

### Step 3 — Make Servo derive from IPositionMotor canonically

In `source/hal/Servo.h`:
```cpp
// Change:
class Servo : public IServo {
// To:
#include "io/capability/IPositionMotor.h"
class Servo : public IPositionMotor {
```
Update `setAngle(uint8_t degrees)` → `setAngleDeg(uint16_t deg, uint8_t mode)` to
match the `IPositionMotor` interface. Keep `setAngle(uint8_t)` as a compatibility
wrapper or update `ServoController` to call `setAngleDeg`.

**Note on `IPositionMotor::setAngleDeg` vs `IServo::setAngle`:** The old `IServo`
method is `setAngle(uint8_t degrees)`. The new `IPositionMotor::setAngleDeg` takes
`uint16_t deg, uint8_t mode`. Determine what `mode` means for the servo (the Nezha
servo mode byte for `moveToAngle` is 1=shortest, 2=CW, 3=CCW). Use a default
`mode = 0` for the simple hobby-servo case, or add a `setAngle(uint8_t)` convenience
overload on `Servo` that calls `setAngleDeg(deg, 0)`. Update `ServoController` to
use whichever overload keeps the existing behavior.

### Step 4 — Update MockServo

```cpp
// source/hal/mock/MockServo.h
class MockServo : public IPositionMotor {
```
Update methods to match `IPositionMotor` signatures.

### Step 5 — Update Hardware and implementations

`Hardware::gripper()` return type: already `IServo&`; since `IServo = IPositionMotor`
(alias), callers already compile. But canonicalize to `IPositionMotor&`:
```cpp
virtual IPositionMotor& gripper() = 0;
```
Update `NezhaHAL::gripper()` and `MockHAL::gripper()` accordingly.

`Hardware::motorL/R()` return type: already `IMotor&`; canonicalize to `IVelocityMotor&`.

### Step 6 — Update ServoController

`ServoController` stores `IServo& _servo` → `IPositionMotor& _servo`. Update the
constructor and any calls from `setAngle`/`currentAngle` to the new interface methods.

### Step 7 — Update WedgeTest.cpp (ARM only)

`WedgeTest` may call motor methods via `IMotor&`. Since `IMotor = IVelocityMotor`
(alias), it should compile unchanged. Verify textually that no `IMotor`-specific
methods are called that don't exist on `IVelocityMotor`.

## Files to Modify

- `source/io/capability/IVelocityMotor.h` — confirm `asPositionMotor()` accessor present
- `source/hal/Motor.h/.cpp` — derive from `IPositionMotor` (or add inner impl); implement `setAngleDeg`, `currentAngleDeg`
- `source/hal/Servo.h/.cpp` — derive from `IPositionMotor` canonically; update method signatures
- `source/hal/mock/MockServo.h/.cpp` — derive from `IPositionMotor`; update methods
- `source/hal/Hardware.h` — `motorL/R()` → `IVelocityMotor&`; `gripper()` → `IPositionMotor&`
- `source/hal/NezhaHAL.h/.cpp` — update `gripper()` return type
- `source/hal/mock/MockHAL.h/.cpp` — update `gripper()` return type
- `source/control/ServoController.h/.cpp` — use `IPositionMotor&`
- `source/robot/Robot.h/.cpp` — confirm `gripper` ref type; update if needed

## Acceptance Criteria

- [ ] `Motor` implements `IPositionMotor` (directly or via inner impl); `asPositionMotor()` returns non-null.
- [ ] `Servo` derives from `IPositionMotor` (not `IServo` concretely).
- [ ] `MockServo` derives from `IPositionMotor`.
- [ ] `Hardware::gripper()` declared as `IPositionMotor&`.
- [ ] `Hardware::motorL/R()` declared as `IVelocityMotor&`.
- [ ] `ServoController` stores `IPositionMotor&`.
- [ ] `source/hal/IMotor.h` is a shim only (`using IMotor = IVelocityMotor;`).
- [ ] `source/hal/IServo.h` is a shim only (`using IServo = IPositionMotor;`).
- [ ] Simulation tier green: `uv run --with pytest python -m pytest -q`.
- [ ] Golden-TLM canary passes.

## Testing Plan

- Run `uv run --with pytest python -m pytest -q` (full simulation tier).
- Run `uv run --with pytest python -m pytest tests/simulation/unit/test_golden_tlm.py -v`.
- If ARM toolchain present: `python3 build.py` (confirms Servo, NezhaHAL, WedgeTest compile).
- **ARM-only files:** `Servo.cpp` (motor position calls), `NezhaHAL.cpp` (gripper() type).
  Verify textually if no ARM toolchain.
