---
id: '003'
title: Gripper subsystem with GripperIONull null-object
status: done
use-cases:
- SUC-003
depends-on:
- 043-001
github-issue: ''
issue: migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 043-003: Gripper subsystem with GripperIONull null-object

## Description

Create `source/subsystems/gripper/Gripper.{h,cpp}` — the structural seam for the optional
servo actuator — and the `GripperIONull` null-object for `has_gripper = false`.

The gripper is command-driven (actuation happens via the GRIP command handler through
`ServoController`), not polled each tick. Therefore:
- `Gripper::periodic()` is a no-op this sprint.
- `Gripper::updateInputs()` is a no-op this sprint (no gripper state in `HardwareState`).
- `GripperIONull` provides a no-op implementation of the same interface.
- `loopTickOnce` does NOT call `gripper.periodic()` this sprint.

The `Robot` struct gains a `Gripper` value member. The existing `ServoController servoController`
member remains unchanged — it still wraps the `IPositionMotor&` gripper ref for command dispatch.
This ticket is purely structural: it names the seam and establishes `GripperIONull` so Phase F
has a clean hook.

Depends on 043-001 because `source/subsystems/` directory and CMakeLists wiring is established
in that ticket.

## Acceptance Criteria

- [x] `source/subsystems/gripper/Gripper.{h,cpp}` exist and compile.
- [x] `Gripper` has `periodic()` (no-op) and `updateInputs()` (no-op).
- [x] `GripperIONull` exists (as a class or type alias with no-op methods) in
      `source/subsystems/gripper/Gripper.h`.
- [x] `Robot.h` adds `Gripper gripper_subsystem` value member (or `Gripper gripper_sub` —
      pick a name that does not shadow the existing `IServo& gripper` device ref).
      [Chosen: `subsystems::Gripper gripper_sub` — OQ-4 resolved; does not shadow `gripper`.]
- [x] Existing `ServoController servoController` member is unchanged.
- [x] Existing `IServo& gripper` device ref is unchanged.
- [x] `loopTickOnce` is NOT modified by this ticket (gripper periodic not wired in).
      [Verified: `git diff` on source/control/LoopTickOnce.{cpp,h} is empty.]
- [x] No CODAL/MicroBit/I2CBus types in `source/subsystems/gripper/`.
      [Verified: vendor-confinement grep gate green; forbidden-token grep clean.]
- [x] No `printf` / `telemetryEmit` calls in `Gripper` methods.
- [x] Simulation tier green: `uv run --with pytest python -m pytest -q` >= 2001 passed, 0 errors.
      [2001 passed in 28.90s, 0 errors.]
- [x] Golden-TLM canary byte-exact.
      [test_golden_tlm_unchanged passed — byte-exact oracle green.]
- [x] `defaultRobotConfig()` field-pin diff empty.
      [field-pin test passed; DefaultConfig.cpp restored via git checkout.]
- [x] ARM firmware build gate: `python3 build.py --fw-only` -> 0 errors; then
      `git checkout -- source/robot/DefaultConfig.cpp`.
      [0 `error:` lines, MICROBIT.hex produced; DefaultConfig.cpp restored.]
- [x] Servo command behavior unchanged (GRIP command still dispatches via `servoController`).
      [servo/grip targeted suite: 14 passed.]

## Implementation Plan

### Approach

New files only; no existing method bodies are moved. The `Gripper` class is a thin value-type
with two no-op methods. `GripperIONull` is a second concrete type with the same interface.
Both are pure additions — no existing code is removed or changed.

### Files to Create

**`source/subsystems/gripper/Gripper.h`**

```cpp
#pragma once
#include "io/capability/IPositionMotor.h"
#include <stdint.h>

/**
 * Gripper — subsystem wrapper for the optional servo actuator (Phase E seam).
 *
 * Actuation is command-driven via ServoController; periodic() and updateInputs()
 * are no-ops this sprint. Phase F can wire periodic() into loopTickOnce if
 * gripper state ever needs per-cycle polling.
 */
class Gripper {
public:
    explicit Gripper(IPositionMotor& servo);
    void updateInputs();  // no-op: no gripper state in HardwareState yet
    void periodic();      // no-op: gripper is command-driven, not polled

    IPositionMotor& servo() { return _servo; }

private:
    IPositionMotor& _servo;
};

/**
 * GripperIONull — null-object for has_gripper = false.
 * periodic() and updateInputs() are no-ops; servo() is not called.
 * Eliminates if (has_gripper) guards at the call site.
 */
class GripperIONull : public Gripper {
    // Inherits no-op periodic() and updateInputs().
    // Needs a null IPositionMotor — use a static NullPositionMotor.
};
```

Note on `GripperIONull`: it needs an `IPositionMotor` to pass to `Gripper`. The programmer
should create a minimal `NullPositionMotor` (private inner class or file-scope static in
`Gripper.cpp`) with all-no-op `IPositionMotor` methods, or use a simpler approach: make
`GripperIONull` not inherit from `Gripper` but instead be a completely independent struct with
the same `periodic()` / `updateInputs()` interface (no vtable needed — the call in `Robot` is
by concrete type). Either approach compiles and satisfies the null-object requirement. Choose
the simpler one.

**`source/subsystems/gripper/Gripper.cpp`**
- `Gripper::Gripper(IPositionMotor& servo)` — store ref.
- `Gripper::periodic()` — empty body.
- `Gripper::updateInputs()` — empty body.
- `GripperIONull` bodies (if split from header).

### Files to Modify

**`source/robot/Robot.h`**
- Add `#include "subsystems/gripper/Gripper.h"`.
- Add `Gripper gripper_sub;` value member (name chosen to avoid shadowing `IServo& gripper`).
  Declare after `servoController` — it binds the `gripper` (IServo/IPositionMotor) ref.
- No removal of `IServo& gripper` or `ServoController servoController` — both remain.

**`source/robot/Robot.cpp`**
- Add `gripper_sub(gripper)` to constructor init-list (binds the existing `gripper` IServo ref,
  which is an alias for `IPositionMotor`).

### Testing Plan

1. After creating `Gripper.{h,cpp}` + `Robot.h` changes: `python3 build.py --fw-only` — expect 0 errors.
2. Run full simulation suite: `uv run --with pytest python -m pytest -q` — expect >= 2001 passed.
3. Run golden-TLM canary — must be byte-exact (no behavior change expected).
4. Run GRIP command test if one exists:
   `uv run --with pytest python -m pytest tests/ -k "grip or servo" -v`
5. Run field-pin check.

### Documentation Updates

`architecture-update.md` already documents the `Gripper` / `GripperIONull` design.
No additional doc updates.
