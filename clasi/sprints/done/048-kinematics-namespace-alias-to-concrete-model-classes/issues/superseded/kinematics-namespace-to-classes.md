---
status: in-progress
sprint: 048
tickets:
- 048-001
- 048-002
- 048-003
---

# Replace IKinematics namespace alias with concrete kinematics model classes

## Problem

[`source/kinematics/IKinematics.h`](../../source/kinematics/IKinematics.h)
uses `#ifdef ROBOT_DRIVETRAIN_MECANUM` to alias a *namespace*
(`namespace Kinematics = MecanumKinematics` vs `BodyKinematics`). The stated
goal — let the control stack call kinematics without an `#ifdef` at every call
site — is **not** met: the two namespaces expose different free-function
signatures:

- Mecanum: `inverse(BodyTwist3, const RobotGeometry&, const int8_t signs[4], float wheels[4])`
- Differential: `inverse(float v, float omega, float b, float& vL, float& vR)`
  plus a bolted-on array overload `inverse(BodyTwist3, float b, float wheels[2])`
  that exists only to feed the alias and still isn't signature-compatible.

As a result `BodyVelocityController::advance()` **still** carries a full
`#ifdef` split
([`source/control/BodyVelocityController.cpp:165-197`](../../source/control/BodyVelocityController.cpp)).
The namespace-alias abstraction is leaky and confusing.

This is cleanup of the design introduced in sprint **046**.

## Desired (chosen design: "concrete class + `using` alias")

- `MecanumKinematics` and `DiffKinematics` become concrete **classes**, each
  holding geometry / signs / steerHeadroom as state (constructed once), with
  **identical** member signatures:
  - `void inverse(const BodyTwist3&, float* wheels) const`
  - `void forward(const float* wheels, BodyTwist3&) const`
  - `void saturate(const float* wheels, float vWheelMax, float* out) const`

  Moving `steerHeadroom` into the object unifies `saturate()` (mecanum uses 0).
- A single header keeps `using Kinematics = MecanumKinematics | DiffKinematics`
  under **one** `#ifdef`, plus the existing `constexpr int kWheelCount`.
  - `kWheelCount` **must stay compile-time**: it sizes arrays in
    `DesiredState` / `ActualState` / `OutputState` POD structs; a runtime
    object cannot resize those.
- `BodyVelocityController` holds a `Kinematics _kin{...}` member built in its
  constructor; `advance()` loses its `#ifdef` and calls
  `_kin.inverse/saturate/forward` uniformly.
- **Zero runtime cost** — no vtable, no heap (micro:bit target). Do **not** use
  a virtual `IKinematics` interface.

## Out of scope

- Keep the differential scalar free-function helpers (`forward(vL, vR, ...)`)
  used for telemetry in `MotionCommands.cpp` and `MotionControllerBegin.cpp`.
  Those stay as-is (may become `static` helpers on the class, but their
  call sites should not change behavior).
