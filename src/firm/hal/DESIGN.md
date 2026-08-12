---
root: ../../../docs/design/design.md
---

# HAL

**Owner:** Eric Busboom · **Last reviewed:** 2026-08-09 · **Status:** in-flux

---

## 1. Purpose

`hal/` holds the **interfaces for composable devices** — the types a robot
mixes and matches, and the plain-aggregate vocabulary they speak in. It is
what everything above the hardware boundary is written against, so that
swapping a motor controller, an odometry sensor, or a colour sensor is one
new class in `hardware/` plus one line in the composition root, and nothing
else in the firmware changes.

`hal/` sits above `platform/` and `hardware/` and below `kinematics/`,
`motion/`, and `app/`.

## 2. Orientation

```
hal/
  device_types.h   MotorReading, ColorReading, LineReading, PoseReading, Neutral
  device_config.h  MotorConfig, OtosConfig, ColorConfig, LineConfig
  clock.h          Hal::Clock / Hal::Sleeper — time/yield seam
  i2c_bus.h        Hal::I2CBus       — the bus interface
  transport.h      Hal::Transport    — a line-oriented byte pipe (serial/radio)
  motor.h          Hal::Motor        — one actuator channel
  motor_board.h    Hal::MotorBoard   — a multi-channel smart driver board
  otos.h           Hal::Otos         — absolute pose/twist sensor
  color_sensor.h   Hal::ColorSensor  — RGBC
  line_sensor.h    Hal::LineSensor   — N-cell reflectance array
```

Implementations all live in `hardware/` (or, for a part physically welded
to one compute board, in `platform/<target>/hardware/`) — except
`Hal::Transport`, whose two implementations (`Platform::MicroBitSerialPort`,
`Platform::MicroBitRadioLink`) are themselves platform primitives, not
devices, and so live in `platform/microbit/` alongside `MicroBitI2CBus`/
`MicroBitClock` (see `platform/DESIGN.md`) — `TestSupport::FakeTransport`
(`src/tests/sim/support/fake_transport.h`) is the host-test double. The one
deliberate exception among the *device* interfaces used to be
`Core::FakeOtos`, removed as dead code — see §4.

`Hal::Transport` (`readLine()`/`send()`/`sendReliable()`) is the one
interface here consumed by `Core::Comms`, not by `Motion::`/`kinematics::` —
it moved into `hal/` from `core/comms.h`'s own former `#ifndef HOST_BUILD`
block (136-005, "dissolve `com/` into `Hal::Transport` +
`platform/microbit/`"), where it had lived unnamed as `Core::Transport`
since the file's own drafting. Its two ARM-only adapter classes
(`Core::SerialTransport`/`RadioTransport`, thin forwarders onto the
now-deleted `com/SerialPort`/`Radio`) are gone outright, not relocated:
`Platform::MicroBitSerialPort`/`MicroBitRadioLink` implement `Hal::Transport`
directly, the same shape every other `Platform::` leaf already had.

`device_types.h`/`device_config.h` are the vocabulary this whole layer
speaks in: plain aggregates, HAL-local counterparts of the equivalent
`msg::*`/`Config::*` types. They exist as separate types precisely so that
`hardware/` can never see the wire schema (`hardware/DESIGN.md` §3).

## 3. Constraints and Invariants

- **Interfaces only, no chip knowledge.** No register map, no I2C address,
  no vendor timing constant appears in this directory. `Hal::Otos` knows
  there is a pose; `Hardware::RealOtos` knows it lives at 0x17.
- **The interface carries exactly what consumers call, and no more.** Each
  interface here was extracted from a concrete class that the loop was
  already driving through a fixed, small set of methods. Speculative
  surface is worse than none: it has no implementation to keep it honest.
  This is why there is no `Hal::Switch` — the proposal that shaped this
  layer listed one, but no switch exists on any robot in this project and
  nothing would implement or call it.
- **No unit in any identifier.** Units live in a leading bracketed comment
  tag (`// [mm/s]`), per `.claude/rules/coding-standards.md`.

## 4. Design notes and open items

**`Hal::Wheel` is not here yet, on purpose.** The reorganization proposal
describes a `Motor`/`Wheel` split matching Pybricks (`Motor`→`DriveBase`),
WPILib (`MotorController`→`*Drive`+`*Kinematics`) and ROS2
(`ActuatorInterface`→`diff_drive_controller`): the raw actuator commanded
in angular terms, the linear conversion exactly one layer up, and the
velocity PID owned by `Wheel`. Two findings from doing this reorganization
bear on it, and both say the same thing — write it with its migration, not
before:

1. **`Hal::Motor` is duty-commanded, not angular.** Its one command verb is
   `setDuty()` (open loop, `[-1, 1]`). There is no closed-loop
   angular-velocity entry point to build `Wheel` on top of.
2. **The wheel-speed control law lives in `Core::DifferentialDrive`** — `fastPid()`
   plus the Stage A/B/C correction, adaptation and stall machinery in
   `app/drive.cpp`. `Motion::WheelVelocityPid`, which CLAUDE.md's
   architecture note still names as the owner, **does not exist**: sprint
   128 ticket 015 deleted `wheel_velocity_pid.{h,cpp}` as a
   zero-instantiation class (see `src/firm/motion/CMakeLists.txt`'s own note).
   The proposal flagged "conflicting signals on which is authoritative" —
   this is the resolution, and CLAUDE.md is the stale one.

So `Hal::Wheel` is not a new interface next to these; it is the
destination of a migration that moves `Core::DifferentialDrive`'s control law down a
layer and re-scopes it per wheel. Adding an empty `Wheel` interface now
would create a second, competing home for wheel control while the real one
keeps running in `app/` — the exact ambiguity this reorganization exists to
remove.

**`Core::FakeOtos` implemented `Hal::Otos` from `app/`, not `hardware/`**
(kept for history; the class itself is gone, 136-003 — zero robot JSON, CI
script, or justfile recipe ever enabled the `FAKE_OTOS` build variant it
backed). The proposal expected it to land in `hardware/generic/` on the
grounds that it had "zero bus or board dependency (pure math over
`Odometry` + trackWidth)". That was true about the *bus* and wrong about
the *dependencies*: `FakeOtos` read `Motion::Odometry`, which is two
layers ABOVE hardware, so filing it under `hardware/` would have inverted
the layering this reorganization established. Rebasing it onto
`Types::RobotState::pose` instead would have fixed the layering but
changed behavior — `RobotLoop::cycle()` calls `odom_.integrate()` and then
`otos_.tick()`, while `state_.pose` is not written until later in the
cycle, so `FakeOtos` would have started reporting last cycle's pose. It
stayed in the composition layer, as a synthetic device rather than a
driver, until its removal.
