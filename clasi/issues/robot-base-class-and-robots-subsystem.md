---
status: pending
---

# Robot base class and a `src/firm/robots/` subsystem

## Description

Stakeholder directive (2026-08-11). The firmware should be able to run more than
one robot. Today it cannot, because **the identity of "which robot this is" is
spread across a composition root, a generated config header, and a build-time
JSON selection** rather than living in one class.

A plain base class with a concrete subclass per robot. Nothing more elaborate.

### The buses belong to the robot

This is the load-bearing decision, and the current code has it backwards.

Today `Core::composeRobot(bus, clock, sleeper, ...)` takes the I2C bus as a
parameter, and the *only* difference between an ARM build and a sim build is
which `I2CBus` implementation the caller passes in. The bus is a property of the
composition root.

It should be a property of the **robot**. The base class carries the bus
accessors; each robot implements them however it needs:

- Most robots attach their buses to the **platform** bus — on the micro:bit,
  `Platform::MicroBitI2CBus` over `uBit.i2c`.
- A simulated robot attaches an I2C bus that **calls into host callbacks**
  instead of real silicon.
- A robot with no buses at all attaches none.

That inversion is what makes the third robot below expressible as a subclass:
inherit every bit of the Nezha device wiring, change only what the devices talk
over.

### The three robots to plan for

| # | Class | Drivers | Bus | Sensors |
|---|---|---|---|---|
| 1 | `NezhaDifferential` | real | platform I2C | real OTOS, line, colour (as fitted) |
| 2 | fully simulated robot | **none** — simulated plant | none | simulated OTOS, line, colour |
| 3 | simulated Nezha — **derives from #1** | real | callback I2C into the sim | real drivers over simulated silicon |

**#1** exists today, scattered through `Core::RobotGraph`.

**#2 is entirely new.** No buses, no drivers, no register layer — it integrates
a plant directly and answers `Hal::Otos` / `Hal::LineSensor` /
`Hal::ColorSensor` from that plant's truth. **The interfaces it implements
already exist** — `hal/otos.h`, `hal/line_sensor.h`, `hal/color_sensor.h` — so
this robot writes implementations, not new abstractions. That also makes it the
first thing to ever implement `Hal::LineSensor`/`Hal::ColorSensor` other than
the PlanetX drivers, which is a useful test of whether those interfaces are
honest. `TestSim::WheelPlant` and `TestSim::OtosPlant` (`src/tests/sim/plant/`)
are the physics models to build on; simulated line and colour sensing does not
exist in any form yet.

**A fourth is already planned:** a HiWonder robot. Its motor driver is written,
compiled, and bench-characterized (2026-08-02) with a wiring guide at
`docs/hiwonder/hiwonder-motor-board.md` — and has **zero callers**. It is the
cheapest proof the base class generalizes past simulation, and it is the robot
that makes cycle ownership pay: the HiWonder driver reads all four channels in
one exchange, so its `cycle()` has no select/settle phase at all. Out of scope
here, but the base class must not assume the Nezha shape.

**#3 is roughly today's sim path, expressed properly.** `TestSim::SimPlant`
(`src/firm/platform/host/sim_plant.cpp`) is already an `I2CBus` that answers
Nezha motor and OTOS register traffic, and it already has `ReadHook`/`WriteHook`
`std::function` middleware for the ctypes bridge — so the callback mechanism
partly exists. **Known gap: it does not simulate line or colour at all.** Both
addresses NAK (`sim_plant.cpp:121-125` for writes, the fall-through at `:131`
for reads), so those two sensors have always read as absent in sim. Robot #3
should close that or record it deliberately.

### What the base class holds vs. what stays in core

The split is **what hardware this robot has, how it is wired, and what it talks
over** (robot class) versus **what the firmware does with any robot** (core):

| Stays in `Core::RobotGraph` / core | Moves to the robot class |
|---|---|
| `Comms`, `Telemetry`, `Configurator`, `Preamble`, `RobotLoop` | the two drive motors and their `MotorArmor` decorators |
| `Motion::Odometry`, `Planner`, `Navigator`, `NavigatorLimits` | `RealOtos` / whatever odometry sensor this robot has |
| `Control::DifferentialDrive` (the control law) | the colour and line sensor leaves, if fitted |
| `BootOverrides` (the sim/hardware exception seam) | `BootValues` / `bakeBootValues()` — per-device construction-time config |
| `Clock`/`Sleeper`, transports, `TuningStore` | **the I2C / SPI / UART buses** |
| the *policy* — what each module does | **the cycle schedule** — when each module runs, and the period |

Two rows deserve emphasis. `I2CBus` moves *out* of the injected-leaves column;
`Clock`, `Sleeper`, transports and `TuningStore` stay injected, because they are
platform services every robot shares rather than hardware this robot has.

### The robot owns its cycle schedule

Stakeholder decision, restated 2026-08-11. The hardware imposes the loop shape,
so the loop shape belongs to the hardware class.

`RobotLoop::cycle()` (`src/firm/core/robot_loop.cpp:754`) is not a generic loop
with a hardware hook in it — it *is* the Nezha schedule, because the brick
latches exactly one pending encoder read per select:

```
mark time → zeroUnownedMotion + haltOnStall → drive.tick (stage duty)
  → requestSample(L) → settle 4 ms (pump comms) → tick(L)   [collect]
  → requestSample(R) → settle 4 ms (pump + route commands) → tick(R)
  → publishWheels()        <- deliberately HERE: same-generation L/R coherence
  → runAndWaitUntil(cycleStart + kCycle) { pump; odometry; otos; publishes;
                                            planner; telemetry }
```

Every element of that is Nezha-specific: the split-phase select/settle/collect
dance, the 4 ms settle windows, the two-pass structure, and the exact point
`publishWheels()` lands. A different motor driver wants a different shape — the
HiWonder board reads all four channels in one exchange, so it has no select
phase at all.

So `cyclePeriod()` and `cycle()` go on the base class, and the core phases
become **services the robot calls at points it chooses**.

**Little should be left of `RobotLoop`** (stakeholder, 2026-08-11) — the loop is
in the robot now. What remains is the outer shell, the `CycleServices`
implementation, and whatever command-routing and blackboard-publishing bodies
those services delegate to. `robot_loop.cpp` is 942 lines today; if the sprint
finishes with it still near that size, the schedule did not actually move.
Expect the command router and the nine `publish*` methods to become their own
classes rather than staying in a shell that is supposed to be thin.

Two invariants the sprint must preserve, both hard-won:

- **`idleFor()` pumps comms, then sleeps.** The settle windows are not dead time;
  they are where inbound commands get drained. A robot class must never own a
  private sleeper or call a bare sleep — `.claude/rules/` is explicit that a
  wedged fiber makes the radio look dead.
- **Pacing is against an ABSOLUTE end-of-cycle deadline** (131-005), not a
  fixed trailing gap. `computeUntil(deadline)` keeps that by construction.

Control period becomes per-robot. Nezha stays at `kCycle`. Validating the motion
library at other periods is deferred until a robot class with a different period
actually lands.

## Base class sketch

Namespace `Robots`, header `src/firm/robots/robot.h`. Every member below is
justified by something `RobotGraph` or `RobotLoop` consumes today, or by one of
the three robots above. Nothing speculative.

```cpp
#pragma once

#include <cstdint>

#include "hal/color_sensor.h"
#include "hal/i2c_bus.h"
#include "hal/line_sensor.h"
#include "hal/motor.h"
#include "hal/otos.h"

namespace Hal {
class SPIBus;   // not written yet -- see "Buses" below
class Uart;     // not written yet
}  // namespace Hal

namespace Robots {

// CycleServices -- the core phases a robot's cycle() calls, handed to it by
// the outer shell. Every one of these is a slice of today's
// Core::RobotLoop::cycle(); the robot chooses the ORDER and the timing, the
// core keeps the POLICY.
class CycleServices {
 public:
  virtual ~CycleServices() = default;

  virtual uint64_t nowMicros() const = 0;   // [us]

  // Borrow a required gap: pump comms, THEN sleep the remainder. The only
  // sanctioned way for a robot to wait -- a bare sleep inside a cycle
  // starves the fiber scheduler and the radio goes quiet.
  virtual void idleFor(uint32_t gap) = 0;   // [ms]

  // Drain the command ring and dispatch. Safe to call inside an idleFor()
  // body, which is what the Nezha schedule does in its second window.
  virtual void routeCommands() = 0;

  // Motion arbitration + the control law's tick: stages this cycle's duty
  // on the robot's motors. Called BEFORE any bus traffic.
  virtual void actuate() = 0;

  // Publish the wheel half of the blackboard. Separate from computeUntil()
  // because WHERE it lands is a robot decision -- Nezha calls it at the
  // point both wheels hold same-generation readings.
  virtual void publishWheels() = 0;

  // The tail, bounded by an ABSOLUTE deadline (131-005): pump, odometry,
  // odometry sensor, remaining publishes, planner, telemetry, then pace to
  // `deadline`. The robot passes its own cycle start + cyclePeriod().
  virtual void computeUntil(uint32_t deadline) = 0;   // [ms] absolute
};

// Robot -- everything that is true of ONE physical (or simulated) robot: the
// buses its devices talk over, the devices themselves, its chassis geometry,
// and its cycle schedule. Core owns the comms plane, the control law and the
// motion library, and reaches hardware only through this interface.
//
// Per-device construction-time configuration arrives through the concrete
// class's own constructor, NOT through a setter here: the device leaves
// (NezhaMotor, RealOtos, the PlanetX sensors) have no post-construction
// setters for their wiring, lever arm, or scales. See "Open questions".
class Robot {
 public:
  virtual ~Robot() = default;

  // --- Identity -------------------------------------------------------
  // Stable, lower-case, wire-visible once robot selection exists.
  virtual const char* name() const = 0;   // "nezha-differential"

  // --- Buses ----------------------------------------------------------
  // What this robot's devices talk over. Default: nothing attached; a
  // robot overrides only the buses it actually has. The fully-simulated
  // robot overrides none of them.
  //
  // These are ACCESSORS, not owners-by-contract: whether the bus is a
  // member of the concrete robot or a reference handed to its constructor
  // is that robot's business. See "The bus and the constructor" below for
  // why the base must not call these during construction.
  virtual Hal::I2CBus* i2c() { return nullptr; }
  virtual Hal::SPIBus* spi() { return nullptr; }
  virtual Hal::Uart* uart() { return nullptr; }

  // --- Lifecycle ------------------------------------------------------
  // detect() is ONE bounded probe step -- it must not block or sleep, and
  // returns false to mean "call me again next pass". begin() runs once,
  // after detect() has succeeded. Core::Preamble drives these and keeps
  // its own job: emitting boot frames and gating done().
  virtual bool detect() = 0;
  virtual void begin() = 0;

  // --- The cycle ------------------------------------------------------
  // This robot's loop shape and period. cycle() runs ONE full iteration:
  // the robot decides when duty is written, when encoders are sampled,
  // where settle windows fall, and where the blackboard is published --
  // calling back into `services` for every core phase. The outer shell
  // (Core::RobotLoop) provides the services and paces nothing itself.
  //
  // cycle() must not sleep except through services.idleFor(), and must
  // end by handing its remaining time to services.computeUntil() so the
  // absolute end-of-cycle deadline is honoured.
  virtual uint32_t cyclePeriod() const = 0;          // [ms] nominal
  virtual void cycle(CycleServices& services) = 0;

  // --- Drive ----------------------------------------------------------
  // Hal::Motor&, never the concrete armor type: core must not know that a
  // Nezha wheel is a NezhaMotor wrapped in a MotorArmor decorator.
  virtual Hal::Motor& driveLeft() = 0;
  virtual Hal::Motor& driveRight() = 0;

  // Chassis geometry this robot's devices were configured against. The
  // resolved value, after any BootOverrides the caller applied.
  virtual float trackWidth() const = 0;   // [mm]

  // --- Optional sensors -----------------------------------------------
  // nullptr means NOT FITTED, and that is the normal case, not an error:
  // gopiv has none of the three (telemetry flags 216) and drives fine.
  // Core must branch on the pointer, never assume presence.
  virtual Hal::Otos* otos() { return nullptr; }
  virtual Hal::LineSensor* lineSensor() { return nullptr; }
  virtual Hal::ColorSensor* colorSensor() { return nullptr; }
};

}  // namespace Robots
```

### Notes on the sketch

**Buses.** `Hal::I2CBus` is real after the layering-cleanup issue moves it out
of `Platform::`. `Hal::SPIBus` and `Hal::Uart` **do not exist** — there is no SPI
or UART interface anywhere in `src/firm`. The accessors are declared here over
forward declarations so the seam is reserved as the stakeholder asked; the
interfaces themselves get written when the first robot actually needs one, not
speculatively.

**`NezhaDifferentialSim` is a subclass** (stakeholder decision, 2026-08-11) — it
derives from `NezhaDifferential` and swaps the bus, so it inherits every bit of
the device wiring by construction rather than by copy.

**The bus and the constructor — a C++ trap the implementation must respect.**
The subclass cannot swap the bus by overriding `i2c()`, because a virtual call
during *base* construction resolves to the base's version: `NezhaDifferential`
would build its devices on the platform bus before the override ever existed.
The bus must therefore reach `NezhaDifferential`'s constructor as a parameter,
and `NezhaDifferentialSim` passes its own callback bus upward. Since a member of
the derived class is not constructed until after the base is, the callback bus
needs the base-from-member idiom (a small private holder base declared first) or
an equivalent. The subclass then earns its keep by **owning the callback bus's
lifetime and exposing the host callback-registration surface** — which is
exactly what a bare constructor parameter could not give you.

**No `configure()`.** Deliberately absent. The values a robot needs at
construction — per-port motor wiring, the OTOS lever arm and scales,
`ColorConfig`, `LineConfig` — have no setters on the device leaves, which is the
entire reason `BootValues` exists. Adding a `configure()` that silently cannot
apply half of what it is given would be worse than not having one. Configuration
reaches a robot through its own constructor, from the generated artifact below.

**`NezhaDifferential::cycle()` is a literal transcription** of
`robot_loop.cpp:754-793`, service call for service call. The literalness is the
regression defense: this is bench-calibrated timing, and the 4 ms settle windows
and absolute-deadline pacing were each bought with measured hardware sessions.
Transcribe; do not rewrite.

## Configuration stays a build-time generated artifact

Stakeholder decision (2026-08-11): **there should still be a configuration object
created at configuration time, before compilation, that generates a data file.**
The robot class consumes that artifact; it does not replace it and it does not
hand-author values.

That is the existing shape — `data/robots/<robot>.json` → `gen_boot_config.py` →
a generated artifact the firmware reads at boot — and it is what
`.claude/rules/configuration-discipline.md` requires: one file per configured
value, the same file feeding both the runtime push path and the bake path.

One thing to settle at sprint time: **generated C++ or a genuine data blob?**
Today `gen_boot_config.py` emits `src/firm/config/boot_config.cpp` — generated
*code*, compiled in. The stakeholder said "data file." The distinction is not
cosmetic:

| | generated C++ (today) | data blob |
|---|---|---|
| multi-robot bake table | N robots' constants all linked in | one indexed blob, one lookup |
| adding a robot | recompile | reflash data, possibly not code |
| type safety | compiler-checked | parsed at boot, must fail closed |

The multi-robot future (a table of every `data/robots/*.json`, selection at
runtime) is the case that actually pushes toward a blob. Decide it deliberately —
and note the change is orthogonal to the robot class itself, so it can land
before, with, or after.

Where the artifact lives is also open. It is per-robot data, so `robots/` is
arguable; it is generated config, so `config/` is where its ancestor lives.

## Cause

Three separate decisions accumulated into one hard-coded robot:

1. **Sprint 130-002 unified the two composition roots** (`main.cpp` and
   `TestSim::SimHarness` had drifted, silently disabling wheel trim in every sim
   session). That was the right fix and must be preserved — but it unified them
   around a graph that names `Hardware::NezhaMotor` by concrete type and takes
   the bus as a parameter, so the single root is now also a single-robot root.
2. **`BootValues` exists because device leaves have no setters.** Its values must
   be complete before construction, so they cannot come from `Core::Configurator`
   (itself constructed after the leaves it references). That forces per-device
   config into the composition root.
3. **Robot selection is a build-time file swap.** `data/robots/active_robot.json`
   points at one robot; `gen_boot_config.py` bakes it into
   `src/firm/config/boot_config.cpp`. There is no runtime concept of "which
   robot" — and `Config::kDrivetrainType` is baked from the JSON but **nothing
   reads it**, which is exactly the hook a robot-class selector wants.

## Proposed fix

1. **Write `Robots::Robot`** per the sketch above.
2. **`NezhaDifferential`** as a literal transcription of what `RobotGraph`
   constructs today — same declaration order, same armor wiring, same
   `bakeBootValues()` content, plus the platform I2C bus. Literalness is the
   regression defense; this is the code that drives the robot.
3. **Move the schedule.** `NezhaDifferential::cycle()` transcribes
   `robot_loop.cpp:754-793`; `RobotLoop` becomes the outer shell and the
   `CycleServices` implementation. **Zero timing change** — this step is
   verified by the delivered period being indistinguishable before and after,
   not by tests alone.
4. **`RobotGraph` takes `Robots::Robot&`** the way it already takes
   `Clock`/transports, and drops the `bus` parameter. It keeps `Comms`/
   `Telemetry`/`Drive`/motion/`RobotLoop`, wired against `robot.driveLeft()` etc.
5. **Both composition roots construct a robot class** — `main.cpp` builds
   `NezhaDifferential`, `SimHarness` builds `NezhaDifferentialSim` — with
   `test_composition_root_parity.py` green throughout.
6. **Then the two sim robots**, in whichever order the sprint prefers:
   `NezhaDifferentialSim` (#3, mostly relocating `SimPlant`) and the fully
   simulated robot (#2, genuinely new, and the one that needs simulated line and
   colour sensors written from scratch).
7. **Leave runtime selection for later.** A baked table of all
   `data/robots/*.json`, a selection verb, flash persistence — separate, larger
   work. This issue's deliverable is that a new robot class becomes *writable*
   without touching core.

### Open questions for the sprint

- **The two-path config problem, restated plainly.** Some settings can only be
  applied when a device object is *built*, because the device classes have no
  method to change them afterward — there is no `NezhaMotor::setPort()`, no way
  to re-seat the OTOS lever arm on a live `RealOtos`. About fourteen values are
  like this (per-port motor wiring, the OTOS offsets and scales, the colour and
  line sensor configs). They must be known *before* construction, which is the
  only reason `BootValues`/`bakeBootValues()` exists as a separate struct
  computed first.

  Meanwhile `Core::Configurator` holds its *own* copy of the robot's settings,
  read from the same JSON through a different generated function family, for
  read-back and live pushes. **Two code paths, one source file.** That is the
  wart. `.claude/rules/configuration-discipline.md` wants one file feeding both
  paths, and it does — but through two generators that can drift.

  Moving `BootValues` into the robot class **relocates this without curing it**.
  Curing it means either giving the device classes setters (so everything can go
  through `Configurator`) or making `Configurator` a pure projection of one
  object. Both are bigger than this issue. Decide whether to attempt it here or
  write it down and move on.

**Not a contradiction:** the layering-cleanup issue deletes `Core::FakeOtos`, and
robot #2 needs a simulated OTOS. These are different things. `FakeOtos`
synthesizes a pose from *encoder odometry* on a real ARM robot with no OTOS chip
fitted; robot #2's simulated OTOS reports its *plant's ground truth*. Deleting
one does not remove the other.

## Verification

- `test_composition_root_parity.py` green at every step — it is the guard that
  made this refactor safe last time.
- **Timing, for the cycle move specifically.** Moving the schedule is the one
  step here that can quietly wreck a working robot, and no unit test will catch
  it. Two checks, both required: a harness asserting the same window sequence
  before and after, and a **delivered-period measurement on hardware** compared
  against the current number. Anything but "indistinguishable" is a failure, not
  a tuning opportunity.
- Full `uv run python -m pytest` at sprint end.
- **Hardware gate on the stand.** This touches motor construction and armor
  wiring, so tests alone do not close it: flash by UID, then
  `src/tests/bench/twist_drive.py` and `src/tests/bench/move_protocol_bench.py`.
  `gopiv` is the better target than `tovez` here — two motors, no OTOS, line, or
  colour, so it exercises the optional-hardware paths (the ones most likely to
  break) by construction.
- Robot #2 needs its own acceptance that does **not** route through hardware: it
  should drive the same protocol surface the sim already proves
  (`test_sim_wire_loopback.py`) with no `SimPlant` and no register traffic
  anywhere in the path.

## Related

- `src/firm/core/boot_wiring.h` — `Core::RobotGraph`, the file this is about.
- `src/firm/platform/host/sim_plant.{h,cpp}` — the existing callback-capable
  simulated I2C bus robot #3 is built from, including its `ReadHook`/`WriteHook`
  middleware and its line/colour NAK gap.
- `src/tests/sim/plant/{wheel_plant,otos_plant}.{h,cpp}` — the physics models
  robot #2 builds on.
- `clasi/issues/firmware-layering-cleanup-interfaces-to-hal-impls-to-platform-microbit.md`
  — should land first; it puts `Hal::I2CBus` in `hal/` (this sketch assumes that
  name), moves the control law out of `core/`, and renames
  `Hal::MotorBoard`/`HiwonderBoard`/`BoardMotor` to motor-*driver* naming, which
  the HiWonder robot class then inherits.
- `docs/hiwonder/hiwonder-motor-board.md` — the wiring guide for the HiWonder
  driver, characterized on the bench 2026-08-02.
- `clasi/issues/main-cpp-holds-code-that-does-not-belong-in-main.md` — the small
  companion cleanup of the same composition root.
- `clasi/issues/kernel-packaging-host-sim-rigor-and-hardware-abstraction-program-plan.md`
  — the larger program this is one piece of. **Stale** (written 2026-08-07; the
  platform/hardware/hal reorganization landed 2026-08-09) and flagged for
  re-planning: it names `src/firm/app/`, `src/firm/devices/`, `src/sim/`,
  `src/motion/`, `App::`, `Devices::`, none of which exist. Read its intent, not
  its paths.
- `src/archive/source_old/hal/` — the deleted prior-generation HAL (capability
  interfaces + a Hardware registry), removed by sprint 102 in favour of the
  injected-leaves pattern. Worth reading as a record of what was tried and
  rejected.
