---
status: pending
sprint: '136'
tickets:
- 136-008
---

# Platform / Hardware / HAL / Core reorganization

## Description

A large, mostly-mechanical reorganization of `src/firm` to make the firmware
portable across compute platforms (micro:bit today; Raspberry Pi, Spike
Prime / Kipp-Wallaby-style variants, possibly an ESP32-class board, later)
instead of micro:bit-specific throughout. Introduces a layered
Platform → Hardware → HAL → Kinematics → Core/Robot architecture, folds
`src/motion` back into `src/firm`, and cleans up naming (`app` → `core`,
`Drive` → `DifferentialDrive`, `BodyKinematics` → a swappable Kinematics
subsystem) along the way.

Target layout:

```
src/firm/
  platform/
    i2c_bus.h, clock.h, ...   platform-primitive interfaces
    microbit/
      microbit_i2c_bus.*, microbit_clock.*, ...   primitive impls
      hardware/               hardware that ONLY exists on this board
                               (e.g. the micro:bit's onboard compass)
    host/                     the sim platform (was src/sim/)
      hardware/               sim-only fake hardware, if any turns out to be
    nezha/                    a named hardware-board family, not a compute
                               target (see Proposed fix / Hardware)
      hardware/                NezhaMotor lives here, not under generic/
  hardware/
    generic/    device drivers that are truly standard-protocol and
                reusable by anyone on any I2C/SPI/UART-capable platform —
                RealOtos (a commercial, publicly-documented part) qualifies;
                NezhaMotor does not.
  hal/        interfaces for composable devices (Motor, Wheel, sensors).
  kinematics/ differential / mecanum / (rare) X-drive twist<->wheel maps.
  motion/     Planner, Navigator, odometry — moved back in whole.
  core/       orchestration: RobotLoop/cycle, Comms, Telemetry, Configurator,
              Robot composition. (Was `app/`.)
  messages/, types/, config/, com/   — unchanged.
```

Everything nests under `src/firm/`, not as new top-level `src/` siblings —
`src/firm` is the tree CLAUDE.md already designates for an eventual
`git subtree split` into its own repo, so platform/hardware/hal/core/motion
should travel together as one unit when that happens; the `motion/`/
`kinematics/` stub directories that already exist inside `src/firm/`
(DESIGN.md only, no code) only make sense under that reading.

## Cause

`src/firm` is micro:bit-specific throughout: device drivers, buses, and
orchestration code are not separated by portability, so there is no clean
seam for a second compute platform. Specific symptoms motivating the
reorg:

- `src/motion` was split out as a sibling tree in sprint 122 specifically
  "so agents didn't get confused" — that reason no longer applies now that
  the system works, and the split costs cross-tree include-path friction.
- `App::Drive` hardcodes `trackWidth` and differential-drive kinematics
  directly (`drive.h:11-13`) rather than delegating to a swappable
  kinematics model, so a mecanum or X-drive robot has no home.
- `App::Comms` hardcodes exactly two named transport slots
  (`Transport& serialLink_`, `Transport& radioLink_`, `comms.h:208-209`)
  rather than a collection, blocking a third transport (WiFi) without a
  structural change.
- `Types::RobotState` is a flat POD struct with no inheritance or extension
  mechanism — every robot variant would need to edit the one shared struct
  directly.
- `App::FakeOtos`, a real-hardware fallback (see Hardware below), currently
  sits inside `boot_wiring.h` next to pure orchestration code rather than
  with the other hardware drivers.
- Naming: `app` doesn't communicate "orchestration core," and `Drive` reads
  as generic when it is differential-drive-specific in everything but name.

Encouragingly, several of the target pieces already exist, half-formed, in
the right shape, which substantially de-risks this reorg:

- `Devices::I2CBus`, `Devices::Clock`/`Sleeper`, `Devices::Motor`,
  `Devices::Otos` are already pure abstract interfaces with separate real
  (micro:bit) and sim implementations — Platform/HAL work already done,
  just not organized under that name.
- `App::composeRobot()` / `App::RobotGraph` (`boot_wiring.h`/`.cpp`) is
  already the Robot composition root, and already platform-parameterized —
  `main.cpp` and `src/sim/sim_harness.h` call the identical function,
  differing only in which `I2CBus`/`Clock`/`Sleeper` gets passed in.
  `src/sim` is, today, the "host platform."
- `App::Comms` already has a `Transport` interface (`readLine`/`send`/
  `sendReliable`, `comms.h:23-32`) with `SerialTransport`/`RadioTransport`
  implementations — adding WiFi extends an existing pattern.

## Proposed fix

### Platform

Board/runtime primitives only: bus I/O and timing, nothing device-specific.
Mostly a relocation, not new design — `Devices::I2CBus` and
`Devices::Clock`/`Sleeper` (`src/firm/devices/i2c_bus.h`, `clock.h`) are
already exactly this shape (pure interface; `MicroBitI2CBus`/`MicroBitClock`
real impls; `TestSim::SimPlant`/`SimClock`/`SimSleeper` sim impls in
`src/sim/`). Move and rename `Devices::` → `Platform::` for these
specifically, and promote `src/sim` into `src/firm/platform/host/` — it
already builds real firmware against substituted bus/clock, which is the
whole job of a platform.

```
src/firm/platform/
  i2c_bus.h, clock.h, ...       interfaces (today's devices/i2c_bus.h, clock.h)
  microbit/                     MicroBitI2CBus, MicroBitClock, MicroBitSleeper
  host/                         SimPlant, SimClock, SimSleeper (was src/sim/)
```

New surface to add (not present today): UART, GPIO, analog/digital pin
interfaces. No current device needs them yet (everything today is I2C), so
these should be shaped by the first Raspberry-Pi or WiFi-module port that
actually needs them, rather than speculatively designed now.

### Hardware

`I2CBus&`-only is not sufficient to make something "generic" — a custom,
project-specific board with its own bespoke wire protocol riding over I2C
is not something another integrator could reuse, even though the bus itself
is generic. `NezhaMotor` is exactly this: it is this project's own
motor-controller board, so it does not belong in `hardware/generic/`
despite only touching `I2CBus&`. Three categories, not two:

- **Generic hardware** (`src/firm/hardware/generic/`) — genuinely
  standard-protocol parts reusable by anyone on any I2C/SPI/UART-capable
  platform without project-specific knowledge. `RealOtos` (a commercial,
  publicly-documented sensor) is the clean example. Whether
  `ColorSensorLeaf`/`LineSensorLeaf` are standard parts or project-specific
  boards like Nezha needs the same check before being filed here — not yet
  done.
- **Named-hardware-family directories, nested under `platform/`** —
  `NezhaMotor` goes to `src/firm/platform/nezha/hardware/`. This stretches
  "platform" beyond "the processor you're running on" to also cover a
  specific peripheral-board family that isn't itself a compute target —
  Nezha doesn't run this firmware, it's a motor controller the firmware
  talks to. An alternative that keeps "platform" strictly compute-target-only
  is a sibling tree instead, `src/firm/hardware/nezha/` — open decision, see
  below.
- **Compute-platform-intrinsic hardware** — hardware physically part of one
  compute board that cannot exist anywhere else (the micro:bit's onboard
  compass): `src/firm/platform/microbit/hardware/`. The
  KIPR-Wallaby-is-a-Pi-variant case fits the same slot when it shows up.

All three still implement the same `Hal::` interfaces where one applies
(the onboard compass would still be a `Hal::Compass`, `NezhaMotor` is still
a `Hal::Motor`) — the split is about where the driver lives and what else
can reuse it, not about whether it participates in HAL.

`App::FakeOtos` (`boot_wiring.h:326-332`) is a build-time option for a real
micro:bit robot built **without** a physical OTOS chip (encoder +
trackWidth stands in) — its own comment states "FAKE_OTOS builds only —
never true for HOST_BUILD/sim," i.e. it is not a sim-only construct. It has
zero bus or board dependency (pure math over `Odometry` + `trackWidth`),
which makes `hardware/generic/` the likely home, arguably an even better fit
than RealOtos. Placement not yet confirmed.

### HAL

Interfaces for devices meant to be mixed and matched or shared across
robots. `Devices::Motor` (`src/firm/devices/motor.h:43-137`) is already
exactly this: a pure abstract interface (14 virtuals), two implementations
(`NezhaMotor`, `MotorArmor` as a decorator). Move and rename
`Devices::Motor` → `Hal::Motor`, same treatment for `Otos`.

Motor vs. Wheel split, matching the pattern used by every framework surveyed
(Pybricks' `Motor` → `DriveBase`, WPILib's `MotorController` →
`*Drive`+`*Kinematics`, ROS2's `ActuatorInterface` → `diff_drive_controller`:
the raw actuator is commanded in angular terms, linear conversion happens
exactly one layer up):

- `Hal::Motor` — angular, commanded in deg/s or rad/s. Today's
  `Devices::Motor` interface is close, but its command entry point is
  `setDuty()` (open-loop) — whether a real closed-loop angular-velocity
  command entry point already exists elsewhere (candidates:
  `App::Drive::fastPid()`, or `Motion::WheelVelocityPid` per CLAUDE.md's
  architecture note) needs a direct check before `Hal::Wheel` is written —
  current research surfaced conflicting signals on which is authoritative.
- `Hal::Wheel` — linear, mm/s or m/s. Wraps a `Hal::Motor&` + diameter (+
  optionally inertia), owns the velocity PID that holds the commanded
  speed. New class — doesn't exist yet. This is where today's
  `App::Drive`-embedded wheel-speed control (PID, stall detection,
  deadband/wedge handling — the "motor armor" policy) eventually migrates,
  as its own follow-on piece of work, not part of this reorg's first pass.
- Chassis-level concerns (track width, wheelbase) stay above Wheel, in
  Kinematics — per-wheel scoping (Wheel, not a whole-chassis DriveBase) is
  the more composable choice, matching how ROS2 treats a single wheel joint
  before a chassis-level controller sits on top.
- `Hal::LineSensor` — N reflectance cells, exposing a computed centerline
  position, not just raw channel values (today's `Devices::LineSensorLeaf`
  returns raw packed bytes; centroid math currently lives host-side in
  `src/tests/bench/line_follow.py`'s `line_error()`). Ties directly into
  the on-robot LineFollower proposal (see Related) — its centroid math
  would become this HAL method rather than a bench-script formula.
- `Hal::ColorSensor`, `Hal::Switch` — ColorSensor maps onto today's
  `Devices::ColorSensorLeaf`; Switch is new (not currently modeled).

### Kinematics (new top-level subsystem)

Differential and mecanum cover almost everything, with X-drive as a rare
third case — not a large plugin system. `src/motion/body_kinematics.h` is
already a stateless twist↔wheel-speed map, but hardcoded to differential
(`inverse`/`forward` take a single track width `b`, no interface).
Formalize as a small interface with named implementations:

```
src/firm/kinematics/
  kinematics.h              interface: inverse(twist) -> wheel speeds,
                             forward(wheel speeds) -> twist
  differential_kinematics.*  today's BodyKinematics, renamed
  mecanum_kinematics.*        new
```

A robot's composition (`RobotGraph`/Robot) picks one at construction, the
same way it picks Serial vs. Radio transports today. `Motion::Planner`
(which currently calls `BodyKinematics` functions directly for its
per-wheel profiling) takes a `Kinematics&` instead of assuming differential.

### Motion — moved back in whole

`src/motion/` → `src/firm/motion/` (the placeholder directory already
there). Three separate CMake projects today (`motion_tests`, `planner_tests`,
`navigator_tests`, ~30 files / ~10k LOC total) — move the directories as a
unit; merging the three CMake projects into one build is a separate,
optional decision, not required for the move itself.

Keep the existing dependency-isolation rule (`src/motion/DESIGN.md` §3: no
`Config::`/`App::`/`Devices::` imports, only `firm/types` and `messages/`)
after the physical move — that rule is exactly what makes Motion portable
across platforms, which is this reorg's whole point. The same discipline
(no upward dependencies) should extend to the new Platform/Hardware/HAL/
Kinematics layers: Platform knows nothing about Hardware, Hardware knows
nothing about HAL, HAL knows nothing about Kinematics or Core.

`.clasi/config.yaml`'s validated `sources:` list (`[src/firm, src/host]`)
already covers `src/firm` — since `motion/`/`kinematics/` stub dirs already
exist inside it, this is a same-root move, not the vendor-symlink validator
trap `docs/design/design.md` §4 documents as the reason bare `src` is kept
off the roots list. The concrete housekeeping item is porting each
subsystem's `DESIGN.md` content into the (currently near-empty) stub files
at `src/firm/motion/DESIGN.md` / `src/firm/kinematics/DESIGN.md`.

### Core (renamed from `app`)

Orchestration only: `RobotLoop`/`cycle()`, `Comms`, `Telemetry`,
`Configurator`, boot sequence, and the Robot composition
(`composeRobot()`/`RobotGraph`, formalized as a proper `Robot` class — see
below). `FakeOtos` moves out of `app`/`core` regardless of final placement
(generic hardware is the current best read — see Hardware above); flag
anything else spotted misplaced during the actual move.

### Drivetrain naming: `Drive` → `DifferentialDrive`

`App::Drive` (`drive.h:11-13`) already takes `trackWidth` directly in its
constructor and owns per-wheel PID against raw `Devices::Motor&` — it is
differential-specific today in everything but name. Once Kinematics is
pulled out and Wheel exists, `Drive` becomes `DifferentialDrive`
implementing a small common drivetrain interface (tick, command a body
twist, report wheel state) that `MecanumDrive` implements alongside it —
both built from `Hal::Wheel`s + a `Kinematics&`, with `RobotLoop` ticking
whichever one the robot's composition picked, generically.

### Comms / Transport → multiple transports

`App::Transport` (`comms.h:23-32`) already exists — `readLine`/`send`/
`sendReliable` — with `SerialTransport`/`RadioTransport` implementations.
The gap: `Comms` hardcodes exactly two named slots rather than a
collection. Generalize to a small fixed-size array of `Transport*` (no
heap, matching the rest of this codebase) registered at Robot-composition
time, with `pump()` looping over all registered transports instead of
calling two named methods. Add `WifiTransport` wrapping the AI-Thinker
Ai-WB2-12F module — already bench-verified (UDP/TCP, ~50ms passthrough RTT,
per `src/tests/bench/wifi/`) — as the third implementation. The WiFi
module's own driver (AT-command/UART handling) lives in Hardware;
`WifiTransport` itself sits beside `SerialTransport`/`RadioTransport` in
Core, since `Transport` is a wire-protocol-adjacent concept (COBS framing,
line reliability), not raw device I/O.

### Robot

Formalizes what `App::composeRobot()`/`App::RobotGraph` already does. Today
it is pure wiring — constructs the device graph, does not own bus
arbitration (the caller constructs and passes in the shared `I2CBus`), and
does not own an extensible state (`Types::RobotState` is a flat POD struct,
no inheritance). Two design decisions remain open when this becomes
concrete:

1. **Bus arbitration.** Today's I2C exclusivity is implicit: one shared bus
   instance, a single-threaded cooperative loop serializing access by tick
   order, a diagnostic-only re-entrancy counter (`MicroBitI2CBus::inUse_` —
   counts violations, not a lock), and a cooperative "clearance safety net"
   that makes callers sleep until a settle deadline rather than spinning.
   Making Robot "own the buses" does not necessarily mean adding a runtime
   lock — the cooperative single-fiber model may still be correct — but
   Robot should be the explicit, documented owner of "who may touch the bus
   and when," rather than that being an emergent property of `RobotLoop`'s
   tick order as it is now. Worth a closer look before locking in, given how
   central the Nezha-bus-exclusivity rule is to the hardware working at all.
2. **RobotState extension.** Flat POD today; every current consumer assumes
   the one concrete type. Making it something robots can inherit and extend
   is the highest-blast-radius single change in this reorg. Options range
   from a base struct + per-robot derived struct to a plain composition slot
   (a generic base plus an opaque robot-specific "extra" member) — deserves
   its own design pass rather than a call made here.

### Verification (136-008, independently re-checked against the tree)

Steps 1-6 below are CONFIRMED landed, each checked directly against the
current tree/git history rather than re-asserted (see full evidence in
sprint 136 ticket 008's commit message):

- **Step 1 (Platform split):** commit `8a86f5bd` ("split Platform out of
  devices/ (reorg step 1)"). `src/firm/platform/{host,microbit}/` exist;
  `src/firm/devices/` does not (`ls src/firm/devices` -> no such file).
- **Step 2 (Hardware split) / Step 3 (HAL rename):** commit `13b3116d`
  ("split devices/ into hal/ (interfaces) and hardware/ (drivers)"),
  between `8a86f5bd` and `1c7b70d3` in `git log`. `grep -rn "namespace
  Devices" src/firm/{hal,hardware,platform,core,control,kinematics}`
  returns zero hits; `hal/`/`hardware/` exist with `Hal::`/`Hardware::`
  namespaces throughout.
- **Step 4 (Motion move):** commit `812d6708`. `git ls-files src/motion
  src/sim` returns empty (both trees are stale, untracked build residue
  only, confirmed deleted outright by this sprint's ticket 003); `src/firm/
  motion/` is populated and tracked.
- **Step 5 (Kinematics extraction):** commit `09892f60`. `src/firm/
  kinematics/kinematics.h` declares `Kinematics::Model`; `Differential`/
  `Mecanum` implementations exist (further renamed from
  `DifferentialKinematics`/`MecanumKinematics` by this sprint's ticket
  007, since those names stuttered against their own namespace).
- **Step 6 (app/ -> core/, Drive -> DifferentialDrive, FakeOtos):** commit
  `1c7b70d3`. `src/firm/app/` does not exist; `src/firm/core/` does, with
  no `App::`-namespaced code. That same commit's own message records that
  `FakeOtos` deliberately did NOT move to `hardware/generic/` as this
  proposal originally expected — it stayed in `core/` because it reads
  `Motion::Odometry`, two layers above `hardware/` (inverting the
  layering would have been the actual bug). **Step 6's `FakeOtos`
  placement question is resolved, not by relocation but by deletion**:
  this sprint's ticket 003 (commit `43843ff3`) removed `Core::FakeOtos`
  and the `FAKE_OTOS` build variant entirely as dead code (zero robot
  JSON/CI script/justfile recipe ever enabled it) — confirmed by `find
  src/firm -iname "fake_otos*"` returning nothing and
  `option(FAKE_OTOS ...)` no longer present in the root `CMakeLists.txt`.

Steps 7 and 8 are the only work this proposal describes that has not
landed:

- **Step 7 (transport generalization + `WifiTransport`)** is owned by
  [`clasi/issues/wifi-alternative-command-path.md`](../../../../clasi/issues/wifi-alternative-command-path.md)
  (status: pending) — its Scope item 1 is exactly this proposal's
  `WifiTransport` leaf, generalizing `Core::Comms`'s two hardcoded
  transport slots to N in the process.
- **Step 8a (Robot/RobotState formalization)** is owned by
  [`clasi/issues/robot-base-class-and-robots-subsystem.md`](../../../../clasi/issues/robot-base-class-and-robots-subsystem.md)
  (status: pending) — a stakeholder-directed (2026-08-11) plain
  Robot base class + per-robot subclass, including the bus-arbitration
  decision this proposal's own "Robot" section left open.
- **Step 8b (`Hal::Wheel`)** is owned by
  [`clasi/issues/hal-wheel-migration-needs-its-own-issue.md`](../../../../clasi/issues/hal-wheel-migration-needs-its-own-issue.md)
  (status: pending, filed by this ticket — it previously had no
  standalone issue, only the design rationale in
  [`src/firm/hal/DESIGN.md`](../../../../src/firm/hal/DESIGN.md) §4 and
  [`src/firm/control/DESIGN.md`](../../../../src/firm/control/DESIGN.md)
  §4, which the new issue points back to rather than duplicating).

This proposal is therefore **re-scoped to steps 7-8 only** and left open
— not moved to done — because steps 7-8 are real, undone, in-scope work
this document itself specifies (WifiTransport's design questions, the
Robot bus-arbitration and RobotState-extension open decisions) and no
other document owns that design content; the three linked issues above
own scheduling and delivery, not the design questions this proposal's own
"Proposed fix" sections (HAL, Robot) still answer better than a fresh
issue would from scratch.

### Sequencing (dependency order, not a ticket breakdown)

A rough order that keeps things compiling at each step, matching a
compile-smoke-test-per-step workflow:

1. Platform split out of `devices/` (interfaces + real/sim impls move,
   shape unchanged) — lowest risk, just include-path fixups.
2. Hardware split out of `devices/` (the concrete leaves) — sort each leaf
   into generic vs. named-hardware-family vs. platform-intrinsic per the
   three-way test above, not a blanket move into `generic/`.
3. HAL rename (`Devices::Motor` → `Hal::Motor`, etc.) — touches every call
   site that names the type, no behavior change.
4. Motion moves into `src/firm/motion/` as a unit — biggest file count, but
   mechanically the simplest (whole-directory move + include-path fixup,
   dependency rule unchanged).
5. Kinematics extraction (`BodyKinematics` → `DifferentialKinematics` behind
   an interface) — first real behavior-preserving refactor, not just a move.
6. `app/` → `core/` rename, `FakeOtos` relocation, `Drive` →
   `DifferentialDrive` rename.
7. Transport generalization (N transports) + `WifiTransport` — additive,
   can land any time after step 3.
8. `Hal::Wheel` as a new class, Robot/RobotState formalization — the two
   genuinely new-design pieces, saved for last since they benefit most from
   everything else already being settled.

### Decisions needed before/while executing

- Directory nesting: everything under `src/firm/` (current recommendation)
  vs. new top-level `src/` siblings.
- Whether "platform" deliberately widens to cover named peripheral-board
  families (Nezha) alongside compute targets (micro:bit/host/Pi), or Nezha
  gets a sibling tree (`hardware/nezha/`) instead, keeping "platform"
  strictly compute-target-only.
- `FakeOtos` final placement, now that it's confirmed to be a real-hardware
  (OTOS-less micro:bit build) fallback, not sim-only.
- Whether `ColorSensorLeaf`/`LineSensorLeaf` are genuinely generic (standard,
  publicly-documented parts) or project-specific boards like Nezha — not
  checked yet.
- Where the current wheel-speed PID actually lives — `App::Drive::fastPid()`
  or `Motion::WheelVelocityPid` — needed before `Hal::Wheel` is written.
- Bus arbitration: keep the cooperative single-fiber model (Robot as
  documented owner) vs. something more structural.
- RobotState extension mechanism: inheritance vs. composition.

## Verification

A compile smoke test after each mechanical move (both the ARM build and
`src/firm/platform/host/`'s host build, since a move that only breaks the
sim-side link is just as real a break), no full test-suite runs until the
whole reorganization is done — then one full run (unit +
`motion_tests`/`planner_tests`/`navigator_tests` + the host build) before
calling it complete. Hardware-on-the-stand verification
(`hardware-bench-testing.md`'s standing gate) applies once behavior — not
just file location — starts changing (Kinematics extraction onward), not to
the pure-move steps.

## Related

- `src/firm/devices/{i2c_bus,clock,motor,otos}.h` — existing interfaces this
  reorg relocates/renames rather than redesigns.
- `src/firm/app/boot_wiring.h`/`.cpp` — `App::composeRobot()`/`RobotGraph`,
  the existing Robot-composition precedent, including `FakeOtos`'s current
  location.
- `src/firm/app/comms.h` — existing `Transport`/`SerialTransport`/
  `RadioTransport`.
- `src/firm/app/drive.h` — current `Drive` class to be renamed
  `DifferentialDrive`.
- `src/motion/body_kinematics.h`, `src/motion/DESIGN.md` — current
  differential-only kinematics and the dependency-isolation rule to
  preserve after the move.
- `src/firm/motion/DESIGN.md`, `src/firm/kinematics/DESIGN.md` — existing
  stub landing zones inside `src/firm`.
- `docs/design/design.md` §2/§4/§5 — current firm/motion split rationale and
  the vendor-symlink validator constraint on source roots.
- `.clasi/config.yaml` `sources:` — validated source roots.
- `src/tests/bench/wifi/` — the bench-verified Ai-WB2-12F WiFi bring-up
  `WifiTransport` would wrap.
- `src/tests/bench/line_follow.py` — host-side centroid math that
  `Hal::LineSensor` would absorb.
- `clasi/issues/proposal-on-robot-linefollower-subsystem.md` — the prior
  on-robot LineFollower proposal this HAL layer would support.
- CLAUDE.md's "Two Firmware Layers" section — background on the sprint-122
  motion split this reorg reverses.
- External prior art referenced during design: WPILib's `hal`/device-class/
  robot-code layering (closest match for the Platform/Hardware/HAL split),
  Pybricks' `Motor`/`DriveBase` split (closest match for Motor/Wheel), and
  ROS2 `ros2_control`'s `hardware_interface` plugin model.
