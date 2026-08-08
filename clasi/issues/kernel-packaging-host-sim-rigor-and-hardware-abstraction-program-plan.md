---
status: pending
---

# Kernel Packaging, Host/Sim Rigor, and Hardware Abstraction — Program Plan

## Description

Stakeholder directive (2026-08-07): reorganize the firmware so one codebase
produces three artifacts — the micro:bit firmware (current), a host
program/library that turns protocol commands into simulated robot state, and
the real firmware compiled to WASM running in a browser driving simulated
robots for students. Around that, three formalizations:

1. **Top end** — from outside, the kernel looks like a serial port: send a
   datagram, poll for waiting data, receive, receive-with-wait.
2. **Bottom end / HAL** — one "robot class" per hardware family capturing all
   of a robot's hardware (motors declarable as velocity/power/position/servo,
   servos, digital/analog ports, I2C/UART passthrough), because the hardware
   (Nezha today; Hi-Wonder, Yahboom, Cutebot Pro, Spike Prime, REV Hub later)
   imposes structure — including on loop timing.
3. **Three layers** — CORE (planner, command processor, stop conditions,
   future MissionScript) / PLATFORM (serial, radio, threading/timing per MCU
   or OS) / ROBOT CLASS (per-hardware, including two sim variants: a
   realistic parameterized test sim and an avatar sim for students).

Configuration instantiates the robot class (several may compile into one
firmware, selected over serial with persistence). MissionScript is future work
but its binding seam must be reserved. Explicit anti-goal: over-abstraction —
extension happens by writing code against good base classes with good docs,
not by pre-abstracting every possible sensor.

**Stakeholder decisions (2026-08-07):**

1. Phase order as given in the breakdown below.
2. **The robot hardware class implements its own `cycle()`** — it owns the
   loop timing and schedule, not just a hardware-exchange hook inside a
   generic loop. Core work (comms pump, command routing, publish,
   planner/telemetry tail) is provided as services the hardware class invokes
   within its cycle. Control period is therefore per-robot-class; Nezha stays
   50 ms; validating the motion library at other periods is deferred until a
   non-50 ms robot class lands (Phase 6 at earliest).
3. **One multi-robot micro:bit image** (full Phase 5 scope: bake table of all
   robot configs, selection verb, flash persistence).
4. **One-time systest golden re-baseline** with tolerance bands when the sim
   plant gets honest; a seed-0 fully-deterministic mode is kept.

**What already exists — build on these, don't replace them** (researched
2026-08-07):

- **HAL seams exist and are clean.** Pure-virtual `Devices::I2CBus`, `Clock`,
  `Sleeper`, `Motor`, `Otos`, plus `App::Transport{readLine, send,
  sendReliable}` (src/firm/app/comms.h:23) and `Config::TuningStore`. Each has
  an ARM impl and a host/sim impl; no `#ifdef` forks in shared headers. Only
  8 of 78 src/firm files touch CODAL (main.cpp, devices/microbit_*, com/*).
- **One composition root.** `App::composeRobot()` / `RobotGraph`
  (src/firm/app/boot_wiring.h) builds the whole graph for BOTH main.cpp and
  the sim harness; divergence confined to typed `App::BootOverrides`; parity
  guarded by test_composition_root_parity.py.
- **Firmware-on-host already works.** src/sim builds `libfirmware_host.dylib`
  (-DHOST_BUILD, C++20, no CODAL, ~8 s clean). `sim_ctypes.cpp` exposes
  `sim_inject_command` / `sim_drain_tlm` — real protocol-v5 wire bytes in,
  real telemetry bytes out, proven end-to-end by test_sim_wire_loopback.py.
  Every sim consumer (pytest, TestGUI, square_tour --sim, systest) drives this
  one C++ sim via ctypes ("one sim object" policy).
- **A multi-vendor motor abstraction is already written but dead.**
  `Devices::MotorBoard` (normalized −1..+1 commands, encoderTick(),
  supplyMillivolts()) + `HiwonderBoard` + `BoardMotor : Motor` adapter —
  compiled, characterized on a bench rig, wiring guide at
  docs/hiwonder/hiwonder-motor-board.md, zero callers.
- **src/motion is already a portable core library** — imports exactly two
  src/firm headers (messages/common.h, types/robot_state.h); no clocks,
  devices, or config; own standalone CMake/ctest projects. Stop conditions
  live inside `Motion::Planner::tick()`.
- **Host side already has the four-verb interface shape** (send_fast / send /
  read_pending_lines / read_lines / wait_for_ack in SerialConnection +
  NezhaProtocol), and the rogo daemon re-exports it over TCP.

## Cause

The gaps between that state and the directive:

1. **`App::RobotLoop` is the tangle** (796 lines): scheduler (kCycle=50 ms
   constexpr), Nezha I2C schedule owner (the split-phase 0x46 dance with
   kSettle/kClear windows is CONTROL FLOW, not parameters — HiWonder wants a
   different loop shape), command dispatcher, blackboard publisher, ownership
   arbiter, wire→core translator. 15 held references.
2. **`RobotGraph` hard-codes `Devices::NezhaMotor` by concrete type**
   (boot_wiring.h:303) despite the clean `Motor` interface. Highest-leverage
   single fix.
3. **Motion ownership is a convention, not a mechanism** — three writers of
   `RobotState::Wheel::cmdVelocity`; sprint 135 added an owner, missed a
   guard, 17x accuracy regression found on the playfield. MissionScript would
   be a fourth writer.
4. **`App::Comms` hard-codes exactly two transports** and fan-outs every
   binary reply to both; carries side channels (SeedRequest with Transport*
   pointer, DBG ring, Status). com/ has no Link interface of its own.
5. **SimPlant simulates Nezha silicon, not "a robot"** (I2C-register-level),
   and has a known-CRITICAL fidelity gap (clasi/issues/later/
   B-sim-plant-is-idealized-and-biases-belief-not-motion.md): ideal linear
   plant, hardcoded tau/gain, controller feedforward installed as the plant's
   exact inverse (feedforward bugs structurally unfalsifiable), all error
   knobs bias reported values not true motion, no RNG/seed at all.
6. **Build source lists duplicated in 27 places** (3 CMake + 24 pytest
   `_APP_SOURCES` lists) — must collapse before adding host-exe/WASM targets.
7. **No host executable main(), no socket transport, zero WASM traces.**
   The sim pacing loop lives in Python (SimLoop._tick_loop).
8. **Config two-path bake wart**: bakeBootValues() vs Configurator::loadBaked()
   read the same JSON through different generated families; ~14 values are
   construction-time-only — blocks one-binary-many-robots.
9. **`Config::kDrivetrainType` is baked from JSON but nothing reads it** —
   the natural hook for robot-class selection already exists.
10. **Docs drift**: docs/protocol-v5.md is ~6 verbs behind commands.proto.

## Proposed fix

### Layering and directory layout

The robot class is injected into the existing composition root exactly like
the bus/clock/transports are today — it becomes the biggest leaf, not a new
framework. File moves are minimal.

```
CORE (compiles everywhere; no vendor headers, no #ifdefs)
  src/motion/                   — untouched (parallel effort owns it)
  src/firm/app/                 — CommandRouter, StatePublisher, MotionArbiter,
                                  Comms, Drive, Configurator, Telemetry,
                                  Preamble (slimmed), boot_wiring (root)
  src/firm/messages/, types/    — wire codec, RobotState (unchanged)
  src/firm/config/              — Config::Robot, bake, TuningStore iface
  src/firm/hal/                 — NEW: Hal::RobotHardware, Hal::CycleServices,
                                  Hal::VerbBinding, robot factory/registry

ROBOT HAL (per robot family; several may compile into one binary)
  src/firm/robots/nezha/        — NezhaHardware: owns NezhaMotor x2 + MotorArmor
                                  + RealOtos + color/line + the 50 ms split-phase
                                  cycle schedule (literal transcription of
                                  robot_loop.cpp:604-685)
  src/firm/robots/hiwonder/     — HiwonderHardware (resurrects the dead
                                  MotorBoard/HiwonderBoard/BoardMotor)
  src/firm/robots/sim/          — SimBotHardware (motor-level plant, avatar sim;
                                  host/WASM builds only)
  src/firm/devices/             — stays: reusable device-driver leaves that
                                  robot classes COMPOSE (Motor, Otos, I2CBus,
                                  Clock, MotorBoard, ...)

PLATFORM (per MCU/OS)
  src/firm/platform/microbit/   — microbit_clock, microbit_i2c_bus, com/*
                                  (SerialTransport/RadioTransport),
                                  MicroBitTuningStore
  src/sim/                      — host+WASM platform: SimPlant, SimClock,
                                  sim_ctypes ABI, NEW sim_main.cpp (socket/stdio
                                  executable), NEW web/ (Emscripten)
  src/sim/plant/                — WheelPlant/OtosPlant/MotorChannelPlant,
                                  promoted out of src/tests/sim/plant (they
                                  become production student-facing code)
```

The sprint-122 "freeze the base / subtree-split" intent is explicitly
superseded until after Phase 2 — the HAL boundary makes the base boundary
real first; a repo split can follow later if still wanted.

### The robot class owns the cycle

Per stakeholder decision 2: `RobotHardware::cycle(CycleServices&)` runs one
full loop iteration — the hardware class decides when duty writes happen,
when encoders are sampled, where settle windows fall, and what the period is.
Core phases are services it calls at the points it chooses:

```cpp
// src/firm/hal/cycle_services.h — handed to RobotHardware::cycle() by the
// thin outer loop. The hardware class never owns a private sleeper: settle
// windows go through idleFor(), which pumps comms then sleeps — preserving
// today's pump-during-settle behavior exactly.
namespace Hal {
class CycleServices {
 public:
  virtual ~CycleServices() = default;
  virtual uint64_t nowMicros() const = 0;         // [us]
  virtual void idleFor(uint32_t gap) = 0;         // [ms] pump comms + sleep
  virtual void routeCommands() = 0;               // drain the command ring
  virtual void actuate() = 0;                     // arbitration + Drive::tick →
                                                  // staged duty writes on hw motors
  virtual void computeUntil(uint32_t deadline) = 0; // [ms] absolute; odometry/
                                                  // sensors/telemetry/planner/
                                                  // Drive::update tail
};
}  // namespace Hal
```

`NezhaHardware::cycle()` is a literal transcription of today's
`robot_loop.cpp:604-685` (mark → actuate → select-L → idleFor(4) → collect-L →
idleFor(4) → select-R → idleFor(4) → collect-R → publish → computeUntil(start
+ 50)) — the literalness is the regression defense, including the hard-won
131-005 absolute-deadline pacing. The exact split of what stays in the outer
shell vs. `CycleServices` is Phase 2's architecture work; the invariant is
that the hardware class holds the schedule and the core holds the policy.

### The RobotHardware interface (lean sketch)

```cpp
// src/firm/hal/robot_hardware.h
namespace Hal {

enum class MotorMode : uint8_t { kPower, kVelocity, kPosition, kServo };

struct AuxDecl {          // one aux (non-drive) actuator
  const char* name;       // "gripper" — stable; used by verb bindings
  uint8_t port;
  MotorMode mode;         // position/servo commanded in radians // [rad]
};

class RobotHardware {
 public:
  virtual ~RobotHardware() = default;

  // identity & configuration (whole-object apply; called by the root after
  // construction and again on live CONFIG pushes)
  virtual const char* name() const = 0;               // "nezha"
  virtual void configure(const Config::Robot& config) = 0;

  // lifecycle — absorbs Preamble's device-probe body
  virtual bool detect() = 0;    // one probe step; false = keep probing
  virtual void begin() = 0;     // one-time init after detect()

  // timing + THE cycle (see CycleServices above)
  virtual uint32_t cyclePeriod() const = 0;           // [ms] nominal
  virtual void cycle(CycleServices& services) = 0;

  // drive inventory — the two consumers that exist today
  virtual Devices::Motor& driveLeft() = 0;
  virtual Devices::Motor& driveRight() = 0;
  virtual Devices::Otos* otos() { return nullptr; }
  virtual Devices::ColorSensorLeaf* colorSensor() { return nullptr; }
  virtual Devices::LineSensorLeaf* lineSensor() { return nullptr; }

  // aux actuators (MissionScript-facing; default none). Commands are STAGED
  // and land on the next cycle's bus traffic (MotorBoard::stageSpeed
  // discipline). kPower/kVelocity normalized [-1,1]; kPosition/kServo [rad].
  virtual uint8_t auxCount() const { return 0; }
  virtual const AuxDecl* auxDecl(uint8_t) const { return nullptr; }
  virtual bool stageAux(uint8_t index, float value) { return false; }
  virtual float auxPosition(uint8_t) const { return 0.0f; }  // [rad]

  // raw ports & passthrough — defaults refuse; robots override what exists.
  // UART passthrough: reserved comment only, no virtuals until a consumer.
  virtual bool digitalRead(uint8_t pin, bool* out) { return false; }
  virtual bool digitalWrite(uint8_t pin, bool level) { return false; }
  virtual bool analogRead(uint8_t pin, float* out) { return false; }
  virtual Devices::I2CBus* passthroughBus() { return nullptr; }

  // MissionScript verb contribution seam (see below)
  virtual uint8_t verbCount() const { return 0; }
  virtual const VerbBinding* verb(uint8_t) const { return nullptr; }
};
}  // namespace Hal
```

Lean-ness rules: everything with two real consumers today is pure virtual;
aux/ports are default-empty vocabulary implemented first only for Nezha's
servo ports; `Devices::Motor` is NOT extended with velocity/position setters
(the motion library owns the control law; mode is `AuxDecl` metadata; drive
motors remain duty+encoder).

### What maps where (existing → target)

| Existing | Target |
|---|---|
| `App::RobotLoop` (796 lines) | Split: `CommandRouter` (routeCommand + 7 handlers + dedupe), `StatePublisher` (9 publish*), `MotionArbiter` (new), thin outer shell providing `CycleServices`. `RobotLoop` survives as a facade with today's public API so sim_ctypes + ~25 pytest harnesses don't churn. |
| Nezha schedule in `cycle()` + kSettle/kClear + `Motor::requestSample()` | `Robots::NezhaHardware::cycle()`. `requestSample()` leaves `Devices::Motor` (it was always Nezha-shaped). |
| `RobotGraph` concrete `NezhaMotor motorL_` (boot_wiring.h:303) | `RobotGraph(..., Hal::RobotHardware& hw, ...)`; graph wires Drive/Odometry/Planner against `hw.driveLeft()` etc. |
| `App::Preamble` device probing | Body → `NezhaHardware::detect()/begin()`; Preamble keeps boot-frame emission + done() gate. |
| Dead `MotorBoard`/`HiwonderBoard`/`BoardMotor` | Resurrected inside `Robots::HiwonderHardware` (Phase 6). |
| `Config::kDrivetrainType` (baked, unread) | Deleted; replaced by `identity.robot_class` + `Hal::makeRobot()` factory (Phase 5). |
| `src/tests/sim/plant/*` | Promoted to `src/sim/plant/`. |

**MotionArbiter** (fixes the three-writers convention; prerequisite for
MissionScript as a fourth motion owner): `enum class MotionOwner { kNone,
kDrive, kPlanner, kNavigator, kScript }` held in one place; `setOwner(next)`
invokes the displaced owner's registered `cancel()` before handing over;
`zeroUnownedMotion()` becomes `owner() == kNone` with the zero-only monotone
contract preserved verbatim. The sprint-135 bug class becomes structurally
impossible.

### The two sim variants (both are robot classes)

- **Realistic test sim** (exists, gets rigor in Phase 4): the *real*
  `NezhaHardware` running over `TestSim::SimPlant` as its I2CBus — "real
  drivers over simulated silicon" is kept for the Nezha; error parameters
  move to a `simulation` section of the robot JSON.
- **Avatar sim** (new, Phase 7): `SimBotHardware` — no I2C; `cycle()`
  integrates a `MotorChannelPlant` per wheel directly. "Callbacks for the
  UI" is implemented as **polling, not callbacks**: the existing sim_ctypes
  state getters (pose, wheels, sensors) are the drawing API; a JS
  `requestAnimationFrame` loop polls them. Works identically over ctypes and
  WASM; a push-style `sim_set_frame_hook()` is one additive function later if
  ever wanted.

### Top-end packaging

One datagram spec, three bindings:

```
send(bytes)            — one protocol-v5 line (cleartext or verb:COBS+CRC)
dataWaiting() -> int   — count of complete inbound lines available
recv() -> bytes|None   — pop one line, non-blocking
recvWait(timeout) -> bytes|None
```

- **Firmware side**: `App::Transport` itself already is this shape and gains
  nothing. `App::Comms` changes: the hard-coded `serialLink_/radioLink_` pair
  becomes a fixed-capacity link list (`Transport* links_[kMaxLinks=4]`), and
  replies route to the *originating* transport (`App::Cmd` gains
  `Transport* origin`) — killing the fan-out-to-both and the
  `SeedRequest::reply` / TLM side-channel `Transport*` pointers. Broadcast
  (banner, telemetry) iterates the list. A host/WASM build registers exactly
  one injected-datagram link; micro:bit registers serial+radio — one code path.
- **Host executable**: new `src/sim/sim_main.cpp` — same objects as
  `libfirmware_host` plus a `HostLink` (TCP server or `--stdio`,
  newline-framed, byte-identical to the serial wire) and a wall-clock pacing
  loop (`--speed N`, `--robot <name>`). This moves the cadence that today
  lives only in Python's `SimLoop._tick_loop` into C++ where WASM can reuse it.
- **rogo sim backend**: `rogo serve --sim` spawns `sim_main` and talks to it
  as it talks to a serial port. Host-side, formalize a `RobotLink`
  `typing.Protocol` (send/send_fast/data_waiting/recv/recv_wait/wait_for_ack)
  in src/host/robot_radio/io/, satisfied by `SerialConnection` (gains
  `data_waiting()`), a new `SocketConnection`, and the TestGUI `SimTransport`;
  the GUI-side ABC re-exports it instead of owning it.
- **WASM (Phase 7)**: Emscripten build over the unified source manifest;
  **ABI = the existing sim_ctypes extern-C surface, verbatim** (no embind) —
  `sim_inject_command` is send, `sim_drain_tlm` is recv, `sim_step(n)` is the
  pacer, called via cwrap. Pacing in JS (setInterval at cycle rate; burst
  `sim_step(k)` for fast-forward); drawing polls pose getters per
  requestAnimationFrame. Full telemetry decode in JS is deferred — the state
  getters cover the student UI. Deliverable: `src/sim/web/index.html` running
  a square tour offline. Working assumption: the student web UI starts life
  in this repo as a reference page, so the C ABI can stay informal until an
  external front end consumes it.

### Simulation rigor track (Phase 4)

Executes the 6-step path already written in
`clasi/issues/later/B-sim-plant-is-idealized-and-biases-belief-not-motion.md`:

1. **JSON `simulation` section** (per-robot): per-wheel gain asymmetry,
   tau_left/right, deadband, latency, sensor-noise sigmas, gyro drift,
   optical stray. `gen_boot_config.py` emits it host-only (never ARM flash).
2. **True-motion error injection** — knobs move from reported-value bias into
   WheelPlant/OtosPlant dynamics, so errors change what the robot *does*.
3. **Seedable noise** — one xorshift32 in the plant, `sim_set_seed(uint32)` on
   the ABI; seed 0 = legacy deterministic so existing C++ tests stay
   bit-stable until deliberately re-baselined.
4. **Break the feedforward tautology** — stop installing the plant's exact
   inverse (sim_harness.h:167); harness installs the JSON-fitted
   duty_per_speed while the plant runs the simulation-section true gains;
   revive `calibration/fit_sim_error_model.py` (currently dead — targets a
   deleted SIMSET registry) against the CONFIG plane to close the loop.
   Ticket 1 of the sprint is the falsifiability proof: a deliberately-wrong
   feedforward test must FAIL.
5. **`Plant::MotorChannelPlant`** — first-order lag + deadband + encoder
   quantization per channel; the physics unit WheelPlant refactors onto and
   HiWonder/SimBot classes reuse, so non-Nezha robots never need
   register-level silicon sims.
6. **Systest goldens** — one blessed re-baseline at seed 0 with tolerance
   bands where dynamics changed; add a noisy tier (seeds 1..N, statistical
   envelopes on tour-closure error).

### Config & robot selection (Phase 5)

- `identity.robot_class` ("nezha" | "hiwonder" | "simbot") in
  robot_config.schema.json; `kDrivetrainType` deleted.
- `Hal::makeRobot(robotClass, bus, config)` factory — placement-new into
  static aligned storage sized for the largest compiled-in class (no heap);
  the compiled-in class list is a CMake list per firmware flavor; a dedicated
  one-robot firmware is a two-line main.
- **One-firmware-many-robots (micro:bit)**: bake ALL data/robots/*.json as an
  indexed table in boot_config.cpp; boot order: persisted robot selection
  (TuningStore) → matching baked entry → factory → configure(). New cleartext
  verbs `ROBOT?` / `ROBOT:<name>` (set over serial once, persist, reboot to
  apply). `active_robot.json` remains the dev-time default.
- **Two-path bake wart resolved here** (it blocks one-binary-many-robots):
  extend `Config::Robot` with the missing groups (per-port motor wiring, OTOS
  mount, PlannerLimits residue); `RobotGraph::bakeBootValues()` becomes a pure
  projection of one `Config::Robot`; construct-after-selection removes most
  construction-time-only constraints for free;
  test_composition_root_parity.py guards the migration.

### MissionScript seam reservation (mechanism only — no language design)

```cpp
// src/firm/hal/verb_binding.h
struct VerbBinding {
  const char* name;   // "GRIP" — upper-case, HELP-listed
  uint8_t verbId;     // from the reserved hardware block
  msg::ErrCode (*invoke)(RobotHardware& hw, const msg::CommandEnvelope& env,
                         msg::ReplyEnvelope* reply);
};
```

- `commands.proto` reserves verb ids 64–127 as the hardware block — never
  assigned to core verbs.
- `CommandRouter::routeCommand()` falls through its core switch to a ~10-line
  lookup over `hw.verb(i)`.
- `HELP` and the Python wire_commands generator enumerate hardware verbs
  (host discovers, never hard-codes).
- Result: add a sensor to your robot class + register a VerbBinding → it's
  commandable (and later scriptable) with zero core changes. The "extend
  with AI from the Nezha base class" pattern doc lands as
  `src/firm/robots/DESIGN.md` in Phase 6 with HiWonder as the worked example.

### Phased sprint breakdown

| Phase | Sprints | Scope | ~Tickets | Risk | Deps | Hardware gate |
|---|---|---|---|---|---|---|
| **0. Build unification** | 1 | One source-of-truth manifest (`src/cmake/firmware_sources.cmake` + generated list the 24 pytest `_APP_SOURCES` import, or relink harnesses against prebuilt libfirmware_host); docs-drift fixes ride along (protocol-v5.md, design.md §1) | 5 | Low | — | none |
| **1. RobotLoop decomposition** | 1 | Extract CommandRouter, StatePublisher, MotionArbiter; RobotLoop facade keeps API; ZERO timing change; pacing harness proves identical schedule | 6 | Low-med | 0 | tovez smoke (expect nil delta) |
| **2. HAL introduction** | 1–2 | Hal::RobotHardware/CycleServices; NezhaHardware (literal cycle transcription + Preamble probe body); RobotGraph takes RobotHardware&; both roots construct NezhaHardware (ARM over MicroBitI2CBus, sim over SimPlant); motor.h sheds requestSample() | 8–10 | **Highest** (bench-calibrated timing) | 1 | **Full bench**: square tour on tovez, delivered-period re-measurement, tour-closure gate |
| **3. Top-end packaging** | 1 | Comms link list + origin-routed replies; sim_main.cpp + HostLink (socket/stdio); rogo serve --sim; Python RobotLink Protocol + data_waiting(); square_tour --sim runs against the daemon | 7 | Med | 0 (can start before 2 completes) | tovez radio+serial regression (reply routing changed) |
| **4. Sim rigor** | 1 | The 6-step honest-plant plan; falsifiability test first; golden re-baseline (blessed) | 6 | Med (golden churn, contained) | 0; parallel with 3 | one tovez run to sanity-check the fitted error model |
| **5. Robot selection** | 1 | robot_class + factory + static-storage construction; multi-robot bake table; ROBOT?/ROBOT: verbs + persistence; two-path bake consolidation | 7 | Med-high (touches boot) | 2 | reflash tovez, select robot over serial, tour |
| **6. Second robot + verb seam** | 1 | HiwonderHardware (resurrect MotorBoard per the wiring guide); MotorChannelPlant-backed sim variant; VerbBinding + one real HiWonder verb; robots/DESIGN.md extension pattern doc | 7 | Med | 2, 4, 5 | HiWonder bench rig (characterized 2026-08-02) |
| **7. WASM / browser** | 1 | Emscripten target over the unified manifest; sim_ctypes ABI exported via cwrap; src/sim/web/ demo (pose-polling canvas, offline square tour); SimBotHardware avatar class | 5 | Low-med (toolchain unknowns) | 0, 3, 4 (nicer after 6) | none |

**Phase 2 regression protocol** (the one place this can hurt a working
robot): transcribe, don't rewrite; pacing harness asserts the same window
sequence pre/post; the RobotGraph signature change lands LAST in the sprint
(breaking steps go last); bench gate mandatory before close.

**Explicitly deferred**: MissionScript language design (seam only); Spike
Prime / REV / Cutebot / Yahboom classes (HiWonder proves the pattern); UART
passthrough implementation; push-style avatar callbacks; repo subtree-split;
the stuck `cycleCount_` line-sensor parity defect (pre-existing,
timing-sensitive — its own measured change).

**Delivery mechanics**: promote this issue's architecture content into
`docs/design/kernel-packaging-and-hal.md` so sprint work references a repo
design doc; split this program issue into one issue per phase (via the CLASI
issue tooling, never hand-created files), each carrying its scope-table row;
Phase 4 links the existing B-sim-plant-is-idealized issue rather than
duplicating it. Phases then run through the normal CLASI sprint flow.

## Verification

- **Per-phase**: each sprint ends runnable per CLASI rules; hardware gates as
  in the table (tovez by UID on the stand; the HiWonder bench rig for
  Phase 6).
- **Phase 1/2 timing**: pacing harness asserting the window sequence and a
  delivered-period bench measurement (the 131-005 rock-stable 54 ms number is
  the reference) before/after.
- **Sim parity throughout**: test_composition_root_parity.py and
  test_sim_wire_loopback.py stay green in every phase; the "one sim object"
  policy extends to the new C++ pacing loop (no second divergent loop in
  Python once sim_main exists).
- **Phase 4**: the deliberately-wrong-feedforward falsifiability test; seed-0
  bit-stability for legacy scenarios; blessed golden re-baseline.
- **Phase 7**: the web demo runs the same .tour input as systest and the
  square tour closes within the sim-tier tolerance in the browser.

## Related

- `clasi/issues/later/B-sim-plant-is-idealized-and-biases-belief-not-motion.md`
  — the sim-fidelity issue Phase 4 executes (6-step fix path already written).
- `clasi/issues/later/firmware-base-hardening-characterization-gate-and-freeze.md`
  — the sprint-122 freeze/subtree-split intent this plan supersedes until
  after Phase 2.
- `docs/hiwonder/hiwonder-motor-board.md` — the MotorBoard wiring guide
  Phase 6 follows.
- `docs/design/base-explicit-loop-sketch.md` — prior loop-visible-dataflow
  draft; its "wheel layer makes no velocity decisions" principle is honored.
- `src/archive/source_old/hal/` — the deleted prior-generation HAL (capability
  interfaces + Hardware registry + PhysicsWorld), removed by sprint 102 in
  favor of the injected-leaves pattern this plan builds on; worth reading
  before Phase 2 detail design.
- `clasi/issues/done/plan-c-port-of-radio-robot-firmware.md` — the original
  port plan that first proposed hal/ + CommandProcessor separation.
