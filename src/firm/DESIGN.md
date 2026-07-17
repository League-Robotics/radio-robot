---
template: /Volumes/Proj/proj/ai-projects/clasi/docs/design/SUBSYSTEM_DESIGN_TEMPLATE.md
role: root design document — owns system-wide context, the subsystem map, and
  the global conventions every subsystem doc may assume without restating.
subsystem-docs:
  - app/DESIGN.md
  - com/DESIGN.md
  - config/DESIGN.md
  - devices/DESIGN.md
  - kinematics/DESIGN.md
  - messages/DESIGN.md
  - motion/DESIGN.md
  - types/DESIGN.md
---

# Firmware (src/firm) — Root Design

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-16 · **Status:** in-flux

---

## 1. Purpose

`src/firm` is the robot firmware: a single C++ program for the BBC micro:bit V2
(nRF52833) that drives a PlanetX Nezha V2 differential-drive robot. It reads
wheel encoders and sensors over one shared I2C bus, closes per-wheel velocity
loops, integrates odometry, and exchanges binary-armored protobuf-style
messages with a host over USB serial and the micro:bit radio. It is the
"plant" end of a host/robot split: the host plans motion (profiled twists,
tours); the firmware follows twist commands, enforces a deadman, and streams
telemetry. Everything under this directory compiles into one image
(`main.cpp` is the ARM entry point); the same modules minus the ARM adapters
also compile under `-DHOST_BUILD` for host-side tests and simulation.

## 2. Orientation

The architecture is a **single cooperatively-timed loop** (`App::RobotLoop`)
that owns all I2C bus access and all timing, calling into passive modules that
never sleep and never touch the bus on their own. This replaced an earlier
subsystem/message-dispatch stack (deleted in sprints 102–107; see git history
and `clasi/sprints/done/`) — the current tree is a deliberate greenfield
rebuild around one visible schedule.

Flow of one cycle, at orientation altitude:

1. **Comms in** — `App::Comms` polls the two transports (serial, radio) for
   one armored `*B` line, dearmors and decodes it into a
   `msg::CommandEnvelope`.
2. **Dispatch** — the loop's own switch acts on the command: a Twist stages a
   target on `App::Drive` and arms `App::Deadman`; config/queries reply via
   `Comms::sendReply()`.
3. **Motor service** — the loop runs each `Devices::NezhaMotor`'s split-phase
   encoder request → settle → collect → PID → duty-write sequence, with the
   settle/clearance gaps expressed as `runAndWait(gap, body)` blocks whose
   wait time is borrowed for other bounded work (OTOS sampling, odometry
   integration, telemetry assembly).
4. **State out** — `App::Odometry` integrates encoder deltas through
   `BodyKinematics::forward()`; `App::Telemetry` emits the primary TLM frame
   (or the slower secondary diagnostic frame) through Comms.
5. **Pace** — a final `runAndWait` paces the cycle (~16 ms nominal).

Boot is a separate loop: `App::Preamble` steps per-device detection (one
bounded probe per pass) while telemetry frames report detection status;
command consumption starts only when `preamble.done()`.

The directory map:

| Directory | Namespace | Role | Doc |
|---|---|---|---|
| `app/` | `App` | The loop and its passive app modules: RobotLoop, Comms, Telemetry, Drive, Odometry, Deadman, Preamble | [app/DESIGN.md](app/DESIGN.md) |
| `devices/` | `Devices` | Device leaves (NezhaMotor, Otos, ColorSensorLeaf, LineSensorLeaf), the MotorArmor policy base, velocity PID, and the pure seam interfaces `I2CBus`/`Clock`/`Sleeper` plus their `MicroBit*` ARM impls | [devices/DESIGN.md](devices/DESIGN.md) |
| `com/` | (global / `radiochan`) | Raw transports: `SerialPort` (USB CDC), `Radio` (micro:bit radio), persisted radio-channel storage | [com/DESIGN.md](com/DESIGN.md) |
| `messages/` | `msg` | Wire schema: generated message structs, generated envelope codec (`wire.{h,cpp}`), hand-written byte-level runtime (`wire_runtime.{h,cpp}`), layout gates | [messages/DESIGN.md](messages/DESIGN.md) |
| `config/` | `Config` | Generated boot configuration — per-robot calibration baked at build time from `data/robots/active_robot.json` | [config/DESIGN.md](config/DESIGN.md) |
| `kinematics/` | `BodyKinematics` | Stateless differential-drive math: inverse/forward twist↔wheel maps, saturation | [kinematics/DESIGN.md](kinematics/DESIGN.md) |
| `motion/` | `Motion` | Jerk-limited single-channel trajectory solving (`JerkTrajectory`, wrapping vendored Ruckig, `src/vendor/ruckig/`); restored 109-001, a leaf like `kinematics/` — dormant until a future queue/executor calls it | [motion/DESIGN.md](motion/DESIGN.md) |
| `types/` | (global) | Protocol v2 text-tag constants, protocol/firmware version, reply-context plumbing types | [types/DESIGN.md](types/DESIGN.md) |
| `main.cpp` | — | ARM entry point: constructs the real hardware singletons and every module, wires them, hands off to `RobotLoop::run()` (never returns) | this doc, §4 |

Dependency direction (arrows = "includes/uses"):

```
main.cpp ──► app ──► devices ──► (nothing project-local except itself)
   │          │  └─► messages, kinematics
   │          └────► com (via ARM-only Transport adapters)
   ├────────► config ──► messages
   └────────► com, devices, config

motion ──► messages   (109-001: restored leaf, no incoming edge yet —
                        ticket 003 adds the app ──► motion edge)
```

`devices/` is the bottom of the stack and deliberately includes nothing from
`messages/` or `config/` (see §3). `kinematics/`, `motion/`, and `messages/`
are leaf libraries with no project dependencies of their own (`motion/`
depends only on `messages/`, for `msg::PlannerConfig`'s field types — see
[motion/DESIGN.md](motion/DESIGN.md)). As of 109-001 nothing in `app/` (or
anywhere else) calls into `motion/` yet — it is a restored, dormant leaf,
not yet wired into the loop.

## 3. Constraints and Invariants

- **Single-loop bus ownership:** all I2C traffic happens from the loop's own
  cycle, in the loop's documented order; no module ever initiates bus traffic
  from its own `tick()`/staging methods. Violating this reintroduces the
  shared-bus timing collisions that wrecked motion timing (see
  `.clasi/knowledge/`, OTOS per-pass tick incident) and can hard-stall the
  nRF52 TWIM peripheral.
- **App modules are passive and bounded:** `Drive::tick()`,
  `Odometry` integration, `Telemetry` assembly, `Preamble::step()`, and every
  `runAndWait` body must be bounded, non-sleeping, non-bus-touching work. A
  sleep or blocking I2C call inside one silently destroys the cycle's timing
  budget and starves the CODAL fiber scheduler (the radio *looks* dead when
  the loop doesn't yield).
- **Critical waits are explicit:** every required gap in the schedule is a
  `runAndWait(gap, body)` block in `robot_loop.cpp` — the name carries the
  wait, the block scopes the work that borrows it.
  `grep 'runAndWait\|sleepUntil' app/robot_loop.cpp` must remain the
  firmware's complete timing schedule. Never hide a sleep inside a work
  function.
- **Devices isolation invariant:** `devices/` must not include `messages/` or
  `config/` headers. Wire-plane types (`msg::*`) stop at the `app/` layer;
  conversion between `msg::MotorConfig` and `Devices::MotorConfig` happens in
  `main.cpp`, the one place both types are reachable. Breaking this couples
  device leaves to the wire schema and kills their host-side reuse.
- **HOST_BUILD purity:** every module except the explicitly ARM-only files
  (`com/*`, `devices/microbit_*`, `main.cpp`, and the `#ifndef HOST_BUILD`
  transport adapters in `app/comms.h`) must compile under `-DHOST_BUILD` with
  no `MicroBit.h` anywhere in the translation unit. Hardware seams are plain
  virtual bases (`Devices::I2CBus`, `Devices::Clock`, `Devices::Sleeper`,
  `App::Transport`) — never `#ifdef` forks inside a shared header. Host fakes
  live under `tests/`, not `src/firm/`.
- **Generated files are never hand-edited:** `messages/*.h` (except
  `wire_runtime.*` and `layout_checks.*`), `messages/wire.cpp`,
  `config/boot_config.cpp`, and `types/version_generated.h` are emitted by
  `scripts/gen_messages.py` / `gen_boot_config.py` / `gen_version.py` at
  build time. Fixes go in the generator. Hand edits are silently destroyed
  by the next build.
- **Wire compatibility outranks naming:** wire key strings, TLM field tokens,
  reply tag strings, and the `DEVICE:NEZHA2:...` banner format are frozen
  protocol surface — the project's no-units-in-identifiers rename convention
  explicitly excludes them (`.claude/rules/coding-standards.md`).
- **Deadman is the only staleness gate:** one `App::Deadman`, armed by every
  actuation command, checked by the loop, expiry → `Drive::stop()`. No second
  ad hoc watchdog timer belongs anywhere in the firmware.
- **`newlib-nano` has no `%f`:** `printf`-family float formatting emits
  nothing on ARM (works fine in host builds). Floats cross the wire as
  scaled integers or via the binary codec.
- **Bench gate:** firmware changes to the HAL, motor control, sensing, or
  protocol are not done until exercised on the robot on its stand
  (`.claude/rules/hardware-bench-testing.md`). Host tests alone do not close
  a change.

## 4. Design

**Why one loop.** The previous architecture (subsystems, message routers,
per-device fibers) hid the bus schedule and the sleeps inside layers, which
made the two hard realtime problems — I2C bus discipline on a shared
single-master bus, and cooperative yielding to the CODAL fiber scheduler —
undebuggable. The rebuild inverts this: `RobotLoop::cycle()` is one page of
code in which every bus transaction and every wait is visible in call order.
Modules were then factored *out* of that page only as passive, bounded
helpers (stage/compute/carry), never as actors with their own timing.

**The timing primitive.** `runAndWait(gap, body)` = mark time, run the body,
sleep until `mark + gap`. Each block anchors its gap to its *own* mark, so a
slow body degrades gracefully (the sleep shrinks to zero but never goes
negative into a busy-wait or starves later blocks). The final pacing block
uses the same shape anchored to its own mark — anchoring to the cycle start
was a diagnosed defect (a never-achievable deadline that turned the pace
sleep into a no-op).

**The time seam.** All cycle-level time flows through `Devices::Clock`
(`nowMicros()`) and `Devices::Sleeper` (`sleepMillis()`), constructor-injected
into `RobotLoop` and the modules that need "now" (Deadman, Preamble). ARM
impls wrap `system_timer_current_time_us()`/`fiber_sleep()`; host tests inject
steppable fakes and advance time explicitly. This is a separate seam from
`I2CBus`'s internal clearance-timer bookkeeping.

**Construction and wiring (`main.cpp`).** All objects are function-`static`
in `main()` — no heap, no globals with cross-TU construction order concerns.
Construction order is deliberate: bus before leaves, leaves before app
modules that read them. Boot config comes from generated `Config::default*()`
functions; `main.cpp` converts wire-plane `msg::MotorConfig` to
`Devices::MotorConfig` (the isolation invariant means only `main.cpp` can see
both types) and maps 1-based port labels from the drivetrain config to
0-based config-array indices. It then calls `RobotLoop::run()`, which never
returns.

**Two build targets, one tree.** ARM (`main.cpp` + CODAL) is the product.
`-DHOST_BUILD` compiles the same `app/`/`devices/`/`messages/`/`kinematics/`
sources for host tests and the SimPlant simulator (`src/sim/`,
`tests/_infra/sim/`), which inject fake `I2CBus`/`Clock`/`Sleeper`/`Transport`
implementations. `RobotLoop` exposes `boot()`/`cycle()` separately from
`run()` precisely so a harness can step bounded cycles and inspect state.

**Command plane.** The current protocol is the binary-armored envelope
codec (`*B`-framed base64 lines carrying `msg::CommandEnvelope` /
`msg::ReplyEnvelope`), generated from `protos/*.proto` by
`scripts/gen_messages.py`. A minimal text rump (HELLO/PING banner replies)
survives for bring-up and safety. Legacy text clients go through a host-side
translator proxy, never through firmware text parsing (stakeholder decision,
2026-07-10).

## 5. Interfaces

### Exposes (system boundary — the wire)

- **Armored binary command/reply protocol:** `*B<base64>` lines over USB
  serial (115200 CDC) and the micro:bit radio (group 10, channel 0–35
  persisted in flash). Payloads are `msg::CommandEnvelope` in,
  `msg::ReplyEnvelope` (ok/err/tlm) out, plus an independently-armored
  `msg::TelemetrySecondary` frame. Schema source of truth: `protos/*.proto`.
  See [messages/DESIGN.md](messages/DESIGN.md) and
  [app/DESIGN.md](app/DESIGN.md).
- **Boot banner:** `DEVICE:NEZHA2:robot:<name>:<serial>` — byte-frozen; host
  banner parsers depend on it.
- **Telemetry stream:** primary TLM frame at fixed cadence from power-on
  (boot frames carry device-detection status), secondary diagnostic frame
  interleaved on other cycles.

### Consumes

- **CODAL / codal-microbit-v2 vendor SDK:** `MicroBit` singleton, fiber
  scheduler, `MicroBitI2C`, serial, radio, `system_timer_current_time_us()`.
  Vendor names are exempt from project naming rules.
- **Build-time generators:** `scripts/gen_messages.py`,
  `scripts/gen_boot_config.py`, `scripts/gen_version.py` (run by `build.py`
  codegen before every firmware build).
- **Robot calibration data:** `data/robots/active_robot.json` → baked into
  `config/boot_config.cpp`.

## 6. Open Questions / Known Limitations

- **Line/color steady-state sampling is absent.** `Preamble` detects the
  line and color sensors at boot, but no cycle slot samples them and the
  telemetry schema carries no line/color fields yet. The full perception
  round-robin (otos|line|color) is deliberately deferred.
- **Status is in-flux:** the single-loop rebuild closed sprint 108
  (2026-07-16); open follow-ups live in `clasi/issues/` (turn outlier,
  wedge-latch flicker, sim-mode tour polish). Subsystem docs date their own
  review lines independently.
- **`messages/event.h` is orphaned:** hand-written pre-rebuild code with no
  live consumers (see [messages/DESIGN.md](messages/DESIGN.md) §6) — delete,
  revive, or keep is an open stakeholder call.
- **`types/` is a vestigial grab-bag** — protocol v2 text tags predate the
  binary cutover; what still consumes them (and whether `ReplyFn`/`ReplyCtx`
  survive) needs an audit. See [types/DESIGN.md](types/DESIGN.md).
