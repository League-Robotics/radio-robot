# src/firm — Firmware (root overview)

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-22 · **Status:** in-flux

---

## 1. Purpose

`src/firm` is the robot firmware: a single C++ program for the BBC
micro:bit V2 (nRF52833) that drives a PlanetX Nezha V2
differential-drive robot. It reads wheel encoders and sensors over one
shared I2C bus, closes per-wheel velocity loops, integrates odometry,
and exchanges COBS+CRC-framed protobuf-style messages with a host over
USB serial and the micro:bit radio (sprint 123's COBS+CRC framing,
further cut over by sprint 124's protocol v5 uniform packet grammar —
see §4 below). NOTE: this file predates and was not updated for sprint
122's motion-library extraction, sprint 125–127's `Motion::Planner`
integration, or sprint 128's deletion of the resulting dead `Motion::
WheelSink`/`Motion::MoveQueue`/`Motion::StopCondition`/`Motion::
VelocityShaper` generation (`docs/design/design.md` §2/§5) — its module
list and dependency diagram below still describe the pre-122 shape
(`App::MoveQueue`/`App::Odometry`/`App::StateEstimator`,
`src/firm/kinematics/`, a small `src/firm/motion/`); `docs/design/design.md`
is the current, reconciled source for all of that restructuring. Only
this file's wire-framing content (§4) is kept current — updated again
for sprint 124 ticket 014's protocol v5 doc pass.

It is the **plant** end of the host/robot split — the host side is
[`src/host`](../host/DESIGN.md). The host plans motion (currently just
profiled twists / wheel-velocity MOVEs); the firmware **follows bounded
MOVE commands** — each self-bounding via a stop condition
(time / distance / angle) and a required `timeout` backstop, queued
1-active + 4-pending — and streams telemetry. There is **no deadman**
and no jerk-limited trajectory solver on this side: sprint 115
("gut-to-minimal-firmware S1") deleted the old motion stack, sprint 116
("MOVE protocol cutover", S2) replaced the interim TWIST+deadman surface
with the bounded, queued `Move` command rather than reviving it, and
every motion is now structurally self-bounding. Sprint 117
("predict-to-now estimator v1") added `App::StateEstimator`, a passive
module that extrapolates wheel/body state from the telemetered readings
but does **not** yet drive motion (the trajectory controller that will
consume it is a later sprint, gated on this one being bench-proven).

Everything under this directory compiles into one image (`main.cpp` is
the ARM entry point); the same modules minus the ARM adapters also
compile under `-DHOST_BUILD` for host-side tests and simulation
([`src/sim`](../sim/DESIGN.md)) — see §4.

`src/firm` is one of the two declared design-doc-set source roots
(`.clasi/config.yaml`'s `sources:` — the other is `src/host`). Each
directory one level below it owns its own co-located `DESIGN.md` (§2).
System-wide context that spans **both** roots — the project overview,
the global naming/style conventions, and why the roots are exactly these
two — lives one level up in
[`docs/design/design.md`](../../docs/design/design.md); this doc is the
firmware-tree map only.

## 2. Subsystem Map

One row per one-level-down directory, each linking to its own co-located
`DESIGN.md`.

| Subsystem | Role |
|---|---|
| [`app/`](app/DESIGN.md) | The single cooperatively-timed control loop (`App::RobotLoop`) and its passive modules: Comms, Telemetry, Drive, Odometry, MoveQueue, StateEstimator, Preamble. |
| [`com/`](com/DESIGN.md) | ARM-only raw transports: USB CDC serial, the micro:bit radio, persisted radio-channel storage. |
| [`config/`](config/DESIGN.md) | Generated boot configuration — per-robot calibration baked at build time from `data/robots/active_robot.json`. |
| [`devices/`](devices/DESIGN.md) | I2C-attached device leaves (Nezha motors, OTOS, color/line sensors), the shared `MotorArmor` policy, the velocity PID, and the pure `I2CBus`/`Clock`/`Sleeper` hardware seams. |
| [`kinematics/`](kinematics/DESIGN.md) | Stateless differential-drive math: inverse/forward twist↔wheel maps, curvature-preserving saturation. |
| [`messages/`](messages/DESIGN.md) | The wire schema: generated message structs, the generated envelope codec, the hand-written byte-level wire runtime. |
| [`motion/`](motion/DESIGN.md) | Pure, bounded-motion stop/timeout comparison logic (`Motion::StopCondition`) — a fresh, tiny directory (116), not a revival of the larger `motion/` tree sprint 115 deleted. |
| [`types/`](types/DESIGN.md) | `Types::RobotState` (sprint 124) — the dependency-free per-cycle blackboard struct `src/firm` and `src/motion` both stand on, alongside `Motion::WheelSink`. Also holds vestigial protocol-v2 text-tag constants and the firmware-version generation seam (mostly dead code — see its own §6). |

(`src/firm` has no README stub — this file *is* the tree overview. The
former `src/firm/README-DESIGN.md` pointer, which existed only while a
root-level `DESIGN.md` was disallowed, was removed when this doc was
written; see [`docs/design/design.md`](../../docs/design/design.md) §4
for the rule change that made it obsolete.)

## 3. Architecture — One Cooperatively-Timed Loop

A single cooperatively-timed loop (`App::RobotLoop`) owns **all** I2C bus
access and **all** timing, calling into passive modules that never sleep
and never touch the bus on their own. This replaced an earlier
subsystem/message-dispatch stack (deleted in sprints 102–107).

Flow of one cycle, at orientation altitude:

1. **Comms in** — `App::Comms` polls the two transports (serial, radio)
   for one complete `\n`-terminated line, parses its `<COMMAND>` prefix
   and dispatches by the generated registry's binary/cleartext flag
   (sprint 124 protocol v5 — supersedes the pre-124 `0x00`-delimited
   frame-vs-HELLO/PING-text-rump demux this step used to describe), then
   decodes a binary command line into a `msg::CommandEnvelope`.
2. **Dispatch** — the loop's own switch acts on the command: a `Move`
   enqueues onto `App::MoveQueue` (1 active + 4 pending; `replace=true`
   flushes pending and preempts the active `Move`, `replace=false`
   enqueues or acks `ERR_FULL` past 4 pending), which stages the active
   motion's velocity onto `App::Drive` and drives its own
   `Motion::StopCondition`; a `Stop` flushes the queue and halts `Drive`
   immediately; config/queries reply via the primary telemetry frame's
   bounded ack ring (sprint 124 ticket 008 deleted the older single
   scalar ack slot — ring membership alone means "acked").
3. **Motor service** — the loop runs each `Devices::NezhaMotor`'s
   split-phase encoder request → settle → collect → PID → duty-write
   sequence, with the settle/clearance gaps expressed as
   `runAndWait(gap, body)` blocks whose wait time is borrowed for other
   bounded work (OTOS sampling, odometry integration, telemetry
   assembly).
4. **State out** — `App::Odometry` integrates encoder deltas through
   `BodyKinematics::forward()`; `App::StateEstimator` (117) ingests the
   same cycle's staged `Types::RobotState` and refreshes its wheel/body
   zero-order-hold predict-to-now estimates; `App::Telemetry` projects
   `RobotState` and emits the ONE primary TLM frame through Comms — there
   is no second/secondary frame any more (`msg::TelemetrySecondary` is
   deleted, sprint 124 ticket 009).
5. **Pace** — a final `runAndWait` paces the cycle to `kCycle` = 20 ms
   (~50 Hz), matching `Telemetry::kPrimaryPeriod` so every cycle emits a
   primary frame.

Boot is a separate loop: `App::Preamble` steps per-device detection (one
bounded probe per pass) while telemetry frames report detection status;
command consumption starts only when `preamble.done()`.

**Dependency direction** (arrows = "includes/uses"):

```
main.cpp ──► app ──► devices ──► (nothing project-local except itself)
   │          │  └─► messages, kinematics
   │          └────► com (via ARM-only Transport adapters)
   ├────────► config ──► messages
   └────────► com, devices, config
```

`devices/` is the bottom of the stack and deliberately includes nothing
from `messages/` or `config/`. `kinematics/` and `messages/` are leaf
libraries with no project dependencies of their own. The per-subsystem
docs in §2 carry the module-level detail; this section is only the shape.

## 4. Two Build Targets and the Wire Boundary

**Two build targets, one tree.** Every module under `src/firm` compiles
twice: as the ARM firmware image (`main.cpp` entry, real
`MicroBit.h`-backed transports and I2C), and — minus the explicitly
ARM-only files — under `-DHOST_BUILD` with no `MicroBit.h` anywhere in
the translation unit, linked into the host-side simulator and unit tests
([`src/sim`](../sim/DESIGN.md), `src/tests/sim/`). Hardware seams
(`I2CBus`, `Clock`, `Sleeper`, the transports) are plain virtual bases
with an ARM implementation and a host/sim implementation — never
`#ifdef` forks inside a shared header. This dual-target discipline is
why `devices/` stays free of wire-plane and config types (§5): the more
of the graph that compiles host-side, the more of the robot the sim and
the pytest suite can exercise without hardware.

**Wire boundary — protocol v5 (sprint 124, supersedes sprint 123's
COBS+CRC framing described below).** Every packet, both directions, text
or binary, is one `\n`-terminated line: `<COMMAND>[':' <data>]'\n'`. A
binary command's `<data>` is CRC-then-COBS composed (CRC-16/CCITT-FALSE
over `COMMAND ':' payload`, then COBS-encoded, keyed on `0x0A` — not
`0x00`, so the trailing `\n` terminator is genuinely unconditional; sprint
123's original COBS+CRC cutover keyed on `0x00`, see the sentence this one
replaces). Which verb a line names, and whether its data is cleartext or
binary, is looked up in a generated command registry
(`src/protos/commands.proto` → `messages/commands.h`'s `kVerbTable[]`) —
the sole text/binary discriminator, never a guess about the data's own
bytes. Payloads are `msg::CommandEnvelope` in (`move`/`config`/`stop`
oneof — three binary command verbs, `MOVE`/`CONFIG`/`STOP`), `msg::
ReplyEnvelope` out (`tlm` oneof — the only arm with a live producer;
`ok`/`err` remain declared-only), plus four cleartext verbs answered
inline: `HELLO`→`DEVICE:...` (byte-frozen boot banner), `PING`→
`PONG:t=<ms>`, `ID`→`ID:<drivetrain>:<profile>:<version>` (configured
identity), `VER`→`VER:<version>` (build identity). `msg::
TelemetrySecondary` — a second, independently-framed diagnostic frame —
is DELETED outright (sprint 124 ticket 009); there is only ever one wire
telemetry shape now. The schema source of truth is `src/protos/*.proto`
([`src/protos`](../protos/DESIGN.md)); the codec and byte-level runtime
live in [`messages/`](messages/DESIGN.md). See
[`docs/protocol-v5.md`](../../docs/protocol-v5.md) for the full wire
reference (supersedes `docs/protocol-v4.md`).

**The firmware has no free-form text-plane command parser.** Exactly four
cleartext verbs (`HELLO`/`PING`/`ID`/`VER`) exist, all in the ONE
generated command registry above, alongside the three binary verbs
(`MOVE`/`CONFIG`/`STOP`) — there is no separate "text rump" mechanism any
more (sprint 124 deleted the old two-heuristic text/binary demux along
with the pre-v5 `HELP`/`STOP`-as-text surface this paragraph used to
describe). A line naming an unrecognized command increments
`Comms::malformedCount()` (surfaced as `kFlagFaultCommsMalformed`), no
reply. By longstanding stakeholder decision, legacy (pre-v5) text clients
are served by a **host-side translator proxy**
(`src/host/robot_radio/testgui/binary_bridge.py`), never by reviving a
larger firmware text parser — see [`src/host/robot_radio/DESIGN.md`](../host/robot_radio/DESIGN.md)
§3 for the host end of that contract.

## 5. Cross-Cutting Constraints and Conventions

Every subsystem doc under this root may assume the following without
restating it; each repeats only what is *specific* to it. (Project-wide
naming/style conventions that also apply to `src/host` — CamelCase case
rules, "no units in identifiers", generated-file rules — live in
[`docs/design/design.md`](../../docs/design/design.md) §3; the items
below are the firmware-tree-specific ones.)

- **Single-loop bus ownership:** all I2C traffic happens from the loop's
  own cycle, in the loop's documented order; no module ever initiates
  bus traffic from its own `tick()`/staging methods. Violating this
  reintroduces the shared-bus timing collisions that wrecked motion
  timing and can hard-stall the nRF52 TWIM peripheral.
- **App modules are passive and bounded:** `Drive::tick()`, `Odometry`
  integration, `Telemetry` assembly, `Preamble::step()`, and every
  `runAndWait` body must be bounded, non-sleeping, non-bus-touching
  work. A sleep or blocking I2C call inside one silently destroys the
  cycle's timing budget and starves the CODAL fiber scheduler (the radio
  *looks* dead when the loop doesn't yield).
- **Critical waits are explicit:** every required gap in the schedule is
  a `runAndWait(gap, body)` block in `robot_loop.cpp` — the name carries
  the wait, the block scopes the work that borrows it. Never hide a
  sleep inside a work function.
- **Devices isolation invariant:** `devices/` must not include
  `messages/` or `config/` headers. Wire-plane types (`msg::*`) stop at
  the `app/` layer; conversion between `msg::MotorConfig` and
  `Devices::MotorConfig` happens in `main.cpp`, the one place both types
  are reachable.
- **HOST_BUILD purity:** every module except the explicitly ARM-only
  files must compile under `-DHOST_BUILD` with no `MicroBit.h` anywhere
  in the translation unit (§4). Hardware seams are plain virtual bases.
- **No deadman — every `Move` is structurally self-bounding:**
  `App::MoveQueue::tick()` runs unconditionally every cycle and drains
  to `Drive::stop()` once the active `Move`'s stop condition or
  `timeout` fires and nothing is pending — an emergent property of every
  queued command carrying its own bound, not a second, independently
  timed staleness timer. `App::Deadman` does not exist in this tree, and
  no ad hoc watchdog belongs anywhere in the firmware.
- **Wire compatibility outranks naming:** wire key strings, TLM field
  tokens, reply tag strings, and the `DEVICE:NEZHA2:…` banner format are
  frozen protocol surface, excluded from the naming-convention rename
  sweep.
- **Vendor names are exempt from the naming conventions:** functions the
  CODAL/micro:bit SDK declares (e.g. `system_timer_current_time_us()`)
  keep their upstream names — this project does not control them and
  cannot rename them without forking the SDK. Only each call site's own
  local variables derived from a vendor return value follow the normal
  convention. See `.claude/rules/coding-standards.md`'s
  "external/vendor function names are excluded" clause.
- **`newlib-nano` has no `%f`:** `printf`-family float formatting emits
  nothing on ARM (works fine in host builds). Floats cross the wire as
  scaled integers or via the binary codec.
- **Bench gate:** firmware changes to the HAL, motor control, sensing,
  or protocol are not done until exercised on the robot on its stand
  (`.claude/rules/hardware-bench-testing.md`). Host tests alone do not
  close a change.

## 6. Recent Structural Changes & Open Questions

**Landed (the current shape of this tree):**

- **115 (gut-to-minimal-firmware S1):** `Motion::Executor`/
  `Motion::JerkTrajectory`/`vendor/ruckig`, `App::Pilot`, and
  `App::HeadingSource` were DELETED wholesale, along with
  `msg::PlannerConfig`/its own curated live-tuning message (`planner.proto` deleted).
  Tagged `pre-gut-motion-stack` for full recoverability — read that tag
  and sprint 115's own `architecture-update.md` for the pre-gut
  architecture, not this doc.
- **116 (MOVE protocol cutover, S2):** `Twist` (arm 19) and
  `ConfigDelta.watchdog` (field 4) are `reserved`, not reused; `App::
  Deadman` is deleted. The new `Move` arm (21) carries its own velocity
  (twist or wheels variant), a stop condition, and a required `timeout`,
  dispatched through `App::MoveQueue` (1 active + 4 pending) driving one
  `Motion::StopCondition` per active `Move`. `motion/` was recreated as a
  fresh, tiny directory holding only that pure comparison logic.
  `kFlagFaultMoveTimeout` (bit 15) is wired (set on the cycle an active
  `Move` ends via `timeout` rather than its stop condition).
- **117 (predict-to-now estimator v1):** `App::StateEstimator` ticks once
  per cycle reading the same staged `Frame` — no new on-chip measurement
  storage, no bus access of its own. It holds per-wheel and body state as
  independently-valid/stale peer estimates, extrapolated
  zero-order-hold, plus a v1 complementary blend against OTOS
  heading/omega whose weights are fail-closed baked config defaulting to
  0.0 (encoder-only this sprint) and were live-tunable via a dedicated
  wire arm — **not** persisted to flash (a reboot reverts to the baked
  default). Its predictions are not exposed on the wire; validation runs
  host-side against the raw `EncoderReading`/`OtosReading` fields via a
  captured TLM-log CSV. HISTORICAL as of 132-013 (patch-surface
  retirement) / 128-016 (App::StateEstimator itself deleted as dead
  code): the ESTIMATOR group (robot_config.proto's `Estimator`) still
  decodes on the wire for read-back but reaches no live consumer
  (Configurator::install(ESTIMATOR) permanently returns
  ERR_UNIMPLEMENTED).

**Open, firmware-tree-wide** (each subsystem doc's own §6 carries its
local items):

- **The trajectory controller that consumes `StateEstimator` is future
  work**, gated on this estimator being bench-proven; fake OTOS,
  external/camera pose fusion, and the remaining-distance controller are
  not part of 117.
- **Reviving the host-side higher-level motion machinery** (tour / path /
  nav) onto the new MOVE wire surface is explicit future work — 116's
  host scope was limited to the low-level `move_twist()`/`move_wheels()`
  builders. See [`src/host/robot_radio/DESIGN.md`](../host/robot_radio/DESIGN.md)
  §6.
- **`messages/event.h` remains orphaned dead code** and **`types/`
  remains a vestigial grab-bag** — see those docs' own §6.
