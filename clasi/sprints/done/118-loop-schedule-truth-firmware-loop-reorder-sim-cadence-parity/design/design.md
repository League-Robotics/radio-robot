---
source_paths:
- /Volumes/Proj/proj/RobotProjects/radio-robot-elite/src/firm
- /Volumes/Proj/proj/RobotProjects/radio-robot-elite/src/host
---
# radio-robot-elite — System Design

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-21 · **Status:** in-flux

---

## 1. Project Overview

This project builds and drives a small differential-drive robot (a
PlanetX Nezha V2 chassis on a BBC micro:bit V2 / nRF52833) over a
host/robot split: a minimal C++ firmware that follows bounded MOVE
commands and streams telemetry, and a Python host package that talks to
it over USB serial or a radio relay, with a parallel host-build
simulator for development without hardware. The current architecture
(post sprint 118, "loop schedule truth", on `master`) is
deliberately minimal — the firmware speaks exactly three inbound
commands (**MOVE / CONFIG / STOP**): `Move` carries its own velocity
(twist or wheels variant), a stop condition (time/distance/angle), and a
required `timeout` backstop, queued 1-active + 4-pending — and emits one
telemetry frame (**frame v2**: per-wheel `EncoderReading`/`OtosReading`
with their own sample times, a single `flags` bit-string, a single ack
slot, packed line/color words) every 40 ms cycle (118 — restored from a
fictional 20 ms; see §5's "Pace" step below). There is still **no**
jerk-limited trajectory solver and no heading-source policy on the
firmware side — sprint 115 ("gut-to-minimal-firmware S1") deleted the
old motion stack, and sprint 116 ("MOVE protocol cutover", S2) replaced
the interim TWIST+deadman surface with the bounded, queued `Move`
command rather than reviving it. Every motion is now structurally
self-bounding (its own stop condition or timeout), which supersedes the
deadman it replaces — there is no `App::Deadman` anywhere in this tree.
Sprint 117 adds `App::StateEstimator`, a passive predict-to-now module
that extrapolates wheel/body state from the same telemetered readings —
it does not yet drive motion (the trajectory controller that will
consume it is a later sprint, gated on this one being bench-proven).

The host side (`src/host/robot_radio/`) still carries the code that was
built against the pre-115 motion stack — tour/path/navigation planning —
but by deliberate stakeholder decision (sprint 115's Design Rationale,
Decision 6) that code was left in the tree rather than deleted. Sprint
116 gave the host a new low-level wire surface to target
(`NezhaProtocol.move_twist()`/`move_wheels()`) but deliberately did NOT
revive the higher-level tour/path/navigation machinery — `planner/`,
`path/`, `nav/`, and the TestGUI tour/turn modules stay dormant, by the
same stakeholder decision, until a separate future sprint takes that on.
See [`src/host/robot_radio/DESIGN.md`](../../src/host/robot_radio/DESIGN.md)
for exactly which parts are live today and which are dormant.

## 2. Subsystem Map

One line per subsystem, each linking to its own co-located `DESIGN.md`.
The design-doc-set's declared source roots
(`.clasi/config.yaml`'s `sources:`) are **exactly** `src/firm` and
`src/host` — see §4 for why, and §5 for the other, real-but-unvalidated
docs living outside those two roots.

### `src/firm` — firmware (declared root)

| Subsystem | Role |
|---|---|
| [`app/`](../../src/firm/app/DESIGN.md) | The single cooperatively-timed control loop (`App::RobotLoop`) and its passive modules: Comms, Telemetry, Drive, Odometry, MoveQueue, StateEstimator, Preamble. |
| [`com/`](../../src/firm/com/DESIGN.md) | ARM-only raw transports: USB CDC serial, the micro:bit radio, persisted radio-channel storage. |
| [`config/`](../../src/firm/config/DESIGN.md) | Generated boot configuration — per-robot calibration baked at build time from `data/robots/active_robot.json`. |
| [`devices/`](../../src/firm/devices/DESIGN.md) | I2C-attached device leaves (Nezha motors, OTOS, color/line sensors), the shared `MotorArmor` policy, the velocity PID, and the pure `I2CBus`/`Clock`/`Sleeper` hardware seams. |
| [`kinematics/`](../../src/firm/kinematics/DESIGN.md) | Stateless differential-drive math: inverse/forward twist↔wheel maps, curvature-preserving saturation. |
| [`messages/`](../../src/firm/messages/DESIGN.md) | The wire schema: generated message structs, the generated envelope codec, the hand-written byte-level wire runtime. |
| [`motion/`](../../src/firm/motion/DESIGN.md) | Pure, bounded-motion stop/timeout comparison logic (`Motion::StopCondition`) — no owned state beyond what's passed into `tick()`, no dependency on `MoveQueue`/`Drive`/wire types. A fresh, tiny directory (116) — not a revival of the larger `motion/` tree sprint 115 deleted. |
| [`types/`](../../src/firm/types/DESIGN.md) | Vestigial protocol-v2 text-tag constants and the firmware-version generation seam (mostly dead code — see its own §6). |

(`src/firm/README-DESIGN.md` is a one-paragraph pointer back to this
document — `src/firm` itself has no co-located `DESIGN.md`; see §4.)

### `src/host` — host-side Python (declared root)

| Subsystem | Role |
|---|---|
| [`robot_radio/`](../../src/host/robot_radio/DESIGN.md) | The importable host package: transports, the wire-protocol adapter, per-robot config loading, calibration, sensor decoding, the `rogo` CLI, an MCP server, and the PySide6 TestGUI. Mixed live/dormant — see its own doc for the file-by-file split. |

### Other source trees (real documentation, outside the declared roots)

These directories are **not** part of the mechanically-validated design
doc set (see §4) but carry real, current `DESIGN.md` files anyway,
because they have architecturally significant content worth documenting
even though nothing requires it:

| Subsystem | Role |
|---|---|
| [`src/sim/`](../../src/sim/DESIGN.md) | The host-build firmware simulator: compiles the real firmware into a shared library, drives it from Python over an `extern "C"` ABI. One sim object shared by the pytest suite and the TestGUI. |
| [`src/protos/`](../../src/protos/DESIGN.md) | The wire-schema source of truth (`.proto` files) both the firmware and host codegen compile from. |
| [`src/scripts/`](../../src/scripts/DESIGN.md) | Build-time code generators (messages, host protobuf bindings, boot config, firmware version) plus one CI-only config-sync lint. |
| [`src/tests/`](../../src/tests/DESIGN.md) | Three never-combined test domains (`sim/`, `bench/`, `playfield/`) plus flat `unit/`/`tools/`/`notebooks/`/`testgui/` categories. |
| [`src/utils/`](../../src/utils/DESIGN.md) | Build/flash tooling: CMake helper modules, UF2/hex conversion scripts, a couple of debug-console snippets. |

### Trees with no architecturally significant content

These are deliberately **excluded** from `sources:` (see §4) and were
given no `DESIGN.md` requirement — the three that are safely in-repo
still got a short, honest stub anyway (linked below); the fourth,
`src/vendor`, was deliberately left undocumented because it resolves
outside this repository entirely (see §4's last paragraph).

| Subsystem | Role |
|---|---|
| [`src/archive/`](../../src/archive/DESIGN.md) | Parked pre-rebuild trees and one-off historical artifacts. Never imported by anything live. |
| [`src/libraries/`](../../src/libraries/DESIGN.md) | The vendored CODAL SDK — entirely `.gitignore`d, fetched by build tooling, zero project-authored content. |
| [`src/recordings/`](../../src/recordings/DESIGN.md) | Recorded telemetry JSONL output — a data directory, not source. |
| `src/vendor` | A symlink to an unrelated, external, actively-developed project (`/Volumes/Proj/proj/league-projects/scratch/radio-robot/vendor`) — **not documented here** for the reason given in §4; never written to by this project's tooling. |

## 3. Global Conventions

Every subsystem doc in this set may assume the following without
restating it:

- **Naming case (CamelCase, Google's default overridden — stakeholder
  rule, 2026-07-04):** UpperCamelCase for class/struct/namespace names
  (acronyms fully capitalized, e.g. `HTTPServer`); lowerCamelCase for
  variable and function names (acronyms fully lowercased at the start,
  e.g. `httpRequest`) — **function and method names never start with an
  uppercase letter.** Class data members keep a trailing underscore
  (`lastPosition_`). Filenames stay snake_case. See
  `.claude/rules/naming-and-style.md` and
  `docs/reference/google-cppguide-condensed.md` (the operative C++
  style reference — the project follows the Google C++ Style Guide
  except for this naming-case override).
- **No units in any identifier.** Field, method, function, and parameter
  names describe the *kind* of quantity (`speed`, `velocity`, `position`),
  never the unit. Units go in a leading bracketed comment tag:
  `// [mm/s]` in C++, `# [ms]` in Python — first token of the trailing
  or block comment. `speed` is a directionless magnitude; `velocity` is
  directed; a body twist always has explicit components (`v_x`, `v_y`,
  `omega`), never a bare directionless `v`. Full convention, vocabulary,
  and exclusions in `.claude/rules/coding-standards.md`.
- **Wire keys are protocol, not identifiers — excluded from the naming
  rules above.** `SET`/`GET`/`SIMSET`/`SIMGET` wire key strings,
  `TLM`/`SNAP` field-name tokens, and JSON config keys in
  `data/robots/*.json` (mirrored 1:1 by
  `src/host/robot_radio/config/robot_config.py`'s pydantic fields) are
  serialized/persisted or cross a wire boundary; renaming one is a
  protocol change, not a code-readability change, and stays stable even
  when the internal identifier next to it is renamed.
- **Config is fail-closed truth from `data/robots/*.json` — no
  behavioral defaults baked into source.** Per sprint 114
  ("config-as-truth"), an unconfigured device or a robot JSON missing a
  required calibration key fails loudly (`ERR_NOT_CONFIGURED` on the
  wire; `MissingRobotConfigKeyError`/`sys.exit(1)` at codegen time) —
  never a silently-substituted bench-placeholder constant. See
  [`src/firm/config/DESIGN.md`](../../src/firm/config/DESIGN.md) §4 and
  [`src/scripts/DESIGN.md`](../../src/scripts/DESIGN.md) §3.
- **Google C++ Style Guide, condensed, project overrides applied
  inline.** The operative reference is
  `docs/reference/google-cppguide-condensed.md`, not the full vendored
  HTML guide — read it, not the upstream doc. Project overrides
  (naming case above; others in `.claude/rules/coding-standards.md` and
  `.claude/rules/naming-and-style.md`) take precedence where they
  conflict with the vendored guide.
- **Generated files are never hand-edited.** Every codegen output this
  project produces (`src/firm/messages/*.h`,
  `src/firm/config/boot_config.cpp`,
  `src/firm/types/version_generated.h`,
  `src/host/robot_radio/robot/pb2/*_pb2.py`) carries this rule; a hand
  edit is silently destroyed the next build with no error. Fix the
  generator (`src/scripts/`) or its source
  (`src/protos/*.proto`, `data/robots/*.json`, root `pyproject.toml`).

## 4. Design-Doc-Set Source Roots — Why Exactly `src/firm` and `src/host`

`.clasi/config.yaml`'s `sources:` declares **exactly two roots**:
`src/firm` and `src/host`. This is a deliberate, narrower choice than
the obvious `[src]`, made because the design-doc-set's mechanical model
— "every one-level-down child of a declared root must have its own
co-located `DESIGN.md`, no exceptions, no exclusion list" — collides
with two real properties of this repository:

1. **`src/vendor` is a symlink resolving OUTSIDE this repository**, to
   an unrelated, actively-developed git checkout at
   `/Volumes/Proj/proj/league-projects/scratch/radio-robot/vendor`. If
   `src` were a declared root, `vendor` would become a "required"
   one-level-down subsystem forever — the only way to satisfy that
   requirement is a file write into someone else's live project tree,
   which this project's tooling will never do. `clasi` 0.20260720.1 has
   no exclusion mechanism for this (`Project.excluded_paths` exists but
   is consumed only by role-guard's `protected_paths` carve-out, not by
   `clasi.design.store._subsystem_dirs`/the validator — confirmed by
   reading the installed package, not assumed).
2. **A permanently-failing `validate_design` is not a cosmetic gap.**
   `close_sprint`'s overlay-apply step runs full canonical validation
   and fails closed — a bare `[src]` (with its unresolvable `vendor`
   requirement) would silently block every future sprint close, not
   just leave one line item unchecked in this bootstrap.

Declaring `src/firm` and `src/host` as their own roots instead sidesteps
this cleanly: their real children (`app`/`com`/`config`/`devices`/
`kinematics`/`messages`/`types` under `src/firm`; `robot_radio` under
`src/host`) become the one-level-down subsystems the validator expects
— matching the actual doc placement exactly — while `src/vendor`,
`src/archive`, `src/libraries`, `src/protos`, `src/recordings`,
`src/scripts`, `src/sim`, `src/tests`, and `src/utils` never enter the
enumeration at all, not even as a required stub. Most of those trees
still have real, current `DESIGN.md` files (§2's "Other source trees"
table) — they are simply unvalidated-but-real documentation, kept honest
by hand rather than by the mechanical gate. `src/archive`,
`src/libraries`, and `src/recordings` got a short, honest "no
architecturally significant content" doc for the same reason (§2's last
table) — `src/vendor` alone was left undocumented, because writing even
a one-line stub there means writing into that other, unrelated
repository, which was deliberately never done.

One structural consequence: `src/firm` is itself a declared root, so a
`DESIGN.md` sitting directly inside it (rather than inside one of its
own children) has no home to validate against and is swept up as an
orphaned doc by `clasi.design.validator`'s per-root `rglob` check. That
firmware-tree overview therefore lives here instead — see §5 below,
folded in from the file that used to be `src/firm/DESIGN.md` — with a
one-paragraph pointer left at `src/firm/README-DESIGN.md` so a
stakeholder's habit of opening that path doesn't dead-end.

## 5. Firmware-Tree Overview (folded from the former `src/firm/DESIGN.md`)

`src/firm` is the robot firmware: a single C++ program for the BBC
micro:bit V2 (nRF52833) that drives a PlanetX Nezha V2 differential-drive
robot. It reads wheel encoders and sensors over one shared I2C bus,
closes per-wheel velocity loops, integrates odometry, and exchanges
binary-armored protobuf-style messages with a host over USB serial and
the micro:bit radio. It is the "plant" end of the host/robot split: the
host plans motion (currently just profiled twists/wheel-velocity
MOVEs — see
[`src/host/robot_radio/DESIGN.md`](../../src/host/robot_radio/DESIGN.md));
the firmware follows bounded MOVE commands — each self-bounding via a
stop condition and a required timeout, queued 1-active + 4-pending —
and streams telemetry; there is no deadman. Everything under this
directory compiles into one image
(`main.cpp` is the ARM entry point); the same modules minus the ARM
adapters also compile under `-DHOST_BUILD` for host-side tests and
simulation (`src/sim/`).

**Architecture: a single cooperatively-timed loop** (`App::RobotLoop`)
owns all I2C bus access and all timing, calling into passive modules
that never sleep and never touch the bus on their own. This replaced an
earlier subsystem/message-dispatch stack (deleted in sprints 102–107).

**115-002/115-003/115-005/115-006 (gut-to-minimal-firmware S1
motion-stack excision):** `Motion::Executor`/`Motion::JerkTrajectory`/
`vendor/ruckig`, `App::Pilot`, and `App::HeadingSource` are DELETED
wholesale — the `motion/` directory (and `motion/DESIGN.md`) no longer
exist. There is no arc/segment queue and no heading-source policy in
S1's minimal firmware; the robot was, at that point, a pure
TWIST-follower plus a deadman. `msg::PlannerConfig` and
`PlannerConfigPatch` are gone with them (`planner.proto` deleted). This
is tagged `pre-gut-motion-stack` for full recoverability — the tag and
sprint 115's own `architecture-update.md` are where to read about the
pre-gut architecture, not this doc.

**116 (MOVE protocol cutover, S2) — landed.** The TWIST+deadman surface
above is superseded, not extended: `Twist` (arm 19) and
`ConfigDelta.watchdog` (field 4) are `reserved`, not reused; `App::
Deadman` is deleted (`app/deadman.{h,cpp}`, both test harnesses). A new
`Move` arm (21) carries its own velocity (twist or wheels variant), a
stop condition (time/distance/angle), and a required `timeout`,
dispatched through a new `App::MoveQueue` (1 active + 4 pending) that
drives one `Motion::StopCondition` per active `Move`. `motion/` is
recreated as a fresh, tiny directory containing only
`Motion::StopCondition` — pure stop/timeout comparison logic, unrelated
to and much smaller than the deleted `Motion::Executor`/
`Motion::JerkTrajectory` tree above. See
[`src/firm/app/DESIGN.md`](../../src/firm/app/DESIGN.md) and
[`src/firm/motion/DESIGN.md`](../../src/firm/motion/DESIGN.md) for the
full detail.

**117 (predict-to-now estimator v1) — landed.** A new passive `app/`
module, `App::StateEstimator`, ticks once per cycle (trailing `kPace`
block, after OTOS sampling and odometry integration) reading the SAME
`Frame` data `Telemetry` already stages — no new on-chip measurement
storage, no bus access of its own. It holds per-wheel and body state as
PEER estimates (each independently valid/stale), extrapolated
zero-order-hold ("predict to now": `distance = basis.position +
basis.velocity × age`, generalizing the deleted `HeadingSource::
headingLead()` equation to the full body pose) plus a v1 complementary
blend against OTOS heading/omega whose weights are fail-closed baked
config, defaulting to 0.0 (encoder-only output this sprint, per
stakeholder decision) and live-tunable via a new `ConfigDelta.estimator`
(`EstimatorConfigPatch`) oneof arm, mirroring `OtosConfigPatch`'s
existing merge-then-apply pattern — NOT persisted to flash (unlike motor
gains/OTOS calibration; a reboot reverts to the baked default). The
estimator's predictions are NOT exposed on the wire this sprint —
validation (leave-one-out one-step-ahead RMS analysis) runs host-side
directly against the raw `EncoderReading`/`OtosReading` fields sprint 115
already telemetered, via a captured TLM-log CSV, not a live query
against the on-chip estimator instance. See
[`src/firm/app/DESIGN.md`](../../src/firm/app/DESIGN.md) for the full
detail.

Flow of one cycle, at orientation altitude:

1. **Comms in** — `App::Comms` polls the two transports (serial, radio)
   for one armored `*B` line, dearmors and decodes it into a
   `msg::CommandEnvelope`.
2. **Dispatch** — the loop's own switch acts on the command: a Move
   enqueues onto `App::MoveQueue` (1 active + 4 pending; `replace=true`
   flushes pending and preempts the active `Move`, `replace=false`
   enqueues or acks `ERR_FULL` past 4 pending), which stages the active
   motion's velocity onto `App::Drive` and drives its own
   `Motion::StopCondition`; a Stop flushes the queue and halts `Drive`
   immediately; config/queries reply via the primary telemetry frame's
   single ack slot (`ack_corr`/`ack_err`, valid iff `flags` bit 5 — see
   [`src/firm/app/DESIGN.md`](../../src/firm/app/DESIGN.md) §2).
3. **Motor service** — the loop runs each `Devices::NezhaMotor`'s
   split-phase encoder request → settle → collect → PID → duty-write
   sequence, with the settle/clearance gaps expressed as
   `runAndWait(gap, body)` blocks whose wait time is borrowed for other
   bounded work (OTOS sampling, odometry integration, telemetry
   assembly).
4. **State out** — `App::Odometry` integrates encoder deltas through
   `BodyKinematics::forward()`; `App::StateEstimator` (117) ingests the
   same cycle's staged `Frame` and refreshes its wheel/body ZOH
   predict-to-now estimates; `App::Telemetry` emits the primary TLM frame
   (or the slower secondary diagnostic frame) through Comms.
5. **Pace** — a final `runAndWait` paces the cycle to `kCycle` = 40 ms
   (~25 Hz), matching `Telemetry::kPrimaryPeriod` so every cycle emits a
   primary frame. (118 — restores the schedule's genuine 4ms/4ms
   settle/clearance budget, regressed to a fictional 20ms/~50Hz by commit
   `5f5a2ba7`; the sim's own `SimHarness::kCycleDtUs` now matches this
   value exactly, closing the sim/firmware cadence gap — see
   [`src/sim/DESIGN.md`](../../src/sim/DESIGN.md).)

Boot is a separate loop: `App::Preamble` steps per-device detection (one
bounded probe per pass) while telemetry frames report detection status;
command consumption starts only when `preamble.done()`.

Dependency direction (arrows = "includes/uses"):

```
main.cpp ──► app ──► devices ──► (nothing project-local except itself)
   │          │  └─► messages, kinematics
   │          └────► com (via ARM-only Transport adapters)
   ├────────► config ──► messages
   └────────► com, devices, config
```

`devices/` is the bottom of the stack and deliberately includes nothing
from `messages/` or `config/`. `kinematics/` and `messages/` are leaf
libraries with no project dependencies of their own.

**Cross-cutting constraints and invariants** (each subsystem doc repeats
only what's specific to it — this is the shared set):

- **Single-loop bus ownership:** all I2C traffic happens from the loop's
  own cycle, in the loop's documented order; no module ever initiates
  bus traffic from its own `tick()`/staging methods. Violating this
  reintroduces the shared-bus timing collisions that wrecked motion
  timing and can hard-stall the nRF52 TWIM peripheral.
- **App modules are passive and bounded:** `Drive::tick()`, `Odometry`
  integration, `Telemetry` assembly, `Preamble::step()`, and every
  `runAndWait` body must be bounded, non-sleeping, non-bus-touching
  work. A sleep or blocking I2C call inside one silently destroys the
  cycle's timing budget and starves the CODAL fiber scheduler (the
  radio *looks* dead when the loop doesn't yield).
- **Critical waits are explicit:** every required gap in the schedule is
  a `runAndWait(gap, body)` block in `robot_loop.cpp` — the name
  carries the wait, the block scopes the work that borrows it. Never
  hide a sleep inside a work function.
- **Devices isolation invariant:** `devices/` must not include
  `messages/` or `config/` headers. Wire-plane types (`msg::*`) stop at
  the `app/` layer; conversion between `msg::MotorConfig` and
  `Devices::MotorConfig` happens in `main.cpp`, the one place both types
  are reachable.
- **HOST_BUILD purity:** every module except the explicitly ARM-only
  files must compile under `-DHOST_BUILD` with no `MicroBit.h` anywhere
  in the translation unit. Hardware seams are plain virtual bases —
  never `#ifdef` forks inside a shared header.
- **Generated files are never hand-edited** — see §3's global
  convention.
- **Wire compatibility outranks naming:** wire key strings, TLM field
  tokens, reply tag strings, and the `DEVICE:NEZHA2:...` banner format
  are frozen protocol surface, excluded from the naming-convention
  rename sweep — see §3.
- **No deadman — every `Move` is structurally self-bounding:**
  `App::MoveQueue::tick()` runs unconditionally every cycle and drains
  to `Drive::stop()` once the active `Move`'s stop condition or
  `timeout` fires and nothing is pending — an emergent property of every
  queued command carrying its own bound, not a second, independently-
  timed staleness timer. `App::Deadman` does not exist in this tree. No
  ad hoc watchdog belongs anywhere in the firmware.
- **`newlib-nano` has no `%f`:** `printf`-family float formatting emits
  nothing on ARM (works fine in host builds). Floats cross the wire as
  scaled integers or via the binary codec.
- **Bench gate:** firmware changes to the HAL, motor control, sensing,
  or protocol are not done until exercised on the robot on its stand
  (`.claude/rules/hardware-bench-testing.md`). Host tests alone do not
  close a change.

**Wire boundary.** Armored binary command/reply protocol: `*B<base64>`
lines over USB serial (115200 CDC) and the micro:bit radio (group 10,
channel 0–35 persisted in flash). Payloads are `msg::CommandEnvelope` in
(`move`/`config`/`stop` oneof), `msg::ReplyEnvelope` (`ok`/`err`/`tlm`
oneof) out, plus an independently-armored `msg::TelemetrySecondary`
frame. Schema source of truth: `src/protos/*.proto`. Boot banner:
`DEVICE:NEZHA2:robot:<name>:<serial>` — byte-frozen. See
[`src/firm/messages/DESIGN.md`](../../src/firm/messages/DESIGN.md) and
[`src/firm/app/DESIGN.md`](../../src/firm/app/DESIGN.md) for the full
wire shape and dispatch detail, and
[`src/protos/DESIGN.md`](../../src/protos/DESIGN.md) for the schema
source of truth itself.

**Open, firmware-tree-wide items** (each subsystem doc's own §6 carries
its local ones): line/color steady-state sampling has since landed
(115-005 — see [`src/firm/app/DESIGN.md`](../../src/firm/app/DESIGN.md)
§2's `updateLineColor()`); `src/firm/messages/event.h` remains orphaned
dead code (see that doc's own §6); `src/firm/types/` remains a
vestigial grab-bag (see that doc's own §6); sprint 116's MOVE protocol
has landed — `Twist` (arm 19) and `ConfigDelta.watchdog` (field 4) are
`reserved`, not reused; `App::Deadman` is deleted; see
[`src/firm/motion/DESIGN.md`](../../src/firm/motion/DESIGN.md) for the
new `Motion::StopCondition` module. Sprint 117's `App::StateEstimator`
has landed — see [`src/firm/app/DESIGN.md`](../../src/firm/app/DESIGN.md)
for its full boundary/interface detail.

## 6. Open Questions / Known Limitations (system-level)

- **Sprint 116 (the bounded MOVE protocol) has landed.**
  `kFlagFaultMoveTimeout` (bit 15) is now wired firmware-side (set on
  the cycle an active `Move` ends via `timeout` rather than its stop
  condition). Host-side `src/host/robot_radio/planner/`, `path/`,
  `nav/`, and the TestGUI tour/turn modules remain dormant — 116's
  host-side scope was limited to `protocol.py`'s low-level
  `move_twist()`/`move_wheels()` builders; reviving the higher-level
  tour/nav machinery onto the new wire surface is explicit future work,
  not part of 116.
- **Sprint 117 (predict-to-now estimator v1) has landed.**
  `App::StateEstimator` ticks every cycle with wheel/body peer ZOH
  estimates; its OTOS-fusion weights are fail-closed baked config,
  defaulting to 0.0 (encoder-only v1) and live-tunable via the new
  `ConfigDelta.estimator` arm — NOT persisted to flash. Its predictions
  are not exposed on the wire; validation runs host-side against the raw
  telemetered readings (a captured TLM-log CSV), per the stakeholder's
  leave-one-out one-step-ahead RMS methodology. Fake OTOS, external/
  camera pose fusion, and the remaining-distance trajectory controller —
  the source issue's further-out goals — remain future work, not part of
  117.
- **The design-doc-set's mechanical validator cannot express "this
  child is out of scope because it symlinks outside the repository."**
  `src/vendor` remains permanently undocumented for that reason (§4).
  Revisit if `clasi` ever grows an `excluded_paths`-equivalent that the
  design validator itself consults.
- **`src/host/robot_radio`'s live/dormant split is not clean at the
  file level** — see that doc's own §2/§3 for the specific traps
  (several nominally-live directories contain dormant functions).
