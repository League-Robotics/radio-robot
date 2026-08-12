---
source_paths:
- /Volumes/Proj/proj/RobotProjects/radio-robot-elite/src/firm
- /Volumes/Proj/proj/RobotProjects/radio-robot-elite/src/host
---
# radio-robot-elite — System Design

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-24 · **Status:** in-flux

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
| [`platform/`](../../src/firm/platform/DESIGN.md) | Board/runtime primitives ONLY: `Hal::I2CBus`/`Hal::Clock`/`Hal::Sleeper`/`Hal::Transport` implementations, per target — `microbit/` (CODAL: `MicroBitI2CBus`, `MicroBitClock`/`Sleeper`, and, since 136-005 dissolved `com/` here, `MicroBitSerialPort`/`MicroBitRadioLink`) and `host/` (the sim, was `src/sim`). Knows nothing about what is on the bus. |
| [`hardware/`](../../src/firm/hardware/DESIGN.md) | Concrete device drivers, one class per physical part, filed by who else could reuse them: `generic/` (RealOtos, MotorArmor, BoardMotor), `nezha/`, `hiwonder/`, `planetx/`. Register maps, timing quirks, and hardware workarounds live here and nowhere else. |
| [`hal/`](../../src/firm/hal/DESIGN.md) | Interfaces for composable devices — `Hal::Motor`, `Hal::MotorBoard`, `Hal::Otos`, `Hal::ColorSensor`, `Hal::LineSensor` — plus the plain-aggregate reading/config vocabulary they speak in. Interfaces only: no chip knowledge. |
| [`kinematics/`](../../src/firm/kinematics/DESIGN.md) | `Kinematics::Model`, the swappable twist↔wheel-speed map, with `DifferentialKinematics` (the former `BodyKinematics` math) and `MecanumKinematics` behind it. The one sanctioned home for chassis geometry (track width, wheelbase). |
| [`motion/`](../../src/firm/motion/DESIGN.md) | The motion-control library: `Motion::Planner`, `Motion::Navigator`, `Motion::Odometry`. Under active development, with its own standalone Python-free CMake builds (`motion_tests`, `planner_tests`, `navigator_tests`). Imports nothing from the rest of `src/firm` except `messages/` and `firm/types/`. |
| [`control/`](../../src/firm/control/DESIGN.md) | The wheel-speed control law — `Control::DifferentialDrive` (`fastPid()` plus the Stage A/B/C correction, bias adaptation, and stall/deficit-latch machinery). Relocated out of `core/` (136-006, "control law out of `core/` — new `control/` layer"): neither orchestration nor geometry nor an interface, so it gets its own single-purpose layer between `hal/` and `core/`. |
| [`core/`](../../src/firm/core/DESIGN.md) | Orchestration (was `app/`): the single cooperatively-timed control loop (`Core::RobotLoop`), the Robot composition root (`Core::composeRobot()`/`RobotGraph`), and the passive modules the loop owns — Comms, Telemetry, `Control::DifferentialDrive`, Configurator, Preamble. `RobotLoop` drives the motion library, which writes `Types::RobotState::Wheel::cmdVelocity` directly (128 — the plain blackboard field IS the boundary, no interface). |
| [`config/`](../../src/firm/config/DESIGN.md) | Generated boot configuration — per-robot calibration baked at build time from `data/robots/active_robot.json`. |
| [`messages/`](../../src/firm/messages/DESIGN.md) | The wire schema: generated message structs, the generated envelope codec, the hand-written byte-level wire runtime. |
| [`types/`](../../src/firm/types/DESIGN.md) | `Types::RobotState` (sprint 124) — the dependency-free, per-cycle blackboard struct that is a shared floor the whole tree stands on; its own `Wheel::cmdVelocity` field is THE core/motion actuation boundary (128, see §5's "Wire boundary" note and the dependency graph below). Also holds vestigial protocol-v2 text-tag constants and the firmware-version generation seam (mostly dead code — see its own §6). |

The rows above are listed in **dependency order, bottom to top**: platform →
hardware → hal → kinematics → motion → control → core, with `config/`,
`messages/` and `types/` as cross-cutting floors any layer may stand on.
(`com/` used to be a fourth cross-cutting floor; 136-005 dissolved it —
its two transports now implement `Hal::Transport` directly from
`platform/microbit/`, and its DESIGN.md content folded into `platform/`'s
and `hal/`'s own.)
Dependencies run strictly downward — `hal/` names no bus, `hardware/`
reaches down to `platform/` and up only as far as the `hal/` interface it
implements, and `motion/` imports nothing from the rest of `src/firm`
except `messages/` and `firm/types/`.
`src/tests/sim/unit/test_layer_isolation.py` enforces the first three
mechanically.

This layout replaced a flat `app/` + `devices/` split, and folded the
sibling `src/motion` and `src/sim` trees back in, in August 2026 —
see [`clasi/issues/proposal-platform-hardware-hal-core-reorganization.md`](../../clasi/issues/proposal-platform-hardware-hal-core-reorganization.md)
for the motivation and each subsystem's own `DESIGN.md` for what was and
was not carried out.

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
| ~~`src/firm/motion/`~~ | **Moved into the `src/firm` table above** (August 2026 reorganization) — it is a validated subsystem now, not an unvalidated sibling. Historical description follows: the motion-control library (sprint 122's two-layer base/motion split): `StateEstimator`, `Odometry`, `BodyKinematics`, `WheelVelocityPid`, and `planner/` — `Motion::Planner`, its own standalone CMake project, the larger and now-live half of this tree, writing `Types::RobotState::Wheel::cmdVelocity` directly (128 — the plain blackboard field is the boundary; `MoveQueue`/`WheelSink`/`StopCondition`/`VelocityShaper` were deleted as dead code, zero callers). A SIBLING tree of `src/firm` (not a child), imports nothing from `src/firm` except `messages/`/`firm/types`, and builds its own standalone `motion_tests` CMake target (no sim library, no Python). Real, current documentation — deliberately kept OUTSIDE the validated `sources:` list (Design Rationale Decision 3, sprint 122), same unvalidated-but-real treatment this table's other rows already get. |
| [`src/firm/platform/host/`](../../src/firm/platform/host/DESIGN.md) | (Was `src/sim/`.) The host-build firmware simulator: compiles the real firmware into a shared library, drives it from Python over an `extern "C"` ABI. One sim object shared by the pytest suite and the TestGUI. |
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
`src/archive`, `src/libraries`, `src/protos`,
`src/scripts`, `src/firm/platform/host`, `src/tests`, and `src/utils` never enter the
enumeration at all, not even as a required stub. Most of those trees
still have real, current `DESIGN.md` files (§2's "Other source trees"
table) — they are simply unvalidated-but-real documentation, kept honest
by hand rather than by the mechanical gate. `src/archive` and
`src/libraries` got a short, honest "no
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
COBS+CRC-framed protobuf-style messages with a host over USB serial and
the micro:bit radio (123). It is the "plant" end of the host/robot split: the
host plans motion (currently just profiled twists/wheel-velocity
MOVEs — see
[`src/host/robot_radio/DESIGN.md`](../../src/host/robot_radio/DESIGN.md));
the firmware follows bounded MOVE commands — each self-bounding via a
stop condition and a required timeout, queued 1-active + 4-pending —
and streams telemetry; there is no deadman. Everything under this
directory compiles into one image
(`main.cpp` is the ARM entry point); the same modules minus the ARM
adapters also compile under `-DHOST_BUILD` for host-side tests and
simulation (`src/firm/platform/host/`).

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
TWIST-follower plus a deadman. `msg::PlannerConfig` and its own curated
live-tuning wire message are gone with them (`planner.proto` deleted). This
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
[`src/firm/core/DESIGN.md`](../../src/firm/core/DESIGN.md) and
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
stakeholder decision) and were live-tunable via a `ConfigDelta.estimator`
oneof arm, mirroring the Otos live-tuning message's own existing
merge-then-apply pattern — NOT persisted to flash (unlike motor
gains/OTOS calibration; a reboot reverts to the baked default). The
estimator's predictions are NOT exposed on the wire this sprint —
validation (leave-one-out one-step-ahead RMS analysis) runs host-side
directly against the raw `EncoderReading`/`OtosReading` fields sprint 115
already telemetered, via a captured TLM-log CSV, not a live query
against the on-chip estimator instance. See
[`src/firm/core/DESIGN.md`](../../src/firm/core/DESIGN.md) for the full
detail. **HISTORICAL as of 132-013 (patch-surface retirement) / 128-016
(`App::StateEstimator` itself deleted as dead code):** `ConfigDelta` and
the curated per-target live-tuning messages it carried (drivetrain/motor/
otos/estimator) are deleted outright, replaced by `robot_config.proto`'s
group/field wire arms (`SetConfigGroup`/`GetConfig`/`ConfigSnapshot`/
`SetConfigField`); the ESTIMATOR group still decodes for read-back but
reaches no live consumer (`Configurator::install(ESTIMATOR)` permanently
returns `ERR_UNIMPLEMENTED`).

**122 (motion-library extraction + loop-timing telemetry) — landed.**
Stakeholder-directed two-layer restructuring (2026-07-24): the firmware
splits into a hardened FIRMWARE BASE (`src/firm`, meant to eventually
freeze and move to its own repository) and a separate MOTION LIBRARY
(`src/firm/motion`, a new sibling tree, not a child of `src/firm`) holding the
motion-control logic still under active development. `Motion::MoveQueue`/
`Motion::StateEstimator`/`Motion::Odometry` move out of `src/firm/app/`;
`BodyKinematics` and `Motion::StopCondition`/`Motion::VelocityShaper` move
out of `src/firm/kinematics/` and `src/firm/motion/` respectively — both
of those `src/firm` directories are now retired (redirect `DESIGN.md`
only, see §2's table). One new boundary header, motion-owned,
`Motion::WheelSink` (`src/firm/motion/wheel_sink.h`) — a plain VELOCITY sink
(`setWheels(v_left, v_right)`/`stop()`), NOT a duty sink (that rewrite,
folding sprint 2's PID-placement decision in, is deliberately deferred) —
`App::Drive` narrows to implement it, losing `setTwist()`/its
`BodyKinematics` dependency to `Motion::MoveQueue`, which now calls
`BodyKinematics::inverse()` directly. `velocity_pid.*` and
`Devices::NezhaMotor`'s PID ownership stay in the base, unchanged.
Independently, `Telemetry::SecondaryFrame` gains `cycle_busy`/
`cycle_period` (`uint32 [us]`, additive fields) reporting real per-cycle
loop timing — landed on the secondary, not primary, frame as an interim
placement (the primary frame's armored envelope was then 1 byte under its
186-byte budget). **Sprint 123 (COBS+CRC binary framing + telemetry
migration) has since landed this migration** — see this section's own
"123" paragraph below. Zero behavior change, zero wire change beyond that one
additive field pair — see sprint 122's own `sprint.md` for the full
architecture, diagrams, and Design Rationale (why a velocity sink, why
`src/firm/motion` is a sibling rather than a nested child, why `motion_tests`
is a standalone CMake target). See
[`src/firm/motion/DESIGN.md`](../../src/firm/motion/DESIGN.md) for the
motion library's own current orientation and
[`src/firm/core/DESIGN.md`](../../src/firm/core/DESIGN.md) for the base's.

**123 (firmware base hardening: COBS+CRC binary framing + telemetry
migration) — landed.** The wire's `*B<base64>\r\n` line armor is
replaced end-to-end (both transports, `App::Comms`, every host decoder)
with COBS framing (`0x00`-delimited, ~0.4% overhead, self-resynchronizing
on byte loss) plus a CRC-16/CCITT-FALSE integrity check — the wire's
first integrity check of any kind. This is a flag-day cutover with no
dual-stack: base64 itself is retained in `wire_runtime.{h,cpp}` only
because an unrelated debug harness (`wire_differential_harness.cpp`)
still depends on the primitive, but no `Comms`/`Telemetry` call site
encodes/decodes it any more. The freed headroom (envelope budget
recomputed 186→240 bytes) is what let this sprint's own ticket 004
complete 122's own forward-referenced migration: `cycle_busy`/
`cycle_period` move off `TelemetrySecondary` (122-003's interim
placement, fields 11/12, now `reserved`) onto the primary `Telemetry`
frame (fields 15/16) every cycle. See sprint 123's own `sprint.md` for
the full architecture and Design Rationale (CRC width choice, the
COBS-vs-length-prefix-vs-SLIP alternatives considered), and
[`src/firm/messages/DESIGN.md`](../../src/firm/messages/DESIGN.md) §3/§4
and [`src/firm/core/DESIGN.md`](../../src/firm/core/DESIGN.md) §1/§4 for
the subsystem-level detail.

**124 (protocol v5, `RobotState` blackboard, radio bench gate) —
landed.** Supersedes 123's own COBS+CRC framing with one uniform
`<COMMAND>[':' <data>]'\n'` grammar in both directions, both text and
binary — the COBS delimiter moves from `0x00` to `0x0A` and the CRC's
scope extends to cover `COMMAND ':'`, closing "a bit-flip inside the
command name lands on another valid verb and still passes CRC." A
generated command registry (`src/protos/commands.proto`) is the single
source firmware dispatch, the host codec, and
[`docs/protocol-v5.md`](../protocol-v5.md) are generated from or checked
against — closing a firmware/host/doc three-way drift risk. The reply
plane gains the same grammar: `PONG:t=<ms>` (was `OK pong t=<ms>`),
`ID:<drivetrain>:<profile>:<version>` (configured identity, distinct from
`DEVICE:`'s hardware identity), `VER:<version>` (reads the existing
generated build-version constant). Telemetry's position/velocity/pose/
twist/OTOS fields switch from `float` to `sint32`+zigzag with a
generated `(scale)` conversion; the ack ring's elements pack to a single
`uint32` each; the older single scalar "freshest ack" slot is deleted —
the bounded `acks` ring is the sole ack-observability path now.
`Types::RobotState` (`src/firm/types/robot_state.h`) becomes a SECOND
dependency-free shared floor `src/firm` and `src/firm/motion` both stand on —
its own `Wheel::cmdVelocity` field later (128) becomes THE base/motion
actuation boundary in its own right, superseding `Motion::WheelSink` — the
one struct every subsystem publishes
its per-cycle section to and the one source `Telemetry::update(state)`
projects from; `Motion::StateEstimator::Input` is now a type alias onto
it, not a hand-copied near-duplicate. `msg::TelemetrySecondary` — frame
type, wire arm, and tie-break/alternation cadence machinery — is deleted
outright. See sprint 124's own `sprint.md` for the full architecture and
Design Rationale (the command-registry/scale-generation/ID-VER-content
decisions, and the position-rebaseline policy — `RobotLoop` calls the
existing, unmodified `Devices::Motor::rebaseline()` in software only,
never a device command, when a wheel's position nears the wire's ±32m
bound, owning a new `positionEpoch` counter the host can watch for a
rebase), and
[`src/firm/messages/DESIGN.md`](../../src/firm/messages/DESIGN.md) §3/§4
and [`src/firm/core/DESIGN.md`](../../src/firm/core/DESIGN.md) §1/§4/§5 for
the subsystem-level detail.

**125–127 (velocity-PID relocation + `Motion::Planner` integration —
landed, superseding the `MoveQueue`/`WheelSink` story two paragraphs
above).** `Motion::WheelVelocityPid` (125-003) relocates the closed-loop
velocity control law from `Devices::NezhaMotor` into `src/firm/motion` — same
control law, motion-local `Gains` type only. Separately, `Motion::Planner`
(`src/firm/motion/planner/`, its own standalone CMake project — profile
generation, jerk-limited shaping, a duty-stage PID/trim, wheel-command
estimation) becomes the on-robot motion decider in place of `Motion::
MoveQueue`: `Motion::Planner::update()` writes `Types::RobotState::
Wheel::cmdVelocity` directly, and `App::RobotLoop::cycle()` reads it back
directly to drive `App::Drive::tick()`. `Motion::WheelSink` — the
boundary interface `MoveQueue` drove `App::Drive` through — is left with
zero callers at this point (Planner never adopts it; Drive's own WHEELS
teleop path already wrote `cmdVelocity` directly too, via `App::Drive::
update()`), though it is not yet deleted.

**128 (complexity reduction: delete dead `WheelSink`/`MoveQueue`
generation) — landed, SUC-002 Decision 1.** With zero callers confirmed
(`Motion::Planner::update()`/`App::Drive::update()` already write
`cmdVelocity` directly; `RobotLoop::cycle()` already reads it directly),
`Motion::WheelSink`, `Motion::MoveQueue`, `Motion::StopCondition`, and
`Motion::VelocityShaper` (~1,500 lines) are deleted outright — not
retired, not redirect-only, deleted — along with their `motion_tests`
ctest targets and `test_app_move_queue.py`. `Types::RobotState::Wheel::
cmdVelocity` (`src/firm/types/robot_state.h`) is promoted to be THE
documented actuation boundary between `src/firm` and `src/firm/motion` in
`WheelSink`'s place: sole-or-arbitrated writer (`Motion::Planner::
update()` when a Move owns motion, `App::Drive::update()` for WHEELS
teleop, arbitrated by `RobotLoop`'s own ordering — exactly one of the two
`update()` calls writes it per cycle), consumer `RobotLoop::cycle()`. No
new interface was introduced to replace `WheelSink` — the plain
blackboard field IS the boundary, the same way `Types::RobotState`
already is for every other base/motion crossing (117/124). The
land-at-zero completion predicate's own empirical margin-factor
derivation (118/119/121, ~250 lines of sweep history) is preserved,
verbatim, as dated design history rather than lost with the code:
[`docs/design/history/land-at-zero-margin-derivation.md`](history/land-at-zero-margin-derivation.md).
See [`src/firm/motion/DESIGN.md`](../../src/firm/motion/DESIGN.md) and
[`src/firm/core/DESIGN.md`](../../src/firm/core/DESIGN.md) for the
subsystem-level detail.

Flow of one cycle, at orientation altitude:

1. **Comms in** — `App::Comms` polls the two transports (serial, radio)
   for one complete `\n`-terminated line, parses its `<COMMAND>` prefix
   and dispatches by a generated command registry's binary/cleartext
   flag (protocol v5, sprint 124 — supersedes the pre-124 `0x00`-
   delimited-frame-vs-HELLO/PING-text-rump demux this step used to
   describe), then decodes a binary command line into a
   `msg::CommandEnvelope`.
2. **Dispatch** — the loop's own switch acts on the command: a Move is
   handed to `Motion::Planner` (`src/firm/motion/planner/`, 125–128 — replaces
   the deleted `Motion::MoveQueue`), which owns the active/pending queue,
   profiles and shapes the commanded motion, and each cycle writes
   `Types::RobotState::Wheel::cmdVelocity` directly — no boundary
   interface, the blackboard field IS the boundary (128, superseding the
   `Motion::WheelSink` story below); a Stop flushes the queue and halts
   `Drive` immediately; config/queries reply via the primary telemetry
   frame's bounded ack ring (sprint 124 ticket 008 deleted the older
   single scalar ack slot — ring membership alone means "acked" — see
   [`src/firm/core/DESIGN.md`](../../src/firm/core/DESIGN.md) §2/§4).
3. **Motor service** — the loop runs each `Devices::NezhaMotor`'s
   split-phase encoder request → settle → collect → PID → duty-write
   sequence, with the settle/clearance gaps expressed as
   `runAndWait(gap, body)` blocks whose wait time is borrowed for other
   bounded work (OTOS sampling, odometry integration, telemetry
   assembly).
4. **State out** — `Motion::Odometry` (122 — moved to `src/firm/motion`)
   integrates encoder deltas through `BodyKinematics::forward()`;
   `Motion::StateEstimator` (117; 122 — moved to `src/firm/motion`) ingests the
   same cycle's published `Types::RobotState` (124 — `Input` is now a type
   alias onto `RobotState`, not a hand-copied near-duplicate struct) and
   refreshes its wheel/body ZOH predict-to-now estimates; `App::Telemetry`
   projects `RobotState` and emits the ONE primary TLM frame — carrying
   the `cycle_busy`/`cycle_period` loop-timing fields every cycle (123)
   and packed fixed-point sensor/pose fields (124) — through Comms. There
   is no second/secondary frame any more: `msg::TelemetrySecondary` is
   deleted outright (124), not merely unused.
5. **Pace** — a final `runAndWait` paces the cycle to `kCycle` = 50 ms
   (~20 Hz; sprint 130 ticket 007 raised this from 40 ms — the 40ms value
   was an overrun artifact the loop could not fit inside, measured busy
   ~21 ms plus vendor-bus-clearance overrun drifting the delivered period
   to ~46–48 ms; at 50 ms the pacer paces again, exactly and stably).
   `Telemetry::kPrimaryPeriod` stays 40 ms unchanged — it gates
   `elapsed >= kPrimaryPeriod`, a floor the 50 ms cycle still clears every
   single iteration, so a primary frame still emits every cycle (effective
   rate ~20 Hz). (118 — restores the schedule's genuine 4ms/4ms
   settle/clearance budget, regressed to a fictional 20ms/~50Hz by commit
   `5f5a2ba7`; the sim's own `SimHarness::kCycleDtUs` still matches
   `kCycle` exactly, closing the sim/firmware cadence gap — see
   [`src/firm/platform/host/DESIGN.md`](../../src/firm/platform/host/DESIGN.md).)

Boot is a separate loop: `App::Preamble` steps per-device detection (one
bounded probe per pass) while telemetry frames report detection status;
command consumption starts only when `preamble.done()`.

Dependency direction (arrows = "includes/uses"). **122 changed this
diagram's shape**: `motion/`/`kinematics/` stop being children of
`src/firm` and become `src/firm/motion`, a SIBLING tree `app` depends on —
**128** deletes the boundary INTERFACE that crossing briefly went
through (`Motion::WheelSink`, zero callers) in favor of the plain
`Types::RobotState::Wheel::cmdVelocity` field already crossing it in
practice:

```
main.cpp ──► app ──► devices ──► (nothing project-local except itself)
   │          │  └─► messages
   │          ├────► src/firm/motion (sibling tree, via RobotState::Wheel::cmdVelocity) ──► messages
   │          └────► com (via ARM-only Transport adapters)
   ├────────► config ──► messages
   └────────► com, devices, config, src/firm/motion
```

`devices/` is the bottom of the `src/firm` stack and deliberately
includes nothing from `messages/` or `config/`. `messages/` is a leaf
library with no project dependencies of its own. `src/firm/motion` (122) is
similarly leaf-like from `src/firm`'s point of view — it imports nothing
from `src/firm` except `messages/` — but is a whole sibling tree, not a
single-directory leaf; see
[`src/firm/motion/DESIGN.md`](../../src/firm/motion/DESIGN.md) for its own
internal module graph. `src/firm/kinematics/` and `src/firm/motion/` are
both now retired, empty-of-code directories (each keeps a redirect
`DESIGN.md` only — see §2's table above).

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
  `Motion::Planner::tick()` (125–128 — replaces the deleted `Motion::
  MoveQueue`) runs unconditionally every cycle and drains once the
  active `Move`'s stop condition or `timeout` fires and nothing is
  pending — an emergent property of every queued command carrying its
  own bound, not a second, independently-timed staleness timer.
  `App::Deadman` does not exist in this tree. No ad hoc watchdog belongs
  anywhere in the firmware.
- **`newlib-nano` has no `%f`:** `printf`-family float formatting emits
  nothing on ARM (works fine in host builds). Floats cross the wire as
  scaled integers or via the binary codec.
- **Bench gate:** firmware changes to the HAL, motor control, sensing,
  or protocol are not done until exercised on the robot on its stand
  (`.claude/rules/hardware-bench-testing.md`). Host tests alone do not
  close a change.

**Wire boundary — protocol v5 (124, superseding the framing this
paragraph describes through 123).** Every packet, both directions, text
or binary, is one `\n`-terminated line: `<COMMAND>[':' <data>]'\n'`. A
binary command's `<data>` is CRC-then-COBS framed — CRC-16/CCITT-FALSE
now scoped over `COMMAND ':' payload` (not payload alone), then
COBS-encoded keyed on `0x0A` (not `0x00`, so `\n` is a genuine,
unconditional terminator with no text/binary demux heuristic at the
transport layer) — over USB serial (115200 CDC) and the micro:bit radio
(group 10, channel 0–35 persisted in flash). Which of a generated,
closed set of verbs a line names, and whether its data is cleartext or
binary, comes from one generated registry (`src/protos/commands.proto`),
the single source firmware dispatch, the host codec, and
[`docs/protocol-v5.md`](../protocol-v5.md) are all generated from or
checked against. Payloads are `msg::CommandEnvelope` in
(`move`/`config`/`stop` — three binary command verbs), `msg::
ReplyEnvelope` (`tlm` oneof — the only arm with a live producer) out, plus
four cleartext verbs answered inline: `HELLO`→`DEVICE:...` (byte-frozen
boot banner), `PING`→`PONG:t=<ms>`, `ID`→`ID:<drivetrain>:<profile>:
<version>` (configured identity), `VER`→`VER:<version>` (build identity).
`msg::TelemetrySecondary` — a second, independently-framed diagnostic
frame that existed through 123 — is DELETED outright (124), not merely
unused; there is only ever one outbound telemetry frame now, carrying
packed `sint32`/zigzag + `(scale)` fixed-point sensor/pose fields and a
bounded ack ring (`acks`, the sole ack-observability path — the older
single scalar "freshest ack" slot is deleted). Schema source of truth:
`src/protos/*.proto`. Boot banner: `DEVICE:NEZHA2:robot:<name>:<serial>`
— byte-frozen, unchanged by the v5 cutover. See
[`docs/protocol-v5.md`](../protocol-v5.md) for the full wire reference
(supersedes `docs/protocol-v4.md`),
[`src/firm/messages/DESIGN.md`](../../src/firm/messages/DESIGN.md) and
[`src/firm/core/DESIGN.md`](../../src/firm/core/DESIGN.md) for the
dispatch/codec detail, and
[`src/protos/DESIGN.md`](../../src/protos/DESIGN.md) for the schema
source of truth itself.

**Open, firmware-tree-wide items** (each subsystem doc's own §6 carries
its local ones): line/color steady-state sampling has since landed
(115-005 — see [`src/firm/core/DESIGN.md`](../../src/firm/core/DESIGN.md)
§2's `updateLineColor()`); `src/firm/messages/event.h` remains orphaned
dead code (see that doc's own §6); `src/firm/types/` remains a
vestigial grab-bag (see that doc's own §6); sprint 116's MOVE protocol
has landed — `Twist` (arm 19) and `ConfigDelta.watchdog` (field 4) are
`reserved`, not reused; `App::Deadman` is deleted; see
[`src/firm/motion/DESIGN.md`](../../src/firm/motion/DESIGN.md) for the
new `Motion::StopCondition` module. Sprint 117's `App::StateEstimator`
has landed — see [`src/firm/core/DESIGN.md`](../../src/firm/core/DESIGN.md)
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
  `Motion::StateEstimator` (122 — moved to `src/firm/motion`, `App::` through
  sprint 121) ticks every cycle with wheel/body peer ZOH
  estimates; its OTOS-fusion weights are fail-closed baked config,
  defaulting to 0.0 (encoder-only v1) and live-tunable via the new
  `ConfigDelta.estimator` arm — NOT persisted to flash. Its predictions
  are not exposed on the wire; validation runs host-side against the raw
  telemetered readings (a captured TLM-log CSV), per the stakeholder's
  leave-one-out one-step-ahead RMS methodology. Fake OTOS, external/
  camera pose fusion, and the remaining-distance trajectory controller —
  the source issue's further-out goals — remain future work, not part of
  117.
- **Sprint 122 (motion-library extraction + loop-timing telemetry) has
  landed.** `src/firm` splits into a hardened base and a separate
  `src/firm/motion` library (sibling tree) per the stakeholder's two-sprint
  restructuring directive (2026-07-24) — see this section's own §5 "122"
  paragraph above for the full change, and
  [`src/firm/motion/DESIGN.md`](../../src/firm/motion/DESIGN.md) for the library's
  current orientation. `src/firm/motion/`/`src/firm/kinematics/` are
  retired (redirect `DESIGN.md` only, kept so the design-doc validator
  still finds a doc for each still-declared child of `src/firm` — §2's
  table). Sprint 2 (base hardening: the duty-sink boundary, bounded wheel
  moves, a per-wheel command observer) is explicitly NOT part of 122 —
  tracked by
  `clasi/issues/firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md`.
- **Sprint 123 (firmware base hardening: COBS+CRC binary framing +
  telemetry migration) has landed.** The wire's `*B<base64>\r\n` line
  armor is replaced end-to-end with COBS framing + a CRC-16/CCITT-FALSE
  integrity check (flag-day cutover, no dual-stack) — see this section's
  own §5 "123" paragraph above and "Wire boundary" note for the full
  change, [`src/firm/messages/DESIGN.md`](../../src/firm/messages/DESIGN.md)
  §3/§4 for the codec/budget detail, and
  [`src/firm/core/DESIGN.md`](../../src/firm/core/DESIGN.md) §1/§4 for the
  `Comms`/`Telemetry` detail. `cycle_busy`/`cycle_period` complete the
  migration 122-003 forward-referenced, moving from `TelemetrySecondary`
  (now `reserved` fields 11/12) onto the primary `Telemetry` frame
  (fields 15/16) every cycle. The duty-sink boundary/bounded-wheel-moves/
  per-wheel-observer work the paragraph above tracks remains a distinct,
  not-yet-scheduled future sprint — 123 did not touch that surface.
- **Sprint 124 (protocol v5, `RobotState` blackboard, radio bench gate)
  has landed.** See this section's own §5 "124" paragraph above and "Wire
  boundary" note for the full change,
  [`src/firm/messages/DESIGN.md`](../../src/firm/messages/DESIGN.md) §3/§4
  for the codec/registry/size-budget detail (now `kReplyEnvelopeMaxEncodedSize
  <= 130` bytes), and [`src/firm/core/DESIGN.md`](../../src/firm/core/DESIGN.md)
  §1/§4/§5 for the `Comms`/`Telemetry`/`RobotLoop` detail. The Drive/
  Sensors device-ownership reshuffle the blackboard issue's own cycle-body
  sketch illustrates is explicitly deferred to sprint 125 (Design
  Rationale Decision 1, "the scope valve") — `RobotLoop` still owns
  `motorL_`/`motorR_`/`otos_`/`line_`/`color_` directly; only state
  ASSEMBLY (not device ownership) moved to the one-`RobotState`-per-cycle
  shape this sprint required. The radio-relay standing bench gate
  (`src/tests/bench/`) is this sprint's own acceptance mechanism, run over
  the relay per stakeholder directive, not merely over USB.
- **The design-doc-set's mechanical validator cannot express "this
  child is out of scope because it symlinks outside the repository."**
  `src/vendor` remains permanently undocumented for that reason (§4).
  Revisit if `clasi` ever grows an `excluded_paths`-equivalent that the
  design validator itself consults.
- **`src/host/robot_radio`'s live/dormant split is not clean at the
  file level** — see that doc's own §2/§3 for the specific traps
  (several nominally-live directories contain dormant functions).
