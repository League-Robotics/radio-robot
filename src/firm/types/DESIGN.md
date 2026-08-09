---
root: ../../../docs/design/design.md
---

# types/ — RobotState Blackboard, Protocol Constants, and Version Plumbing

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-26 · **Status:** mixed — `robot_state.h` is live, load-bearing, AND PUBLISHED every cycle (sprint 124 tickets 007-009); the rest is vestigial (see §6)

---

## 1. Purpose

`types/` holds `robot_state.h` (sprint 124 ticket 007, live) and two older,
unrelated things that predate the single-loop rebuild and never got a real
home: text-protocol tag constants left over from the pre-binary-cutover
command set, and the firmware-version generation seam (`FIRMWARE_VERSION` /
`PROTO_VERSION`, fed by `version_generated.h`). The directory is not a
subsystem in the sense the rest of `src/firm` uses the word — §6 preserves
the earlier audit's findings on the vestigial half; this revision documents
the new, load-bearing half.

`robot_state.h` (`Types::RobotState`) is the ONE dependency-free struct that
is the sole cross-subsystem AND cross-tree data contract (sprint 124
architecture Step 3; `clasi/issues/robot-state-blackboard-one-struct-for-
all-shared-state-and-telemetry.md`). It gives `types/` a second, deliberate
purpose alongside version plumbing: a shared floor `src/firm` and
`src/firm/motion` both stand on — its own `Wheel::cmdVelocity` field is THE
actuation boundary in its own right (sprint 128, superseding the deleted
`Motion::WheelSink` interface). Three roles, one struct:
the blackboard every subsystem publishes its own per-cycle section to and
reads other subsystems' sections from; the source `App::Telemetry` projects
into the lossy, scaled wire frame (`msg::Telemetry` — a DIFFERENT, later
ticket's job, not this file's); and a test fixture (construct one, fill the
fields under test, call `tick(state, now)`, assert on what it wrote —
trivially copyable, so tests copy it for golden comparisons).

## 2. Orientation

`robot_state.h` declares `namespace Types`: the `Mode` enum (mirrors
`msg::DriveMode`'s value set without depending on it) and `struct
RobotState`, sectioned by writer — `Time`, `Wheel` (×2, `wheelLeft`/
`wheelRight`), `Otos`, `Perception`, `Pose`, `Estimate` (nested
`WheelEstimate`×2/`BodyEstimate`/`Innovations`, ZOH bases — value +
velocity + `basisTime` + `valid`, so "predict to time t" is a pure function
over the struct alone), `Command`, `Health`. See the header's own file
comment and each section's own doc comment for the field-by-field
writer/rationale — not duplicated here, to avoid the two drifting.
`Motion::StateEstimator` (`src/firm/motion/state_estimator.h`) is the first real
cross-tree consumer: its own `Input` type is now a type alias onto
`Types::RobotState` rather than a private near-duplicate struct (124-007;
see `src/firm/motion/DESIGN.md` §3 for the crossing's own rationale).

`protocol.h` is a single header, included by nothing in the live tree (see
§6). It declares, in order: six `PROTO_TAG_*` reply-tag string constants,
`PROTO_VERSION`, the `FIRMWARE_VERSION` string (sourced from the generated,
git-ignored `version_generated.h`, falling back to `"0.0.0-dev"` when that
file is absent), the `ReplyFn`/`ReplyCtx` reply-sink types, and the `KVPair`
key=value token struct. `version_generated.h` is emitted by
`scripts/gen_version.py` (run from `build.py`'s codegen step, alongside
`gen_messages.py`) and is never hand-edited — see the system doc's
"generated files are never hand-edited" convention
(`docs/design/design.md` §3).

## 3. Constraints and Invariants

- **`robot_state.h` is dependency-free: cstdint-level includes ONLY.** No
  `msg::`/`messages/*` type, no protobuf-generated anything, nothing from
  `src/firm/app`, `src/firm/config`, or `src/firm/devices`. Grep-enforceable:
  `grep -n "messages/\|msg::" src/firm/types/robot_state.h` returns nothing —
  enforced by `src/tests/sim/unit/test_firm_types_robot_state.py`, both by
  that literal grep and by compiling the header's own harness
  (`firm_types_robot_state_harness.cpp`) against a narrow `-I <repo>/src`
  path with no other `src/firm` subtree on it. This is what lets
  `src/firm/motion` (a sibling tree that must build independently of `src/firm`,
  per §3's own rule in `src/firm/motion/DESIGN.md`) include this one header.
- **`Types::RobotState` is trivially copyable.** Plain scalar/bool/nested-
  plain-struct fields only — no pointers, no heap, no virtuals, no
  user-defined constructors/destructors. Asserted directly:
  `static_assert(std::is_trivially_copyable_v<Types::RobotState>)` in the
  same test harness. This is what makes "copy the state for a golden
  comparison in a subsystem unit test" a real, cheap operation rather than
  an aspiration.
- **What's IN `RobotState`: per-cycle dynamics only, grouped by writer, not
  by consumer.** Time, per-wheel sensed+commanded, OTOS, line/color
  perception, dead-reckoned pose, the estimator's ZOH bases, the current
  command, and health/fault counters — see `robot_state.h`'s own section
  comments for exactly which subsystem writes each one.
- **What's OUT of `RobotState` (deliberately, per the issue's own "What
  stays OUT" section): config, PID gains, calibration, fusion weights,
  persisted tuning, and ACKS.** Acks are protocol bookkeeping (`App::
  Telemetry`'s own ack ring), not robot state, even though they change
  every cycle a command lands — the exclusion is about WHAT KIND of thing
  an ack is (a protocol receipt), not how often it changes. Config keeps
  its own patch/persistence path (`Config::TuningSnapshot` and friends).
- **Superseded by tickets 008/009 — `RobotState` is now published every
  cycle, not merely defined.** Ticket 007 defined the struct only (`App::
  RobotLoop` still built a cycle-local `Motion::StateEstimator::Input`
  variable field-by-field back then); tickets 008 (packed telemetry +
  position-rebaseline trigger) and 009 (the `RobotLoop`/`Telemetry`
  restructure) wired `RobotLoop` to own a persistent `state_` member,
  publish each section at its coherence point exactly once per cycle, and
  call `Telemetry::update(state_)` as the ONE projection point — see
  [`../app/DESIGN.md`](../app/DESIGN.md) §2/§4 for the exact call sites.
  `Motion::StateEstimator::update(state_, now)` and `Telemetry::
  update(state_)` are both now real, live consumers of a genuinely
  RobotLoop-owned instance, not a throwaway local.
- **`version_generated.h` is generated, git-ignored, and never hand-edited.**
  `scripts/gen_version.py` overwrites it at every build from `pyproject.toml`.
  Edits here are silently destroyed by the next build.
- **The `__has_include` fallback must stay.** `protocol.h` guards its include
  of `version_generated.h` with `#if __has_include(...)` and defines
  `FIRMWARE_VERSION_STR "0.0.0-dev"` if it's missing. This keeps clangd (which
  doesn't run codegen) and any ad-hoc codegen-less compile building. Deleting
  the fallback because "the generator always runs" breaks IDE tooling.
- **Wire tag strings are frozen.** `PROTO_TAG_*` and the `DEVICE:NEZHA2:...`
  banner format are wire surface, exempt from the naming-convention rename
  sweep (`.claude/rules/coding-standards.md`) — see also §6 on whether these
  particular constants still matter.

## 4. Design

**`RobotState`'s seed material.** `Motion::StateEstimator::Input` was
already ~90% of what `RobotState` needs — same encoder/pose/twist/OTOS
fields, already float-typed, already dependency-shaped the same way
(motion-owned, no `App::`/`msg::` spelling anywhere) — so it is the
struct's natural starting point, per this ticket's own instructions.
`RobotState` supersedes it field-for-field (every one of `Input`'s former
16 flat fields maps onto a section field with no information loss — see
`robot_state.h`'s own file header for the mapping) while reorganizing them
by WRITER instead of leaving them flat, and adding the sections `Input`
never carried at all: `Time`, per-wheel `cmdVelocity`/`connected`/
`positionEpoch`, `Perception`, `Command`, `Health`. `Motion::
StateEstimator::Input` is now a type alias onto `Types::RobotState`
(`using Input = Types::RobotState;`) rather than a second, hand-maintained
near-duplicate — the alias is source-compatibility only, kept so
`state_estimator.h`'s own doc comments and call sites can keep saying
`Input` without forcing every caller to spell `Types::RobotState` inline;
there is no behavioral difference between the two spellings, and no third
struct exists anywhere.

**Naming: sections named for what they hold, not who writes them.** Each
top-level section (`Wheel`, `Otos`, `Pose`, `Estimate`, `Command`,
`Health`, ...) is named for the KIND of data it holds; the writer is
documented in the section's own comment, not encoded into the name. This
matters because Decision 1 (sprint 124's own architecture, the "scope
valve") explicitly defers the Drive/Sensors device-ownership reshuffle to
sprint 125 — `RobotLoop` still reads `Devices::Motor`/`Devices::Otos`
directly today, but `App::Drive`/a future `Sensors` subsystem take over
that reading next sprint. If a section were named `RobotLoopWheelData` or
similar, that rename would be forced the moment ownership moves; naming it
`Wheel` means the section survives the ownership reshuffle unchanged.

**`command.v_x`/`command.omega`, not `targetVx`/`targetOmega`.** The
issue's own struct sketch names these `targetVx`/`targetOmega`; this
ticket instead reuses the same `v_x`/`omega` names `Pose` and
`BodyEstimate` already use, disambiguated by the enclosing `command.`
section prefix rather than by a `target`-prefixed compound name — avoids
mashing a word prefix onto a mathematical subscript
(`naming-and-style.md` rule 2's "a twist is never a bare `v`" already
governs the subscript itself) and stays consistent with how every other
section in this struct disambiguates by section, not by per-field prefix
(`wheelLeft.cmdVelocity` vs. `wheelLeft.velocity`, not
`wheelLeftCmdVelocity`).

**`Health` drops the issue sketch's `deadmanExpired`, adds
`moveTimeout`/`shapingDisabled`.** `App::Deadman` was fully retired in an
earlier sprint — there is no live "deadman expired" signal left to carry
(its former telemetry flag, `kFlagEventDeadmanExpired`, is declared but
unwired — `src/firm/app/telemetry.h`'s own comment). This ticket's own
instructions are to derive the field list from what genuinely exists
today, not to reproduce the issue's illustrative sketch verbatim, so
`Health` carries the two fault signals that ARE genuinely live instead
(`App::RobotLoop`'s own `kFlagFaultMoveTimeout`/`kFlagFaultShapingDisabled`
derivation, sourced from `Motion::Planner::tick()`'s own `Motion::
TickResult` outcome and `Motion::Planner::shaperConfigured()` — 128,
superseding the deleted `Motion::MoveQueue`).

**Version generation pipeline.** The firmware needs to report a build
version over the wire without a hand-edited constant silently drifting (this
happened: `FIRMWARE_VERSION` sat at `0.20260704.6` while `pyproject.toml`
advanced past it). `gen_version.py` reads the canonical version out of the
root `pyproject.toml` and writes it as `#define FIRMWARE_VERSION_STR "..."`
into `version_generated.h`, which `protocol.h` includes. The file is only
rewritten when its content changes, to avoid needless rebuilds. Because
codegen doesn't run under clangd or a bare compile, `protocol.h`'s
`__has_include` guard falls back to a literal `"0.0.0-dev"` so those builds
still succeed — the fallback string is a marker, not a real version, and
should never appear in a wire reply from a real build.

**Everything in `protocol.h` is inert today** (see §6) — there is no
control flow to describe there. `robot_state.h` is the opposite: pure
data, by design (§3's trivially-copyable/dependency-free constraints), so
it likewise has no control flow of its own to describe here — its
"design" is entirely the section list and naming rationale above.

## 5. Interfaces

### Exposes
- **`Types::RobotState` (`robot_state.h`):** the blackboard struct itself
  — see §2/§3 above for its section list and constraints. One current
  consumer: `Motion::StateEstimator::update(const Input&, uint32_t now)`
  (`src/firm/motion/state_estimator.h`), where `Input` is a type alias onto
  this struct. `App::RobotLoop` builds a cycle-local instance
  field-by-field today (ticket 008/009 wire a persistent, `RobotLoop`-
  owned instance through the rest of the cycle).
- **`Types::Mode` (`robot_state.h`):** mirrors `msg::DriveMode`'s value
  set without depending on it — the one place the two enums are converted
  between is `App::Telemetry`'s own projection step (ticket 008/009, not
  yet built).
- **`PROTO_TAG_OK/ERR/EVT/TLM/CFG/ID`, `PROTO_VERSION`, `FIRMWARE_VERSION`,
  `ReplyFn`/`ReplyCtx`, `KVPair`:** declared, header-only, no current
  callers in `src/firm` or `src/firm/platform/host` (verified by repo-wide grep — see §6).
  Any future consumer would take these as-is; no contract beyond the C++
  types themselves.

### Consumes
- **`pyproject.toml` (via `scripts/gen_version.py`):** canonical version
  string, at build time — see [`../../scripts/DESIGN.md`](../../scripts/DESIGN.md).
- **`robot_state.h` consumes nothing** — dependency-free by construction
  (§3); it has no build-time or runtime input of its own.

## 6. Open Questions / Known Limitations

- **RESOLVED (was open through 124-007): `RobotState` is now published
  every cycle.** Tickets 008/009 closed this gap — `App::RobotLoop` owns a
  persistent `state_` member, publishes every section (`wheelLeft`/
  `wheelRight` including `cmdVelocity`/`positionEpoch`, `otos`,
  `perception`, `pose`, `estimate`, `command`, `health`) at its own
  coherence point exactly once per cycle, and `Telemetry::update(state_)`
  is the ONE projection call (`RobotLoop::cycle()`'s own grep-enforceable
  contract: zero `tlm_.setFlag()` calls outside `Telemetry::update()`).
  `RobotLoop` still owns the underlying devices directly (Decision 1's
  "scope valve" — the Drive/Sensors ownership reshuffle remains deferred
  to sprint 125), but that is an ownership question, orthogonal to
  publication: every `RobotState` section is genuinely live now.
- **`protocol.h` is currently included by nothing.** A repo-wide grep
  (`grep -rn "PROTO_TAG_\|ReplyCtx\|ReplyFn\|FIRMWARE_VERSION\|PROTO_VERSION\|KVPair" src --include='*.cpp' --include='*.h'`)
  finds real consumers only in `src/archive/source_old/` (the pre-rebuild
  tree, deleted from the live build in sprints 102–107) — `Protocol.h`,
  `CommandTypes.h`, `Superstructure.h`, `CommandProcessor.*`,
  `MotionCommands.cpp`, etc. Nothing under `src/firm/app`, `src/firm/com`,
  `src/firm/devices`, `src/firm/messages`, `src/firm/config`, or `src/firm/platform/host`
  includes `types/protocol.h` at all. `main.cpp`'s banner
  (`DEVICE:NEZHA2:robot:<name>:<serial>`) is hand-formatted from name and
  serial only; it does not use `FIRMWARE_VERSION` or `PROTO_VERSION`.
  `App::Comms::dispatchCleartext()` answers `HELLO`/`PING`/`ID`/`VER`
  with the literal strings `banner_`/`"PONG:t=<ms>"`/`idLine_`/
  `"VER:" FIRMWARE_VERSION_STR` (protocol v5, sprint 124 — supersedes the
  pre-124 `"OK pong"` reply this bullet used to cite), not `PROTO_TAG_OK`.
- **`PROTO_TAG_*` predate the binary cutover.** They belong to the old
  text-tag reply format (`OK`/`ERR`/`EVT`/`TLM`/`CFG`/`ID` as a leading
  token). The current wire protocol is the binary-armored envelope codec
  (`msg::ReplyEnvelope` with an ok/err/tlm discriminant) — see
  `docs/design/design.md` §5, "Command plane." These constants have no
  counterpart need in that scheme.
- **`ReplyFn`/`ReplyCtx`/`KVPair` are artifacts of the deleted
  dispatch-table architecture** (per-command handlers taking a reply
  sink + parsed kv-pairs), not the current single-loop design where
  `App::Comms::sendReply()` takes a typed `msg::ReplyEnvelope` directly and
  there is no generic kv-pair command parser.
- **Recommendation (not actioned here):** this ticket is documentation-only
  and changes no code. A follow-up cleanup ticket should decide whether to
  delete the unused declarations (`PROTO_TAG_*`, `ReplyFn`, `ReplyCtx`,
  `KVPair`) outright, keeping only the version-generation machinery
  (`PROTO_VERSION`, `FIRMWARE_VERSION`, the `__has_include` fallback) which
  is the one piece with a real, if currently unwired, purpose. File as a
  `clasi/issues/` item rather than deciding it inline.
