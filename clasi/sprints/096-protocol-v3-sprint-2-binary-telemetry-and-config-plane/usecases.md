---
status: done
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 096 Use Cases

This sprint lands "Sprint 2" of the protocol-v3 program
(`clasi/issues/protocol-v3-schema-driven-binary-command-plane-protobuf.md`):
the binary telemetry and config planes, built on 095's codec foundation
(`wire_runtime`, `BinaryChannel`, generated field-descriptor tables,
`CommandEnvelope`/`ReplyEnvelope`). Most use cases here are infrastructure
(schema, periodic-emission plumbing, differential testing) with no direct
parent in `docs/usecases.md` — they exist to *serve* UC-005 (Query Encoder
Positions), UC-006 (Query and Zero Dead-Reckoning Odometry), and UC-014
(Tune Calibration Parameters at Runtime) over a second wire format, the
same posture 095's SUC-001..005 took toward UC-001/UC-004/UC-018.
SUC-003/SUC-004/SUC-006 are the user/host-visible use cases that actually
ride the new planes this sprint; SUC-001/002/005/007 are the machinery
underneath them.

## SUC-001: Schema author declares the config and telemetry wire contracts
Parent: (infrastructure — schema foundation for SUC-003/SUC-004)

- **Actor**: Firmware/protocol developer editing `protos/*.proto`.
- **Preconditions**: `envelope.proto`'s `Telemetry`/`ConfigDelta`/
  `ConfigSnapshot` are empty placeholder messages (095); `ConfigGet.target`
  is an untyped `uint32`; `CommandEnvelope.cmd`'s `config`/`get`/`stream`
  arms and `ReplyEnvelope.body`'s `tlm`/`cfg` arms decode successfully but
  carry no real fields.
- **Main Flow**:
  1. Add `protos/telemetry.proto`: a real `Telemetry` message, curated to
     the union of the STREAM/SNAP text frame's fields (`enc`/`vel`/`cmd`/
     `pose`/`encpose`/`otos`+`otosconn`/`twist`/`mode`/`seq`/`now`) and the
     one-shot `TLM` verb's bench-diagnostic fields (`acc`/`active`/`conn`/
     `glitch`/`ts`) not previously on any periodic wire path.
  2. Add `protos/config.proto`: a `ConfigTarget` enum (drivetrain / motor /
     planner / watchdog) and three small, curated `*Patch` messages
     (`DrivetrainConfigPatch`, `MotorConfigPatch`, `PlannerConfigPatch`)
     mirroring ONLY the 15 keys `config_commands.cpp`'s existing SET/GET
     surface already exposes — not the full 41-field `DrivetrainConfig`/
     10-field `MotorConfig`/10-field `PlannerConfig` messages.
  3. Replace `envelope.proto`'s placeholder `ConfigDelta`/`ConfigSnapshot`
     with real messages built from the Patch types; retype `ConfigGet.target`
     from `uint32` to `ConfigTarget`.
  4. `python scripts/gen_messages.py` regenerates every header, including
     two new ones (`telemetry.h`, `config.h`), without touching the shape
     of any pre-existing, unrelated header.
- **Postconditions**: The full binary wire contract for this sprint's three
  newly-implemented arms (`stream`/`config`/`get`) plus the `tlm`/`cfg`
  reply bodies exists as proto source, with every curated field traceable
  to an existing text surface (STREAM/SNAP/TLM/SET/GET), and every
  top-level envelope's `kMaxEncodedSize` still `<= 186` bytes (Decision 6,
  architecture-update-r1.md (095)).
- **Acceptance Criteria**:
  - [ ] `protos/telemetry.proto`'s `Telemetry` fields are traceable 1:1 to
        either `Telemetry::TlmFrameInput` (text STREAM/SNAP) or
        `handleTlm()`'s own field computation (`motion_commands.cpp`) —
        no field invents new firmware capability.
  - [ ] `protos/config.proto`'s three Patch messages' fields are traceable
        1:1 to `config_commands.cpp`'s `kAllKeys` (15 keys) — no key
        outside that list is exposed.
  - [ ] `python scripts/gen_messages.py --dry-run` succeeds; the generated
        `static_assert(kMaxEncodedSize<=186)` passes for `CommandEnvelope`
        and `ReplyEnvelope` with these arms now real.
  - [ ] `just build-sim` and the full existing sim suite stay green.

## SUC-002: Periodic telemetry emission works for the first time since sprint 093's loop rewrite
Parent: (infrastructure — restores a dead subsystem so SUC-003 has a live
text baseline to compare against)

- **Actor**: Firmware developer; indirectly, any client issuing `STREAM`/
  `stream`.
- **Preconditions**: `telemetryCommands()` (STREAM/SNAP) is NOT registered
  in `Rt::CommandRouter::buildTable()` (unwired since sprint 093, same as
  `configCommands()`); `Rt::MainLoop`'s own header comment confirms
  "loop-originated wire output (EVT/periodic TLM) are gone from the tick
  entirely... their classes remain parked" — no per-pass periodic
  re-emission code exists anywhere in the current tree (verified: zero
  hits for `telemetryEmit` outside `telemetry_commands.cpp` itself).
- **Main Flow**:
  1. `telemetryCommands()` is re-added to `buildTable()` (`command_router.cpp`),
     restoring STREAM/SNAP to the live text table (SET/GET stay
     unregistered — see architecture-update.md Decision 1).
  2. A new loop-owned periodic-emission step (`tickTelemetry()`, extending
     `commands/telemetry_commands.{h,cpp}`) is called once per pass from
     BOTH `source/main.cpp`'s bare loop and
     `tests/_infra/sim/sim_api.cpp`'s advance step — the same "both real
     hardware and sim call the identical function" invariant
     `Rt::MainLoop::tick()` already establishes for motion.
  3. `Rt::CommandRouter` gains a small accessor that resolves
     `bb.telemetryChannel` (a `Subsystems::Channel` enum) into an actual
     `ReplyFn`/`void*` pair outside of an active `route()` call.
- **Postconditions**: `STREAM <ms>` produces real, ongoing periodic `TLM ...`
  text frames on the bench (not just the one immediate frame the pre-093
  handler still manages inline); the SAME periodic step is the foundation
  SUC-003's binary emission builds on.
- **Acceptance Criteria**:
  - [ ] `STREAM 50` followed by waiting >= 200ms yields >= 3 periodic
        `TLM ...` frames with strictly increasing `seq=`, on both the sim
        harness and (per the bench gate) real hardware.
  - [ ] `STREAM 0` stops periodic emission; `SNAP` still works standalone.
  - [ ] The existing (pre-096) sim suite stays green — no existing text
        verb's behavior changes.

## SUC-003: Operator or host client streams telemetry over the binary plane at parity with text
Parent: UC-005 (Query Encoder Positions) / UC-006 (Query and Zero
Dead-Reckoning Odometry) — served over a second wire format and a second
(bench-diagnostic) field set

- **Actor**: `robot_radio` host software or any binary-speaking client.
- **Preconditions**: SUC-001's `Telemetry` schema and SUC-002's periodic
  tick both exist; `CommandEnvelope.cmd.stream` (`StreamControl`) decodes
  but replies `Error{ERR_UNIMPLEMENTED}` (095).
- **Main Flow**:
  1. Client sends `*B<base64(CommandEnvelope{stream:{binary:true,
     period:50}})>`.
  2. `BinaryChannel` sets `bb.telemetryPeriod`/`bb.telemetryChannel`
     (from `routerCtx`'s `currentChannel()`)/`bb.telemetryBinary`, and
     acks — mirroring `handleStream()`'s state-setting exactly, minus the
     text schema/parsing layer.
  3. SUC-002's periodic tick, on a later pass, sees `bb.telemetryBinary`
     and calls the NEW binary formatter (extends `Telemetry::tick()`/a new
     `Telemetry::buildTelemetryMessage()`) instead of `buildTlmFrame()`,
     encodes+armors a `ReplyEnvelope{tlm: Telemetry}` (`corr_id = 0`,
     unsolicited, per `envelope.proto`'s own forward-looking doc comment),
     and sends it on the bound channel.
- **Postconditions**: A binary client receives periodic `Telemetry` frames
  carrying the same information the text `TLM` frame carries (plus the
  bench-diagnostic fields `acc`/`active`/`conn`/`glitch`/`ts`), on a
  framing a relay byte-pipe passes through unmodified.
- **Acceptance Criteria**:
  - [ ] A binary `stream{binary:true, period:N}` command produces periodic
        `ReplyEnvelope{tlm}` frames at the requested (floor-clamped)
        period, with `seq` shared/monotonic against the SAME counter text
        STREAM uses.
  - [ ] `stream{binary:false, ...}` (or period 0) behaves exactly like text
        STREAM's own on/off semantics.
  - [ ] The text STREAM/SNAP frame's own wire text is byte-identical
        before and after this sprint (the shared `TlmFrameInput` struct
        gains fields; `buildTlmFrame()` itself is untouched).
  - [ ] Confirmed by the team-lead on the stand (per
        `.claude/rules/hardware-bench-testing.md`, after this sprint's
        tickets close): text vs. binary TLM streamed at matched rates
        shows no regression in `tlm_drop_rate()` for the binary path;
        gamepad teleop runs cleanly on binary TLM with Ack `q`/`rem` flow
        control.

## SUC-004: Host or operator reads and writes robot configuration over the binary plane
Parent: UC-014 (Tune Calibration Parameters at Runtime) — served over a
second wire format, restoring config_commands.cpp's parked SET/GET
capability via the binary arm specifically (not by re-registering text
SET/GET — see architecture-update.md Decision 1)

- **Actor**: `robot_radio` host software or any binary-speaking client.
- **Preconditions**: SUC-001's `ConfigDelta`/`ConfigSnapshot`/`ConfigGet`
  schema exists; `CommandEnvelope.cmd.config`/`.get` decode but reply
  `Error{ERR_UNIMPLEMENTED}` (095); `config_commands.cpp`'s SET/GET stay
  unregistered in `buildTable()` throughout this sprint.
- **Main Flow**:
  1. Client sends `*B<base64(CommandEnvelope{config:{drivetrain:{
     trackwidth: 142}}})>`. `BinaryChannel` translates the ONE populated
     Patch's present (`Opt<T>`) fields into a freshly-built
     `Rt::ConfigDelta{target, mask, value}` — mirroring
     `applyConfigKey()`'s existing per-field assignment shape (minus the
     `strcmp` dispatch and manual float/long parsing, both subsumed by the
     generated decoder's typed decode + `min`/`max`/`abs_max`/`req`
     validation) — and posts to `bb.configIn`. The Configurator (unchanged,
     `source/runtime/configurator.cpp`) folds+applies it exactly as it
     does for a text-SET-originated delta today.
  2. Client sends `*B<base64(CommandEnvelope{get:{target:CONFIG_DRIVETRAIN}})>`.
     `BinaryChannel` reads the CURRENT published `bb.drivetrainConfig`
     cell, populates a `ConfigSnapshot{target, drivetrain: Patch}`, and
     replies.
- **Postconditions**: A binary client can change and read back every one
  of `config_commands.cpp`'s 15 registered keys, chunked one
  `ConfigTarget` slice per reply (never a whole-`DrivetrainConfig` dump —
  that would not fit 186B; architecture-update.md Decision 4), with zero
  new functionality beyond that existing key surface.
- **Acceptance Criteria**:
  - [ ] Every one of the 15 keys in `config_commands.cpp`'s `kAllKeys`
        round-trips (`config` then `get` on the matching target) correctly
        over the binary path.
  - [ ] `ml`/`mr` (per-side `travel_calib`) address the correct bound
        motor independently; `pid.kp`/`ki`/`kff`/`iMax`/`kaw` apply to
        BOTH bound motors identically, mirroring `applyConfigKey()`'s
        existing both-sides behavior (architecture-update.md Decision 5).
  - [ ] An out-of-range or malformed field yields a typed `Error{code,
        field}`, never a crash, never a silent drop — validated by the
        generated decoder's `min`/`max`/`abs_max`/`req` checks, not
        hand-written range checks.
  - [ ] Confirmed by the team-lead on the stand: every config slice
        round-trips over the binary path; changing a PID gain over binary
        produces an observable, correct wheel-behavior change.

## SUC-005: The firmware telemetry/config codec is proven correct against the host's reference protobuf implementation
Parent: (infrastructure — closes issue Risk 6, "parked text families have
no live regression tests," for the two arms this sprint implements)

- **Actor**: Developer running the test suite; CI.
- **Preconditions**: 095's differential/fuzz/range harness (M8) exists for
  `drive`/`segment`/`replace`/`stop`/`ping`/`echo`/`id`; `Telemetry`/
  `ConfigDelta`/`ConfigSnapshot` have no differential coverage yet (they
  were empty placeholders in 095).
- **Main Flow**:
  1. Extend the existing differential harness (`tests/sim/unit/*_harness.cpp`
     + Python drivers) to cover `Telemetry` (encode-only, firmware-to-host
     direction) and `ConfigDelta`/`ConfigSnapshot` (both directions).
  2. Add fresh sim-level regression tests for binary `stream`/`config`/
     `get` — the issue's own Risk 6 ("parked text families have no live
     regression tests... binary arms for config/pose/otos need fresh sim
     coverage") applied to this sprint's two arms specifically.
- **Postconditions**: The self-written codec's correctness claim extends to
  every field this sprint adds, machine-checked against `google.protobuf`,
  not just "it compiled."
- **Acceptance Criteria**:
  - [ ] Differential round-trip passes for `Telemetry` (firmware-encode ->
        host-decode) and both directions for `ConfigDelta`/`ConfigSnapshot`.
  - [ ] A sim-level test drives `config`/`get`/`stream` through
        `BinaryChannel` end-to-end (not just the codec) and asserts the
        resulting `bb.configIn`/`bb.drivetrainConfig`/`bb.telemetryPeriod`
        effects match what the equivalent text verb would have produced.
  - [ ] The full pre-096 sim suite (~469 tests, per the sprint's baseline)
        stays green alongside the new tests.

## SUC-006: Host tooling speaks binary telemetry and config over the same transport
Parent: UC-014 / UC-005 — host-side half of SUC-003/SUC-004's wire

- **Actor**: A developer running `rogo`, TestGUI, teleop, or bench scripts
  built on `robot_radio`.
- **Preconditions**: 095's `pb2`/`serial_conn.py` envelope machinery exists
  (generic `ReplyEnvelope` demux via `_reply_queues`, already oneof-arm
  agnostic); `NezhaProtocol`'s public API is the compatibility shim 095
  established; `TLMFrame` is currently built only from parsed text.
- **Main Flow**:
  1. `TLMFrame` gains an alternate constructor built from a decoded `pb2.Telemetry`
     message — the SAME dataclass shape, so no downstream call site
     (TestGUI/teleop/bench/MCP) changes.
  2. `NezhaProtocol` gains binary set/get config methods (`send_envelope()`
     building `CommandEnvelope{config}`/`{get}`, parsing the
     `ConfigSnapshot`/`Ack` reply) alongside its existing text `SET`/`GET`
     wrappers — same public-API-stability posture 095 established for
     drive/segment/replace.
- **Postconditions**: The host can drive the binary telemetry and config
  planes through the exact same demux machinery (`_reply_queues`/
  `_tlm_queue`) the text plane and 095's drive/segment/replace already use
  — no new reader-thread branch needed (`ReplyEnvelope`'s oneof dispatch is
  already generic).
- **Acceptance Criteria**:
  - [ ] `TLMFrame.from_pb2(telemetry)` (or equivalent) produces a `TLMFrame`
        field-for-field equal to what parsing the matching text TLM line
        would have produced, for every field both formats carry.
  - [ ] `NezhaProtocol`'s binary config set/get round-trips against the
        differential test harness's host-side codec (SUC-005) without
        needing live hardware.
  - [ ] No existing `NezhaProtocol`/`TestGUI`/teleop call site changes
        signature or behavior.

## SUC-007: Config-sync check verifies the pydantic model against the wire schema, not a nonexistent file pair
Parent: (infrastructure — closes the config-sync divergence risk the
driving issue calls out)

- **Actor**: Developer running `scripts/check_config_sync.py`; CI.
- **Preconditions**: The CURRENT `check_config_sync.py` diffs
  `source/types/Config.h` against `source/robot/ConfigRegistry.cpp` —
  **neither file exists anywhere in the current `source/` tree** (verified
  by direct search; both are source_old-era artifacts). The script has
  been silently non-functional since the pre-rebuild tree, independent of
  this sprint.
- **Main Flow**:
  1. `check_config_sync.py` is rewritten to diff
     `host/robot_radio/config/robot_config.py`'s pydantic fields against
     the generated `pb2` descriptors for `DrivetrainConfigPatch`/
     `MotorConfigPatch`/`PlannerConfigPatch` (SUC-001's curated wire
     surface) — the fields a binary client can actually set/get, not the
     full internal `DrivetrainConfig`/`MotorConfig`/`PlannerConfig`
     messages (most of which have no wire-config verb at all).
- **Postconditions**: The script runs clean (or reports real, actionable
  drift) against the ACTUAL current schema and config model, closing the
  false-confidence risk of a check that silently does nothing.
- **Acceptance Criteria**:
  - [ ] `python scripts/check_config_sync.py` exits 0 against the current
        tree (or reports genuine, fixable drift — not a crash from a
        missing input file).
  - [ ] A field present in the pydantic model but absent from the curated
        pb2 Patch descriptors (or vice versa) is reported, not silently
        ignored.
  - [ ] The allowlist mechanism (`scripts/config_sync_allowlist.json`) is
        preserved in spirit (an escape hatch for known-intentional
        exceptions) even though the underlying comparison changes.
