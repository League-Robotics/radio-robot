---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 097 Use Cases

This sprint lands "Sprint 3" (the final sprint) of the protocol-v3 program
(`clasi/issues/protocol-v3-schema-driven-binary-command-plane-protobuf.md`):
host completion (every proven `NezhaProtocol` method becomes an envelope
builder, public API unchanged) and text retirement (delete the now-redundant
text parse/format code once its binary replacement has host-side parity,
down to a minimal five-verb safety rump). As with 095/096, most use cases
here are infrastructure — completing the host side of the wire and then
removing dead code — that exist to *keep serving* UC-001 (Drive Robot at
Continuous Speed), UC-004 (Stop Robot), UC-005 (Query Encoder Positions),
UC-006 (Query and Zero Dead-Reckoning Odometry), UC-014 (Tune Calibration
Parameters at Runtime), and UC-018 (Device Discovery) once the text plane
they rode on for those flows is gone. SUC-009 (the STOP safety rump) is the
one use case that is directly user/operator-visible in the traditional
sense.

## SUC-001: Host receives unsolicited binary telemetry push frames
Parent: (infrastructure — corrects a gap found in `serial_conn.py`'s reply
routing that blocks SUC-003)

- **Actor**: `SerialConnection`'s background reader thread.
- **Preconditions**: The firmware emits a `ReplyEnvelope{tlm}` push frame
  with `corr_id=0` on every periodic telemetry tick when a binary client
  has armed `StreamControl{binary: true}` (096, `tickTelemetry()` ->
  `telemetryEmitBinary()`). Today, `_handle_binary_reply()` routes every
  binary reply — pushed or requested — through
  `_reply_queues[str(reply.corr_id)]`, a table populated only while a
  `send()`/`send_envelope()` call is actively awaiting that specific
  corr_id. A `corr_id=0` push frame almost never matches a live entry, so
  it is silently dropped: binary telemetry streaming is currently
  undrainable from the host side even though the wire and firmware side are
  both correct (096).
- **Main Flow**:
  1. The reader thread receives a `*B<base64>` line and decodes it to a
     `ReplyEnvelope`.
  2. If `WhichOneof("body") == "tlm"`, the frame is routed to a new bounded
     `_binary_tlm_queue` (same depth/drop-oldest policy as the existing
     `_tlm_queue`) instead of the corr-id reply-queue lookup.
  3. Any other binary reply body keeps using the existing corr-id routing
     unchanged.
- **Postconditions**: A binary client that armed periodic streaming can
  drain a steady sequence of `TLMFrame`-convertible push frames from
  `SerialConnection`, the same way the text plane's `_tlm_queue` has always
  worked.
- **Acceptance Criteria**:
  - [ ] A new `_binary_tlm_queue` (or equivalent) exists on
        `SerialConnection`, matching `_tlm_queue`'s bounded/drop-oldest
        policy.
  - [ ] `_handle_binary_reply()` (or `_reader_loop()`) special-cases
        `body == "tlm"` BEFORE the corr-id lookup, mirroring how the text
        plane's `text.startswith("TLM")` branch is checked before the
        `OK`/`ERR`/`CFG`/`ID` corr-id branch.
  - [ ] Every other binary reply body (`ok`/`err`/`cfg`/`id`/`echo`) keeps
        routing through the unchanged corr-id path — a host unit test
        proves both a corr-id-keyed direct reply and a `corr_id=0` push
        frame are each routed correctly in the same reader-thread session.
  - [ ] No text-plane behavior changes (this ticket touches only the binary
        branch).

## SUC-002: Host speaks binary for liveness, drive, and config — API unchanged
Parent: UC-001 (Drive Robot at Continuous Speed) / UC-004 (Stop Robot) /
UC-014 (Tune Calibration Parameters at Runtime) / UC-018 (Device Discovery)

- **Actor**: Any existing `NezhaProtocol` caller (TestGUI, gamepad teleop,
  bench scripts, the MCP server) that calls `ping()`, `echo()`, `get_id()`,
  `get_ver()`, `stop()`, `drive()`, `timed()`, `distance()`, `get_config()`,
  or `set_config()`.
- **Preconditions**: 095/096 proved binary `ping`/`echo`/`id`/`stop`/`drive`
  (hardware-bench-smoke-tested) and `config`/`get` (sim-exhaustively-tested)
  arms exist and work. `NezhaProtocol` still sends TEXT for every one of
  these methods today; `set_config_binary()`/`get_config_binary()` exist
  only as separate, additively-named methods (096-007), not yet the
  primary path.
- **Main Flow**:
  1. Each listed method's body is rewritten to build the matching
     `CommandEnvelope` oneof arm and call `SerialConnection.send_envelope()`,
     instead of formatting and sending a text line.
  2. `timed()`/`distance()` translate their legacy `l`/`r`/`ms`-or-`mm`
     wire shape into a `MotionSegment` the same way `handleT()`/`handleD()`
     already do firmware-side (`BodyKinematics.forward()`-equivalent sign/
     distance computation, ported host-side once, shared with SUC-004's
     translator).
  3. `get_config()`/`set_config()` become thin wrappers over the existing
     `get_config_binary()`/`set_config_binary()` logic (096-007);
     `set_config_binary()`/`get_config_binary()` remain as direct-access
     methods for callers that already build a `pb2.ConfigDelta` themselves.
  4. Method **signatures and return shapes are unchanged** — a caller
     cannot tell, from the outside, that the wire format changed.
- **Postconditions**: Every listed method round-trips correctly against the
  firmware over the binary plane; no caller's source needed to change.
- **Acceptance Criteria**:
  - [ ] `ping()`, `echo()`, `get_id()`, `get_ver()`, `stop()`, `drive()`,
        `timed()`, `distance()`, `get_config()`, `set_config()` all send
        `*B<base64>` on the wire (verified by a host unit test asserting
        the bytes written to a fake serial port, not just the parsed
        return value).
  - [ ] `get_ver()`'s `fw`/`proto` dict keys are populated from the binary
        `id` arm's `DeviceId.fw_version`/`.proto_version` fields (Decision:
        VER's content is a strict subset of ID's; no independent binary
        `ver` arm is added — see architecture Decision on VER).
  - [ ] Every method's existing docstring-documented return type/shape is
        unchanged; existing unit tests for these methods (if any) pass
        unmodified.
  - [ ] `cancel()` (`X`), `arc()` (`R`), `vw()` (`VW`), `go_to()` (`G`),
        `turn()` (`TURN`), `drive_until_sensor()`, `grip()`, `zero_otos()`/
        `zero_all()`, `otos_*()`, `port_*()`, `stream_fields()` are
        explicitly OUT of this use case's scope — each targets a verb that
        does not exist in the current firmware at all (already broken,
        pre-dating this sprint; tracked by the separate
        `realign-host-tooling-to-gutted-four-verb-wire-surface.md` issue).
        This ticket does not touch them.

## SUC-003: Host speaks binary for telemetry streaming and one-shot reads
Parent: UC-005 (Query Encoder Positions) / UC-006 (Query and Zero
Dead-Reckoning Odometry)

- **Actor**: Any existing caller of `NezhaProtocol.stream()`/`.snap()`, and
  every host module that currently parses a raw text `TLM ...` line
  (`nezha.py`, `nezha_state.py`, `testgui/transport.py`,
  `calibration/linear.py`, `calibration/angular.py`,
  `calibration/fit_sim_error_model.py`, `sensors/odom_tracker.py`,
  `io/cli.py`, `playfield/world_goto_chart.py`).
- **Preconditions**: SUC-001's `_binary_tlm_queue` exists.
  `TLMFrame.from_pb2()` already adapts a binary `Telemetry` message onto
  the same dataclass shape `parse_tlm()` produces (096-007). No binary
  one-shot ("give me exactly one frame, right now") request exists — 096's
  own Open Question 2 deferred it, and adding a new wire arm is out of this
  sprint's scope.
- **Main Flow**:
  1. `stream(period)` sends `CommandEnvelope{stream: StreamControl{period,
     binary: true}}`.
  2. `snap()` is reimplemented, with NO new firmware wire capability, by
     sequencing the existing binary `stream` arm: drain
     `_binary_tlm_queue`, arm a brief period via `stream{period: <floor>,
     binary: true}`, wait for exactly one frame off `_binary_tlm_queue`,
     disarm with `stream{period: 0, binary: true}`, and return the
     resulting `TLMFrame` (or `None` on timeout) — the same public
     contract `snap()` has always had.
  3. Every listed internal consumer is swept from reading raw text off
     `_tlm_queue`/`read_lines()` and calling `parse_tlm(line)`, onto
     draining `_binary_tlm_queue` and using the already-adapted
     `TLMFrame` object directly.
  4. The module-level `parse_tlm()`/`parse_cfg()` functions and
     `NezhaProtocol.parse_tlm`/`.parse_cfg` static wrappers are deleted
     once zero real (non-test-of-itself) call sites remain.
- **Postconditions**: Every existing `TLMFrame`-consuming code path keeps
  working, sourced entirely from the binary plane; no text `TLM ...` line
  is parsed anywhere in `host/` after this ticket.
- **Acceptance Criteria**:
  - [ ] `stream()`/`snap()` send `*B<base64>` on the wire; `snap()`'s
        return type/shape (`TLMFrame | None`) is unchanged.
  - [ ] Every listed internal consumer file is updated to source its
        `TLMFrame` from the binary plane; `grep -rn "parse_tlm" host/`
        (excluding the deleted function's own former definition) returns
        no hits outside test files exercising `TLMFrame.from_pb2`'s
        historical parity claim.
  - [ ] `parse_tlm`/`parse_cfg` (module-level functions) and
        `NezhaProtocol.parse_tlm`/`.parse_cfg` (static wrappers) are
        deleted.
  - [ ] `stream_fields()` is explicitly OUT of scope (already calls a
        `fields=` kv the current text `STREAM` handler has never accepted
        — pre-existing broken method, same bucket as SUC-002's exclusions).

## SUC-004: A human at `rogo send` keeps typing v2 text while the wire carries binary
Parent: UC-018 (Device Discovery) — the human-operator entry point to every
other use case above

- **Actor**: A developer or bench operator at a terminal running `rogo
  send <text>`.
- **Preconditions**: `rogo binary <arm>` (095/096) already exists as a
  separate, explicit subcommand family for building envelopes by hand;
  `rogo send` today is a raw pass-through to `NezhaProtocol.send()` (plain
  text).
- **Main Flow**:
  1. `rogo send` gains a text-v2-to-envelope translator, built on the same
     legacy-verb-to-`CommandEnvelope` translation logic SUC-002/SUC-003
     introduced for `timed()`/`distance()`/`snap()` (shared, not
     duplicated).
  2. For a verb with a binary replacement, the translator builds and sends
     the matching envelope. For a retained rump verb (PING, ID, HELLO,
     HELP, STOP), it sends the verb as plain text, unchanged.
  3. A `--decode` flag pretty-prints a received `*B<base64>` reply's
     decoded fields instead of the raw armored line.
- **Postconditions**: A human never needs to hand-construct a
  `pb2.CommandEnvelope` to drive the robot interactively; `rogo binary
  <arm>` remains available for direct, low-level envelope construction.
- **Acceptance Criteria**:
  - [ ] `rogo send S 200 200`, `rogo send D 200 200 300`, `rogo send STOP`
        each produce the expected on-wire effect (binary for S/D, text for
        STOP) and a human-readable reply.
  - [ ] `rogo send --decode ...` prints decoded reply fields, not a raw
        `*B...` line.
  - [ ] `rogo binary <arm>` subcommands are unaffected.

## SUC-005: Every existing host consumer test suite passes unmodified
Parent: (infrastructure — protects SUC-002/SUC-003/SUC-004 before firmware
deletion begins)

- **Actor**: The sprint's own verification step (team-lead-run, this
  ticket).
- **Preconditions**: SUC-002/003/004 land. Firmware text handlers are STILL
  present (deletion has not started) — this ticket's whole purpose is to
  prove the host is safe to depend on binary BEFORE any text handler is
  removed.
- **Main Flow**:
  1. Run `tests/unit`, `tests/sim` (the CI gate), and every
     `NezhaProtocol`-consuming test file's existing suite unmodified.
  2. Confirm the `tests/testgui` tier's pre-existing 16 failures do not
     increase (informational only — fixing them is explicitly out of
     scope, tracked by the separate
     `realign-host-tooling-to-gutted-four-verb-wire-surface.md` issue).
  3. Exercise `rogo send`/`rogo binary` manually against the sim harness
     (or bench, if available) for each converted verb.
- **Postconditions**: A documented, verified go/no-go for starting firmware
  text deletion.
- **Acceptance Criteria**:
  - [ ] `tests/sim` is green.
  - [ ] `tests/unit` is green.
  - [ ] `tests/testgui` failure count is recorded and is <= the
        pre-sprint baseline (16), not fixed, not increased.
  - [ ] Bench scripts under `tests/bench/` that call converted
        `NezhaProtocol` methods are reviewed for continued correctness
        (hardware bench execution itself is the team-lead's post-sprint
        consolidated gate, not this ticket's).

## SUC-006: Firmware retires the migrated motion and liveness text families
Parent: UC-001 (Drive Robot at Continuous Speed) / UC-004 (Stop Robot)

- **Actor**: Firmware maintainer / the build's flash budget.
- **Preconditions**: SUC-005's gate passed. Binary `drive`/`segment`/
  `replace`/`echo`/`id` arms are proven (095, hardware-bench-smoke-tested
  for drive/stop/ping/echo/id; sim-exhaustively-tested for segment/replace,
  hardware bench for MOVE/MOVER deferred to the team-lead's post-sprint
  consolidated session). `S`/`D`/`T`/`RT`/`MOVE`/`MOVER` (motion_commands.cpp)
  and `ECHO`/`VER` (system_commands.cpp) are currently live, registered
  text verbs; `source/types/command_types.h`'s `ParsedCommand` struct has
  zero references anywhere in the tree.
- **Main Flow**:
  1. Delete `parseS`/`handleS`, `parseD`/`handleD`, `parseT`/`handleT`,
     `parseRT`/`handleRT`, `parseMove`/`handleMove`, `parseMover`/
     `handleMover` and their `motionCommands()` registrations.
  2. Delete `handleEcho`'s text registration (`ECHO`) and `handleVer`
     (`VER`) from `systemCommands()` — VER's content (`fw`/`proto`) is a
     strict subset of the binary `id` arm's `DeviceId` reply.
  3. Delete `ParsedCommand` from `command_types.h`.
  4. `STOP` stays registered and unchanged (the rump — SUC-009).
  5. `parseR`/`handleR` (`R`), `parseTURN`/`handleTURN` (`TURN`),
     `parseG`/`handleG` (`G`), and the shared stop-clause grammar helpers
     they depend on (`parseStopClauseValue`/`collectStopClauses`/
     `packStopKVs`/`kMaxStopConds`/`replyStopBadarg`) are explicitly
     PRESERVED, unregistered, exactly as before this sprint — see
     architecture Decision on the parked Planner family.
  6. `handleTlm` (one-shot `TLM`) and `handleQlen` (`QLEN`) are explicitly
     PRESERVED, registered, exactly as before this sprint — see
     architecture Decision on bench-diagnostic verbs.
- **Postconditions**: `source/commands/motion_commands.cpp` and
  `system_commands.cpp` shrink measurably; the flash delta is recorded.
- **Acceptance Criteria**:
  - [ ] `grep -rn '"S"\|"D"\|"T"\|"RT"\|"MOVE"\|"MOVER"' source/commands/motion_commands.cpp`
        (registration call sites only) returns no hits.
  - [ ] `grep -rn "ParsedCommand" source/` returns no hits.
  - [ ] `parseR`/`handleR`/`parseTURN`/`handleTURN`/`parseG`/`handleG` and
        the shared stop-clause helpers still compile (unregistered, source
        present).
  - [ ] `handleTlm`/`handleQlen` still registered and functioning.
  - [ ] `tests/sim` green; flash delta (`.map`) recorded.

## SUC-007: Firmware retires the text config family
Parent: UC-014 (Tune Calibration Parameters at Runtime)

- **Actor**: Firmware maintainer / the build's flash budget.
- **Preconditions**: SUC-005's gate passed. Binary `config`/`get` arms are
  sim-exhaustively-tested (096). `config_commands.cpp`'s `SET`/`GET` are
  already UNREGISTERED (096 Decision 1) but the file, both `strcmp` chains
  (`applyConfigKey`, `formatConfigKeyFromBb`), and the `CFG` snprintf
  emitter are still on disk.
- **Main Flow**:
  1. Delete `source/commands/config_commands.{h,cpp}` in full.
  2. `dev_commands.cpp`'s own, separate, lower-level `DEV *CFG` strcmp
     chains are explicitly OUT of scope (096 Decision 3's boundary,
     unchanged this sprint — no binary `dev` arm exists or is planned).
- **Postconditions**: `source/commands/` loses ~644 lines; the flash delta
  is recorded.
- **Acceptance Criteria**:
  - [ ] `config_commands.{h,cpp}` no longer exist.
  - [ ] No remaining `#include "commands/config_commands.h"` anywhere.
  - [ ] `dev_commands.cpp` is untouched.
  - [ ] `tests/sim` green; flash delta recorded.

## SUC-008: Firmware retires the text telemetry family
Parent: UC-005 (Query Encoder Positions) / UC-006 (Query and Zero
Dead-Reckoning Odometry)

- **Actor**: Firmware maintainer / the build's flash budget.
- **Preconditions**: SUC-005's gate passed (including SUC-003's `snap()`
  synthesis verified end-to-end). The binary `stream` arm and
  `Telemetry`/`buildTelemetryMessage()` are sim-exhaustively-tested (096).
  `handleStream`/`handleSnap`, `kStreamSchema`, and the text
  `telemetryEmit()`/`Telemetry::buildTlmFrame()` formatter are still live.
- **Main Flow**:
  1. Delete `handleStream`/`handleSnap`'s `telemetryCommands()`
     registrations, `kStreamSchema`, and `telemetryEmit()`/
     `Telemetry::buildTlmFrame()` (the text-only formatter — NOT
     `Telemetry::tick()`, which both planes share and stays).
  2. `tickTelemetry()` keeps its `bb.telemetryBinary` branch structurally,
     but since only the binary `stream` arm can ever set
     `bb.telemetryPeriod`/`.telemetryBinary` now, the text branch becomes
     unreachable — remove it (or leave a documented `assert`/dead branch
     removal, per the ticket's own judgment call, cited explicitly either
     way).
  3. `handleTlm` (one-shot `TLM` verb) is untouched — see SUC-006's
     Decision (a distinct, disjoint text surface from STREAM/SNAP, per
     096's own Step 1 finding).
- **Postconditions**: `source/telemetry/tlm_frame.cpp`'s text formatter is
  gone; `source/commands/telemetry_commands.cpp` shrinks; the flash delta
  is recorded.
- **Acceptance Criteria**:
  - [ ] `STREAM`/`SNAP` no longer registered as text verbs.
  - [ ] `Telemetry::buildTlmFrame()` (text formatter) deleted;
        `Telemetry::tick()`/`buildTelemetryMessage()` (binary) untouched.
  - [ ] `handleTlm`/`handleQlen` still registered (SUC-006).
  - [ ] `tests/sim` green (with sim tests for STREAM/SNAP updated to drive
        the binary `stream` arm instead of the deleted text verbs); flash
        delta recorded.

## SUC-009: A bare-terminal human retains the text safety rump
Parent: UC-004 (Stop Robot)

- **Actor**: A human with nothing but a raw serial terminal (no host
  program).
- **Preconditions**: SUC-006 lands (motion text families deleted, STOP
  explicitly retained).
- **Main Flow**:
  1. The human types `STOP` at a bare terminal while the robot is moving
     (via any means — a prior binary `drive`/`segment` command, or a
     lingering physical motion).
  2. The firmware's unchanged `handleStop()` posts a NEUTRAL
     `DrivetrainCommand` to `bb.driveIn` exactly as it does today.
- **Postconditions**: The robot halts. PING/ID/HELLO/HELP also remain
  hand-typeable for liveness/identity/discoverability.
- **Acceptance Criteria**:
  - [ ] Bench: a bare-terminal typed `STOP` halts a moving robot (the
        team-lead's post-sprint consolidated bench gate — this sprint's
        job is to confirm nothing in SUC-006/007/008 touches `handleStop`,
        `PING`, `ID`, `HELLO`, or `HELP`'s registrations or bodies).
  - [ ] `grep -n '"PING"\|"ID"\|"HELLO"\|"HELP"\|"STOP"' source/runtime/command_router.cpp source/commands/system_commands.cpp source/commands/motion_commands.cpp`
        shows exactly these five still registered, unchanged.

## SUC-010: docs/protocol-v2.md is rewritten as an accurate protocol-v3 document
Parent: (infrastructure — keeps the wire's only prose spec truthful)

- **Actor**: Any developer reading protocol documentation.
- **Preconditions**: SUC-006/007/008 land (final wire surface is stable).
  `docs/protocol-v2.md` (2252 lines) currently documents the pre-v3 text
  grammar exclusively — sections 7 (SET/GET), 8 (STREAM/SNAP/TLM), 10
  (motion verbs), 11 (OTOS/port) are all now partly or wholly stale.
- **Main Flow**:
  1. Rewrite as `docs/protocol-v3.md`: envelope framing (`*B<base64>`),
     every implemented `CommandEnvelope`/`ReplyEnvelope` oneof arm, the
     five-verb text rump, and an explicit "parked, not on the wire" note
     for R/TURN/G/OTOS/pose/DEV (naming which future sprint, if any, is
     expected to touch each).
  2. Retire or clearly mark `docs/protocol-v2.md` as superseded (do not
     silently delete history — link forward).
- **Postconditions**: A new reader can learn the actual, current wire
  surface from one document.
- **Acceptance Criteria**:
  - [ ] `docs/protocol-v3.md` exists and documents every implemented
        binary arm plus the text rump.
  - [ ] `docs/protocol-v2.md` carries a clear superseded-by pointer.
  - [ ] No section describes a verb this sprint deleted as if it were
        still live.

## SUC-011: The protocol-v3 migration issue is fully resolved
Parent: (infrastructure — closes
`protocol-v3-schema-driven-binary-command-plane-protobuf.md`)

- **Actor**: The sprint's closing ticket.
- **Preconditions**: SUC-001..010 land.
- **Main Flow**:
  1. Grep-confirm every deletion target from the issue's "What gets
     deleted" list is gone, except the named five-verb rump and the
     explicitly-preserved parked families (R/TURN/G, OTOS/pose text,
     `dev_commands.cpp`, `handleTlm`/`handleQlen`).
  2. Record final `source/commands/` line count against the issue's own
     ~1,000–1,300-line estimate.
  3. Record the final flash-footprint report (before/after the full
     3-sprint program), per the issue's own net-negative expectation.
- **Postconditions**: The issue is marked resolved; the ticket carries
  `completes_issue: true`.
- **Acceptance Criteria**:
  - [ ] Grep-clean report produced (see Main Flow item 1).
  - [ ] Final `source/commands/` line count recorded.
  - [ ] Final flash report recorded, compared against the pre-project
        (pre-095) baseline.
