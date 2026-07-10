---
id: '002'
title: NezhaProtocol core conversion (liveness/drive/config) + Legacy Verb Translator
status: done
use-cases:
- SUC-002
- SUC-004
depends-on: []
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# NezhaProtocol core conversion (liveness/drive/config) + Legacy Verb Translator

## Description

Convert `NezhaProtocol.ping()`, `.echo()`, `.get_id()`, `.get_ver()`,
`.stop()`, `.drive()`, `.timed()`, `.distance()`, `.get_config()`, and
`.set_config()` from text-line senders into `CommandEnvelope` builders,
using `SerialConnection.send_envelope()` (095) — with **every method's
signature and return type/shape held unchanged**, per the compatibility-
shim design 095 established for this class. `set_config()`/`get_config()`
become thin wrappers over the existing `set_config_binary()`/
`get_config_binary()` logic (096-007), which stay available as
direct-access methods.

This ticket also builds **M4, the Legacy Verb Translator** — a small set
of pure, stateless functions that turn a legacy verb's wire-shaped
arguments into the matching binary message. `timed()`/`distance()` need
this to reproduce `handleT()`/`handleD()`'s own l/r-sign-then-distance
computation (`BodyKinematics::forward()`, firmware) host-side — port it
exactly, per the "transcribe, never re-derive" discipline 095 Decision 5
established, verified by a unit test against known firmware behavior, not
re-derived from first principles. This translator is a shared dependency
of ticket 004 (`rogo` REPL) — build it once, here, so ticket 004 reuses it
rather than reimplementing the same mapping.

**Explicitly out of scope** (architecture Decision 1): `cancel()`
(`X`), `arc()` (`R`), `vw()` (`VW`), `go_to()` (`G`), `turn()` (`TURN`),
`drive_until_sensor()`, `grip()`, `zero_otos()`/`zero_all()`,
`otos_*()`, `port_*()`, `stream_fields()`. Every one of these targets a
verb that either does not exist anywhere in the current firmware source
tree, or is parked with no binary replacement — converting them is not
achievable within this sprint's "no new binary functionality" boundary.
Leave their bodies exactly as they are today (already non-functional
against the current firmware — pre-existing drift tracked by the separate
`realign-host-tooling-to-gutted-four-verb-wire-surface.md` issue). Do not
touch them.

## Acceptance Criteria

- [x] `ping()`, `echo()`, `get_id()`, `get_ver()`, `stop()`, `drive()`,
      `timed()`, `distance()`, `get_config()`, `set_config()` all send
      `*B<base64>` on the wire — verified by a host unit test asserting
      the actual bytes written to a fake serial port (not just the parsed
      return value).
- [x] `get_ver()`'s `fw`/`proto` dict keys are populated from the binary
      `id` arm's `DeviceId.fw_version`/`.proto_version` fields (VER's
      content is a strict subset of ID's reply; no independent binary
      `ver` arm exists or is added).
- [x] Every method's existing docstring-documented return type/shape is
      unchanged; any existing unit test for these methods passes
      unmodified.
- [x] `cancel()`, `arc()`, `vw()`, `go_to()`, `turn()`,
      `drive_until_sensor()`, `grip()`, `zero_otos()`, `zero_all()`,
      `otos_*()`, `port_*()`, `stream_fields()` are byte-for-byte
      untouched by this ticket's diff.
- [x] M4 (Legacy Verb Translator) exists as pure, stateless functions with
      no `SerialConnection`/I/O reference, ported from and unit-tested
      against `handleT()`/`handleD()`'s own firmware computation for
      representative `l`/`r`/`mm`/`ms` inputs.
- [x] `tests/sim` stays green (this ticket is host-only; run it as a
      sanity check — no firmware files change).
- [x] `tests/unit` is green, including new tests for every converted
      method and for M4.

## Implementation Plan

### Approach

1. Build M4 first (`host/robot_radio/robot/legacy_translate.py`, new
   file, or module-level functions in `protocol.py` — either satisfies
   the architecture's boundary; document the choice in the file's own
   header comment). Port `handleT()`/`handleD()`/`handleS()`'s exact
   sign/distance/wheel-target computation from `motion_commands.cpp`.
2. Rewrite each of the ten methods' bodies in `protocol.py` to build the
   matching `pb2.CommandEnvelope` oneof arm and call
   `self._conn.send_envelope(env, read_timeout=...)`, unwrapping the
   reply the same way `set_config_binary()`/`get_config_binary()` (096-007)
   already demonstrate — return `None` on timeout/error, matching each
   method's existing "no failure detail" posture.
3. `get_config()`/`set_config()`: keep their existing `**kwargs`/`*keys`
   signatures; internally build the matching `pb2.ConfigDelta`/`ConfigGet`
   the way `set_config_binary()`/`get_config_binary()` already do, or call
   those methods directly if convenient.
4. Leave every other method's source byte-for-byte unchanged.

### Files to modify

- `host/robot_radio/robot/protocol.py` — the ten methods' bodies.
- `host/robot_radio/robot/legacy_translate.py` (new, or equivalent
  module-level functions in `protocol.py`) — M4.

### Testing plan

- New host unit tests per converted method: assert the envelope's wire
  bytes match expectations for representative inputs, and that the
  parsed return value matches the pre-conversion text-plane contract.
- M4 unit tests: compare translator output against hand-computed
  expected `MotionSegment`/`DrivetrainCommand` values for several `l`/
  `r`/`mm`/`ms` combinations, cross-checked against
  `BodyKinematics::forward()`'s documented behavior (cite the exact
  firmware function/file in the test's own comment, per the
  transcribe-don't-re-derive discipline).
- Run `tests/unit` (host suite) and `tests/sim` (sanity — unaffected).

### Documentation updates

- None required beyond docstrings (each converted method's own docstring
  gets a one-line note that the implementation is now binary, contract
  unchanged). `docs/protocol-v3.md` (ticket 009) documents the wire level.

## Resolution

Implemented per the plan, M4 first:

**M4 (`host/robot_radio/robot/legacy_translate.py`, new file)** — chose a
standalone module over module-level functions in `protocol.py` (Open
Question 5's own "left to this ticket's judgment"), documented in the
file's own header: ticket 004's `rogo` REPL translator is a second real
caller, and a standalone module lets it import the mapping without pulling
in `NezhaProtocol`/`SerialConnection`. Scoped to exactly what this ticket's
own ten methods need — `wheel_targets_for_drive()` (handleS()'s direct
per-wheel passthrough), `segment_for_timed()`/`segment_for_distance()`
(handleT()/handleD()'s `BodyKinematics::forward()`-based sign/distance
computation) — plus a `forward()` helper transcribing the full two-output
kinematic map for completeness/testability against the cited source. RT/
MOVE/MOVER (also named in architecture-update.md's M4 description) are
explicitly left for ticket 004: `NezhaProtocol` has no `rt()`/`move()`/
`mover()` method today, so nothing in this ticket's acceptance criteria
needs them — noted in the module's own file header so ticket 004 knows to
extend, not duplicate. Key finding that simplified the design: `omega`
(the only trackwidth-dependent output of `forward()`) is discarded by both
`handleT()`/`handleD()` (`(void)omega;`), so `segment_for_timed()`/
`segment_for_distance()` need no `trackwidth` parameter at all — avoiding
a signature-breaking addition to `NezhaProtocol.timed()`/`.distance()`.

**M2 (`host/robot_radio/robot/protocol.py`, ten method bodies)**:
- `ping()`/`echo()`/`get_id()`/`stop()`/`drive()`: direct 1:1 mapping onto
  `CommandEnvelope{ping|echo|id|stop|drive}` via `send_envelope()`,
  unwrapping the matching reply arm (`ok`/`echo`/`id`).
- `get_ver()`: reuses the SAME `id` arm `get_id()` sends (no independent
  binary `ver` arm), reading only `fw_version`/`proto_version` off the
  reply — the ticket's own required behavior.
- `timed()`/`distance()`: build a `MotionSegment` via
  `legacy_translate.segment_for_timed()`/`segment_for_distance()`, send via
  `CommandEnvelope{segment: ...}`. Return value is a synthesized
  single-line `list[str]` (`["OK drive ..."]` on Ack, `[]` on timeout) —
  preserves the pre-conversion `list[str]` SHAPE; grep-verified no caller
  in this tree inspects the actual line text.
- `get_config()`/`set_config()`: thin wrappers over
  `get_config_binary()`/`set_config_binary()` (096-007), via a new
  module-level 15-key mapping table (`_DRIVETRAIN_KEYS`/`_MOTOR_PID_KEYS`/
  `_PLANNER_KEYS`/`_TARGET_FOR_KEY`) transcribed from
  `config_commands.cpp`'s `kAllKeys` (config.proto's own header comment
  already establishes the 1:1 correspondence, so this invents no new
  vocabulary). Flagged, not silently reconciled: unlike the text plane's
  single atomic `SET` line, `ConfigDelta`'s oneof carries only ONE Patch at
  a time, so a `set_config()` call spanning multiple targets (e.g. `tw=` +
  `sTimeout=`) costs multiple round trips and is NOT atomic across targets
  (a true cross-target atomic SET needs new binary wire capability, out of
  this sprint's "no new binary functionality" scope). An unknown key fails
  the whole call with no wire traffic (mirrors the text plane's own
  atomic-SET "one bad key rejects the whole line" posture). `pid.*` is
  applied to both bound motors from ONE envelope (server-side fan-out,
  `handleConfigMotor()`); `ml`/`mr` need two envelopes when both given
  (side-selected `travel_calib`).
- Deliberate safety choice on `drive(..., stop=[...])`: since the CURRENT
  text `S` handler (`parseS()`, 093-001) already rejects any `stop=`/
  `sensor=` kv with `ERR badarg` (no motor effect), and `drive()`'s prior
  `send_fast()` never read that reply anyway, the binary implementation
  sends NO envelope at all when `stop` is passed — preserving "no motor
  effect" rather than silently starting to drive (which
  `WheelTargets`/`DrivetrainCommand` has no mechanism to reject). No caller
  in this tree passes `stop` to `drive()` (grep-verified).

Every one of the 12 named out-of-scope methods, plus every OTHER
`NezhaProtocol` method not in the ten-method list (`zero_encoders()`,
`get_help()`, `stream()`, `snap()`, `wait_for_evt_done()`, `stream_drive()`,
`send()`, `send_fast()`, `read_lines()`, `read_pending_lines()`,
`set_config_binary()`, `get_config_binary()`), was verified byte-for-byte
identical to `HEAD` via an AST-based per-method source-segment diff (not
just eyeballing `git diff`) — only the ten target methods' `FunctionDef`
source segments differ from `HEAD`.

**Verification**:
- `uv run python -m pytest tests/unit -q` — 83 passed (42 pre-existing
  unmodified [27 in other host test files + 15 in
  `test_protocol_binary_client.py`] + 41 new: 18 in a new
  `tests/unit/test_legacy_translate.py` for M4, 23 appended to the
  existing `tests/unit/test_protocol_binary_client.py` for the ten
  converted methods and `get_config()`/`set_config()`'s
  multi-target/key-mapping behavior).
- `uv run python -m pytest tests/sim -q` — 600 passed, unaffected
  (host-only change, no firmware/sim files touched).
