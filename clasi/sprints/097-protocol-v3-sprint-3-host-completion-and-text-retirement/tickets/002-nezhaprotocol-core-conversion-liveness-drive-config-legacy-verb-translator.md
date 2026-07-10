---
id: '002'
title: NezhaProtocol core conversion (liveness/drive/config) + Legacy Verb Translator
status: open
use-cases: [SUC-002, SUC-004]
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

- [ ] `ping()`, `echo()`, `get_id()`, `get_ver()`, `stop()`, `drive()`,
      `timed()`, `distance()`, `get_config()`, `set_config()` all send
      `*B<base64>` on the wire — verified by a host unit test asserting
      the actual bytes written to a fake serial port (not just the parsed
      return value).
- [ ] `get_ver()`'s `fw`/`proto` dict keys are populated from the binary
      `id` arm's `DeviceId.fw_version`/`.proto_version` fields (VER's
      content is a strict subset of ID's reply; no independent binary
      `ver` arm exists or is added).
- [ ] Every method's existing docstring-documented return type/shape is
      unchanged; any existing unit test for these methods passes
      unmodified.
- [ ] `cancel()`, `arc()`, `vw()`, `go_to()`, `turn()`,
      `drive_until_sensor()`, `grip()`, `zero_otos()`, `zero_all()`,
      `otos_*()`, `port_*()`, `stream_fields()` are byte-for-byte
      untouched by this ticket's diff.
- [ ] M4 (Legacy Verb Translator) exists as pure, stateless functions with
      no `SerialConnection`/I/O reference, ported from and unit-tested
      against `handleT()`/`handleD()`'s own firmware computation for
      representative `l`/`r`/`mm`/`ms` inputs.
- [ ] `tests/sim` stays green (this ticket is host-only; run it as a
      sanity check — no firmware files change).
- [ ] `tests/unit` is green, including new tests for every converted
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
