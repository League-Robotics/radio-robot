---
id: '013'
title: Delete the binary plane (protobuf codec, COBS/CRC, ack ring, persisted-tuning/parity/boot-live-split,
  orphan proto schema)
status: open
use-cases: [SUC-008]
depends-on: ['012']
github-issue: ''
issue:
- protocol-v6-spec.md
- protocol-and-implementation-simplification-review.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Delete the binary plane (protobuf codec, COBS/CRC, ack ring, persisted-tuning/parity/boot-live-split, orphan proto schema)

## Description

**Deletion ticket — must be exhaustive: no binary plane may survive.**
Delete the binary plane outright, now that tickets 001-012 have proven the
ASCII replacement complete and conformant (spec §13: "staged so the binary
plane is emptied before it is deleted"). Per `sprint.md`'s Impact on
Existing Components and the review's R1-R4 "deletes" bullets:

- Generated protobuf structs/codec (`wire.{h,cpp}`, most per-proto headers)
  in `src/firm/messages/`.
- COBS framing, CRC-16/CCITT (including `crcOverScope()`, `crcInit`/
  `crcUpdate`), and the varint/zigzag codecs from `wire_runtime.{h,cpp}`
  (the file shrinks — it is NOT deleted outright, since the unrelated debug
  harness still calls `base64Encode`/`Decode`).
- The ack ring (depth 12, 3x repeat, packed `corr_id<<4|err`).
- `PersistedTuning` and `config_parity_capi` (deleted outright, in
  `src/firm/config/`).
- The boot-only/live config split.
- The now-orphaned `src/protos/*.proto` message declarations for the wire
  (`envelope.proto`, `commands.proto`'s binary half, most of
  `robot_config.proto`, `telemetry.proto`, `drivetrain.proto`,
  `motor.proto`, `sensors.proto`, `communicator.proto`, `gripper.proto`,
  `odometer.proto`) — shrink to declaring only the three flat tables, or
  delete, per the ticket 001/013 boundary call left open in `sprint.md`'s
  Step 7 open questions.
- `gen_pb2.py` and the protobuf runtime dependency, both languages.

`boot_config.{h,cpp}` is NOT deleted — boot-time baking from the robot JSON
is unchanged in kind per `.claude/rules/configuration-discipline.md`'s
production-boot rule.

## Acceptance Criteria

- [ ] No reachable reference to COBS, CRC-16, protobuf codegen, the ack
      ring, `PersistedTuning`, or `config_parity_capi` remains anywhere in
      `src/firm` or `src/host` after this ticket, confirmed by grep against
      the built sources (not assumed from the diff alone) — per
      `sprint.md`'s Success Criteria wording.
- [ ] `gen_pb2.py` and the protobuf runtime dependency are removed from
      both the C++ build and the Python dependency manifest
      (`pyproject.toml` / equivalent).
- [ ] `boot_config.{h,cpp}` and the boot-baked-from-JSON path are
      explicitly verified UNCHANGED and still passing after this ticket —
      this is not collateral damage.
- [ ] `wire_runtime.{h,cpp}` shrinks (COBS/CRC/varint/zigzag removed) but
      is not deleted outright — `base64Encode`/`Decode` calls from the
      unrelated debug harness are confirmed still present and working.
- [ ] The full test suite (`uv run python -m pytest`, project-wide) passes
      with zero references to deleted symbols remaining as dead
      imports/includes — this is run here (in addition to the sprint-close
      run) because deletion has the widest blast radius of any ticket in
      the sprint.
- [ ] This ticket only starts after ticket 011 (conformance) has proven the
      ASCII replacement complete, per `sprint.md`'s Migration Concerns
      explicit sequencing note.

## Testing

- **Existing tests to run**: full C++ test suite, full Python test suite
  (`uv run python -m pytest`) — the one full run for this stretch of the
  sprint, since deletion has the widest blast radius.
- **New tests to write**: a grep-based "no dead reference" check
  (COBS/CRC/protobuf/ack-ring symbol names) run against the built tree as
  part of this ticket's own acceptance.
- **Verification command**: `uv run python -m pytest`, plus a full
  `--clean` C++ build with zero warnings about the removed headers.
