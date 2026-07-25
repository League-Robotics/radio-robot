---
id: '003'
title: Host decoder rewrite
status: open
use-cases: [SUC-001, SUC-002, SUC-003]
depends-on: ['002']
github-issue: ''
issue: cobs-crc-binary-framing-replace-base64-armor.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host decoder rewrite

## Description

Update every host consumer/producer of `*B<base64>` to
the COBS+CRC framing: `io/cli.py`, `io/sim_loop.py`, `io/serial_conn.py`,
`io/sim_config.py`, `testgui/transport.py`, `robot/protocol.py`. Preserve
every higher-level API (`NezhaProtocol`'s command builders, `TLMFrame`'s
field semantics) unchanged — only the framing/armor layer changes.

## Acceptance Criteria

- [ ] Each of the six files decodes/encodes via COBS+CRC; no remaining
      `base64.b64decode`/`b64encode` call on the wire path (confirm via
      grep, mirroring `messages/DESIGN.md`'s own existing grep-
      verifiable-invariant style).
  - [ ] `SerialConnection`/`NezhaProtocol`/`sim_loop`/TestGUI's
        `transport.py` all pass their existing higher-level test suites
        unchanged (command builders, `TLMFrame` field access untouched).
  - [ ] HELLO/PING text rump still decodes correctly when interleaved
        with binary frames on the same connection (host-side half of
        SUC-003's acceptance).
  - [ ] Fault-injection test: a corrupted frame is dropped on the host
        decode path with a counted fault, not silently mis-parsed
        (host-side half of SUC-002).

## Implementation Plan

- **Approach:** Port the firmware's COBS+CRC scheme byte-for-byte
  (same algorithm, same CRC width) to Python; centralize the framing
  logic in one place (`io/serial_conn.py` looks like the natural home,
  given it already owns "byte-level transport concerns" per
  `src/host/robot_radio/DESIGN.md`) rather than duplicating it across
  all six call sites.
- **Files:** `src/host/robot_radio/io/cli.py`, `io/sim_loop.py`,
  `io/serial_conn.py`, `io/sim_config.py`, `testgui/transport.py`,
  `robot/protocol.py`.
- **Testing:** Host-side unit tests for the new framer (round-trip,
  fault-injection); existing host test suite must stay green (`uv run
  python -m pytest`, per the project's own pytest/uv gotcha).
- **Documentation:** `src/host/DESIGN.md`, `src/host/robot_radio/DESIGN.md`
  updates ride ticket 005.
