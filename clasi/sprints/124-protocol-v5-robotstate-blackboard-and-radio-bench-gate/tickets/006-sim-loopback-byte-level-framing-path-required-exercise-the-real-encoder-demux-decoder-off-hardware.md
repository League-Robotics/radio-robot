---
id: '006'
title: 'Sim/loopback byte-level framing path (REQUIRED): exercise the real encoder/demux/decoder
  off-hardware'
status: open
use-cases: [SUC-003]
depends-on: ['005']
github-issue: ''
issue:
- telemetry-physical-layer-corruption-and-move-ack-observability.md
- protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sim/loopback byte-level framing path (REQUIRED): exercise the real encoder/demux/decoder off-hardware

## Description

**REQUIRED, non-negotiable** — the second of the stakeholder's two
structural fixes. Sim did not catch either of 123's hardware-only wire
bugs (the 0x0A corruption, the move-enqueue-ack gap) because
`sim_ctypes` bypasses the real encoder/demux/decoder entirely and passes
envelopes directly. Build a test path that serializes a command/reply
through the REAL wire codec (the same one ticket 005 just cut over) and
feeds the resulting bytes back through the REAL demux/decoder, entirely
off-hardware.

This is additive to, not a replacement for, `sim_ctypes`'s existing
envelope-passing tests — those stay as they are; this ticket adds the
byte-level path they don't cover.

## Acceptance Criteria

- [ ] The sim/loopback path is distinct from and additional to
      `sim_ctypes`'s existing envelope tests — it does not replace them.
- [ ] It actually invokes the byte-level codec (encode → COBS → decode),
      not a stub or shortcut around it — verify by reading the
      implementation, not just its test names.
- [ ] A deliberately-reintroduced `0x0A`-in-binary-frame bug (make
      manually during review: skip the delimiter XOR) fails this test.
- [ ] Runs in CI without hardware present.

## Testing

- **Existing tests to run**: existing `sim_ctypes`/`sim_loop.py` tests
  (must remain green and unaffected).
- **New tests to write**: the loopback harness itself (C++ and/or
  Python) — this ticket's entire deliverable — plus the deliberate-bug
  sanity check above (run once during review, not left in the suite).
- **Verification command**: `uv run pytest` plus the C++ sim-tests build,
  with no hardware connected.
