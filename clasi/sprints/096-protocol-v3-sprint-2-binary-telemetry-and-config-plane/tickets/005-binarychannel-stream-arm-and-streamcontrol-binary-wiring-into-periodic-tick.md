---
id: '005'
title: BinaryChannel stream arm and StreamControl.binary wiring into periodic tick
status: open
use-cases: [SUC-003]
depends-on: ['001', '002', '003']
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# BinaryChannel stream arm and StreamControl.binary wiring into periodic tick

## Description

Implement the `stream` arm in `source/commands/binary_channel.cpp` (M4),
replacing its `ERR_UNIMPLEMENTED` stub, and complete the binary half of
the periodic-emission path tickets 002 (loop-owned tick scaffold) and 003
(binary formatter) built. Depends on all three: 001 (schema), 002 (the
tick + `bb.telemetryBinary` field + channel-resolution accessor), 003 (the
binary formatter `tickTelemetry()` calls when `telemetryBinary` is true).

**Approach**:
1. Decode `msg::StreamControl{binary, period}`.
2. Set `bb.telemetryPeriod`/`bb.telemetryChannel` (from `routerCtx`'s
   `currentChannel()`, the SAME `handlerCtx`-idiom every other arm uses,
   per 095 Decision 1)/`bb.telemetryBinary`, mirroring `handleStream()`'s
   own state-setting exactly (including the 20ms floor clamp,
   `docs/protocol-v2.md` §8) — minus the text schema/ArgList parsing
   layer, which the generated decoder already replaced.
3. Reply with `sendAck` (mirroring drive/segment/replace/stop's ack
   shape) — do not invent a new reply shape for `stream`.
4. Do NOT reproduce `handleStream()`'s old same-reply "immediate first
   frame" concatenation (Open Question 5, ticket 002's own note) — the
   first frame arrives one pass later via `tickTelemetry()`'s normal
   `!telemetryHasLastEmit` trigger, uniformly for text and binary.
5. `stream{binary:false, ...}` or `period:0` must behave exactly like
   text STREAM's own on/off semantics (disables periodic emission;
   `bb.telemetryBinary` still gets recorded for bookkeeping symmetry, but
   has no visible effect when period is 0).

**Files to modify**: `source/commands/binary_channel.cpp`.

## Acceptance Criteria

- [ ] A binary `stream{binary:true, period:N}` command produces periodic
      `ReplyEnvelope{tlm: Telemetry}` frames at the requested
      (floor-clamped) period, with `seq` shared/monotonic against the SAME
      counter (`bb.telemetrySeq`) text STREAM uses.
- [ ] `stream{binary:false, ...}` or `period:0` behaves exactly like text
      STREAM's own on/off semantics.
- [ ] `ReplyEnvelope{tlm}` frames carry `corr_id = 0` (unsolicited push),
      per `envelope.proto`'s own forward-looking doc comment.
- [ ] The text STREAM/SNAP frame's own wire text stays byte-identical
      throughout this ticket (verifies ticket 003's own guarantee still
      holds once `stream` can actually toggle `telemetryBinary`).
- [ ] Full sim suite (~469 tests) stays green; 095's differential codec
      gate (`test_wire_differential.py`) does not regress.

## Testing

- **Existing tests to run**: full `tests/sim` suite; `test_binary_channel.py`.
- **New tests to write**: a sim-level test sending `stream{binary:true,
  period:N}` and asserting periodic `ReplyEnvelope{tlm}` frames arrive
  with monotonic `seq`, at the sim-driver's simulated pass cadence; a test
  toggling `binary:false`/`period:0` and asserting emission stops.
- **Verification command**: `just build-sim && uv run python -m pytest
  tests/sim`

## Notes for the bench gate (post-sprint, team-lead-run)

This ticket's own dev-time tests only prove correctness on the sim
harness. The sprint's actual "text vs. binary TLM at matched rates, no
regression in `tlm_drop_rate()`" and "gamepad teleop on binary TLM with
Ack q/rem flow control" criteria are bench-only, run after all of this
sprint's tickets close, per `.claude/rules/hardware-bench-testing.md`.
