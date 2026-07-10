---
id: '005'
title: BinaryChannel stream arm and StreamControl.binary wiring into periodic tick
status: done
use-cases:
- SUC-003
depends-on:
- '001'
- '002'
- '003'
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

- [x] A binary `stream{binary:true, period:N}` command produces periodic
      `ReplyEnvelope{tlm: Telemetry}` frames at the requested
      (floor-clamped) period, with `seq` shared/monotonic against the SAME
      counter (`bb.telemetrySeq`) text STREAM uses.
- [x] `stream{binary:false, ...}` or `period:0` behaves exactly like text
      STREAM's own on/off semantics.
- [x] `ReplyEnvelope{tlm}` frames carry `corr_id = 0` (unsolicited push),
      per `envelope.proto`'s own forward-looking doc comment.
- [x] The text STREAM/SNAP frame's own wire text stays byte-identical
      throughout this ticket (verifies ticket 003's own guarantee still
      holds once `stream` can actually toggle `telemetryBinary`).
- [x] Full sim suite (~469 tests) stays green; 095's differential codec
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

## Completion Notes

**Implementation**: `source/commands/binary_channel.cpp`'s `STREAM` arm
replaces the `ERR_UNIMPLEMENTED` stub. Decodes `msg::StreamControl{binary,
period}`; clamps `period` to a duplicated `kStreamFloorMs = 20` (mirroring
`handleStream()`'s own floor -- `telemetry_commands.cpp` keeps its constant
TU-local in an unnamed namespace, so this file hand-mirrors the value rather
than reaching across TUs for it, the same pattern `toSegment()` already
uses); sets `bb.telemetryPeriod`/`bb.telemetryChannel` (from
`static_cast<Rt::CommandRouter*>(routerCtx)->currentChannel()`, the same
`routerCtx` idiom every other arm in this file uses)/`bb.telemetryBinary`
unconditionally (even for `period:0`, matching `handleStream()`'s own
unconditional channel rebind); replies via `sendAck` (no bespoke reply
shape). Deliberately does NOT reproduce `handleStream()`'s old same-reply
immediate-first-frame concatenation (Open Question 5) -- the first frame
arrives one pass later via `tickTelemetry()`'s own
`!bb.telemetryHasLastEmit` trigger, uniformly for text and binary.
`corr_id=0` on push `ReplyEnvelope{tlm}` frames was already set by ticket
003's `telemetryEmitBinary()` (`telemetry_commands.cpp`); verified, no
change needed here.

**Tests**: extended `tests/sim/unit/test_binary_channel.py` -- removed
`stream` from the declared-only-arms `ERR_UNIMPLEMENTED` parametrization
(now only `pose`/`otos` remain stubs) and added a `stream` section mirroring
`test_telemetry_periodic_tick.py`'s own sim-harness pattern
(`sim.peek_reply_store()`, never `sim.command()`/`send()` to OBSERVE
periodic output, since both reset the target channel's `ReplyStore` before
routing): (1) the ack carries no immediate frame; (2) `stream{binary:true,
period:50}` + >=200ms of ticking yields >=3 binary frames with `corr_id=0`
and strictly increasing `seq=`; (3) toggling `binary:false` (period
unchanged) reverts emission to plain-text `TLM` lines with the SAME shared
`seq=` counter continuing (not resetting) across the transition, and the
text frame's key set is unchanged (`t`/`mode`/`seq` present) -- proving
003's text-format guarantee still holds once `stream` can actually toggle
`bb.telemetryBinary` at runtime; (4) `stream{..., period:0}` stops all
periodic emission outright, independent of `binary`; (5) `SNAP` still works
standalone (plain text, one shot) with a binary stream active.

**Verification**: `just build` (ARM) -- FLASH 326052 B / 364 KB = 87.48%,
RAM 98.33% (expected per project convention, not a regression signal).
`just build-sim` clean. `tests/sim/unit/test_binary_channel.py`: 45 passed
(40 pre-existing/updated + 5 new). Full `tests/sim`: 496 passed (up from
492 pre-ticket, +4 net: +5 new stream tests, -1 removed `stream` param from
the declared-only-arms parametrization). `test_wire_differential.py` (132)
and `test_telemetry_periodic_tick.py` (4) both unregressed, re-run
standalone alongside `test_binary_channel.py`: 181 passed. Noted but NOT
this ticket's regression: `test_move_streaming_chains_at_speed`
(`test_bare_loop_move_and_tlm.py`) fails deterministically when run in
ISOLATION (confirmed pre-existing via `git stash` back to the 096-004
tree, same 3/3 deterministic isolated failure there) but passes as part of
the full `tests/sim` suite run both before and after this ticket's changes
-- unrelated to `stream`/telemetry, not investigated further here.
