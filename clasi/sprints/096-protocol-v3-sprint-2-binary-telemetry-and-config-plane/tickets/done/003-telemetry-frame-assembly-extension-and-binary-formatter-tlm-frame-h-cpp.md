---
id: '003'
title: Telemetry frame assembly extension and binary formatter (tlm_frame.h/.cpp)
status: done
use-cases:
- SUC-003
depends-on:
- '001'
- '002'
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Telemetry frame assembly extension and binary formatter (tlm_frame.h/.cpp)

## Description

Extend the existing pure `tick()`/format split (`source/telemetry/
tlm_frame.{h,cpp}`, M3) with the bench-diagnostic fields, and add a binary
formatter alongside the existing text one. Depends on ticket 001 (the
`Telemetry` message must exist) and ticket 002 (`tickTelemetry()` and
`bb.telemetryBinary` must exist as the call site this ticket's formatter
plugs into).

**Approach**:
1. Extend `Telemetry::TlmFrameInput` (`tlm_frame.h`) with `acc`
   (`accLeft`/`accRight`), `active`, `conn` (`connLeft`/`connRight`),
   `glitch` (`glitchLeft`/`glitchRight`), and `ts` (`tsLeft`/`tsRight`) —
   sourced in `Telemetry::tick()` (`tlm_frame.cpp`) EXACTLY the way
   `handleTlm()` (`motion_commands.cpp`) already computes them
   (`bb.drivetrain.acc[]`/`.busy`, `bb.motors[].connected`/
   `.enc_glitch_count`/`.sampled_at`) — do not re-derive, transcribe. Do
   NOT touch `motion_commands.cpp`'s `handleTlm()` itself; it keeps its
   own separate text wire format unchanged.
2. Add a new binary formatter (e.g. `Telemetry::buildTelemetryMessage(
   msg::Telemetry&, const TlmFrameInput&)`) alongside the existing
   `buildTlmFrame()` — pure, stateless, same input struct, populates the
   generated `msg::Telemetry` POD instead of a text buffer. `buildTlmFrame()`
   itself must NOT change its output for any existing field — this is a
   hard regression gate (SUC-003's "byte-identical before and after"
   acceptance criterion).
3. Wire `tickTelemetry()` (ticket 002, `telemetry_commands.cpp`) to call
   the new binary formatter + `msg::wire::encode` + armor + send when
   `bb.telemetryBinary` is true, and the existing text path otherwise.
4. Verify the REAL (not Step-1-estimated) worst-case `kMaxEncodedSize` for
   `ReplyEnvelope{Telemetry}` — the generated `static_assert` is the
   authority (095 Decision 6's lesson, reapplied — architecture-update.md
   Open Question 3). If it exceeds 186 bytes, apply Decision 6's
   documented trim order: drop `encpose` first (reconstructable from
   `enc`+`twist`), then `otos`+`otosconn` (already diagnostic-only per its
   092-002 doc comment). NEVER trim `enc`/`vel`/`cmd`/`active`/`conn`/
   `glitch` — these are the bench gate's core signals. Record whichever
   outcome (fits as-is, or trimmed-and-what) in this ticket's completion
   notes.

**Files to modify**: `source/telemetry/tlm_frame.{h,cpp}`,
`source/commands/telemetry_commands.cpp` (the binary-branch call site,
completing what ticket 002 stubbed).

## Acceptance Criteria

- [x] `TlmFrameInput` gains `acc`/`active`/`conn`/`glitch`/`ts`, sourced
      identically to `handleTlm()`'s own computation (same Blackboard
      cells, same values).
- [x] A new binary formatter populates `msg::Telemetry` from
      `TlmFrameInput`, pure and stateless (same shape as `buildTlmFrame()`).
- [x] Text `buildTlmFrame()`'s output is byte-identical before and after
      this ticket for every pre-existing field (verified by a
      before/after comparison test or an existing test's continued pass).
- [x] `tickTelemetry()` calls the binary formatter when
      `bb.telemetryBinary` is true, the text formatter otherwise.
- [x] The generated `kMaxEncodedSize` static_assert for
      `ReplyEnvelope{Telemetry}` passes, with the final field list (as
      designed, or trimmed per Decision 6's order) recorded in completion
      notes.
- [x] Full sim suite (~469 tests) stays green; 095's differential codec
      gate (`test_wire_differential.py`) does not regress.

## Completion Notes

**`TlmFrameInput` extension** (`source/telemetry/tlm_frame.h`): added
`driveMode` (`msg::DriveMode`, the raw enum `bb.planner.mode` -- needed
because `mode` (char) is already the lossy text-mapped value and
`protos/telemetry.proto`'s own doc comment requires the binary formatter to
carry the enum itself) plus the five bench-diagnostic groups: `accLeft`/
`accRight`, `active`, `connLeft`/`connRight`, `glitchLeft`/`glitchRight`,
`tsLeft`/`tsRight`. `Telemetry::tick()` (`tlm_frame.cpp`) sources every one
of them EXACTLY as `handleTlm()` (`motion_commands.cpp`) computes them:
`bb.drivetrain.acc()[0]/[1]` (acc), `bb.drivetrain.busy` (active),
`bb.motors[0]`/`bb.motors[1]` DIRECTLY -- the same hardcoded bound-pair
indices `handleTlm()` itself uses, deliberately NOT the
`bb.drivetrainConfig`-derived `leftIdx`/`rightIdx` that `enc=`/`vel=` use --
for `.connected`/`.enc_glitch_count`/`.sampled_at` (conn/glitch/ts).

**Binary formatter**: `Telemetry::buildTelemetryMessage(msg::Telemetry& out,
const TlmFrameInput& in)`, declared/defined alongside `buildTlmFrame()` in
`tlm_frame.{h,cpp}`. Pure/stateless: always resets `out` to a fresh
`msg::Telemetry{}` first, then copies every field 1:1 (including all five
`has_*` presence flags) except `encpose`/`hasEncPose`, which have no
counterpart on `msg::Telemetry` (already trimmed by ticket 001/Decision 6).

**Text output regression proof**: `buildTlmFrame()`'s own code is
byte-for-byte untouched -- it does not read any of the new `TlmFrameInput`
fields at all. `tests/sim/unit/tlm_frame_harness.cpp`'s `baselineInput()`
now also populates the bench-diagnostic fields with distinctive non-default
values, and `scenarioAllFieldsPresentExactMatch()` asserts the exact same
formatted string as before this ticket, plus explicit absence checks that
`acc=`/`active=`/`conn=`/`glitch=`/`ts=` never appear in the text line.

**`tickTelemetry()` wiring** (`telemetry_commands.cpp`): a new
`.cpp`-local `telemetryEmitBinary()` (peer to the existing `telemetryEmit()`)
does the same `Telemetry::tick()`-then-`bb.telemetrySeq++` flow, then
builds `msg::ReplyEnvelope{corr_id: 0, body_kind: TLM, body.tlm: <via
buildTelemetryMessage()>}`, encodes with `msg::wire::encode`, and armors
with the same `"*B" + WireRuntime::base64Encode` scheme
`binary_channel.cpp`'s `sendReply()` uses. `corr_id = 0` because this is
unsolicited push telemetry (no triggering `CommandEnvelope`), per
`envelope.proto`'s own doc comment and ticket 005's stated acceptance
criterion. `tickTelemetry()` now branches: `bb.telemetryBinary` ?
`telemetryEmitBinary()` : `telemetryEmit()`. `handleSnap()` is untouched --
SNAP always uses the text `telemetryEmit()`, regardless of
`bb.telemetryBinary` (per this file's own pre-existing header note that
only `bb.telemetrySeq` is shared between STREAM and SNAP). Nothing sets
`bb.telemetryBinary` true yet (that's ticket 005), so this ticket's own
observable behavior is still unconditionally text.

**`kMaxEncodedSize` static_assert**: unchanged by this ticket (no
`protos/*.proto` edits) -- `kReplyEnvelopeMaxEncodedSize = 171 <= 186`
holds (`source/messages/wire.h`, regenerated on every `just build`/`just
build-sim`). Final `msg::Telemetry` field list (unchanged from ticket 001,
confirmed still current): `now`, `mode`, `seq`, `has_enc`/`enc_left`/
`enc_right`, `has_vel`/`vel_left`/`vel_right`, `has_cmd_vel`/
`cmd_vel_left`/`cmd_vel_right`, `has_pose`/`pose`, `has_otos`/`otos`/
`otos_connected`, `has_twist`/`twist`, `acc_left`/`acc_right`, `active`,
`conn_left`/`conn_right`, `glitch_left`/`glitch_right`, `ts_left`/
`ts_right` -- `encpose`/`has_enc_pose` remain trimmed (001's first trim
step); the second trim step (`otos`/`otosconn`) was not needed.

**Tests**: `tests/sim/unit/tlm_frame_harness.cpp` extended with 4 new
scenarios (`scenarioBuildTelemetryMessageAllFieldsPresent`,
`...PresenceFlagsIndependent`, `...ResetsStaleState`, `...Deterministic`)
plus bench-diagnostic assertions folded into the existing
`scenarioAllFieldsPresentExactMatch()` (text-unchanged proof) and
`scenarioTickAssemblesFromBareBlackboard()` (tick()-sourcing proof against
a bare `Rt::Blackboard`). All scenarios exercised directly (host compile)
and via `tests/sim/unit/test_tlm_frame.py`'s existing pytest wrapper --
still one pytest-collected test (the compile-and-run harness pattern), now
covering 17 internal scenarios instead of 13.

**Verification**: `just build` (ARM) succeeded (FLASH 87.01%, RAM 98.33% --
RAM is always near-full on this build per project convention, not a
regression signal). `just build-sim` succeeded; `kMaxEncodedSize` report
unchanged (`tlm=165B`, `ReplyEnvelope total=171B`). `uv run python -m
pytest tests/sim -q` -- 473 passed, 0 failed. `test_wire_differential.py`
(095's differential codec gate) and `test_telemetry_periodic_tick.py`
(002's periodic-tick tests) both unregressed (explicitly re-verified via
`-k "wire_differential or telemetry_periodic_tick or tlm_frame"`, 137
passed).

## Testing

- **Existing tests to run**: full `tests/sim` suite; any existing
  `tlm_frame`-level unit test (`tests/sim/unit/tlm_frame_harness.cpp` and
  its Python driver, per `tlm_frame.h`'s own doc comment referencing this
  harness).
- **New tests to write**: a unit test asserting `buildTlmFrame()`'s text
  output is unchanged for a representative `TlmFrameInput`; a unit test
  for the new binary formatter's field-for-field correctness against a
  hand-constructed `msg::Telemetry`.
- **Verification command**: `just build-sim && uv run python -m pytest
  tests/sim`
