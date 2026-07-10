---
id: '003'
title: Telemetry frame assembly extension and binary formatter (tlm_frame.h/.cpp)
status: open
use-cases: [SUC-003]
depends-on: ['001', '002']
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

- [ ] `TlmFrameInput` gains `acc`/`active`/`conn`/`glitch`/`ts`, sourced
      identically to `handleTlm()`'s own computation (same Blackboard
      cells, same values).
- [ ] A new binary formatter populates `msg::Telemetry` from
      `TlmFrameInput`, pure and stateless (same shape as `buildTlmFrame()`).
- [ ] Text `buildTlmFrame()`'s output is byte-identical before and after
      this ticket for every pre-existing field (verified by a
      before/after comparison test or an existing test's continued pass).
- [ ] `tickTelemetry()` calls the binary formatter when
      `bb.telemetryBinary` is true, the text formatter otherwise.
- [ ] The generated `kMaxEncodedSize` static_assert for
      `ReplyEnvelope{Telemetry}` passes, with the final field list (as
      designed, or trimmed per Decision 6's order) recorded in completion
      notes.
- [ ] Full sim suite (~469 tests) stays green; 095's differential codec
      gate (`test_wire_differential.py`) does not regress.

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
