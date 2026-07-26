---
id: 008
title: 'Packed telemetry field encoding: sint32/zigzag, generated (scale), packed
  acks, bound fixes, and the position-rebaseline policy'
status: done
use-cases:
- SUC-005
- SUC-006
depends-on:
- '001'
- '007'
github-issue: ''
issue: protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Packed telemetry field encoding: sint32/zigzag, generated (scale), packed acks, bound fixes, and the position-rebaseline policy

## Description

Packed fixed-point telemetry encoding, PLUS the full position-rebaseline
policy end to end (sprint 124 architecture Decision 6, revised twice —
see that section for the complete reasoning; summarized below).

**Packed encoding**:
- `ScalarType::kSint32` + zigzag encoding — `zigzagEncode32()`/
  `zigzagDecode32()` already exist in `wire_runtime.h`, tested, unused;
  add the `wire.cpp` table-layer enumerator and generator mapping so
  `gen_messages.py` can emit a zigzag field.
- A generated `(scale)` field option in `options.proto` — schema carries
  the divisor, codegen emits the conversion on both firmware and host
  (NOT hand-honoured documentation like the existing `(units)` option —
  a generated scale mismatch is a build-time-checkable non-issue; a
  hand-honoured one is the classic silent fixed-point bug).
- Packed `acks`: `repeated uint32 acks = 14 [(max_count) = 4]`, packing
  `corr_id<<4 | err` — the engine's first real `FieldKind::kRepeatedScalar`
  use (declared "PACKED on the wire," currently unreached by the
  schema). Delete `ack_corr`/`ack_err` (fields 5/6) and
  `kFlagAckFresh` — ring membership already means "really acked."
- Bound fixes: `flags`'s `(max)` widened past 65535 to cover
  `kFlagFaultShapingDisabled` (bit 16); `ack_err`/`AckEntry.err`'s
  `(max)` sized to `ErrCode::ERR_NOT_CONFIGURED` (8), not 7.

**Position-rebaseline policy** (Decision 6 — read the sprint.md section
in full before implementing; this is the load-bearing part of this
ticket): pin `position`'s `(abs_max) = 32000` (±32 m, `sint32` at 1 mm
scale) against the quantified realistic-session analysis in Decision 6
(storage side — `NezhaMotor::encOffset_`, `int32_t` tenths-of-degree —
is already wide enough by many orders of magnitude and needs no change;
the wire's own ±32 m range is the tight constraint, and a multi-hour
characterization session, per sprint 126's own plans, can plausibly
exceed it). Add `positionEpoch` (new field, `RobotState::Wheel` — from
ticket 007 — and the wire `EncoderReading`). `RobotLoop` (this ticket's
scope includes the call site, even though the broader `cycle()`
restructure is ticket 009 — this specific addition is small and
self-contained) checks each cycle, after reading a wheel's position:
if `|position()|` is within a stated margin (e.g. 30,000 mm, a 2,000 mm
margin) of the wire bound, call the **existing, unmodified**
`Motor::rebaseline()` directly — **never** `Motor::resetPosition()` /
`MotorArmor::processResetIfPending()`'s staged dispatch, which can still
choose a real bus-touching hard reset at verified standstill. This is a
stakeholder-ruled, non-negotiable constraint: zero device commands, ever,
for this policy. `RobotLoop` owns and increments the `positionEpoch`
counter itself; `Devices::Motor`/`NezhaMotor`/`MotorArmor` get NO new
code from this ticket.

Defensive fallback only (not the expected path, given the 2,000 mm
margin dwarfs one cycle's worst-case travel): the fixed-point encode
step clamps `position` to `±32000` rather than wrapping, and sets an
observable fault/flag bit, if the bound is ever approached despite the
margin.

## Acceptance Criteria

- [x] `ScalarType::kSint32` + `zigzagEncode32/64` wired into `wire.cpp`
      (previously declared, unused) and the generator mapping.
- [x] `(scale)` option added to `options.proto`; conversion is
      GENERATED (both firmware and host), not hand-transcribed.
- [x] `sint32`/scale round-trip: negative values at each declared bound
      (`enc_left`/`enc_right`/`otos`/`pose`/`twist` per the issue's B3
      table) encode to the documented width — an explicit regression
      test against the `int32` sign-extension trap (a negative velocity
      costs 3 B, not 10).
- [x] Packed `acks` round-trips at ring depths 0-4, with `corr_id` at
      65535 and `err` at 8 (the real `ErrCode` ceiling).
- [x] `flags` with bit 16 set, and an ack carrying `ERR_NOT_CONFIGURED`,
      both survive a firmware-side decode without `ERR_RANGE`.
- [x] **Position-rebaseline policy**: `RobotLoop` calls the existing
      `Motor::rebaseline()` directly (never `resetPosition()`) when a
      wheel's position nears `(abs_max) = 32000`; `positionEpoch`
      increments observably each time; a sim run driving one wheel's
      cumulative signed travel past 30,000 mm over an extended session
      shows `position` never silently clips and `positionEpoch`
      increments at the expected point.
- [x] `Devices::Motor`/`NezhaMotor`/`MotorArmor` are unmodified by this
      ticket — `git diff` on those three files is empty (grep/diff
      enforceable).
- [x] Defensive fallback: if `position` is ever computed beyond `±32000`
      despite the margin (test by forcing the condition), the encode
      step clamps rather than wraps and sets an observable flag —
      never silent wraparound.
- [x] Regenerated `kReplyEnvelopeMaxEncodedSize` ≤ 130 B.

## Testing

- **Existing tests to run**: `wire.cpp` unit tests, existing
  `app_telemetry_harness.cpp` bound tests.
- **New tests to write**: `sint32`/scale round-trip tests at declared
  bounds; packed-`acks` ring tests at depths 0-4; bound-violation decode
  tests (`flags` bit 16, `ack_err`=8); the position-rebaseline sim test
  (drive cumulative signed position past the margin, assert rebaseline
  fires, `position` stays bounded, `positionEpoch` increments); the
  `git diff --stat` check on `Devices::Motor`/`NezhaMotor`/`MotorArmor`
  files.
- **Verification command**: `uv run pytest` plus the C++ sim-tests
  build.
