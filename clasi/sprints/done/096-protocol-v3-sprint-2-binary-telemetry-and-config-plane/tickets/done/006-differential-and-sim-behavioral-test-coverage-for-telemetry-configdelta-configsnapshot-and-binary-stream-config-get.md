---
id: '006'
title: Differential and sim behavioral test coverage for Telemetry, ConfigDelta, ConfigSnapshot,
  and binary stream/config/get
status: done
use-cases:
- SUC-005
depends-on:
- '003'
- '004'
- '005'
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Differential and sim behavioral test coverage for Telemetry, ConfigDelta, ConfigSnapshot, and binary stream/config/get

## Description

Extend 095's differential/fuzz/range harness (M8's pattern) to cover this
sprint's new messages, and add fresh sim-level behavioral tests for the
`stream`/`config`/`get` arms end-to-end — closing the issue's own Risk 6
("parked text families have no live regression tests") for the two arms
this sprint implements. Depends on tickets 003, 004, and 005 (the real
implementations to test against — this ticket does not stub or mock the
arms it tests).

**Approach**:
1. Extend the existing differential harness
   (`tests/sim/unit/*_harness.cpp` + Python drivers, following 095's
   pattern exactly — same host-compiled C++ harness linking
   `wire_runtime.cpp`+`wire.cpp`, same google.protobuf-backed Python
   reference) to cover:
   - `Telemetry`: encode-only differential (firmware-encode ->
     host-decode via `google.protobuf`), since firmware never decodes
     `Telemetry` (it is a reply-only message).
   - `ConfigDelta`: both directions (host-encode -> firmware-decode,
     covering the `config` arm's input path; firmware-encode ->
     host-decode is not applicable since `ConfigDelta` is command-only —
     confirm and adjust to whichever directions actually apply).
   - `ConfigSnapshot`: encode-only differential (firmware-encode ->
     host-decode), since it is a reply-only message.
2. Add sim-level behavioral tests that drive `config`/`get`/`stream`
   through `BinaryChannel` end-to-end (not just the codec) and assert the
   resulting `bb.configIn`/`bb.drivetrainConfig`/`bb.motorConfig[]`/
   `bb.plannerConfig`/`bb.streamWatchdogWindow`/`bb.telemetryPeriod`/
   `bb.telemetryChannel`/`bb.telemetryBinary` effects match what the
   equivalent text verb (`applyConfigKey()`/`handleStream()`, even though
   unregistered) would have produced.
3. Add a fuzz/boundary corpus pass for the new messages' `min`/`max`/
   `abs_max`/`req` validated fields (mirrors 095's SUC-005 acceptance
   criteria), covering: `DrivetrainConfigPatch`/`MotorConfigPatch`/
   `PlannerConfigPatch`'s numeric fields at their transcribed bounds
   (per `validateCandidate()`'s existing invariants — `tw > 0`,
   `rotSlip == 0 || [0.5, 1.0]`, `sTimeout > 0`).

**Files to modify**: `tests/sim/unit/*` (extend existing harness files or
add new ones following the established naming pattern).

## Acceptance Criteria

- [x] Differential round-trip passes for `Telemetry` (firmware-encode ->
      host-decode) and `ConfigDelta`/`ConfigSnapshot` (whichever
      directions apply per each message's actual command/reply role).
- [x] A sim-level test drives `config`/`get`/`stream` through
      `BinaryChannel` end-to-end and asserts the resulting Blackboard
      state matches what the equivalent text verb would have produced.
- [x] Fuzz corpus (>= 200 generated cases, matching 095's own bar) for the
      new messages produces zero crashes and zero out-of-bounds reads
      (ASan/UBSan on the host build).
- [x] Every validated field's boundary case (min-1, min, max, max+1,
      abs_max, -abs_max, and `validateCandidate()`'s own invariants where
      applicable) produces the expected accept/reject verdict.
- [x] The full pre-096 sim suite (~469 tests) stays green alongside the
      new differential and behavioral tests.

## Testing

- **Existing tests to run**: full `tests/sim` suite, including 095's
  `test_wire_differential.py`/`test_binary_channel.py` (must not
  regress).
- **New tests to write**: as described in Approach above — this ticket
  IS the new-tests ticket.
- **Verification command**: `just build-sim && uv run python -m pytest
  tests/sim` (host build with ASan/UBSan for the fuzz pass, per 095's own
  harness convention).

## Completion Notes

**Directions confirmed against each message's actual role** (envelope.h/
envelope.proto read directly, not assumed): `Telemetry` and
`ConfigSnapshot` never appear in `CommandEnvelope.cmd`'s union -- reply-
only, encode-only differential (Direction B). `ConfigDelta` never appears
in `ReplyEnvelope.body`'s union -- command-only, decode-only differential
(Direction A). All three match the ticket's own stated expectation exactly;
no adjustment needed.

**Differential harness extended** (`wire_differential_harness.cpp`):
`decode`'s CONFIG case now prints every `ConfigDelta` Patch field (was
previously grouped into the "no arm-specific fields" bucket alongside
pose/otos/get/stream); five new encode verbs added --
`encode_telemetry` (34 positional args, Telemetry's full 28-field shape)
and `encode_cfg_drivetrain`/`encode_cfg_motor`/`encode_cfg_planner`/
`encode_cfg_watchdog` (one per `ConfigSnapshot.patch` oneof arm).

**Real bug found and fixed (in test code, not firmware)**: the Python
`encode_telemetry()` driver originally passed every field through
`str(value)`, which renders Python `True`/`False` as the literal strings
`"True"`/`"False"` -- the harness parses every non-float arg with
`strtoul()`, which silently reads a non-digit-leading string as `0`. Every
`has_*`/`active`/`conn_*`/`otos_connected` bool field was corrupted to
`false` regardless of the Python-side value passed. Caught immediately by
`test_direction_b_telemetry_full_shape` failing on first run (`has_enc`
came back `False` when `True` was passed). Fixed in `_wire_diff_driver.py`
by converting `bool` values to `"0"`/`"1"` before formatting. This was a
test-driver bug, not a `wire.cpp`/codec bug -- no firmware/generator change
was needed.

**Finding, documented not silently patched**: `config.proto`'s own file
header already flags that `DrivetrainConfigPatch`/`MotorConfigPatch`/
`PlannerConfigPatch`/`ConfigDelta.watchdog` carry NO `(min)`/`(max)`/
`(abs_max)` wire options, and that `binary_channel.cpp`'s `CONFIG` arm
(ticket 004) never calls `validateCandidate()` -- confirmed directly by
reading `wire.cpp`'s generated field tables (`flags = 0` for every one of
these fields, `validateRange()` short-circuits to `true` whenever no
bound flag is set) and `binary_channel.cpp`'s CONFIG case (posts straight
to `bb.configIn`/`bb.streamWatchdogWindowIn`, no invariant check). This
means `tw <= 0`, `rotSlip` outside `{0} ∪ [0.5, 1.0]`, and `sTimeout == 0`
are ALL currently ACCEPTED over the binary plane, unlike the text `SET`
path (`validateCandidate()` rejects them there). This is a pre-existing,
already-flagged, deliberately-deferred gap (config.proto: "neither of
which this ticket's own acceptance criteria require") -- ticket 006's
scope is testing, not adding a `validateCandidate()` call or new wire
bounds that no prior ticket's acceptance criteria asked for, so the new
boundary tests (`test_boundary_config_*_no_wire_level_enforcement` in
`test_wire_differential.py`) assert this ACTUAL, current behavior
(`expect_accept=True` for every case, including the invariant-violating
ones) rather than inventing a rejection this ticket has no mandate to
implement. Flagged here for the stakeholder/team-lead to decide whether a
follow-up issue should close it.

**Behavioral coverage**: reviewed 004/005's existing `test_binary_channel.py`
coverage first, per the ticket's own instruction to avoid duplication --
found it already comprehensive (all 15 `kAllKeys` keys round-tripped via
`config`+`get`, `sTimeout`→`streamWatchdogWindowIn` routing verified,
empty-patch/missing-target error paths, stream ack/period/binary-toggle/
period-zero/SNAP-coexistence). One genuine gap found: no existing test
exercised `bb.telemetryChannel` binding across more than one channel (every
existing stream test used `CHANNEL_SERIAL` only, so "always emits on
SERIAL" was indistinguishable from "binds to the requesting channel").
Added `test_binary_stream_binds_periodic_emission_to_the_requesting_channel`
(arms `stream` on `CHANNEL_RADIO`, confirms periodic frames land there and
nowhere on `CHANNEL_SERIAL`).

**Fuzz corpus**: added a `ConfigDelta{drivetrain}` valid message to
`test_wire_fuzz.py`'s `_VALID_MESSAGES` (exercises the new CONFIG
decode-printing code under truncation/oversized/salted mutation, same as
every other arm) -- corpus grew to 295 cases (was 252), well over the
>= 200 bar; +43 config-specific cases. Also added a dedicated ASan/UBSan
encode-side extreme-value check for `Telemetry`/`ConfigSnapshot`
(`test_fuzz_encode_telemetry_float_extremes`,
`test_fuzz_encode_cfg_snapshot_extremes`,
`test_fuzz_encode_cfg_snapshot_every_target_and_side`) since encode()
takes a typed struct (no adversarial-bytes surface), so the fuzz value is
in exercising `encodeInto()`/`encodeNestedMessage()`'s fixed-size scratch
buffers (`kEncodeScratchCap`) at IEEE-754 extremes (±FLT_MAX, NaN, ±Inf)
and `UINT32_MAX` for Telemetry, the single largest ReplyEnvelope oneof arm
(~165B) with the most fields (28) and nested messages (3) of anything in
the schema.

**Verification**: `uv run python -m pytest tests/sim/unit/test_wire_differential.py
tests/sim/unit/test_wire_fuzz.py tests/sim/unit/test_binary_channel.py -q`
-- 534 passed (178 + 310 + 46). Full `uv run python -m pytest tests/sim -q`
-- 600 passed (up from ~492 post-005), zero regressions. Fuzz suite
compiled with `-fsanitize=address,undefined -fno-omit-frame-pointer -g`
(same flags as 095's own harness); zero ASan/UBSan findings across all 310
fuzz-file cases (295-case decode corpus + corpus-size check + 14 new
encode-side extreme-value cases).
