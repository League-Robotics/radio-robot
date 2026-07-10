---
id: '006'
title: Differential and sim behavioral test coverage for Telemetry, ConfigDelta, ConfigSnapshot,
  and binary stream/config/get
status: in-progress
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

- [ ] Differential round-trip passes for `Telemetry` (firmware-encode ->
      host-decode) and `ConfigDelta`/`ConfigSnapshot` (whichever
      directions apply per each message's actual command/reply role).
- [ ] A sim-level test drives `config`/`get`/`stream` through
      `BinaryChannel` end-to-end and asserts the resulting Blackboard
      state matches what the equivalent text verb would have produced.
- [ ] Fuzz corpus (>= 200 generated cases, matching 095's own bar) for the
      new messages produces zero crashes and zero out-of-bounds reads
      (ASan/UBSan on the host build).
- [ ] Every validated field's boundary case (min-1, min, max, max+1,
      abs_max, -abs_max, and `validateCandidate()`'s own invariants where
      applicable) produces the expected accept/reject verdict.
- [ ] The full pre-096 sim suite (~469 tests) stays green alongside the
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
