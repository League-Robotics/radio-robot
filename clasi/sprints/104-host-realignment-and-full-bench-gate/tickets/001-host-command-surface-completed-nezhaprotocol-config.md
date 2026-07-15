---
id: '001'
title: "Host command surface completed — NezhaProtocol.config()"
status: open
use-cases:
- SUC-011
depends-on: []
github-issue: ''
issue: single-loop-firmware-p3-p7-continuation.md
completes_issue:
  single-loop-firmware-p3-p7-continuation.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host command surface completed — NezhaProtocol.config()

## Description

Sprint 103 shipped `twist()`/`stop()` host builders for the pruned
`CommandEnvelope` schema but left `ConfigDelta` — a schema-defined oneof
arm since 103-001 — without a host-side builder (103 Step 7 Open
Question 3 explicitly deferred whether it's even live-applied on the
firmware side). This ticket adds `NezhaProtocol.config()` so every arm
the wire schema defines has a host builder, and resolves the open
question about firmware-side behavior by testing against the actual
merged tree instead of assuming.

This ticket is ordered first in the sprint (no dependencies) because it
is purely additive — it can land cleanly before ticket 002's large
deletion sweep touches the same files.

## Acceptance Criteria

- [ ] `NezhaProtocol.config(**deltas)` builds and sends a `ConfigDelta`
      envelope using the pruned schema; matches the construction style of
      103's `twist()`/`stop()`.
- [ ] Ack for a sent `config` command is observed via the existing
      (103) ack-ring matcher — no new matching logic added here (ticket
      003 is where the matcher itself gets hardened/promoted).
- [ ] Firmware-side `config` dispatch behavior (live-apply vs.
      `ERR_UNIMPLEMENTED`) is confirmed by reading `main.cpp`'s actual
      dispatch switch against the merged 103 tree and recorded in this
      ticket's completion notes — do not assume either answer.
- [ ] If firmware dispatch is confirmed to be `ERR_UNIMPLEMENTED` (no live
      apply), `NezhaProtocol.config()` still ships as a builder — its
      test coverage asserts the envelope/ack round-trip, not config
      application (which would then be a future ticket's scope, not
      this one's).

## Testing

- **Existing tests to run**: `uv run python -m pytest tests/unit -k protocol`
  (baseline — expect many pre-existing failures from dead methods this
  ticket does not touch; ticket 002 cleans those up).
- **New tests to write**: a unit test constructing a `ConfigDelta`
  envelope via `NezhaProtocol.config()` and asserting the encoded bytes
  match a hand-built reference envelope; an ack-round-trip test using the
  existing (103) matcher, mirroring 103's own `twist()`/`stop()` test
  pattern.
- **Verification command**: `uv run python -m pytest
  tests/unit/test_protocol_config.py -v` (new file, name at
  implementation discretion).

## Implementation Plan

**Approach**: Mirror 103's `twist()`/`stop()` implementation pattern in
`host/robot_radio/robot/protocol.py` exactly — same envelope-construction
style, same reliance on the existing ack-ring matcher. Read `main.cpp`'s
dispatch switch first to settle the open question before writing tests
that assume an answer.

**Files to create/modify**:
- `host/robot_radio/robot/protocol.py` — add `config()` method.
- `tests/unit/test_protocol_config.py` (new) — encoding + ack-round-trip
  tests.

**Testing plan**: covered above.

**Documentation updates**: note the confirmed firmware dispatch behavior
(live-apply or stub) in this ticket's completion notes and, if it
resolves 103's Step 7 Open Question 3, add a one-line pointer from this
ticket back to that question so a future reader doesn't re-ask it.

## SUC-011: Host command surface completed — config arm + ack-ring ergonomics

Parent: `single-loop-firmware-p3-p7-continuation.md` (P5 remainder).

- **Actor**: Firmware/host engineer scripting the rig.
- **Preconditions**: Sprint 103's `NezhaProtocol.twist()`/`stop()` +
  ack-ring matcher exist; `ConfigDelta` is schema-defined but has no host
  builder.
- **Main Flow**: A host script calls `NezhaProtocol.config(**deltas)`; it
  constructs and sends a `ConfigDelta` envelope; the ack-ring matcher
  confirms receipt.
- **Postconditions**: Every `CommandEnvelope` oneof arm has a host-side
  builder.
- **Acceptance Criteria**: see above.
