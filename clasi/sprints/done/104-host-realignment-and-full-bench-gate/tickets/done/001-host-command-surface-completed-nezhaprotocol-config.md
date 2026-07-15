---
id: '001'
title: "Host command surface completed \u2014 NezhaProtocol.config()"
status: done
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

- [x] `NezhaProtocol.config(**deltas)` builds and sends a `ConfigDelta`
      envelope using the pruned schema; matches the construction style of
      103's `twist()`/`stop()`.
- [x] Ack for a sent `config` command is observed via the existing
      (103) ack-ring matcher — no new matching logic added here (ticket
      003 is where the matcher itself gets hardened/promoted).
- [x] Firmware-side `config` dispatch behavior (live-apply vs.
      `ERR_UNIMPLEMENTED`) is confirmed by reading `main.cpp`'s actual
      dispatch switch against the merged 103 tree and recorded in this
      ticket's completion notes — do not assume either answer.
- [x] If firmware dispatch is confirmed to be `ERR_UNIMPLEMENTED` (no live
      apply), `NezhaProtocol.config()` still ships as a builder — its
      test coverage asserts the envelope/ack round-trip, not config
      application (which would then be a future ticket's scope, not
      this one's).

## Completion Notes

**Firmware dispatch behavior (resolves 103's Step 7 Open Question 3):**
read `source/main.cpp`'s main-loop dispatch switch directly against the
merged 103 tree (the `runAndWait(kSettle, ...)` block, `CmdKind::CONFIG`
case, around line 275). Confirmed: the switch decodes a `ConfigDelta`
successfully but does **not** apply it — it unconditionally acks
`ACK_STATUS_ERR` / `ERR_UNIMPLEMENTED` ("ConfigDelta runtime application
deferred this sprint"). No live-apply path exists yet. This answers
Open Question 3 definitively: **stub, not live-apply.** A future ticket
(not this one) is required to wire `bb`-equivalent state for the CONFIG
arm before `config()` has any actuation effect.

**API added** — `host/robot_radio/robot/protocol.py`,
`NezhaProtocol.config(self, **deltas: Any) -> int`:
- Reuses the existing flat wire-key vocabulary `set_config()` already
  curates (`_DRIVETRAIN_KEYS`/`_MOTOR_PID_KEYS`/`_PLANNER_KEYS`/`ml`/`mr`/
  `sTimeout`, i.e. `_ALL_SET_KEYS`) rather than inventing a second
  vocabulary.
- Unlike `set_config()` (which fans a multi-target kwargs dict into
  multiple round trips, one per touched `ConfigDelta.patch` oneof arm),
  `config()` builds and sends exactly ONE `CommandEnvelope` carrying
  exactly ONE `ConfigDelta` — matching `twist()`/`stop()`'s "one call,
  one envelope, one corr_id" shape. Kwargs spanning more than one
  `ConfigDelta.patch` target (e.g. `tw=` + `pid.kp=`) raise `ValueError`,
  as does an empty or unknown-key call.
- `pid.*` keys and `ml`/`mr` may be combined in one call (both target the
  same `MotorConfigPatch` oneof arm); with no `ml`/`mr` present, `side`
  defaults to `LEFT` (meaningless for `pid.*` fields per
  `config.proto`'s own comment, but the wire message still needs some
  value), mirroring `set_config()`'s own `motor_left_patch` branch.
- Fire-and-poll, same as `twist()`/`stop()`: sends via
  `SerialConnection.send_envelope_fast()` and returns the assigned
  `corr_id` immediately; the caller passes it to the existing (103-009)
  `wait_for_ack()` to observe the outcome riding the ack ring. No new
  matching logic was added to `wait_for_ack()` — its docstring was
  updated only to name `config()` alongside `twist()`/`stop()` instead of
  referring to it as a hypothetical "future fire-and-poll command".

**Tests** — new file `tests/unit/test_protocol_config.py` (17 tests, all
passing): envelope-construction tests for each `ConfigDelta.patch` target
(drivetrain/motor-ml/motor-mr/motor-pid/motor-combined/planner/watchdog),
each asserting the built envelope's `SerializeToString()` bytes match a
hand-built reference envelope; input-validation tests (empty kwargs,
unknown key, multi-target kwargs all raise `ValueError` and send nothing);
and ack-round-trip tests reusing the 103-009 `wait_for_ack()` matcher
(first-match, ring re-delivery, timeout), scripting an
`ERR_UNIMPLEMENTED` ack to match the confirmed real firmware outcome
above — no test asserts config *application*, per acceptance criterion 4.

**Test results:**
- `tests/unit/test_protocol_config.py`: 17/17 passed.
- `uv run python -m pytest tests/unit -k protocol`: 38 passed, 35 failed —
  all 35 failures are pre-existing dead-method breakage from 103's
  `CommandEnvelope`/`ReplyEnvelope` prune (`pose_fix`/`ping`/`echo`/
  `get_config_binary`/`drive`/`timed`/`distance`/`get_config`/`stream`/
  `snap` reference oneof arms — e.g. `pose_fix`, `get`, `cfg` — the
  regenerated `envelope_pb2` no longer declares), confirmed unrelated to
  this ticket's change (verified against `HEAD:host/robot_radio/robot/
  protocol.py`, which lacks `config()` entirely and already fails these
  same tests the same way). Ticket 002's scope, not touched here.
- `tests/sim`: 339/339 passed — stayed green, no regressions.
- Full default `uv run python -m pytest` (`testpaths = ["tests/sim",
  "tests/unit"]`): 653 passed, 112 failed, 5 errors — the failures/errors
  are the same pre-existing dead-method set (`test_protocol_binary_client.py`,
  `test_protocol_pose_fix.py`, `test_serial_conn_binary_plane.py`,
  `test_bridge_pty_e2e.py`), not new breakage from this ticket.

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
