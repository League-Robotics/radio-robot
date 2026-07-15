---
id: '001'
title: 'Prune wire protocol: twist/config/stop + ack-ring telemetry'
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: single-loop-firmware-p3-p7-continuation.md
completes_issue:
  single-loop-firmware-p3-p7-continuation.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Prune wire protocol: twist/config/stop + ack-ring telemetry

## Description

Prune `protos/envelope.proto` and `protos/telemetry.proto` to the P4 wire
surface the whole rest of this sprint builds on: `CommandEnvelope` narrows
to `corr_id` + `oneof { Twist; ConfigDelta; Stop }`; `ReplyEnvelope`
narrows to `ok`/`err`/`tlm`; `Telemetry` gains a depth-3 ack ring
(`repeated AckEntry acks`) and `fault_bits`/`event_bits`; the
bench-diagnostic fields move to a new `TelemetrySecondary` message. This is
a re-derivation, not a merge, of sprint 102's spike-003 dry run
(`scratch/102-003-frame-budget`, never merged) — re-run the measurement for
real against this sprint's own field list rather than inheriting the
scratch branch's judgment calls silently.

This ticket is the sprint's foundation: every `source/app/` module
(tickets 004-007) and the host slice (ticket 009) compile against the
types this ticket generates. It has no dependencies and should land first.

## Acceptance Criteria

- [x] `CommandEnvelope.cmd` reduced to exactly `Twist twist`, `ConfigDelta
      config`, `Stop stop`; every other pre-102 arm (drive, segment,
      replace, pose_fix, otos, ping, echo, get, stream, id, hello, ver,
      help, plan_dump) removed; its field number `reserved`, not reused.
- [x] `ReplyEnvelope.body` reduced to exactly `Ack ok`, `Error err`,
      `Telemetry tlm`; every other arm removed and reserved.
- [x] `Twist{v_x, omega, duration}` defined (new message).
- [x] `Telemetry` gains `repeated AckEntry acks` (ring depth 3;
      `AckEntry{corr_id, status, err_code}`) and `fault_bits`/`event_bits`
      (`uint32`, bit layout decided and documented in this ticket's
      completion notes — spike-003 left it undefined).
- [x] `acc_left/acc_right/glitch_left/glitch_right/ts_left/ts_right/
      has_cmd_vel/cmd_vel_left/cmd_vel_right` move to a new
      `TelemetrySecondary` message.
- [x] `TelemetrySecondary`'s wire framing (a second `*B`-armored line vs. a
      `ReplyEnvelope` oneof arm) is decided and documented (Decision 3,
      architecture-update.md) — not left implicit.
- [x] `scripts/gen_messages.py` and `scripts/gen_pb2.py` run clean against
      the pruned protos; `source/messages/{envelope,telemetry,
      layout_checks,wire}.{h,cpp}` and host `envelope_pb2`/`telemetry_pb2`
      regenerated.
- [x] `wire.h`'s `kCommandEnvelopeMaxEncodedSize`/`kReplyEnvelopeMaxEncodedSize`
      static_asserts pass; both worst-case sizes and their margin against
      186B are recorded in the ticket's completion notes (mirroring
      spike-003's own reporting style).
- [x] `tests/sim/unit/wire_codec_harness.cpp`, `test_wire_codec.py`,
      `test_wire_differential.py`, `test_wire_fuzz.py` rewritten against
      the pruned schema; the protobuf differential oracle
      (`_wire_diff_driver.py`) still runs and passes.
- [x] No hardware needed for this ticket (schema/codegen/host-compile
      only).

## Implementation Plan

**Approach**: Work schema-first. Draft the pruned `.proto` files, run the
generators, read the real `kMaxEncodedSize` numbers `gen_messages.py`
prints (not spike-003's numbers — re-derive), adjust the field set if the
budget doesn't fit, then rewrite the wire test harnesses last (they can
only be written against a schema that has already stabilized).
`scratch/102-003-frame-budget` (commit `10985ec1d4`) is a reference
starting point, not something to `git cherry-pick` verbatim — confirm each
of its judgment calls (keeping `active`/`conn_left`/`conn_right` in the
primary frame; `ReplyEnvelope` staying a wrapper type) against this
ticket's own measurement before relying on them.

**Files to create/modify**:
- `protos/envelope.proto` — prune `CommandEnvelope`/`ReplyEnvelope`, add
  `Twist`.
- `protos/telemetry.proto` — add ack ring + fault/event bits; split out
  `TelemetrySecondary`.
- `source/messages/{envelope,telemetry,layout_checks,wire}.{h,cpp}` —
  regenerated, not hand-edited.
- Host `envelope_pb2.py`/`telemetry_pb2.py` (wherever `gen_pb2.py` emits
  them) — regenerated.
- `tests/sim/unit/wire_codec_harness.cpp`,
  `tests/sim/unit/wire_differential_harness.cpp`,
  `tests/sim/unit/test_wire_codec.py`,
  `tests/sim/unit/test_wire_differential.py`,
  `tests/sim/unit/test_wire_fuzz.py` — rewritten against the pruned arm
  set.

**Testing plan**:
- Existing tests to run: `tests/sim/unit/test_wire_runtime.py` (armor/
  base64 primitives — unchanged, should stay green untouched).
- New/rewritten tests: the four wire test files above, targeting
  `twist`/`config`/`stop`/`tlm`/`ok`/`err` only.
- Verification command: `uv run python -m pytest tests/sim/unit/test_wire_codec.py
  tests/sim/unit/test_wire_differential.py tests/sim/unit/test_wire_fuzz.py
  tests/sim/unit/test_wire_runtime.py`; plus a direct compile check
  (`c++ -std=c++20 -Wall -Wextra -I source -c source/messages/wire.cpp`)
  matching spike-003's own verification method.

**Documentation updates**: record the final worst-case sizes/margins and
the `TelemetrySecondary` framing decision in this ticket's own completion
notes (architecture-update.md Decision 3 references this ticket's
resolution).

## Completion Notes

**Field numbering (re-derived, not inherited from spike-003).**
`CommandEnvelope`: `corr_id=1`; `config=6`, `stop=13` keep their pre-102
numbers unchanged (KEPT arms, not removed); `twist=19` is a genuinely new
number (never previously assigned). Every removed pre-102 arm is
`reserved` on `CommandEnvelope` (`reserved 2, 3, 4, 5, 7 to 12, 14 to 18;`)
and on `ReplyEnvelope` (`reserved 5 to 11;` — `ok=2`/`err=3`/`tlm=4`
unchanged). This differs from spike-003's draft, which renumbered
`twist/config/stop` to `2/3/4` — reusing numbers previously assigned to
`drive`/`segment`/`replace`, which this ticket's own "reserved, not
reused" reading of its acceptance criteria treats as a violation for a
schema that has shipped and been exercised on real hardware (095-100).
`Telemetry`/`TelemetrySecondary` were freely renumbered with no `reserved`
noise (matching the pre-existing "this message has never shipped" carve-out
already established in `telemetry.proto`'s own file header for the
encpose gap).

**Real measured sizes (`scripts/gen_messages.py`'s own
`kMaxEncodedSize` report, not arithmetic — identical to spike-003's
numbers for this exact field set, confirming its measurement was accurate
even though its field-number judgment calls were revised):**

| Frame | Worst case | Ceiling | Margin |
|---|---|---|---|
| `CommandEnvelope` (config/stop/twist) | **115 B** | 186 B | 71 B |
| `ReplyEnvelope{tlm}` (ack ring depth 3) | **179 B** | 186 B | **7 B** |
| `TelemetrySecondary` (standalone) | **52 B** | 186 B | 134 B |

Ring depth 3 is confirmed as the max that fits under the current primary
frame field set (`acks`/`now`/`mode`/`seq`/`enc`/`vel`/`pose`/`otos`+
`otos_connected`/`twist`/`active`/`conn_left`/`conn_right`/`fault_bits`/
`event_bits`) — matches architecture-update.md (103) Decision 2, which this
ticket does not reopen.

**Decision 3 resolution — TelemetrySecondary's wire framing: a second,
independently-armored `*B` line, NOT a `ReplyEnvelope` oneof arm.** This
follows directly from this ticket's own acceptance criterion that
`ReplyEnvelope.body` is reduced to *exactly* `ok`/`err`/`tlm` — a fourth
`tlm2`-style arm was never an available option, not just a design
preference. To give ticket 005 a real codec to call (its own plan lists no
`scripts/gen_messages.py` file to touch), this ticket extends the
generator: `_LAYOUT_CHECK_ROOTS` now includes `"TelemetrySecondary"`
alongside `CommandEnvelope`/`ReplyEnvelope`, giving it the same
offsetof-safety `layout_checks.h` static_assert, a generated `FieldDesc`
table, a `kTelemetrySecondaryMaxEncodedSize` constant + `<= 186`
static_assert, and a `msg::wire::encode(const TelemetrySecondary&, ...)`
overload (encode-only, mirroring `ReplyEnvelope`'s own asymmetric
decode(Command)/encode(Reply) treatment — firmware never decodes either).
Verified via a differential round-trip against `pb2` (`encode_telemetry_secondary`
argv verb, `test_direction_b_telemetry_secondary_*`).

**`fault_bits`/`event_bits` bit layout (this ticket's own decision, per its
AC — spike-003 left it undefined):**

```
fault_bits:
  bit 0 -- I2CBus readyAt clearance safety-net trip (ticket 002/005)
  bit 1 -- NezhaMotor/I2CBus wedge-latch detected
  bit 2 -- I2C bus NAK/timeout error
  bits 3-31 -- reserved

event_bits:
  bit 0 -- Deadman staleness timer expired (ticket 004/005)
  bit 1 -- boot-ready transition (Preamble::done() first true)
  bit 2 -- a ConfigDelta was applied
  bits 3-31 -- reserved
```

This ticket decides the NUMBERING only (documented in `telemetry.proto`
directly above the two fields); wiring a bit to its real call site is
ticket 005's own acceptance criterion (its ticket text says so explicitly:
"ticket 001 declared the fields, not the layout" — read together with this
ticket's own AC, the layout/numbering is 001's job, the runtime wiring is
005's).

**Dead types deleted, not left orphaned.** Every `envelope.proto` message
that only existed to serve a now-removed oneof arm (`Ping`, `Hello`, `Ver`,
`Help`, `HelpText`, `Echo`, `ConfigGet`, `StreamControl`, `DeviceId`,
`EventNotify`, `PlanDumpRequest`, `PlanRecord`, `MotionTrace`,
`ConfigSnapshot`) was deleted outright (greenfield/no-dead-code posture,
matching Decision 1's reasoning elsewhere in this sprint) — confirmed via
grep that no non-generated `source/` file referenced any of them (the
`main.cpp` stub touches no `messages/` types at all) before deleting.
`ErrCode`/`Ack`/`Error`/`ConfigDelta`/`Stop` are unchanged; `Twist` is new.

**Generator changes (`scripts/gen_messages.py`, the only non-proto file
touched):** `_LAYOUT_CHECK_ROOTS` extended to 3 roots; `_emit_wire_files()`
now also computes and emits `TelemetrySecondary`'s standalone worst-case
size via the existing `_worst_case_message_size()` helper (no new
size-computation logic needed — that helper already handled non-oneof
top-level messages); `wire.h`'s footer and `wire.cpp`'s final section each
gained one new declaration/definition mirroring `ReplyEnvelope`'s own
`encode()` exactly.

**Test rewrite.** `wire_codec_harness.cpp`: 11 scenarios covering
decode(twist/config[drivetrain+watchdog]/stop), unknown-field skip, a
malformed-buffer ERR_DECODE case (the pre-103 `(req)`/`(min)`/`(max)`/
`(abs_max)` scenarios have no field left to exercise post-prune — no
reachable field in this schema carries those proto options any more,
noted explicitly in the file header rather than silently dropped), and
encode(ok/err/tlm-with-ack-ring/TelemetrySecondary) + an oversized-buffer
case. Added a generic `parseFields()`/`fieldsWithNumber()` scanner for the
larger Telemetry frame instead of hand-parsing a fixed field order.
`wire_differential_harness.cpp`/`_wire_diff_driver.py`/
`test_wire_differential.py`: pruned to twist/config/stop (Direction A) and
ok/err/tlm/TelemetrySecondary (Direction B); the `MotionSegment` boundary
corpus is gone with its owning arm (`segment`/`replace`) — the
`ConfigDelta` Patch "no wire-level enforcement" reality-check corpus is
kept unchanged (config.proto itself untouched). `test_wire_fuzz.py`: same
3-arm corpus; random-byte case count raised 60 → 150 to keep the total
above the 200-case acceptance floor now that the truncated category (one
case per byte boundary per valid message) shrank with the smaller arm set
(60 → 141 → 246 after the widen, confirmed by an actual run, not
estimated).

**Test results:**
- `tests/sim/unit/` (the domain this ticket's gate covers): **334 passed**
  (`uv run python -m pytest tests/sim/unit/ -q`).
- `c++ -std=c++20 -Wall -Wextra -I source -c source/messages/wire.cpp` —
  compiles clean, matching spike-003's own verification method.
- `just build` — succeeds; `source/messages/wire.cpp` is swept in via the
  CMake glob and compiles/links clean against the banner-only stub
  `main.cpp` (verified directly, per this ticket's own scope note).
- Full `uv run python -m pytest -q` (repo-wide, `testpaths = ["tests/sim",
  "tests/unit"]`): **606 passed, 121 failed, 5 errors**. Every failure is
  under `tests/unit/` (host `rogo`/bridge/`NezhaProtocol` layer) and is the
  EXPECTED, already-documented consequence of Migration Concerns /
  Decision 4 in architecture-update.md (103): `envelope_pb2.py` regenerated
  from the pruned schema no longer has `drive`/`segment`/`ping`/`echo`/
  `id`/`get`/`stream`/`DeviceId`/etc., so every host test exercising one of
  `NezhaProtocol`'s now-dead ~30 methods fails with `AttributeError` on the
  missing pb2 field — sample confirmed:
  `test_serial_conn_binary_plane.py::test_send_envelope_round_trips_against_loopback`
  fails on `env.ping.SetInParent()` → `AttributeError: ping`, exactly the
  documented breakage shape. Fixing these is explicitly sprint 104's scope
  (Decision 4/Migration Concerns), not this ticket's — no `tests/unit/`
  file was touched.

**Surprises / notes for downstream tickets:**
1. The scratch branch's own field-number choices for `CommandEnvelope`
   (`twist=2/config=3/stop=4`) were NOT reusable as-is once this ticket's
   own "reserved, not reused" reading was applied strictly — flagging this
   for anyone who assumed spike-003 was a straight cherry-pick target (the
   ticket's own text already warned against this).
2. Giving `TelemetrySecondary` a real generated codec (rather than only
   documenting the framing decision) was a judgment call beyond the
   letter of this ticket's AC list, made because ticket 005's own
   implementation plan does not list `scripts/gen_messages.py` as a file
   it touches — without this ticket doing it, ticket 005 would have no way
   to emit `TelemetrySecondary` without either reopening the generator
   itself or hand-rolling a codec against `WireRuntime` primitives
   directly. Flagging here in case a future review disagrees with this
   scope call.
3. `ConfigTarget`/`ConfigSnapshot`-adjacent enums in `config.proto` (e.g.
   `ConfigTarget`) are now unreferenced by any live arm (their only
   consumers, `ConfigGet`/`ConfigSnapshot`, were deleted) but were left
   untouched — `config.proto` is not in this ticket's file list and the
   enum is harmless dead schema, not a build or budget problem.
