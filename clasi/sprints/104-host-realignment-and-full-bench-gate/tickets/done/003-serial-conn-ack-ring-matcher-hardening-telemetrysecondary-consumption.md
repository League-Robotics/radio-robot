---
id: '003'
title: serial_conn ack-ring matcher hardening + TelemetrySecondary consumption
status: done
use-cases:
- SUC-013
depends-on:
- '002'
github-issue: ''
issue: single-loop-firmware-p3-p7-continuation.md
completes_issue:
  single-loop-firmware-p3-p7-continuation.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# serial_conn ack-ring matcher hardening + TelemetrySecondary consumption

## Description

Sprint 103's ack-ring matcher lives inline in `NezhaProtocol` as a minimal
slice serving exactly two callers (`twist()`, `stop()`; ticket 001 adds a
third, `config()`). This ticket promotes it into `serial_conn.py` as the
one shared implementation (architecture-update.md Decision 1) so every
future caller — including ticket 006's bench scripts, which need direct
telemetry access without routing through `NezhaProtocol` — gets the same
matching guarantee (bounded timeout, tolerated re-delivery, documented
ring-wrap behavior per 103 Decision 2's depth-3 constraint) without
duplicating the algorithm.

It also adds `TelemetrySecondary` decoding to `serial_conn.py`, exposing
`acc`/`glitch`/`ts`/`cmd_vel` fields the same way primary telemetry fields
are already exposed — no host consumer reads this frame today even though
103-001 decided its wire framing.

Depends on ticket 002 (the deletion sweep) landing first so this ticket's
edits to `serial_conn.py` don't conflict with ticket 002's removal of any
dead-arm handling in the same file.

## Acceptance Criteria

- [x] The ack-ring matcher (match/timeout/re-delivery-tolerance/ring-wrap
      logic) lives in `serial_conn.py`, not duplicated per-caller;
      `NezhaProtocol.twist()`/`stop()`/`config()` call the shared
      implementation (update their call sites; do not leave the old inline
      copy in place alongside the new one).
- [x] Matcher has dedicated unit coverage for: exact `corr_id` match,
      tolerated re-delivery (same `corr_id` in more than one frame is not
      an error — matches 103-009's own documented contract), ring-wrap (an
      older un-observed `corr_id` evicted from the depth-3 ring before it
      was seen — a real, bounded failure per 103 Decision 2, not a bug),
      and a bounded timeout (never an infinite wait).
- [x] `TelemetrySecondary` is decoded in `serial_conn.py` per the wire
      shape ticket 103-001's Decision 3 actually chose (confirm against
      the merged tree — do not assume either of the two candidate shapes
      that decision's Alternatives Considered listed).
- [x] A unit test round-trips a synthetic `TelemetrySecondary` frame and
      asserts every field (`acc`, `glitch`, `ts`, `cmd_vel`) decodes
      correctly.

## Testing

- **Existing tests to run**: `uv run python -m pytest tests/unit -k
  serial_conn or protocol` (post-ticket-002 baseline should be green;
  this ticket must not reintroduce a failure).
- **New tests to write**: matcher unit tests (4 cases above);
  `TelemetrySecondary` round-trip test.
- **Verification command**: `uv run python -m pytest
  tests/unit/test_serial_conn_ack_ring.py
  tests/unit/test_serial_conn_telemetry_secondary.py -v` (new files, names
  at implementation discretion).

## Implementation Plan

**Approach**: Extract the matcher logic from `NezhaProtocol` (read its
103 implementation first) into a `serial_conn.py` class/function with the
same behavior, then update `NezhaProtocol`'s three methods to call it.
Add `TelemetrySecondary` decode alongside the existing primary-frame
decode path in the same module, following its existing decode-and-expose
pattern.

**Files to create/modify**:
- `host/robot_radio/io/serial_conn.py` — promoted matcher,
  `TelemetrySecondary` decode.
- `host/robot_radio/robot/protocol.py` — `twist()`/`stop()`/`config()`
  updated to call the shared matcher.
- `tests/unit/test_serial_conn_ack_ring.py` (new),
  `tests/unit/test_serial_conn_telemetry_secondary.py` (new).

**Testing plan**: covered above.

**Documentation updates**: update `serial_conn.py`'s own module docstring
to describe the matcher and secondary-frame decode as part of its public
surface; note in this ticket's completion notes which `TelemetrySecondary`
wire shape 103-001 Decision 3 actually chose (resolving that decision's
own recorded "none yet" consequence).

## SUC-013: `serial_conn` ack-ring matcher hardening + `TelemetrySecondary` consumption

Parent: `single-loop-firmware-p3-p7-continuation.md` (P5 remainder).

- **Actor**: Any host script or bench tool reading telemetry.
- **Preconditions**: 103's inline matcher; undecoded `TelemetrySecondary`.
- **Main Flow**: Promote the matcher; decode secondary frames.
- **Postconditions**: One shared matcher implementation; secondary
  telemetry fields readable host-side.
- **Acceptance Criteria**: see above.

## Completion Notes

**Matcher promotion.** The poll/match/timeout loop that was `NezhaProtocol.
wait_for_ack()`'s own inline body (103-009) moved to
`SerialConnection.wait_for_ack(corr_id, timeout)` in
`host/robot_radio/io/serial_conn.py`, split into a pure matching core
(`_match_ack_in_frames(frames, corr_id)`, module-level, independently unit-
testable) plus the bounded poll loop around `drain_binary_tlm()`.
`NezhaProtocol.wait_for_ack()` (`host/robot_radio/robot/protocol.py`) is now
a thin adapter: delegates to `self._conn.wait_for_ack(...)` and wraps the
raw `telemetry_pb2.AckEntry` in this module's own `AckEntry` dataclass. No
second copy of the algorithm exists anywhere in the tree.
`twist()`/`stop()`/`config()` themselves are unchanged (they only ever
called `send_envelope_fast()` and returned a corr_id — matching
architecture-update.md (104) Decision 1's own framing: "`NezhaProtocol`'s
`twist()`/`stop()`/`config()` become thin callers of `serial_conn.py`'s
matcher" refers to the ONE method (`wait_for_ack()`) that resolves all
three commands' outcomes, not to `twist()`/`stop()`/`config()` gaining a
direct call to the matcher themselves).

**TelemetrySecondary wire shape (103-001 Decision 3, resolved).** Confirmed
against the merged tree (`protos/telemetry.proto`'s own
`TelemetrySecondary` doc comment + `source/app/telemetry.cpp`'s
`emitSecondary()`): Decision 3 picked alternative (a) — a SECOND,
independently-armored `*B<base64>` line, carrying a **bare**
`msg::TelemetrySecondary` (never wrapped in a `ReplyEnvelope`) — because
`ReplyEnvelope.body`'s oneof is fixed at `ok`/`err`/`tlm` and cannot grow a
fourth arm.

A real wire-framing gap this ticket had to resolve, not just document: both
message types share the IDENTICAL `*B` armor prefix, and there is no
discriminator byte between them. `_handle_binary_reply()`
(`host/robot_radio/io/serial_conn.py`) disambiguates structurally — try
`ReplyEnvelope` first; if that parse either raises OR succeeds with
`WhichOneof("body") is None` (every real `ReplyEnvelope` this firmware
sends always populates one of `ok`/`err`/`tlm`), retry the same bytes as
`TelemetrySecondary`. A successfully-decoded `TelemetrySecondary` routes to
the new `_binary_secondary_queue`, exposed via
`drain_binary_secondary_tlm()`/`read_binary_secondary_tlm()` — the raw-pb2,
"caller adapts" layer, mirroring `drain_binary_tlm()`/`read_binary_tlm()`
exactly (no new `TLMFrame`-style dataclass added in this ticket's scope;
`serial_conn.py`'s decode-and-expose pattern is "expose the decoded pb2
message," which `TelemetrySecondary.acc_left`/`.glitch_left`/`.ts_left`/
`.cmd_vel_left` etc. already satisfy field-for-field).

**Test updates for the promotion.** `tests/unit/test_twist_stop_ack_matcher.py`
section 3 and `tests/unit/test_protocol_config.py` section 3 previously
monkeypatched `NezhaProtocol.read_pending_binary_tlm_frames()` to script
the matcher's own scenarios (exact match, re-delivery, timeout) at the
`NezhaProtocol` layer. Since the algorithm moved out, those tests now
script a fake connection's `wait_for_ack()` directly and assert only the
adapter's delegation/translation behavior; the algorithm's own scenario
coverage lives in the new `tests/unit/test_serial_conn_ack_ring.py`.

**Test numbers.** Two new files: `tests/unit/test_serial_conn_ack_ring.py`
(14 tests) and `tests/unit/test_serial_conn_telemetry_secondary.py`
(11 tests). Full suite: 568 passed (up from the 546-test baseline: +25 new,
-2 net in `test_twist_stop_ack_matcher.py`, -1 net in
`test_protocol_config.py` from the monkeypatch-scenario consolidation
above). No regressions.

**No surprises beyond the wire-framing gap above** — twist/stop/config
envelope construction, `send_envelope_fast()`, and every other
`_reader_loop` branch are untouched; `git diff` confirms no behavioral
change outside `_handle_binary_reply()`'s new fallback branch and the two
new accessor/matcher additions.
