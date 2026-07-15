---
id: '003'
title: serial_conn ack-ring matcher hardening + TelemetrySecondary consumption
status: open
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

- [ ] The ack-ring matcher (match/timeout/re-delivery-tolerance/ring-wrap
      logic) lives in `serial_conn.py`, not duplicated per-caller;
      `NezhaProtocol.twist()`/`stop()`/`config()` call the shared
      implementation (update their call sites; do not leave the old inline
      copy in place alongside the new one).
- [ ] Matcher has dedicated unit coverage for: exact `corr_id` match,
      tolerated re-delivery (same `corr_id` in more than one frame is not
      an error — matches 103-009's own documented contract), ring-wrap (an
      older un-observed `corr_id` evicted from the depth-3 ring before it
      was seen — a real, bounded failure per 103 Decision 2, not a bug),
      and a bounded timeout (never an infinite wait).
- [ ] `TelemetrySecondary` is decoded in `serial_conn.py` per the wire
      shape ticket 103-001's Decision 3 actually chose (confirm against
      the merged tree — do not assume either of the two candidate shapes
      that decision's Alternatives Considered listed).
- [ ] A unit test round-trips a synthetic `TelemetrySecondary` frame and
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
