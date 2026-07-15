---
id: '009'
title: "Minimal host slice — NezhaProtocol.twist/stop + ack-ring matcher"
status: open
use-cases: [SUC-009]
depends-on: ['001']
github-issue: ''
issue: single-loop-firmware-p3-p7-continuation.md
completes_issue:
  single-loop-firmware-p3-p7-continuation.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Minimal host slice — NezhaProtocol.twist/stop + ack-ring matcher

## Description

Add `NezhaProtocol.twist(v_x, omega, duration)` and `NezhaProtocol.stop()`
to `host/robot_radio/robot/protocol.py`, plus an ack-ring matcher that
polls incoming `Telemetry.acks` frames for a given `corr_id` (the new
telemetry-only return path has no synchronous per-command reply — the
existing `_send_envelope()`/reply-queue machinery built for the OLD design
does not apply to these two new methods). Add one `tests/bench/` drive
script exercising both. This is deliberately the MINIMAL host slice —
full P5 host realignment (legacy translator deletion, `rig_dev`/`rig_soak`
rewrite) is sprint 104 scope.

Depends on ticket 001 (regenerated `envelope_pb2`/`telemetry_pb2` host-side).

## Acceptance Criteria

- [ ] `NezhaProtocol.twist(v_x, omega, duration)` builds a
      `CommandEnvelope{corr_id, twist:{v_x, omega, duration}}`, assigns a
      fresh `corr_id` (reusing the existing `_corr_counter` pattern), sends
      it (fire — no wait for a synchronous OK/ERR reply, matching the new
      design), and returns the `corr_id` used.
- [ ] `NezhaProtocol.stop()` builds `CommandEnvelope{corr_id, stop:{}}`,
      same send/return shape.
- [ ] An ack-ring matcher function/method exists: given a `corr_id` and a
      timeout, it polls `Telemetry` frames (via the existing
      `read_pending_binary_tlm_frames()`/binary telemetry delivery path)
      for a matching `acks` entry, returns the matched `AckEntry` (or
      `None` on timeout), and correctly tolerates the SAME `corr_id`
      appearing in more than one polled frame (ring re-delivery is not an
      error).
- [ ] The matcher has a bounded timeout — no infinite wait.
- [ ] A new `tests/bench/` script: connects, arms telemetry, calls
      `twist()`, confirms the ack via the matcher, confirms (via telemetry)
      that encoders are moving, calls `stop()`, confirms its ack.
- [ ] `TLMFrame`/telemetry parsing (`TLMFrame.from_pb2`, `protocol.py`) is
      updated if needed to expose the `acks`/`fault_bits`/`event_bits`
      fields the matcher reads (confirm against ticket 001's final schema
      — do not assume the pre-prune `TLMFrame` shape already carries
      them).

## Implementation Plan

**Approach**: Read `NezhaProtocol`'s existing `_send_envelope()`
(`protocol.py:637`) and `read_pending_binary_tlm_frames()` (`:1312`)
methods directly before writing `twist()`/`stop()` — do not assume
`_send_envelope()`'s existing reply-queue-wait behavior is reusable
as-is; it was built for the OLD synchronous-reply design and these two new
methods need a fire-and-poll shape instead (send via the same underlying
serial write path `_send_envelope()` uses, but skip its reply-queue wait).

**Files to create/modify**:
- `host/robot_radio/robot/protocol.py` — `twist()`, `stop()`, ack-ring
  matcher (new methods/functions); `TLMFrame` update if the `acks` field
  isn't already exposed.
- One new `tests/bench/` script (e.g. `tests/bench/twist_drive.py` —
  exact name at implementation time).

**Testing plan**:
- Existing tests to run: any existing `host/` unit tests touching
  `NezhaProtocol`/`TLMFrame` parsing that don't depend on the now-pruned
  wire arms.
- New tests to write: a schema-level unit test for `twist()`/`stop()`'s
  envelope construction (assert the built `CommandEnvelope` has the
  correct oneof arm and field values, no serial connection needed); a
  matcher unit test using synthetic `Telemetry` frames with scripted
  `acks` lists (including a re-delivered `corr_id` across two frames) to
  confirm match/timeout behavior without hardware.
- Verification command: `uv run python -m pytest tests/unit/ -k "twist or nezha_protocol"`
  (adjust path once test files exist); the new `tests/bench/` script is
  exercised for real against hardware in ticket 010, not in this ticket's
  own CI-run tests.

**Documentation updates**: a short docstring on the ack-ring matcher
explaining the telemetry-only return path (no per-command synchronous
reply) so a future host developer doesn't assume `twist()`/`stop()` block
for an OK/ERR the way most of `NezhaProtocol`'s other methods do.
