---
id: 009
title: "Minimal host slice \u2014 NezhaProtocol.twist/stop + ack-ring matcher"
status: done
use-cases:
- SUC-009
depends-on:
- '001'
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

- [x] `NezhaProtocol.twist(v_x, omega, duration)` builds a
      `CommandEnvelope{corr_id, twist:{v_x, omega, duration}}`, assigns a
      fresh `corr_id` (reusing the existing `_corr_counter` pattern), sends
      it (fire — no wait for a synchronous OK/ERR reply, matching the new
      design), and returns the `corr_id` used.
- [x] `NezhaProtocol.stop()` builds `CommandEnvelope{corr_id, stop:{}}`,
      same send/return shape.
- [x] An ack-ring matcher function/method exists: given a `corr_id` and a
      timeout, it polls `Telemetry` frames (via the existing
      `read_pending_binary_tlm_frames()`/binary telemetry delivery path)
      for a matching `acks` entry, returns the matched `AckEntry` (or
      `None` on timeout), and correctly tolerates the SAME `corr_id`
      appearing in more than one polled frame (ring re-delivery is not an
      error).
- [x] The matcher has a bounded timeout — no infinite wait.
- [x] A new `tests/bench/` script: connects, arms telemetry, calls
      `twist()`, confirms the ack via the matcher, confirms (via telemetry)
      that encoders are moving, calls `stop()`, confirms its ack.
- [x] `TLMFrame`/telemetry parsing (`TLMFrame.from_pb2`, `protocol.py`) is
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

## Completion Notes

**API added** (`host/robot_radio/robot/protocol.py`):
- `AckEntry` (new frozen dataclass) — adapts one `telemetry_pb2.AckEntry`
  (`corr_id`, `ok`, `err_code`) onto a host-side shape, `AckEntry.from_pb2()`.
- `TLMFrame` gains `acks: tuple[AckEntry, ...] | None`,
  `fault_bits: int | None`, `event_bits: int | None`, populated
  unconditionally by `from_pb2()` (same "always present, not gated by
  has_*" treatment as the existing `active` field).
- `NezhaProtocol.twist(v_x, omega, duration) -> int` (new) — builds
  `CommandEnvelope{twist: Twist{v_x, omega, duration}}`, sends via the new
  fire-and-poll path, returns the corr_id used.
- `NezhaProtocol.stop() -> int` — **rewritten in place** (was `-> None`,
  pre-103-009): same fire-and-poll shape as `twist()`. The old
  implementation called `send_envelope()` (blocking, reply-queue-wait) for
  an Ack the P4 firmware no longer sends for this arm — it would have
  blocked ~800ms and returned nothing useful on every call. Every existing
  caller in this tree (`nezha.py`, `nezha_state.py`, `cli.py`,
  `calibrate.py`, `navigator.py`, `camera_goto.py`, `testkit/safety.py`'s
  guaranteed-stop-on-exit, etc.) calls `stop()` as a bare statement and
  ignores the return value, so the `None` → `int` change is
  source-compatible; the behavior change (no more pointless post-write
  block) is strictly safety-positive for a panic-stop.
- `NezhaProtocol.wait_for_ack(corr_id, timeout=500) -> AckEntry | None` —
  the ack-ring matcher. Polls `read_pending_binary_tlm_frames()` in a
  `time.sleep(0.01)` loop bounded by `timeout` ms; returns on the first
  frame carrying a matching `corr_id`, tolerating re-delivery across
  multiple frames/polls by construction (it never looks past the first
  match).
- `SerialConnection.send_envelope_fast(envelope) -> int`
  (`host/robot_radio/io/serial_conn.py`, new) — the binary-plane
  fire-and-forget send `twist()`/`stop()` sit on: assigns a corr_id from
  the SAME `_corr_counter` sequence `send_envelope()` uses (no collision
  between the two send paths), writes the armored line, returns
  immediately with **no** `_reply_queues` registration. Raises
  `ConnectionError` if not connected (mirrors `send_fast()`'s contract,
  not `send_envelope()`'s dict-with-`"error"` contract, since this
  method's return type is a bare `int`).

**Surprise / latent bug fixed**: `TLMFrame.from_pb2()` still read
`telemetry.has_cmd_vel`/`cmd_vel_left`/`cmd_vel_right` — fields
103-001/telemetry.proto moved OUT to the new `TelemetrySecondary` message.
Every call crashed with `AttributeError: has_cmd_vel` on ANY real
`Telemetry` frame, not just old tests — this would have silently broken
`read_pending_binary_tlm_frames()` (and therefore `wait_for_ack()`) the
moment it touched a live frame. Fixed: removed the dead read, documented
`cmd_vel` as a new permanent gap on the primary telemetry stream (mirrors
the existing `encpose`/`wedge` gap-documentation pattern in the same
docstring). This also flipped 9 pre-existing tests in
`tests/unit/test_protocol_binary_client.py` from fail to pass as a side
effect (they were failing on this same crash, not on anything this ticket
was asked to fix) — left everything else in that file untouched per the
"do NOT go fixing the 130" instruction.

**Ack-ring matcher semantics**: `wait_for_ack()` is a simple bounded
poll-until-match loop; it does not track "already seen" corr_ids across
calls (each call is independent) — a caller that calls `wait_for_ack()`
again for a corr_id it already matched will simply match it again if the
ring still carries it (harmless — the ack is idempotent — but callers
should call it once per command, not repeatedly).

**Deliverable**: `tests/bench/twist_drive.py` — connects, calls `twist()`,
confirms the ack via `wait_for_ack()`, watches telemetry for encoder
movement during the twist's `duration` window, calls `stop()`, confirms
its ack. No "arm telemetry" step (unlike the ticket's own suggested outline)
— documented in the script's own docstring: the P4 firmware pushes
`Telemetry` unconditionally at all times, there is no `STREAM` verb left
to arm (pruned at 103-001/103-003). Not yet run against real hardware —
that is ticket 010's bench-gate job.

**Tests**: `tests/unit/test_twist_stop_ack_matcher.py` (new, 12 tests) —
schema-level `twist()`/`stop()` envelope construction against a minimal
fake connection, `TLMFrame.from_pb2()` acks/fault_bits/event_bits
round-trip + the cmd_vel regression test, and the `wait_for_ack()` matcher
(immediate match, match-on-a-later-poll, ring re-delivery within one
batch, timeout, empty-ring frames). `tests/unit/test_serial_conn_binary_plane.py`
gained 4 new tests for `send_envelope_fast()` (wire bytes, no reply-queue
registration, shared corr-counter with `send_envelope()`, not-connected
raises).

Full suite (`uv run python -m pytest`, `tests/sim` + `tests/unit` per
`pyproject.toml` `testpaths`): **636 passed, 112 failed, 5 errors** (753
total) — vs. a measured baseline (same tree, this ticket's changes
stashed out) of **611 passed, 121 failed, 5 errors** (737 total). Net: +16 new tests
(all passing) + 9 pre-existing tests flipped fail→pass (the `cmd_vel` fix)
+ 0 regressions (diffing the baseline vs. final FAILED/ERROR test-ID sets
showed zero tests newly failing). `tests/sim` — the always-run, no-hardware
gate — is untouched by this ticket's changes and stays green. The
remaining 112 failures + 5 errors are the pre-existing Decision-4 breakage
(NezhaProtocol methods referencing removed pb2 arms like `ping`/`echo`/
`id`/`drive`/`stream`/`cfg`) — sprint 104 scope, not touched here.
