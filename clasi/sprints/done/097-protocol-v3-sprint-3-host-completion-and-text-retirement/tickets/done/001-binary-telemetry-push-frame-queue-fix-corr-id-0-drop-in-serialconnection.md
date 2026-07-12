---
id: '001'
title: Binary telemetry push-frame queue (fix corr_id=0 drop in SerialConnection)
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Binary telemetry push-frame queue (fix corr_id=0 drop in SerialConnection)

## Description

`host/robot_radio/io/serial_conn.py`'s `_handle_binary_reply()` routes
**every** binary `*B<base64>` reply — solicited or not — through
`_reply_queues[str(reply.corr_id)]`, a table populated only while a
`send()`/`send_envelope()` call is actively blocked awaiting that exact
corr_id. `telemetryEmitBinary()` (firmware, sprint 096) sets
`reply.corr_id = 0` on every unsolicited periodic push frame. No queue is
ever registered under `"0"`, so `_handle_binary_reply()`'s own "no
listener -> drop silently" fallback eats every binary telemetry push
frame the firmware ever sends. The wire and firmware side are correct
(096) — this is a host-only gap, found during sprint 097 architecture
research (`architecture-update.md` Decision 2), that blocks ticket 003
(`NezhaProtocol.stream()`/`.snap()` conversion) from ever being able to
receive a reply.

This ticket adds a new, bounded, drop-oldest `_binary_tlm_queue` and
routes any `ReplyEnvelope` whose `body` oneof is `tlm` into it, checked
BEFORE the corr-id lookup — mirroring how the text plane's
`text.startswith("TLM")` branch is already checked before the
`OK`/`ERR`/`CFG`/`ID` corr-id branch in the same function. No other reply
body's routing changes.

This ticket is host-only. It does not touch firmware, and it does not
convert any `NezhaProtocol` method (that is tickets 002/003). It is
purely foundational plumbing, landed first so ticket 003 can build on a
working delivery path instead of discovering the gap mid-ticket.

## Acceptance Criteria

- [x] A new `_binary_tlm_queue` (or equivalently named) exists on
      `SerialConnection`, matching `_tlm_queue`'s existing bounded/
      drop-oldest-on-overflow policy (same depth constant or a documented,
      deliberate choice of a different one).
- [x] `_handle_binary_reply()` (or `_reader_loop()`, whichever is the
      cleaner insertion point) special-cases `WhichOneof("body") == "tlm"`
      and routes to `_binary_tlm_queue`, checked BEFORE the corr-id
      `_reply_queues` lookup — the same ordering the text plane's
      `text.startswith("TLM")` branch already has relative to its
      `OK`/`ERR`/`CFG`/`ID` corr-id branch.
- [x] Every other binary reply body (`ok`/`err`/`cfg`/`id`/`echo`) keeps
      routing through the unchanged corr-id path — verified by a host
      unit test that exercises BOTH a corr-id-keyed direct reply (e.g. a
      simulated `Ack`) and a `corr_id=0` push frame (`Telemetry`) in the
      same reader-thread session and asserts each lands in the correct
      queue.
- [x] No text-plane behavior changes (this ticket touches only the binary
      reply-routing branch — `_tlm_queue`, `text.startswith("TLM")`, and
      every other existing branch in `_reader_loop()`/
      `_handle_binary_reply()` are untouched).
- [x] `tests/sim` stays green (this is a host-only Python change with no
      firmware/sim surface, so this is a no-op check confirming no
      accidental cross-contamination).
- [x] `tests/unit` is green, including the new test(s) this ticket adds.

## Implementation Plan

### Approach

1. Add `self._binary_tlm_queue: queue.Queue = queue.Queue(maxsize=<depth>)`
   in `SerialConnection.__init__`, alongside the existing
   `self._tlm_queue` declaration — reuse `_TLM_QUEUE_DEPTH` unless a
   documented reason exists to pick a different bound.
2. In `_handle_binary_reply()`, after a successful
   `ReplyEnvelope.FromString(raw_bytes)` parse and BEFORE the
   `corr_id = str(reply.corr_id)` / `_reply_queues.get(corr_id)` lookup,
   add: `if reply.WhichOneof("body") == "tlm": <bounded put, drop oldest
   on overflow, matching _tlm_queue's own pattern>; return`.
3. Update the docstrings of `_reader_loop()` and `_handle_binary_reply()`
   to document the new branch, matching their existing documentation
   style (both already have detailed routing-table docstrings — extend,
   don't replace).
4. Add a small drain/read accessor if `SerialConnection` doesn't already
   expose one generically enough for ticket 003 to use (e.g. a
   `read_binary_tlm_frames(duration)`-shaped method mirroring
   `read_lines()`'s existing pattern for `_tlm_queue`) — or leave this to
   ticket 003 if it's cleaner to add the accessor alongside its first
   real caller. Document the choice either way.

### Files to modify

- `host/robot_radio/io/serial_conn.py` — `SerialConnection.__init__`
  (new queue), `_handle_binary_reply()` (new branch), docstrings.

### Testing plan

- New host unit test(s) in the existing test file for `serial_conn.py`
  (or a new one if none exists) using a fake/mock serial port:
  - Simulate a `*B<base64>` line encoding a `ReplyEnvelope{tlm: ...,
    corr_id: 0}` and assert it lands in `_binary_tlm_queue`, not
    `_reply_queues`.
  - Simulate a `*B<base64>` line encoding a `ReplyEnvelope{ok: ..., corr_id:
    <N>}` while a queue is registered under `str(N)` and assert it still
    lands there, unaffected.
  - Overflow test: push more frames than the queue depth and assert the
    oldest is dropped (matching `_tlm_queue`'s documented behavior).
- Run `tests/unit` (host suite) — must stay green.
- `tests/sim` is unaffected by this ticket (no firmware/sim files
  touched); run it anyway as a sanity check per the sprint's blanket
  "tests/sim stays green at every ticket" requirement.

### Documentation updates

- None required beyond the in-code docstring updates above — this ticket
  has no user-visible wire-format or CLI change. `docs/protocol-v3.md`
  (ticket 009) will describe the binary telemetry push-frame mechanism at
  the wire level; this ticket's host-internal queue is an implementation
  detail below that level.

## Resolution

Implemented exactly per the plan: `_binary_tlm_queue` added in
`SerialConnection.__init__` (`queue.Queue(maxsize=_TLM_QUEUE_DEPTH)`,
reusing the existing depth constant — no separate bound needed).
`_handle_binary_reply()` now checks `reply.WhichOneof("body") == "tlm"`
immediately after the `ReplyEnvelope.FromString` parse, BEFORE the
`corr_id`/`_reply_queues` lookup, and routes unconditionally (not gated on
`corr_id == 0`) to `_binary_tlm_queue` with the same drop-oldest-on-overflow
pattern `_tlm_queue`'s text-plane branch already uses; every other body
(`ok`/`err`/`cfg`/`id`/`echo`) falls through to the unchanged corr-id path.
Docstrings updated on the class, `_reader_loop()`, and `_handle_binary_reply()`.

Drain accessor (plan item 4): deferred to ticket 003
(`NezhaProtocol.stream()`/`.snap()` conversion), documented inline at the
`_binary_tlm_queue` declaration — it is that ticket's first real caller and
best positioned to shape the accessor (blocking-with-duration vs.
non-blocking drain, parsed-object vs. `TLMFrame` shape) around its actual
need, per the plan's own stated option.

One pre-existing test needed updating, not just new tests added: 096-007's
`test_reader_loop_routes_binary_tlm_reply_by_corr_id` asserted the OLD
(corr-id-keyed) routing for a `tlm` body — exactly the routing this ticket
changes. Renamed to `test_reader_loop_routes_binary_tlm_reply_to_binary_tlm_queue`,
content updated to assert the new `_binary_tlm_queue` destination, and its
docstring documents the supersession (architecture-update.md Decision 2
frames the bug in terms of `corr_id=0`, but the fix implemented per this
ticket's own plan step 2 gates on the body oneof, not on `corr_id`, since
firmware never emits a `tlm` body any other way). Three new tests added:
`corr_id=0` push-frame routing, coexistence of a corr-id-keyed `ok` reply
and a `corr_id=0` `tlm` push frame in one reader-thread session (the
ticket's required "both queues, one session" proof), and a drop-oldest
overflow test (monkey-patches a depth-3 queue directly onto the connection
so the test stays fast — the drop-oldest logic itself is depth-agnostic).

**Verification**: `uv run python -m pytest tests/unit -q` — 42 passed
(38 pre-existing/updated + 4 new/renamed in
`test_serial_conn_binary_plane.py`; no other host test file touched).
`uv run python -m pytest tests/sim -q` — 600 passed, unaffected (host-only
change, no firmware/sim surface touched).
