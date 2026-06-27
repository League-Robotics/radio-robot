---
id: 008
title: 'Fix SerialConnection reader routing: ID reply and SNAP/TLM reply'
status: done
use-cases:
- SUC-002
depends-on: []
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix SerialConnection reader routing: ID reply and SNAP/TLM reply

## Description

**Background — pre-existing bug surfaced during sprint-036 bench validation.**

Validating sprint 036 over the **direct serial port** (the reliable link, not
radio) exposed two pre-existing routing bugs in `SerialConnection` that break
ticket 002's `refresh()` call on real hardware. Both bugs exist on `master`
(the reader loop is identical there); they are in-scope for 036 because they
prevent the `refresh()` deliverable from working on hardware.

### Bug 1 — `get_id()` returns None: `ID` reply silently dropped

`_reader_loop` in `host/robot_radio/io/serial_conn.py` (~line 536-567) routes
lines as follows:

- `TLM ...` → `_tlm_queue`
- `EVT ...` → `_evt_queue`
- `OK/ERR/CFG ...` lines that contain `#<id>` → `_reply_queues[corr_id]`
- **Everything else → silently dropped**

The `ID` reply carries a trailing corr-id: hardware-verified on bench:

```
send: ID #7
recv: ID model=Nezha2 fw=0.20260612.28 proto=2 caps=... #7
```

The trailing `#7` is there, but `ID` is not in the `startswith(("OK", "ERR",
"CFG"))` check, so the line is dropped → `get_id()` returns `None`.

**Fix:** Add `"ID"` to the routed tag set:

```python
if text.startswith(("OK", "ERR", "CFG", "ID")):
```

The existing `#(\d+)$` corr-id extraction then routes the reply to the caller's
queue. `get_id()` in `host/robot_radio/robot/protocol.py` already searches for
tag `ID` in the response — it will work once routing is fixed.

### Bug 2 — `snap()` / `refresh()` return None: SNAP reply is a corr-id-less TLM frame

`snap()` in `host/robot_radio/robot/protocol.py` (~line 676) calls
`self._conn.send("SNAP", ...)`. `send()` waits on `_reply_queues[corr_id]`.
But the SNAP reply is a **`TLM` frame** that does NOT carry the corr-id:

```
send: SNAP #8
recv: TLM t=... mode=I enc_l=... enc_r=... ...   (no #8)
```

The TLM frame is routed to `_tlm_queue`; `send()` times out waiting on the
corr-id queue and returns `None` → `snap()` returns `None` → `refresh()`
returns `None`.

**Fix:** Change `snap()` to retrieve the reply from the telemetry-queue path
rather than relying on `send()`'s corr-id reply:

1. Optionally drain any stale frames already in `_tlm_queue` (to avoid picking
   up an old snapshot).
2. Send `SNAP` via `send_fast()` (fire-and-forget, no corr-id wait), or via
   `send()` with a very short reply timeout that is expected to expire
   (acceptable if the timeout is cheap, e.g., 0.05s).
3. Call `self._conn.read_lines(timeout_s=...)` (or the equivalent
   `_tlm_queue.get(timeout=...)`) to pull the next frame from `_tlm_queue`.
4. Parse the first frame that `parse_tlm()` accepts and return it.

`refresh()` in `Nezha` (ticket 002) calls `snap()` and will work once this
fix is applied.

### Out of scope (noted for completeness, no action)

Separately investigated: STREAM idle-throttling is NOT a bug. The firmware
streams at the configured period during motion (bench-verified: 12 TLM
frames/1.3s while driving) and throttles when idle. This is expected behavior;
no code change is needed.

## Acceptance Criteria

- [x] `get_id()` over a real (or mock) connection returns the parsed identity
      dict — the `ID` reply carrying `#<id>` is routed to the caller's corr-id
      reply queue and is not dropped.
- [x] `snap()` returns a `TLMFrame` populated from the `TLM` reply received
      after sending `SNAP`.
- [x] `Nezha.refresh()` calls `snap()` and returns a `RobotState` populated
      from the resulting `TLMFrame`.
- [x] Existing routing is unchanged: `OK/ERR/CFG` corr-id replies, `EVT`
      frames, streamed `TLM` frames, `+`-keepalive handling, and relay `#`-line
      filtering all behave as before; no existing tests are broken.
- [x] Unit tests (mocking the serial/reader boundary) covering:
      - An `ID model=... #<id>` line is delivered to the corr-id reply queue
        and surfaces correctly via `get_id()`.
      - A `SNAP` command whose reply arrives as a corr-id-less `TLM` line is
        returned by `snap()` (i.e., read from `_tlm_queue`, not the corr-id
        queue).
- [x] Test run passes: `uv run --with pytest python -m pytest host/tests/ -q`.
- [ ] **Live bench re-validation** (DEFERRED TO TEAM-LEAD): over the direct
      serial port, `robot.refresh()` returns a populated `RobotState` and
      `get_id()` returns a non-None identity dict. Programmer's code + mocked
      tests are complete; team-lead runs the hardware end-to-end check.

## Implementation Plan

### Files to modify

**`host/robot_radio/io/serial_conn.py`** — reader loop tag set

- Locate the `if text.startswith(("OK", "ERR", "CFG")):` guard (~line 560).
- Add `"ID"` to the tuple: `startswith(("OK", "ERR", "CFG", "ID"))`.
- No other changes to the reader loop required for Bug 1.

**`host/robot_radio/robot/protocol.py`** — `snap()` method (~line 676)

- Replace the current `send("SNAP", ...)` + corr-id-wait pattern.
- New approach:
  1. (Optional) Drain stale frames: clear `self._conn._tlm_queue` or use a
     short `read_lines()` with `timeout_s=0` before sending.
  2. Send `SNAP` without waiting on a corr-id reply. Use `send_fast("SNAP")`,
     or `send("SNAP", reply_timeout_s=0.05)` if `send_fast` is not available.
  3. Read the next TLM frame: `frame_text = self._conn._tlm_queue.get(timeout=2.0)`.
  4. Return `parse_tlm(frame_text)`.
  - If `_tlm_queue` is private, access via the existing `read_lines()` public
    helper if one exists, or access `_tlm_queue` directly (both are in the
    same package).

### Files to create

**`host/tests/test_serial_conn_id_snap_routing.py`**

Two test scenarios, mocking at the `serial.Serial` boundary (same style as
`test_serial_relay_handshake.py`):

1. **`ID` routing test**: Feed the reader thread an `ID model=Nezha2 ... #7`
   line. Assert that calling `get_id()` (via `protocol.py`) returns a dict with
   `model="Nezha2"` and does not return `None`.

2. **`snap()` TLM-queue test**: Simulate a `SNAP #8` send; feed the reader a
   `TLM t=... mode=I ... enc_l=100 enc_r=100 ...` line (no `#8`). Assert that
   `snap()` returns a `TLMFrame` with the correct fields (not `None`).

### Approach notes

- Prefer the minimal targeted change. The reader-loop fix for Bug 1 is a
  single-tuple edit. The `snap()` fix for Bug 2 is localized to that one
  method.
- Do NOT touch `EVT`, `TLM`-stream, keepalive, or relay-comment handling —
  the task brief explicitly requires those remain unchanged.
- Do not introduce new public API; the fix is internal to `serial_conn.py`
  and `protocol.py`.

### Testing plan

```
uv run --with pytest python -m pytest host/tests/ -q
```

Run after each change. Confirm both new tests pass and all 577 pre-existing
tests remain green.

### Documentation updates

- Add an inline comment above the `startswith(...)` guard explaining that `ID`
  carries a corr-id and must be routed like `OK/ERR/CFG`.
- Add an inline comment on the new `snap()` implementation noting that the SNAP
  reply arrives as a corr-id-less `TLM` frame (not a corr-id-keyed reply) —
  this is the authoritative explanation for future readers.
