---
id: '004'
title: Add vw() body-velocity generator to Nezha
status: done
use-cases:
- SUC-006
depends-on:
- '001'
github-issue: ''
issue: plan-stateful-robot-object-playfield-object-for-the-host-module.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Add vw() body-velocity generator to Nezha

## Description

The firmware supports `VW <v_mms> <omega_mrads>` for body-velocity streaming, but
`Nezha` does not expose a generator for it. The existing `speed()` generator
(nezha.py:130) and `NezhaProtocol.stream_drive` (protocol.py:846) demonstrate the
S-streaming generator pattern. This ticket adds `vw()` as the VW-streaming
equivalent.

**`vw()` generator contract:**
1. Enable `STREAM <period_ms>`.
2. Send `VW <v_mms> <omega_mrads>` immediately.
3. Loop: read lines via `self._proto._conn.read_lines(duration_ms=50)`.
   - For each TLM line: `_apply_tlm` then `yield` (caller's loop body runs here).
   - If `EVT safety_stop`: break (generator ends naturally).
4. Re-send `VW` as keepalive whenever the keepalive interval elapses (≤30% of
   firmware watchdog window, so `period_ms * 0.3 / 1000` seconds). This matches
   the `stream_drive` pattern at protocol.py:866.
5. On `GeneratorExit` (caller `break`s): send `STOP` then `STREAM 0`, suppress
   exceptions.

Signature:
```python
def vw(self, v_mms: int, omega_mrads: int, *, period_ms: int = 40) -> Generator[None, None, None]
```

The generator yields `None` each tick (caller reads `robot.state` directly, as
the state is updated before the yield). Modelled on `stream_drive`; does NOT
return `ParsedResponse` — callers that need raw responses should use
`stream_drive` directly.

## Acceptance Criteria

- [x] `Nezha.vw(v_mms, omega_mrads)` is a generator that yields once per TLM
      tick. On each yield, `robot.state` has been updated via `_apply_tlm`.
- [x] `VW` is re-sent as a keepalive at `period_ms * 0.3` second intervals,
      within the firmware watchdog window.
- [x] Caller `break` triggers `GeneratorExit`; `STOP` and `STREAM 0` are sent;
      generator exits cleanly without raising.
- [x] `EVT safety_stop` terminates the generator without error.
- [x] `uv run --with pytest python -m pytest host/tests/test_robot_vw_generator.py
      host/tests/test_nezha_drive.py -q` passes.

## Implementation Plan

### Approach

Add `vw()` as a new method in `host/robot_radio/robot/nezha.py` in the
"Streaming drive" section, immediately below `speed()`. Model the implementation
on `speed()` (nezha.py:130–147) and `NezhaProtocol.stream_drive` (protocol.py:846–893).

The VW command is issued via `self._proto._conn.send_fast(f"VW {v_mms} {omega_mrads}")`.
Read lines via `self._proto._conn.read_lines(duration_ms=50)`. Parse with
`parse_response` (already imported).

### Files to Modify

- `host/robot_radio/robot/nezha.py` — add `vw()` generator in the Streaming
  drive section.

### Files to Create

- `host/tests/test_robot_vw_generator.py`

### Testing Plan

New file `host/tests/test_robot_vw_generator.py`. Mock the serial connection to
feed scripted TLM lines.

1. `test_vw_yields_per_tick` — mock `read_lines` to return three TLM frames then
   `EVT safety_stop`; consume the generator; assert it yielded three times and
   `robot.state` was updated each time.
2. `test_vw_resends_keepalive` — use a mock with a controllable clock; advance
   time past the keepalive interval; assert `VW` was re-sent.
3. `test_vw_break_sends_stop_and_stream_off` — break after the first yield; assert
   `STOP` and `STREAM 0` were sent (inspect `send_fast` call list).
4. `test_vw_safety_stop_exits_cleanly` — inject `EVT safety_stop` as first line;
   generator exits without raising; no `STOP` explicitly sent (safety_stop implies
   the firmware already stopped).

Verification: `uv run --with pytest python -m pytest host/tests/test_robot_vw_generator.py
host/tests/test_robot_state.py host/tests/test_nezha_drive.py -q`

### Documentation Updates

Add `vw()` docstring with unit conventions, keepalive note, and a brief usage
example. Update `Nezha` class docstring to list `vw()` alongside `speed()`.
