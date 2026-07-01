---
id: '005'
title: 'SimTransport: ctypes tick-thread, TLM from get_async_evts, sim-lib build check'
status: done
use-cases:
- SUC-002
- SUC-007
depends-on:
- '004'
issue: plan-robot-test-gui-pyside6.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 005 — SimTransport: ctypes tick-thread, TLM from get_async_evts, sim-lib build check

## Description

Implement `SimTransport` in `testgui/transport.py` — the third concrete backend
that drives the ctypes firmware simulator instead of real hardware. `SimTransport`
owns a `Sim` instance (from `tests/_infra/sim/firmware.py`), a background
tick-thread that advances `sim.tick()` at wall-clock rate, and drains
`sim.get_async_evts()` for TLM/EVT lines. Ground-truth pose comes from
`sim_get_true_pose_x/y/h`. Commands are queued to the tick-thread via a lock.

Before connecting, if the sim lib (`tests/_infra/sim/build/libfirmware_host.
{dylib,so}`) is missing, a `QMessageBox` prompts the user to run `python build.py`.

Applies a realistic error profile (motor slip + OTOS noise) so traces diverge.

Corresponds to item 3 in the approved design's ticket breakdown.

## Acceptance Criteria

- [x] `SimTransport` is added to `testgui/transport.py`, implementing the
  `Transport` ABC.
- [x] On `connect()`: checks for `tests/_infra/sim/build/libfirmware_host.*`;
  if missing, shows `QMessageBox.warning("Build required", "Run: python build.py")`
  and returns without connecting.
- [x] On `connect()` (lib present): loads `Sim`, starts a daemon tick-thread
  that calls `sim.tick()` at wall-clock rate (~20 ms), drains `get_async_evts()`
  for TLM lines, calls `parse_tlm` on TLM lines, invokes the `telemetry`
  callback with parsed `TLMFrame`, and invokes the `truth` callback with
  `(sim_get_true_pose_x(), sim_get_true_pose_y(), sim_get_true_pose_h())`.
- [x] `send(line)` and `command(line)` are forwarded to `sim.send_command(line)`
  via a thread-safe queue or lock (not called from the Qt main thread directly).
- [x] `disconnect()` stops the tick-thread cleanly.
- [x] Selecting **Sim** in the transport selector and clicking **Connect** triggers
  the lib-check and, if the lib is present, connects without error.
- [x] Sim mode applies `sim_set_motor_slip` and/or `sim_set_otos_linear_noise`
  on connect to produce visible trace divergence.
- [x] `uv run python -m pytest tests/simulation` passes.

## Implementation Plan

### Approach

Read `tests/_infra/sim/firmware.py` thoroughly before writing. Key methods:
`Sim.__init__()`, `send_command(line: str) -> str`, `tick()`, `tick_for(ms)`,
`get_async_evts() -> list[str]`, `sim_get_true_pose_x/y/h()`,
`sim_set_motor_slip(left, right)`, `sim_set_otos_linear_noise(stddev)`.

The `Sim` ctypes object is NOT thread-safe for concurrent `tick()` and
`send_command()`. Design: the tick-thread owns the `Sim`; commands from the Qt
thread are placed in a `queue.Queue`; the tick-thread drains the command queue
between ticks, sends each via `send_command`, and stores the reply. The `command()`
call on the Transport puts the command in the queue with a `threading.Event` for
the reply and waits.

### Files to modify

- `host/robot_radio/testgui/transport.py` — add `SimTransport`
- `host/robot_radio/testgui/__main__.py` (or `app.py`) — add Sim to transport
  selector; add lib-present check on connect

### Reuse

- `tests/_infra/sim/firmware.py` — `Sim` class (ctypes)
- `host/robot_radio/robot/protocol.py` — `parse_tlm`, `TLMFrame`

### Testing plan

Manual sim end-to-end: `python -m robot_radio.testgui`, pick Sim, click Connect.
Verify the log shows `STREAM 50` sent; verify TLM lines appear. Send `D 200 200
500`; confirm the robot marker moves. Test lib-missing path by temporarily
renaming the lib; confirm the warning dialog appears. Run simulation gate.

### Documentation updates

None yet. README is written in ticket 010.
