---
id: '004'
title: Transport ABC and Serial/Relay wrappers with TLM reader thread
status: open
use-cases:
- SUC-003
- SUC-004
- SUC-013
- SUC-014
depends-on:
- '003'
issue: plan-robot-test-gui-pyside6.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 004 — Transport ABC and Serial/Relay wrappers with TLM reader thread

## Description

Define `testgui/transport.py` with the `Transport` ABC and two concrete
backends: `SerialTransport` (mode="direct") and `RelayTransport` (mode="relay").
Both wrap `SerialConnection` from `host/robot_radio/io/serial_conn.py`. Wire the
TLM reader thread and connect/disconnect lifecycle to the skeleton window's
transport selector from ticket 003. Wire the log pane to show sent and received
lines.

The aprilcam camera-truth polling thread is also implemented here for both
hardware backends.

Corresponds to item 2 in the approved design's ticket breakdown.

## Acceptance Criteria

- [ ] `host/robot_radio/testgui/transport.py` exists and defines:
  - `Transport` ABC with `send(line: str) -> None`, `command(line: str, read_ms: int = 200) -> str`,
    a `telemetry` callback/signal delivering parsed `TLMFrame`, and a `truth`
    callback delivering `(x_cm, y_cm, yaw_rad) | None`.
  - `SerialTransport(port: str)` — wraps `SerialConnection(port, mode="direct")`.
  - `RelayTransport(port: str)` — wraps `SerialConnection(port, mode="relay")`.
- [ ] Both concrete backends start a reader thread on `connect()` that:
  - Reads lines from the serial connection.
  - Calls `parse_tlm(line)` (from `robot/protocol.py`) on TLM lines;
    invokes the `telemetry` callback with the resulting `TLMFrame`.
  - Forwards all lines (sent and received) to the log pane callback.
- [ ] Both backends start a camera-truth polling thread on `connect()` that:
  - Calls `read_camera_pose` (from `testkit/camera.py`) for tag 100.
  - Invokes the `truth` callback with the world pose.
  - Handles daemon-not-available gracefully (logs warning, does not crash).
- [ ] `disconnect()` joins all threads cleanly; no dangling threads.
- [ ] Transport selector in the window enables a port-picker `QLineEdit` when
  Serial or Relay is selected; clicking **Connect** calls `transport.connect()`
  and `transport.command("STREAM 50")`.
- [ ] Log pane shows each sent line and each received reply/TLM line timestamped.
- [ ] `uv run python -m pytest tests/simulation` passes.

## Implementation Plan

### Approach

Read `host/robot_radio/io/serial_conn.py` before writing anything — understand
`SerialConnection`'s `write()`, `readline()`, and `mode` parameter. Read
`host/robot_radio/robot/protocol.py` for `parse_tlm` and `TLMFrame`. Read
`host/robot_radio/testkit/camera.py` for `read_camera_pose`.

Use `threading.Thread(daemon=True)` for both the TLM reader and the truth
poller. Use a `threading.Event` stop-flag to signal threads on disconnect.

The `telemetry` callback can be a simple callable; in Qt context it should be
delivered via `QMetaObject.invokeMethod` or a Qt signal to avoid cross-thread
widget access. Use `QtCore.Signal` on the Transport subclasses or a wrapper.

### Files to create

- `host/robot_radio/testgui/transport.py`

### Files to modify

- `host/robot_radio/testgui/__main__.py` (or extracted `app.py`) — wire
  transport selector, Connect button, and log pane to the new Transport.

### Reuse

- `host/robot_radio/io/serial_conn.py` — `SerialConnection`
- `host/robot_radio/robot/protocol.py` — `parse_tlm`, `TLMFrame`, `NezhaProtocol`
- `host/robot_radio/testkit/camera.py` — `read_camera_pose`

### Testing plan

Manual with a connected robot: select Serial, pick the port, click Connect.
Confirm TLM lines appear in the log pane. Confirm Disconnect cleans up.
Manual with relay dongle: same via Relay transport. Run simulation gate.

### Documentation updates

None yet. README is written in ticket 010.
