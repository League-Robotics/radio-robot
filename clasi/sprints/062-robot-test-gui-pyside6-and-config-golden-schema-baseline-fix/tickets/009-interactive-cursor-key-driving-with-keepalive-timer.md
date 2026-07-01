---
id: '009'
title: Interactive cursor-key driving with keepalive timer
status: open
use-cases:
- SUC-006
depends-on:
- '008'
issue: plan-robot-test-gui-pyside6.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 009 — Interactive cursor-key driving with keepalive timer

## Description

Implement `testgui/drive.py` — the `KeyboardDriver` class that maps cursor keys
to `VW` commands. `Up`/`Down` = forward/back (`VW ±v 0`); `Left`/`Right` = slow
rotate (`VW 0 ±omega`). A ~100 ms `QTimer` re-sends the command as a keepalive
while the key is held. On release, `STOP` is sent. Qt auto-repeat is suppressed
so a held key produces one logical press (the timer handles re-sends).

Drive speeds are named constants at the top of `drive.py`:
`FWD_SPEED_MMS = 200`, `ROTATE_OMEGA_MRADS = 500` (adjust values to safe defaults).

Guard: if the transport response to a `VW` command contains `vw busy`, log the
message clearly and do not suppress subsequent STOP sends.

Corresponds to item 7 in the approved design's ticket breakdown.

## Acceptance Criteria

- [ ] `host/robot_radio/testgui/drive.py` defines `KeyboardDriver`:
  - `attach(window, transport)` — installs `keyPressEvent` / `keyReleaseEvent`
    overrides on the main window.
  - On press of Up/Down/Left/Right (and no auto-repeat): starts a `QTimer`
    (~100 ms) that calls `transport.send(vw_cmd)` until key is released.
  - On release: stops the timer; calls `transport.send("STOP")`.
  - Qt auto-repeat is suppressed: `event.isAutoRepeat()` check in press handler;
    ignore auto-repeat events.
- [ ] Named constants `FWD_SPEED_MMS` and `ROTATE_OMEGA_MRADS` at the top of
  `drive.py` control the VW speeds.
- [ ] Wire strings emitted:
  - Up held: `VW 200 0` (or `VW <FWD_SPEED_MMS> 0`)
  - Down held: `VW -200 0`
  - Left held: `VW 0 500` (or `VW 0 <ROTATE_OMEGA_MRADS>`)
  - Right held: `VW 0 -500`
  - Release: `STOP`
- [ ] `KeyboardDriver` is inactive (ignores key events) when no transport is
  connected.
- [ ] If the transport reply to a VW command contains `vw busy`: logs the
  warning; does NOT suppress the STOP on release.
- [ ] `uv run python -m pytest tests/simulation` passes.

## Implementation Plan

### Approach

Override `keyPressEvent` and `keyReleaseEvent` on the `QMainWindow` subclass.
Use `event.key()` to identify `Qt.Key_Up`, `Qt.Key_Down`, `Qt.Key_Left`,
`Qt.Key_Right`. Maintain a `QTimer` per-direction or a single timer with a
`_current_cmd` state variable.

Simplest design: one `QTimer` with a `_cmd: str | None` field.
- Press: set `_cmd = "VW ..."`, `_timer.start(100)`.
- Timer timeout: `transport.send(_cmd)`.
- Release: `_timer.stop()`, `transport.send("STOP")`, `_cmd = None`.

### Files to create

- `host/robot_radio/testgui/drive.py` — `KeyboardDriver`

### Files to modify

- `host/robot_radio/testgui/app.py` — instantiate `KeyboardDriver` and call
  `driver.attach(self, transport)` on connect.

### Testing plan

Manual sim: connect to Sim, hold Up arrow, observe robot marker advancing;
release, confirm STOP is logged. Hold Left, observe rotation. The headless
smoke test (ticket 010) validates wire strings programmatically by simulating
key events with `QTest`. Run simulation gate.

### Documentation updates

None yet. README is written in ticket 010.
