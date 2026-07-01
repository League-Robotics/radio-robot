---
id: '005'
title: Record / Pause / Stop command+response logging
status: open
use-cases:
- SUC-006
depends-on: []
github-issue: ''
issue: testgui-record-pause-stop-command-log.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Record / Pause / Stop command+response logging

## Description

Add session-recording controls to the Test GUI so that every command sent to
the robot and every response received can be captured, timestamped, and saved
to a file for later review or replay.

Three controls are added: **Record**, **Pause**, **Stop**.

- **Record** — starts a new recording session. From this point, every TX line
  (command sent via `transport.command()` / `transport.send()`) and every RX
  line (telemetry/response arriving via the `on_log` / `on_telemetry` callbacks
  that feed `_append_log`) is appended to the recorder with a monotonic timestamp
  and wall-clock timestamp, and a direction tag (`TX` or `RX`).
- **Pause** — suspends appending without ending the session. Clicking Record (or
  a Resume button) continues into the same session file.
- **Stop** — finalizes the session: flushes any buffered entries and writes the
  file to `recordings/<ISO-timestamp>.jsonl` under the project root (or the
  current working directory). The session ends; all controls return to idle state.

**Recorder core (`testgui/recorder.py`)**: a Qt-free class `SessionRecorder`
that owns the append logic, pause gating, and JSONL serialization. It has no
PySide6 dependency and is testable headlessly. All append calls happen on the
Qt main thread (TX is already on the main thread; RX arrives via the
`_TelemetryBridge`/`_on_log_from_thread` path which already marshals to the
main thread before calling `_append_log`). Thread safety is therefore inherent
from the existing architecture.

**JSONL format**: one JSON object per line.
```json
{"t_mono": 123.456, "t_wall": "2026-07-01T14:23:00.123Z", "dir": "TX", "line": "S 200 200"}
{"t_mono": 123.789, "t_wall": "2026-07-01T14:23:00.456Z", "dir": "RX", "line": "OK"}
```

**Tap point**: the recorder is wired into `_append_log()` in `__main__.py`.
`_append_log` is already the single funnel for all log-pane entries (both TX
command echoes and RX responses). Pass `direction` along (`"TX"` or `"RX"`)
and route to `recorder.append(direction, line)` when recording is active. The
GUI log pane display is unchanged.

**UI placement**: add three `QPushButton` widgets — `record_btn`, `pause_btn`,
`stop_btn` — in a small horizontal `QHBoxLayout` in the left panel, below the
transport controls and above the operations panel. Enable/disable rules:
- Idle: Record enabled; Pause and Stop disabled.
- Recording: Record disabled; Pause and Stop enabled.
- Paused: Record (as Resume) enabled; Pause disabled; Stop enabled.

Button labels: `"Record"`, `"Pause"`, `"Stop"`. No icons required.

**Output directory**: `recordings/` relative to the working directory from
which the GUI is launched. Create the directory if it does not exist. Filename:
`recording_<YYYYMMDD_HHMMSS>.jsonl` using wall-clock time at session start.

**Transport-independent**: works across Sim, Serial, and Relay transports
because the tap is at `_append_log`, not inside any transport class.

**Files to create:**
- `host/robot_radio/testgui/recorder.py` — `SessionRecorder` core class.
- `tests/testgui/test_recorder.py` — headless tests of the recorder core.

**Files to modify:**
- `host/robot_radio/testgui/__main__.py` — add Record/Pause/Stop UI, wire
  buttons to `SessionRecorder`, route `_append_log` calls through the recorder.

## Acceptance Criteria

### Recorder core (`recorder.py`)

- [ ] `SessionRecorder` is importable without PySide6 or any Qt dependency.
- [ ] `start(output_path)` opens a recording session; subsequent `append(dir,
      line)` calls write JSONL entries with `t_mono`, `t_wall`, `dir`, `line`.
- [ ] `pause()` suspends appending; calls to `append()` while paused are
      silently dropped.
- [ ] `resume()` re-enables appending into the same file.
- [ ] `stop()` flushes any in-memory buffer and closes the file handle; further
      `append()` calls are no-ops until `start()` is called again.
- [ ] Each JSONL entry contains: `t_mono` (float, seconds since epoch via
      `time.monotonic()`), `t_wall` (ISO-8601 UTC string), `dir` (`"TX"` or
      `"RX"`), `line` (the raw wire string, stripped of trailing newline).
- [ ] The output file is valid JSONL: each line is a standalone JSON object.
- [ ] `recordings/` directory is created automatically if absent.
- [ ] `state` property returns `"idle"`, `"recording"`, or `"paused"`.

### UI wiring (`__main__.py`)

- [ ] Record, Pause, Stop buttons exist with object names `"record_btn"`,
      `"pause_btn"`, `"stop_btn"`.
- [ ] Initial state: Record enabled, Pause and Stop disabled.
- [ ] After clicking Record: Record disabled, Pause and Stop enabled; recorder
      starts a new session file in `recordings/`.
- [ ] After clicking Pause: Pause disabled, Record (Resume) and Stop enabled;
      appending is suspended.
- [ ] After clicking Record/Resume while paused: Resume disabled, Pause and
      Stop enabled; appending resumes.
- [ ] After clicking Stop: all buttons return to idle state; file is finalized
      and closed; log shows the path of the saved file.
- [ ] TX lines passed to `_append_log` with direction `"TX"` are appended to
      the active recording when recording is active.
- [ ] RX lines passed to `_append_log` with direction `"RX"` are appended to
      the active recording when recording is active.
- [ ] Appending is skipped (no error) when recorder is paused.
- [ ] Appending is skipped (no error) when recorder is idle (not yet started).

### Transport coverage

- [ ] Recording works in Sim mode (TX/RX echoed by SimTransport appear in
      the recording).
- [ ] Recording works in Serial (bench) mode.
- [ ] Recording works in Relay (playfield) mode.

### Headless tests

- [ ] Headless tests of `SessionRecorder` pass without a QApplication.
- [ ] `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui -q` passes.
- [ ] All existing `tests/testgui/` tests pass unchanged.

## Implementation Plan

### Approach

#### 1. Create `recorder.py`

```python
# host/robot_radio/testgui/recorder.py
"""SessionRecorder — Qt-free TX/RX session logger for the Test GUI.

Writes one JSONL line per entry:
    {"t_mono": <float>, "t_wall": "<ISO-UTC>", "dir": "TX"|"RX", "line": "<str>"}

All append() calls must occur on the same thread (the Qt main thread).
"""
from __future__ import annotations

import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Literal

Direction = Literal["TX", "RX"]


class SessionRecorder:
    """Record GUI TX/RX lines to a JSONL file."""

    def __init__(self, recordings_dir: str | Path = "recordings") -> None:
        self._dir = Path(recordings_dir)
        self._file = None
        self._state: str = "idle"  # "idle" | "recording" | "paused"
        self._path: Path | None = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def state(self) -> str:
        return self._state

    @property
    def current_path(self) -> Path | None:
        return self._path

    def start(self, filename: str | None = None) -> Path:
        """Open a new recording session. Returns the output path."""
        if self._state != "idle":
            raise RuntimeError(f"Cannot start: recorder is {self._state!r}")
        self._dir.mkdir(parents=True, exist_ok=True)
        if filename is None:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"recording_{ts}.jsonl"
        self._path = self._dir / filename
        self._file = open(self._path, "w", encoding="utf-8")  # noqa: WPS515
        self._state = "recording"
        return self._path

    def pause(self) -> None:
        if self._state != "recording":
            return
        self._state = "paused"

    def resume(self) -> None:
        if self._state != "paused":
            return
        self._state = "recording"

    def stop(self) -> Path | None:
        """Finalize and close the session file. Returns the path."""
        if self._state == "idle":
            return None
        path = self._path
        if self._file is not None:
            self._file.flush()
            self._file.close()
            self._file = None
        self._state = "idle"
        self._path = None
        return path

    def append(self, direction: Direction, line: str) -> None:
        """Append one TX/RX entry. No-op if idle or paused."""
        if self._state != "recording":
            return
        entry = {
            "t_mono": time.monotonic(),
            "t_wall": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "dir": direction,
            "line": line.rstrip("\r\n"),
        }
        self._file.write(json.dumps(entry) + "\n")
```

#### 2. Extend `_append_log` in `__main__.py`

`_append_log` currently takes `(line: str)`. Extend its signature to accept an
optional `direction` parameter:

```python
def _append_log(line: str, direction: str | None = None) -> None:
    # ... existing log-pane append logic ...
    if direction is not None:
        recorder.append(direction, line)
```

All existing callers that pass no `direction` continue to work unchanged — their
calls do not feed the recorder, which is correct for internal GUI status messages
(mode label changes, connect/disconnect notices, etc.).

TX commands: update the existing `transport.command()` / `transport.send()` call
sites in `__main__.py` to pass `direction="TX"` when calling `_append_log` with
the outgoing line. These are the places in `_on_send_clicked()` and `_on_connect()`
that already log `[TX] ...` lines.

RX lines: the `on_log` transport callback already routes received lines through
`_append_log` (via `_on_log_from_thread` on the main thread). Pass
`direction="RX"` in that callback so incoming telemetry/responses are captured.

#### 3. Add Record/Pause/Stop UI

In `_build_main_window()`, after the transport controls layout:

```python
from robot_radio.testgui.recorder import SessionRecorder

recorder = SessionRecorder()

record_btn = QPushButton("Record")
record_btn.setObjectName("record_btn")
pause_btn = QPushButton("Pause")
pause_btn.setObjectName("pause_btn")
pause_btn.setEnabled(False)
stop_btn = QPushButton("Stop")
stop_btn.setObjectName("stop_btn")
stop_btn.setEnabled(False)

rec_layout = QHBoxLayout()
rec_layout.addWidget(record_btn)
rec_layout.addWidget(pause_btn)
rec_layout.addWidget(stop_btn)
left_layout.addLayout(rec_layout)


def _on_record_clicked() -> None:
    if recorder.state == "idle":
        path = recorder.start()
        _append_log(f"[REC] Recording started: {path}")
        record_btn.setEnabled(False)
        pause_btn.setEnabled(True)
        stop_btn.setEnabled(True)
    elif recorder.state == "paused":
        recorder.resume()
        _append_log("[REC] Recording resumed")
        record_btn.setEnabled(False)
        pause_btn.setEnabled(True)


def _on_pause_clicked() -> None:
    recorder.pause()
    _append_log("[REC] Recording paused")
    record_btn.setText("Resume")
    record_btn.setEnabled(True)
    pause_btn.setEnabled(False)


def _on_stop_clicked() -> None:
    path = recorder.stop()
    record_btn.setText("Record")
    record_btn.setEnabled(True)
    pause_btn.setEnabled(False)
    stop_btn.setEnabled(False)
    if path is not None:
        _append_log(f"[REC] Recording saved: {path}")


record_btn.clicked.connect(_on_record_clicked)
pause_btn.clicked.connect(_on_pause_clicked)
stop_btn.clicked.connect(_on_stop_clicked)
```

#### Tap point detail for TX

In `_on_send_clicked()` the outgoing wire string is assembled and sent. After
calling `transport.command(line)` add `recorder.append("TX", line)` (or route
through `_append_log(f"[TX] {line}", direction="TX")`).

For `ZERO enc` and `SI ...` sent from `_set_origin()` (ticket 004), those
commands also flow through `transport.command()`. The simplest approach is to
pass `direction="TX"` from all call sites that already call `_append_log("[TX]
...")` with an outgoing line; the recorder then captures them automatically.

#### Tap point detail for RX

The existing `_on_log_from_thread(line: str)` slot already calls
`_append_log(line)`. Change it to `_append_log(line, direction="RX")` so that
all RX lines are automatically routed to the recorder.

### Files to create/modify

- `host/robot_radio/testgui/recorder.py`: new file — `SessionRecorder` as above.
- `host/robot_radio/testgui/__main__.py`: extend `_append_log` signature; add
  Record/Pause/Stop UI and handler closures; wire TX/RX direction tags.
- `tests/testgui/test_recorder.py`: new test file (see Testing plan below).

### Testing plan

Create `tests/testgui/test_recorder.py` (Qt-free):

```python
"""Headless tests for SessionRecorder (ticket 063-005)."""
from __future__ import annotations

import json
import time
from pathlib import Path


def test_recorder_idle_state():
    """Recorder starts in idle state."""
    from robot_radio.testgui.recorder import SessionRecorder
    r = SessionRecorder()
    assert r.state == "idle"
    assert r.current_path is None


def test_recorder_start_creates_file(tmp_path):
    """start() creates the recordings directory and output file."""
    from robot_radio.testgui.recorder import SessionRecorder
    r = SessionRecorder(recordings_dir=tmp_path / "recordings")
    path = r.start()
    assert path.exists()
    assert r.state == "recording"
    r.stop()


def test_recorder_append_writes_jsonl(tmp_path):
    """append() writes valid JSONL entries with required fields."""
    from robot_radio.testgui.recorder import SessionRecorder
    r = SessionRecorder(recordings_dir=tmp_path)
    r.start()
    r.append("TX", "S 200 200")
    r.append("RX", "OK")
    path = r.stop()
    lines = path.read_text().splitlines()
    assert len(lines) == 2
    tx = json.loads(lines[0])
    rx = json.loads(lines[1])
    assert tx["dir"] == "TX"
    assert tx["line"] == "S 200 200"
    assert "t_mono" in tx
    assert "t_wall" in tx
    assert rx["dir"] == "RX"
    assert rx["line"] == "OK"


def test_recorder_pause_suppresses_append(tmp_path):
    """append() is a no-op while paused."""
    from robot_radio.testgui.recorder import SessionRecorder
    r = SessionRecorder(recordings_dir=tmp_path)
    r.start()
    r.append("TX", "before pause")
    r.pause()
    assert r.state == "paused"
    r.append("TX", "during pause — should be dropped")
    r.resume()
    assert r.state == "recording"
    r.append("TX", "after resume")
    path = r.stop()
    lines = path.read_text().splitlines()
    assert len(lines) == 2
    assert json.loads(lines[0])["line"] == "before pause"
    assert json.loads(lines[1])["line"] == "after resume"


def test_recorder_stop_returns_path(tmp_path):
    """stop() returns the path of the saved file."""
    from robot_radio.testgui.recorder import SessionRecorder
    r = SessionRecorder(recordings_dir=tmp_path)
    r.start()
    r.append("RX", "hello")
    path = r.stop()
    assert path is not None
    assert path.exists()
    assert r.state == "idle"


def test_recorder_stop_when_idle_returns_none():
    """stop() is a no-op when idle; returns None."""
    from robot_radio.testgui.recorder import SessionRecorder
    r = SessionRecorder()
    assert r.stop() is None


def test_recorder_idle_append_is_noop(tmp_path):
    """append() while idle does not raise and does not create a file."""
    from robot_radio.testgui.recorder import SessionRecorder
    r = SessionRecorder(recordings_dir=tmp_path)
    r.append("TX", "should be dropped")
    # No file should have been created.
    assert not any(tmp_path.iterdir())


def test_recorder_jsonl_line_strips_trailing_newline(tmp_path):
    """append() strips trailing \\r\\n from the line before writing."""
    from robot_radio.testgui.recorder import SessionRecorder
    r = SessionRecorder(recordings_dir=tmp_path)
    r.start()
    r.append("RX", "telemetry line\r\n")
    path = r.stop()
    entry = json.loads(path.read_text().splitlines()[0])
    assert entry["line"] == "telemetry line"
```

Run with:

```
QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/test_recorder.py -v
```

### Documentation updates

- Add a module docstring to `recorder.py` documenting the state machine
  (`idle` → `recording` ↔ `paused` → `idle`), the JSONL schema, and the
  threading assumption (all append() calls on the Qt main thread).
- Update the `__main__.py` module docstring to mention the Record/Pause/Stop
  controls and `SessionRecorder`.
