"""src/tests/testgui/test_recorder.py -- ticket 085-008: session recorder test
port. Ported from ``tests_old/testgui/test_recorder.py``.

Headless tests for SessionRecorder (ticket 063-005). These tests exercise
the recorder core (append, pause gating, resume, stop, JSONL correctness)
entirely without PySide6 or a QApplication -- the most independent module
in this sprint (zero transport/runner coupling).

No production code change: pure verification pass.

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest src/tests/testgui/test_recorder.py -v
"""
from __future__ import annotations

import json
from pathlib import Path


# ---------------------------------------------------------------------------
# Recorder core -- headless tests (no QApplication required)
# ---------------------------------------------------------------------------


def test_recorder_idle_state():
    """Recorder starts in idle state with no current path."""
    from robot_radio.testgui.recorder import SessionRecorder

    r = SessionRecorder()
    assert r.state == "idle"
    assert r.current_path is None


def test_recorder_start_creates_file(tmp_path):
    """start() creates the recordings directory and a non-empty output file."""
    from robot_radio.testgui.recorder import SessionRecorder

    r = SessionRecorder(recordings_dir=tmp_path / "recordings")
    path = r.start()
    assert path.exists(), f"Recording file not found: {path}"
    assert r.state == "recording"
    r.stop()


def test_recorder_start_returns_path(tmp_path):
    """start() returns a Path inside the recordings_dir."""
    from robot_radio.testgui.recorder import SessionRecorder

    rec_dir = tmp_path / "recordings"
    r = SessionRecorder(recordings_dir=rec_dir)
    path = r.start()
    assert isinstance(path, Path)
    assert path.parent == rec_dir
    r.stop()


def test_recorder_state_transitions(tmp_path):
    """State transitions: idle -> recording -> paused -> recording -> idle."""
    from robot_radio.testgui.recorder import SessionRecorder

    r = SessionRecorder(recordings_dir=tmp_path)
    assert r.state == "idle"
    r.start()
    assert r.state == "recording"
    r.pause()
    assert r.state == "paused"
    r.resume()
    assert r.state == "recording"
    r.stop()
    assert r.state == "idle"


def test_recorder_append_writes_jsonl(tmp_path):
    """append() writes valid JSONL entries with all required fields."""
    from robot_radio.testgui.recorder import SessionRecorder

    r = SessionRecorder(recordings_dir=tmp_path)
    r.start()
    r.append("TX", "S 200 200")
    r.append("RX", "OK")
    path = r.stop()

    assert path is not None
    lines = path.read_text().splitlines()
    assert len(lines) == 2, f"Expected 2 JSONL lines, got {len(lines)}: {lines}"

    tx = json.loads(lines[0])
    rx = json.loads(lines[1])

    # TX entry
    assert tx["dir"] == "TX"
    assert tx["line"] == "S 200 200"
    assert "t_mono" in tx, "Missing t_mono field"
    assert "t_wall" in tx, "Missing t_wall field"
    assert isinstance(tx["t_mono"], float)
    assert isinstance(tx["t_wall"], str)

    # RX entry
    assert rx["dir"] == "RX"
    assert rx["line"] == "OK"
    assert "t_mono" in rx
    assert "t_wall" in rx


def test_recorder_pause_suppresses_append(tmp_path):
    """append() is silently dropped while paused; resumes correctly."""
    from robot_radio.testgui.recorder import SessionRecorder

    r = SessionRecorder(recordings_dir=tmp_path)
    r.start()
    r.append("TX", "before pause")
    r.pause()
    assert r.state == "paused"
    r.append("TX", "during pause -- should be dropped")
    r.append("RX", "also dropped")
    r.resume()
    assert r.state == "recording"
    r.append("TX", "after resume")
    path = r.stop()

    lines = path.read_text().splitlines()
    assert len(lines) == 2, (
        f"Expected 2 lines (before + after resume), got {len(lines)}: {lines}"
    )
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
    """stop() is a no-op when idle; returns None without raising."""
    from robot_radio.testgui.recorder import SessionRecorder

    r = SessionRecorder()
    result = r.stop()
    assert result is None
    assert r.state == "idle"


def test_recorder_idle_append_is_noop(tmp_path):
    """append() while idle does not raise and does not create a file."""
    from robot_radio.testgui.recorder import SessionRecorder

    r = SessionRecorder(recordings_dir=tmp_path)
    r.append("TX", "should be dropped")
    # No file should have been created in the recordings dir.
    assert not any(tmp_path.iterdir()), "No file should be created when idle"


def test_recorder_jsonl_line_strips_trailing_newline(tmp_path):
    """append() strips trailing \\r\\n from the line before writing."""
    from robot_radio.testgui.recorder import SessionRecorder

    r = SessionRecorder(recordings_dir=tmp_path)
    r.start()
    r.append("RX", "telemetry line\r\n")
    path = r.stop()

    entry = json.loads(path.read_text().splitlines()[0])
    assert entry["line"] == "telemetry line", (
        f"Expected stripped line, got {entry['line']!r}"
    )


def test_recorder_jsonl_strips_lf_only(tmp_path):
    """append() strips a bare \\n as well as \\r\\n."""
    from robot_radio.testgui.recorder import SessionRecorder

    r = SessionRecorder(recordings_dir=tmp_path)
    r.start()
    r.append("TX", "cmd\n")
    path = r.stop()

    entry = json.loads(path.read_text().splitlines()[0])
    assert entry["line"] == "cmd"


def test_recorder_each_line_is_valid_json(tmp_path):
    """Every line in the output file is parseable as a standalone JSON object."""
    from robot_radio.testgui.recorder import SessionRecorder

    r = SessionRecorder(recordings_dir=tmp_path)
    r.start()
    for i in range(10):
        r.append("TX" if i % 2 == 0 else "RX", f"line {i}")
    path = r.stop()

    for raw_line in path.read_text().splitlines():
        obj = json.loads(raw_line)  # raises if invalid
        assert "t_mono" in obj
        assert "t_wall" in obj
        assert "dir" in obj
        assert "line" in obj


def test_recorder_start_raises_when_already_recording(tmp_path):
    """start() raises RuntimeError if the recorder is not idle."""
    from robot_radio.testgui.recorder import SessionRecorder
    import pytest

    r = SessionRecorder(recordings_dir=tmp_path)
    r.start()
    with pytest.raises(RuntimeError, match="recording"):
        r.start()
    r.stop()


def test_recorder_start_raises_when_paused(tmp_path):
    """start() raises RuntimeError if the recorder is paused."""
    from robot_radio.testgui.recorder import SessionRecorder
    import pytest

    r = SessionRecorder(recordings_dir=tmp_path)
    r.start()
    r.pause()
    with pytest.raises(RuntimeError, match="paused"):
        r.start()
    r.stop()


def test_recorder_t_wall_is_utc_iso(tmp_path):
    """t_wall field is an ISO-8601 string with UTC timezone info."""
    from robot_radio.testgui.recorder import SessionRecorder
    from datetime import datetime

    r = SessionRecorder(recordings_dir=tmp_path)
    r.start()
    r.append("TX", "ping")
    path = r.stop()

    entry = json.loads(path.read_text().splitlines()[0])
    t_wall = entry["t_wall"]
    # Should parse as a timezone-aware datetime.
    dt = datetime.fromisoformat(t_wall)
    assert dt.tzinfo is not None, f"t_wall has no timezone info: {t_wall!r}"


def test_recorder_current_path_set_during_session(tmp_path):
    """current_path is set while recording and cleared after stop."""
    from robot_radio.testgui.recorder import SessionRecorder

    r = SessionRecorder(recordings_dir=tmp_path)
    assert r.current_path is None
    r.start()
    assert r.current_path is not None
    r.pause()
    assert r.current_path is not None  # still set while paused
    r.resume()
    assert r.current_path is not None
    path_before_stop = r.current_path
    r.stop()
    assert r.current_path is None  # cleared after stop
    assert path_before_stop.exists()


def test_recorder_custom_filename(tmp_path):
    """start(filename=...) uses the supplied filename."""
    from robot_radio.testgui.recorder import SessionRecorder

    r = SessionRecorder(recordings_dir=tmp_path)
    path = r.start(filename="my_session.jsonl")
    assert path.name == "my_session.jsonl"
    r.stop()


def test_recorder_append_after_stop_is_noop(tmp_path):
    """append() after stop() does not raise and does not modify the closed file."""
    from robot_radio.testgui.recorder import SessionRecorder

    r = SessionRecorder(recordings_dir=tmp_path)
    r.start()
    r.append("TX", "recorded")
    path = r.stop()

    size_after_stop = path.stat().st_size

    # These should be no-ops, not errors.
    r.append("TX", "should be dropped")
    r.append("RX", "also dropped")

    assert path.stat().st_size == size_after_stop, (
        "File grew after stop() -- append() should be a no-op when idle"
    )


# ---------------------------------------------------------------------------
# _append_log direction-routing tests (Qt-free simulation)
# ---------------------------------------------------------------------------

def test_append_log_with_tx_direction_feeds_recorder(tmp_path):
    """Simulating _append_log(text, direction='TX') routes to the recorder."""
    from robot_radio.testgui.recorder import SessionRecorder

    recorder = SessionRecorder(recordings_dir=tmp_path)
    recorder.start()

    logged_lines: list[str] = []

    def _append_log(text: str, direction: str | None = None) -> None:
        logged_lines.append(text)
        if direction is not None:
            recorder.append(direction, text)  # type: ignore[arg-type]

    # Simulate a TX command echoed to the log.
    _append_log("TX S 100 100", direction="TX")
    # Simulate an internal status message (no direction -> not recorded).
    _append_log("[INFO] Connected via Sim")

    path = recorder.stop()
    lines = path.read_text().splitlines()
    assert len(lines) == 1, f"Expected 1 recorded line, got {len(lines)}"
    entry = json.loads(lines[0])
    assert entry["dir"] == "TX"
    assert entry["line"] == "TX S 100 100"


def test_append_log_with_rx_direction_feeds_recorder(tmp_path):
    """Simulating _append_log(text, direction='RX') routes to the recorder."""
    from robot_radio.testgui.recorder import SessionRecorder

    recorder = SessionRecorder(recordings_dir=tmp_path)
    recorder.start()

    logged_lines: list[str] = []

    def _append_log(text: str, direction: str | None = None) -> None:
        logged_lines.append(text)
        if direction is not None:
            recorder.append(direction, text)  # type: ignore[arg-type]

    _append_log("OK", direction="RX")
    _append_log("T 100 200 50 ...", direction="RX")
    # Internal GUI message -- not recorded.
    _append_log("[INFO] STREAM 50 sent")

    path = recorder.stop()
    lines = path.read_text().splitlines()
    assert len(lines) == 2, f"Expected 2 recorded RX lines, got {len(lines)}"
    assert json.loads(lines[0])["dir"] == "RX"
    assert json.loads(lines[1])["dir"] == "RX"


def test_append_log_no_direction_not_recorded(tmp_path):
    """_append_log calls without direction are not written to the recorder."""
    from robot_radio.testgui.recorder import SessionRecorder

    recorder = SessionRecorder(recordings_dir=tmp_path)
    recorder.start()

    def _append_log(text: str, direction: str | None = None) -> None:
        if direction is not None:
            recorder.append(direction, text)  # type: ignore[arg-type]

    _append_log("[INFO] Connected via Sim")
    _append_log("[WARN] Set Robot @ 0,0: no robot connected -- display only")
    _append_log("[REC] Recording started: /tmp/recording_xyz.jsonl")

    path = recorder.stop()
    assert path is not None
    content = path.read_text()
    assert content == "", f"Expected empty file, got: {content!r}"


def test_append_log_paused_does_not_record(tmp_path):
    """_append_log with direction is silently dropped while the recorder is paused."""
    from robot_radio.testgui.recorder import SessionRecorder

    recorder = SessionRecorder(recordings_dir=tmp_path)
    recorder.start()

    def _append_log(text: str, direction: str | None = None) -> None:
        if direction is not None:
            recorder.append(direction, text)  # type: ignore[arg-type]

    _append_log("TX cmd1", direction="TX")
    recorder.pause()
    _append_log("TX cmd2", direction="TX")  # should be dropped
    recorder.resume()
    _append_log("TX cmd3", direction="TX")

    path = recorder.stop()
    lines = path.read_text().splitlines()
    assert len(lines) == 2
    assert json.loads(lines[0])["line"] == "TX cmd1"
    assert json.loads(lines[1])["line"] == "TX cmd3"


# ---------------------------------------------------------------------------
# direction_from_marker -- TX/RX inference from formatted log lines
# ---------------------------------------------------------------------------

def test_direction_from_marker_tx():
    """A ``>`` marker after the timestamp is a transmitted command -> TX."""
    from robot_radio.testgui.recorder import direction_from_marker

    assert direction_from_marker("[10:46:44] > SNAP") == "TX"
    assert direction_from_marker("[10:46:44] > RT 4500") == "TX"


def test_direction_from_marker_rx():
    """A ``<`` marker after the timestamp is a received reply/telemetry -> RX."""
    from robot_radio.testgui.recorder import direction_from_marker

    assert direction_from_marker("[10:46:44] < OK oz") == "RX"
    assert direction_from_marker("[10:46:44] < TLM t=6640 mode=V seq=20") == "RX"


def test_direction_from_marker_status_lines_are_none():
    """Internal status lines carry no marker -> None (not recorded)."""
    from robot_radio.testgui.recorder import direction_from_marker

    assert direction_from_marker("[10:46:44] [INFO] Connected via Sim") is None
    assert direction_from_marker("[10:46:44] [WARN] No relay found") is None
    assert direction_from_marker("[10:46:44] [ERROR] Connect failed: boom") is None
    assert direction_from_marker("[REC] Recording started: /tmp/x.jsonl") is None


def test_direction_from_marker_without_timestamp_prefix():
    """Marker inference also works on lines lacking the ``[HH:MM:SS] `` prefix."""
    from robot_radio.testgui.recorder import direction_from_marker

    assert direction_from_marker("> SI 0 0 0") == "TX"
    assert direction_from_marker("< OK setpose") == "RX"


# ---------------------------------------------------------------------------
# Fixed-name "latest" capture -- overwrite-on-restart semantics
# ---------------------------------------------------------------------------


def test_recorder_fixed_name_overwrites_on_restart(tmp_path):
    """Re-starting with the same filename truncates the prior contents.

    This guards the always-on "latest.jsonl" capture: each connect calls
    start("latest.jsonl") again, and the file must be overwritten (not
    appended), so it always reflects only the most recent session.
    """
    from robot_radio.testgui.recorder import SessionRecorder

    rec_dir = tmp_path / "recordings"
    r = SessionRecorder(recordings_dir=rec_dir)

    # First session: two lines.
    p1 = r.start("latest.jsonl")
    r.append("TX", "> RT 4500")
    r.append("RX", "< OK rt rot=4500")
    r.stop()
    first = p1.read_text().strip().splitlines()
    assert len(first) == 2

    # Second session: one line, same fixed filename.
    p2 = r.start("latest.jsonl")
    assert p2 == p1, "latest capture must reuse the same fixed path"
    r.append("TX", "> D 200 200 420")
    r.stop()

    lines = p2.read_text().strip().splitlines()
    assert len(lines) == 1, "restart must overwrite, not append to, latest.jsonl"
    assert json.loads(lines[0])["line"] == "> D 200 200 420"
    assert "RT 4500" not in p2.read_text(), "stale first-session data survived"
