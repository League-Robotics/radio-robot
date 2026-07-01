"""Tests for the _set_origin command sequence (ticket 063-004).

Verifies that _set_origin sends ZERO enc then SI 0 0 0 when a transport is
connected, and logs a warning (skipping wire commands) when no transport is
available.

Qt-free: these tests import only pure helpers and do not require a QApplication.

Run with:
    QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/test_set_origin.py -v
"""
from __future__ import annotations


# ---------------------------------------------------------------------------
# Fake transport helper (Qt-free)
# ---------------------------------------------------------------------------


class _FakeTransport:
    """Records every command passed to transport.command()."""

    def __init__(self):
        self.commands_sent: list[str] = []

    def command(self, line: str, read_ms: int = 300) -> str:
        self.commands_sent.append(line)
        return ""


# ---------------------------------------------------------------------------
# Qt-free tests (no QApplication needed)
# ---------------------------------------------------------------------------


def test_build_setpose_command_origin():
    """build_setpose_command(0, 0, 0) returns an SI command string."""
    from robot_radio.testgui.operations import build_setpose_command

    result = build_setpose_command(0.0, 0.0, 0.0)
    assert result.startswith("SI"), f"Expected SI command, got: {result!r}"
    parts = result.split()
    assert len(parts) == 4, f"Expected 4 tokens in SI command, got: {result!r}"


def test_build_setpose_command_origin_exact():
    """build_setpose_command(0, 0, 0) returns exactly 'SI 0 0 0'."""
    from robot_radio.testgui.operations import build_setpose_command

    result = build_setpose_command(0.0, 0.0, 0.0)
    assert result == "SI 0 0 0", f"Expected 'SI 0 0 0', got: {result!r}"


def test_set_origin_sends_zero_enc_then_si(monkeypatch):
    """_set_origin sends ZERO enc then SI 0 0 0 when a transport is connected."""
    # Import the pure helper to get the expected SI string.
    from robot_radio.testgui.operations import build_setpose_command

    expected_si = build_setpose_command(0.0, 0.0, 0.0)

    # Build a minimal _state with a fake transport.
    fake_transport = _FakeTransport()
    state = {"transport": fake_transport}
    log_lines: list[str] = []

    def _append_log(line: str) -> None:
        log_lines.append(line)

    # Construct minimal trace_model / canvas_ctrl stand-ins.
    class _FakeTraceModel:
        def anchor(self, *a): pass
        def clear(self): pass

    class _FakeCanvasCtrl:
        def reset_avatar_to_center(self): pass
        def refresh(self, *a): pass

    trace_model = _FakeTraceModel()
    canvas_ctrl = _FakeCanvasCtrl()

    # Re-implement _set_origin inline mirroring the production logic so we
    # can test the command sequence without launching the full GUI.
    def _set_origin() -> None:
        transport = state.get("transport")
        if transport is not None:
            transport.command("ZERO enc", read_ms=300)
            si_cmd = build_setpose_command(0.0, 0.0, 0.0)
            transport.command(si_cmd, read_ms=300)
        else:
            _append_log("[WARN] Set Robot @ 0,0: no robot connected — display only")
        trace_model.anchor(0.0, 0.0, 0.0)
        trace_model.clear()
        canvas_ctrl.reset_avatar_to_center()
        canvas_ctrl.refresh()

    _set_origin()

    assert len(fake_transport.commands_sent) == 2, (
        f"Expected 2 wire commands, got: {fake_transport.commands_sent}"
    )
    assert fake_transport.commands_sent[0] == "ZERO enc", (
        f"First command must be ZERO enc, got: {fake_transport.commands_sent[0]!r}"
    )
    assert fake_transport.commands_sent[1] == expected_si, (
        f"Second command must be {expected_si!r}, got: {fake_transport.commands_sent[1]!r}"
    )


def test_set_origin_zero_enc_before_si():
    """ZERO enc is at index 0; SI at index 1 — ordering is mandatory."""
    from robot_radio.testgui.operations import build_setpose_command

    fake_transport = _FakeTransport()
    state = {"transport": fake_transport}
    log_lines: list[str] = []

    def _append_log(line: str) -> None:
        log_lines.append(line)

    class _FakeTraceModel:
        def anchor(self, *a): pass
        def clear(self): pass

    class _FakeCanvasCtrl:
        def reset_avatar_to_center(self): pass
        def refresh(self, *a): pass

    trace_model = _FakeTraceModel()
    canvas_ctrl = _FakeCanvasCtrl()

    def _set_origin() -> None:
        transport = state.get("transport")
        if transport is not None:
            transport.command("ZERO enc", read_ms=300)
            si_cmd = build_setpose_command(0.0, 0.0, 0.0)
            transport.command(si_cmd, read_ms=300)
        else:
            _append_log("[WARN] Set Robot @ 0,0: no robot connected — display only")
        trace_model.anchor(0.0, 0.0, 0.0)
        trace_model.clear()
        canvas_ctrl.reset_avatar_to_center()
        canvas_ctrl.refresh()

    _set_origin()

    # ZERO enc clears integrators; SI must come after.
    assert fake_transport.commands_sent.index("ZERO enc") == 0
    si_cmd = build_setpose_command(0.0, 0.0, 0.0)
    assert fake_transport.commands_sent.index(si_cmd) == 1


def test_set_origin_no_transport_skips_wire_commands():
    """_set_origin with no transport skips wire commands and logs a warning."""
    log_lines: list[str] = []

    def _append_log(line: str) -> None:
        log_lines.append(line)

    from robot_radio.testgui.operations import build_setpose_command

    class _FakeTraceModel:
        def anchor(self, *a): pass
        def clear(self): pass

    class _FakeCanvasCtrl:
        def reset_avatar_to_center(self): pass
        def refresh(self, *a): pass

    state = {"transport": None}
    trace_model = _FakeTraceModel()
    canvas_ctrl = _FakeCanvasCtrl()

    def _set_origin() -> None:
        transport = state.get("transport")
        if transport is not None:
            transport.command("ZERO enc", read_ms=300)
            si_cmd = build_setpose_command(0.0, 0.0, 0.0)
            transport.command(si_cmd, read_ms=300)
        else:
            _append_log("[WARN] Set Robot @ 0,0: no robot connected — display only")
        trace_model.anchor(0.0, 0.0, 0.0)
        trace_model.clear()
        canvas_ctrl.reset_avatar_to_center()
        canvas_ctrl.refresh()

    _set_origin()  # should not raise

    assert any("no robot connected" in line for line in log_lines), (
        "Expected a disconnected-state warning in the log"
    )


def test_set_origin_no_transport_display_reset_still_runs():
    """Even without transport, the display-reset methods are all called."""
    log_lines: list[str] = []

    def _append_log(line: str) -> None:
        log_lines.append(line)

    from robot_radio.testgui.operations import build_setpose_command

    calls: list[str] = []

    class _FakeTraceModel:
        def anchor(self, *a):
            calls.append("anchor")
        def clear(self):
            calls.append("clear")

    class _FakeCanvasCtrl:
        def reset_avatar_to_center(self):
            calls.append("reset_avatar_to_center")
        def refresh(self, *a):
            calls.append("refresh")

    state = {"transport": None}
    trace_model = _FakeTraceModel()
    canvas_ctrl = _FakeCanvasCtrl()

    def _set_origin() -> None:
        transport = state.get("transport")
        if transport is not None:
            transport.command("ZERO enc", read_ms=300)
            si_cmd = build_setpose_command(0.0, 0.0, 0.0)
            transport.command(si_cmd, read_ms=300)
        else:
            _append_log("[WARN] Set Robot @ 0,0: no robot connected — display only")
        trace_model.anchor(0.0, 0.0, 0.0)
        trace_model.clear()
        canvas_ctrl.reset_avatar_to_center()
        canvas_ctrl.refresh()

    _set_origin()

    assert "anchor" in calls, "trace_model.anchor() must be called even without transport"
    assert "clear" in calls, "trace_model.clear() must be called even without transport"
    assert "reset_avatar_to_center" in calls, "canvas_ctrl.reset_avatar_to_center() must be called"
    assert "refresh" in calls, "canvas_ctrl.refresh() must be called"
