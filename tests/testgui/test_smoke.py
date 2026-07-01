"""tests/testgui/test_smoke.py — Headless end-to-end smoke tests for the testgui package.

Runs with ``QT_QPA_PLATFORM=offscreen`` (set by conftest.py).  No display
server, no hardware, and no sim lib are required.  All external I/O is
replaced by :class:`FakeTransport`.

Run:
    QT_QPA_PLATFORM=offscreen uv run --group gui python -m pytest tests/testgui/ -v

Tests
-----
test_app_opens
    Construct a QMainWindow via ``_build_main_window()`` and confirm no
    exception is raised.

test_trace_model_feeds_tlm
    Push 3 synthetic TLMFrame objects through ``TraceModel.feed()`` and
    assert each of the encoder, otos, and fused trace lists accumulates
    3 points.

test_robot_marker_moves
    After feeding TLM frames with increasing fused pose, assert the robot
    marker's scene position differs from the origin (it was updated).

test_command_rows_emit_correct_wire_strings
    For each command row (S, T, D, R, TURN, RT, G), call ``build_wire_string``
    with known values and a FakeTransport, and assert the emitted wire
    string equals the expected value.

test_turn_row_converts_degrees_to_centidegrees
    Set the TURN heading field to 9000 (centidegrees = 90°) and assert the
    wire string contains "9000".
"""

from __future__ import annotations

import math

import pytest


# ---------------------------------------------------------------------------
# FakeTransport — no hardware, no threads, captures sent lines.
# ---------------------------------------------------------------------------


def _make_fake_transport():
    """Return a FakeTransport instance (constructed after imports are safe)."""
    from robot_radio.testgui.transport import Transport

    class FakeTransport(Transport):
        """Drop-in Transport that captures every sent line."""

        def __init__(self) -> None:
            super().__init__()
            self.sent: list[str] = []

        @property
        def last_sent(self) -> str | None:
            """Return the most recently sent line, or None."""
            return self.sent[-1] if self.sent else None

        def connect(self) -> None:
            pass

        def disconnect(self) -> None:
            pass

        def send(self, line: str) -> None:
            self.sent.append(line)

        def command(self, line: str, read_ms: int = 200) -> str:
            self.sent.append(line)
            return "OK"

    return FakeTransport()


# ---------------------------------------------------------------------------
# TLMFrame helper
# ---------------------------------------------------------------------------


def _make_frame(
    *,
    enc: tuple[int, int] | None = None,
    otos: tuple[int, int, int] | None = None,
    pose: tuple[int, int, int] | None = None,
    t: int = 0,
):
    """Build a minimal TLMFrame without going through the firmware parser."""
    from robot_radio.robot.protocol import TLMFrame

    return TLMFrame(t=t, enc=enc, otos=otos, pose=pose)


# ---------------------------------------------------------------------------
# QApplication fixture (session-scoped, shared by all tests in the suite)
# ---------------------------------------------------------------------------


@pytest.fixture(scope="session")
def qapp():
    """Ensure a QApplication exists for the whole test session.

    ``QT_QPA_PLATFORM=offscreen`` is already set by conftest.py before this
    import runs.
    """
    import sys
    from PySide6.QtWidgets import QApplication  # type: ignore[import-untyped]

    app = QApplication.instance() or QApplication(sys.argv)
    yield app
    # Do NOT call app.quit() — other tests in the same session may need it.


# ---------------------------------------------------------------------------
# test_app_opens
# ---------------------------------------------------------------------------


class TestAppOpens:
    """Smoke test: construct QMainWindow with the full build function."""

    def test_app_opens(self, qapp):
        """_build_main_window() must not raise and must return a QMainWindow."""
        from PySide6.QtWidgets import QMainWindow  # type: ignore[import-untyped]
        from robot_radio.testgui.__main__ import _build_main_window

        window, app = _build_main_window()
        assert isinstance(window, QMainWindow), (
            f"Expected QMainWindow, got {type(window)}"
        )
        # Window is valid and has the expected title.
        assert window.windowTitle() == "Robot Test GUI"
        # Clean up — hide rather than close so subsequent tests can still
        # use the same QApplication.
        window.hide()


# ---------------------------------------------------------------------------
# test_tour_button_present
# ---------------------------------------------------------------------------


class TestTourButton:
    """The Tour 1 button exists, starts disabled, and enables on connect."""

    def test_tour_button_present_and_disabled(self, qapp):
        """A 'Tour 1' QPushButton exists and is disabled before connect."""
        from PySide6.QtWidgets import QPushButton  # type: ignore[import-untyped]
        from robot_radio.testgui.__main__ import _build_main_window

        window, _app = _build_main_window()
        try:
            btn = window.findChild(QPushButton, "tour_btn_tour_1")
            assert btn is not None, "Tour 1 button not found"
            assert btn.text() == "Tour 1"
            assert not btn.isEnabled(), "Tour button should start disabled"
        finally:
            window.hide()


# ---------------------------------------------------------------------------
# test_trace_model_feeds_tlm
# ---------------------------------------------------------------------------


class TestTraceModelFeedsTlm:
    """Push 3 synthetic TLMFrames and verify all trace lists accumulate 3 pts."""

    def test_trace_model_feeds_tlm(self):
        """TraceModel.feed() must append one point per sensor per frame.

        The first frame establishes the baseline (anchor point); subsequent
        frames append displacement points.  Feeding 3 frames → 3 points.
        """
        from robot_radio.testgui.traces import TraceModel

        model = TraceModel()
        model.anchor(0.0, 0.0, 0.0)

        # Push 3 frames, each carrying all three on-robot sensors with
        # incrementally increasing values so each frame produces a new point.
        for i in range(3):
            frame = _make_frame(
                enc=(i * 100, i * 100),
                otos=(i * 100, 0, 0),
                pose=(i * 100, 0, 0),
                t=i * 50,
            )
            model.feed(frame)

        assert len(model.encoder) == 3, (
            f"encoder trace: expected 3 points, got {len(model.encoder)}"
        )
        assert len(model.otos) == 3, (
            f"otos trace: expected 3 points, got {len(model.otos)}"
        )
        assert len(model.fused) == 3, (
            f"fused trace: expected 3 points, got {len(model.fused)}"
        )


# ---------------------------------------------------------------------------
# test_robot_marker_moves
# ---------------------------------------------------------------------------


class TestRobotMarkerMoves:
    """After feeding TLM with fused pose data, the robot marker must update."""

    def test_robot_marker_moves(self, qapp):
        """CanvasController.refresh() must move the marker from the origin."""
        from robot_radio.testgui.traces import TraceModel
        from robot_radio.testgui.canvas import build_canvas

        model = TraceModel()
        model.anchor(0.0, 0.0, 0.0)

        _canvas_widget, canvas_ctrl = build_canvas(model)

        # Avatar is always visible — it starts at world (0,0) centre.
        assert canvas_ctrl._marker_group.isVisible(), (
            "Marker should be visible at startup (shown at centre before any TLM)"
        )

        # Feed first frame — establishes baseline at anchor (0, 0).
        model.feed(_make_frame(pose=(0, 0, 0)))
        canvas_ctrl.refresh(fused_yaw_rad=0.0)

        # Capture position after first frame (should be at canvas origin ≈ centre).
        pos_after_first = canvas_ctrl._marker_group.pos()
        assert canvas_ctrl._marker_group.isVisible(), (
            "Marker should be visible after feeding first fused frame"
        )

        # Feed second frame with x displacement = 1000 mm = 100 cm.
        model.feed(_make_frame(pose=(1000, 0, 0)))
        canvas_ctrl.refresh(fused_yaw_rad=0.0)

        pos_after_second = canvas_ctrl._marker_group.pos()

        # Marker position must have changed in x (eastward displacement).
        assert pos_after_second.x() != pos_after_first.x(), (
            f"Marker x position did not change: "
            f"before={pos_after_first.x():.1f}, after={pos_after_second.x():.1f}"
        )


# ---------------------------------------------------------------------------
# test_command_rows_emit_correct_wire_strings
# ---------------------------------------------------------------------------


class TestCommandRowsEmitCorrectWireStrings:
    """build_wire_string emits the expected wire string for every command."""

    def _build_wire(self, label: str, values: dict) -> str:
        from robot_radio.testgui.commands import COMMANDS, build_wire_string

        spec = next(s for s in COMMANDS if s["label"] == label)
        fake = _make_fake_transport()
        line = build_wire_string(spec, values)
        fake.command(line)
        return fake.last_sent  # type: ignore[return-value]

    def test_s_row_wire_string(self):
        result = self._build_wire("S", {"left": 200, "right": -150})
        assert result == "S 200 -150", f"S row: expected 'S 200 -150', got {result!r}"

    def test_t_row_wire_string(self):
        result = self._build_wire("T", {"left": 200, "right": 200, "ms": 1000})
        assert result == "T 200 200 1000", (
            f"T row: expected 'T 200 200 1000', got {result!r}"
        )

    def test_d_row_wire_string(self):
        result = self._build_wire("D", {"left": 300, "right": 300, "mm": 500})
        assert result == "D 300 300 500", (
            f"D row: expected 'D 300 300 500', got {result!r}"
        )

    def test_r_row_wire_string(self):
        result = self._build_wire("R", {"speed": 200, "radius": 500})
        assert result == "R 200 500", (
            f"R row: expected 'R 200 500', got {result!r}"
        )

    def test_turn_row_wire_string(self):
        result = self._build_wire("TURN", {"heading": 9000, "eps": 0})
        assert result == "TURN 9000", (
            f"TURN row: expected 'TURN 9000', got {result!r}"
        )

    def test_rt_row_wire_string(self):
        result = self._build_wire("RT", {"deg": 90})
        assert result == "RT 9000", (
            f"RT row: expected 'RT 9000', got {result!r}"
        )

    def test_g_row_wire_string(self):
        result = self._build_wire("G", {"x": 500, "y": 300, "speed": 200})
        assert result == "G 500 300 200", (
            f"G row: expected 'G 500 300 200', got {result!r}"
        )


# ---------------------------------------------------------------------------
# test_turn_row_converts_degrees_to_centidegrees
# ---------------------------------------------------------------------------


class TestTurnRowCentidegrees:
    """The TURN command field accepts centidegrees; 9000 cdeg = 90°.

    The COMMANDS schema stores TURN heading in centidegrees (unit='cdeg').
    Setting the heading field to 9000 (= 90° in cdeg) must produce a wire
    string containing '9000'.
    """

    def test_turn_row_converts_degrees_to_centidegrees(self):
        """Setting TURN heading spinbox to 9000 cdeg emits '9000' on the wire.

        The TURN row stores its heading field in centidegrees (1 cdeg = 0.01°).
        Setting the field value to 9000 represents 90°.  The wire string must
        contain '9000' (no degree-to-centidegree conversion; the field is
        already in cdeg).
        """
        from robot_radio.testgui.commands import COMMANDS, build_wire_string

        turn_spec = next(s for s in COMMANDS if s["label"] == "TURN")

        # Set heading to 9000 cdeg (= 90°).
        values = {"heading": 9000, "eps": 0}
        wire = build_wire_string(turn_spec, values)

        assert "9000" in wire, (
            f"Expected '9000' in TURN wire string for 9000 cdeg heading, got: {wire!r}"
        )
        assert wire == "TURN 9000", (
            f"Expected 'TURN 9000', got {wire!r}"
        )
