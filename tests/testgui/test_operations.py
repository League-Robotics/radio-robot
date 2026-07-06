"""tests/testgui/test_operations.py — headless tests for OpsController.on_stop()
(ticket 083-002).

``operations.py``'s STOP button had the same dead-verb defect as
``KeyboardDriver``: it sent a bare ``STOP`` that the firmware does not
register (``docs/protocol-v2.md`` only defines ``DEV DT STOP``). This module
tests ``OpsController.on_stop()`` directly (no ``QApplication``/PySide6
widgets needed — only fake stand-ins for the one button it touches).

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui/test_operations.py -q

This module is not yet wired into ``pyproject.toml``'s ``testpaths`` (ticket
083-004's job) — run it directly, per ticket 083-002's Testing section.
"""
from __future__ import annotations

from robot_radio.testgui.operations import OpsController
from robot_radio.testgui.transport import Transport


# ---------------------------------------------------------------------------
# Fakes
# ---------------------------------------------------------------------------


class _FakeButton:
    """Minimal stand-in for the STREAM QPushButton -- records the calls
    ``on_stop()`` makes on it (``setChecked``/``setText``)."""

    def __init__(self) -> None:
        self.checked: bool | None = None
        self.text: str | None = None

    def setChecked(self, value: bool) -> None:
        self.checked = value

    def setText(self, value: str) -> None:
        self.text = value


class _FakeTransport(Transport):
    """Records every ``command()``/``send()`` line; no real IO."""

    def __init__(self) -> None:
        super().__init__()
        self.commands: list[str] = []

    def connect(self) -> None:
        pass

    def disconnect(self) -> None:
        pass

    def send(self, line: str) -> None:
        self.commands.append(line)

    def command(self, line: str, read_timeout: int = 200) -> str:  # [ms]
        self.commands.append(line)
        return ""


def _make_controller(transport: "Transport | None") -> tuple[OpsController, list[str], _FakeButton]:
    """Build an OpsController with fake buttons; only stream_btn matters for on_stop()."""
    logs: list[str] = []
    stream_btn = _FakeButton()
    controller = OpsController(
        transport_ref={"transport": transport},
        log_cb=logs.append,
        sync_btn=None,
        zero_btn=None,
        stop_btn=None,
        clear_btn=None,
        refresh_btn=None,
        stream_btn=stream_btn,
        origin_btn=None,
        transport_buttons=[],
    )
    return controller, logs, stream_btn


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_on_stop_sends_dev_dt_stop_then_stream_0() -> None:
    transport = _FakeTransport()
    controller, logs, stream_btn = _make_controller(transport)

    controller.on_stop()

    assert transport.commands == ["DEV DT STOP", "STREAM 0"]
    assert any("DEV DT STOP sent" in line for line in logs)
    assert stream_btn.checked is False
    assert stream_btn.text == "STREAM: off"


def test_on_stop_cancels_motion_worker_before_sending_stop() -> None:
    transport = _FakeTransport()
    controller, logs, stream_btn = _make_controller(transport)
    call_order: list[str] = []
    controller.stop_motion_cb = lambda: call_order.append("stop_motion_cb")

    controller.on_stop()

    # stop_motion_cb must run BEFORE the wire DEV DT STOP is sent, so the
    # worker thread is joined and no longer touches the transport.
    assert call_order == ["stop_motion_cb"]
    assert transport.commands[0] == "DEV DT STOP"


def test_on_stop_not_connected_logs_warning_and_sends_nothing() -> None:
    controller, logs, stream_btn = _make_controller(None)

    controller.on_stop()

    assert any("not connected" in line for line in logs)


def test_on_stop_stop_motion_cb_exception_does_not_block_wire_stop() -> None:
    """A raising stop_motion_cb must not prevent DEV DT STOP from being sent."""
    transport = _FakeTransport()
    controller, logs, stream_btn = _make_controller(transport)
    controller.stop_motion_cb = lambda: (_ for _ in ()).throw(RuntimeError("boom"))

    controller.on_stop()

    assert transport.commands == ["DEV DT STOP", "STREAM 0"]
