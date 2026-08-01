"""src/tests/testgui/test_operations.py — headless tests for OpsController.on_stop()
(ticket 083-002).

128-003 update: ``on_stop()``'s wire-STOP saga (097's "STOP is the one verb
ALWAYS guaranteed to work") turned out to be false on hardware --
``binary_bridge.translate_command()`` is an unconditional dead stub on
every transport ever since ``legacy_verbs`` was deleted wholesale
(107-003), so ``transport.command("STOP", ...)`` never touched the wire at
all while this method logged success regardless. ``on_stop()`` now calls
``transport.halt()`` directly (estop semantics, not the PLANNED stop
``"STOP"`` used to map to even when the stub "worked") and logs an honest
``[ERROR] HALT FAILED`` on a raise instead of silence. This module tests
``OpsController.on_stop()`` directly (no ``QApplication``/PySide6 widgets
needed — only fake stand-ins for the one button it touches).

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest src/tests/testgui/test_operations.py -q

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


class _FakeOriginButton:
    """Minimal stand-in for ``origin_btn`` -- records ``setEnabled``/
    ``setVisible``/``setToolTip`` calls (OOP sim-motor-state fix: origin_btn
    gating tests below)."""

    def __init__(self) -> None:
        self.enabled: bool | None = None
        self.visible: bool | None = None
        self.tooltip: str | None = None

    def setEnabled(self, value: bool) -> None:
        self.enabled = value

    def setVisible(self, value: bool) -> None:
        self.visible = value

    def setToolTip(self, value: str) -> None:
        self.tooltip = value


class _FakeTransport(Transport):
    """Records every ``command()``/``send()`` line and every ``halt()``
    call; no real IO. No ``.protocol`` surface (mirrors a transport with no
    direct-protocol telemetry-off path -- ``on_stop()``'s step 3 must
    degrade to its "not available" branch against this fake, not raise)."""

    def __init__(self) -> None:
        super().__init__()
        self.commands: list[str] = []
        self.halt_calls: int = 0

    def connect(self) -> None:
        pass

    def disconnect(self) -> None:
        pass

    def send(self, line: str) -> None:
        self.commands.append(line)

    def command(self, line: str, read_timeout: int = 200) -> str:  # [ms]
        self.commands.append(line)
        return ""

    def halt(self) -> None:
        self.halt_calls += 1


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


def _make_controller_with_origin(
    transport: "Transport | None",
) -> tuple[OpsController, _FakeOriginButton]:
    """Build an OpsController with a fake ``origin_btn`` -- for the
    OOP sim-motor-state gating tests below."""
    origin_btn = _FakeOriginButton()
    controller = OpsController(
        transport_ref={"transport": transport},
        log_cb=lambda _msg: None,
        sync_btn=None,
        zero_btn=None,
        stop_btn=None,
        clear_btn=None,
        refresh_btn=None,
        stream_btn=_FakeButton(),
        origin_btn=origin_btn,
        transport_buttons=[],
    )
    return controller, origin_btn


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_on_stop_calls_halt_then_reports_telemetry_off_unavailable() -> None:
    """128-003: on_stop() no longer sends the dead "STOP"/"STREAM 0" wire
    strings -- step 2 calls transport.halt() directly (estop semantics),
    and step 3 (telemetry-off) degrades to an honest "not available" log
    against this fake, which has no ``.protocol`` surface (unlike a real
    hardware/Sim transport)."""
    transport = _FakeTransport()
    controller, logs, stream_btn = _make_controller(transport)

    controller.on_stop()

    assert transport.commands == []
    assert transport.halt_calls == 1
    assert any("estop sent" in line for line in logs)
    assert any("not available" in line for line in logs)
    # step 3 never ran (no .protocol on this fake) -- stream_btn untouched.
    assert stream_btn.checked is None
    assert stream_btn.text is None


# ---------------------------------------------------------------------------
# origin_btn gating (OOP sim-motor-state fix): enabled iff connected AND no
# tour is running.
# ---------------------------------------------------------------------------


def test_origin_btn_starts_disabled_before_any_connect() -> None:
    controller, origin_btn = _make_controller_with_origin(None)
    assert origin_btn.enabled is None  # OpsController never touched it yet


def test_origin_btn_enabled_when_connected_and_no_tour() -> None:
    transport = _FakeTransport()
    controller, origin_btn = _make_controller_with_origin(transport)

    controller.set_connected(True, transport)

    assert origin_btn.enabled is True


def test_origin_btn_disabled_when_disconnected() -> None:
    transport = _FakeTransport()
    controller, origin_btn = _make_controller_with_origin(transport)

    controller.set_connected(True, transport)
    assert origin_btn.enabled is True

    controller.set_connected(False)
    assert origin_btn.enabled is False


def test_origin_btn_ghosted_while_tour_running() -> None:
    transport = _FakeTransport()
    controller, origin_btn = _make_controller_with_origin(transport)

    controller.set_connected(True, transport)
    assert origin_btn.enabled is True

    controller.set_tour_running(True)
    assert origin_btn.enabled is False, "must ghost while a tour runs"

    controller.set_tour_running(False)
    assert origin_btn.enabled is True, "must re-enable once the tour ends (still connected)"


def test_origin_btn_stays_disabled_if_tour_ends_while_disconnected() -> None:
    """A tour-finished callback firing after disconnect (e.g. on app
    teardown) must not spuriously re-enable origin_btn."""
    transport = _FakeTransport()
    controller, origin_btn = _make_controller_with_origin(transport)

    controller.set_connected(True, transport)
    controller.set_tour_running(True)
    controller.set_connected(False)
    assert origin_btn.enabled is False

    controller.set_tour_running(False)
    assert origin_btn.enabled is False, "must stay disabled -- not connected"


def test_on_stop_cancels_motion_worker_before_halting() -> None:
    transport = _FakeTransport()
    controller, logs, stream_btn = _make_controller(transport)
    call_order: list[str] = []
    controller.stop_motion_cb = lambda: call_order.append("stop_motion_cb")

    controller.on_stop()

    # stop_motion_cb must run BEFORE the halt, so the worker thread is
    # joined and no longer touches the transport.
    assert call_order == ["stop_motion_cb"]
    assert transport.halt_calls == 1


def test_on_stop_not_connected_logs_warning_and_sends_nothing() -> None:
    controller, logs, stream_btn = _make_controller(None)

    controller.on_stop()

    assert any("not connected" in line for line in logs)


def test_on_stop_stop_motion_cb_exception_does_not_block_halt() -> None:
    """A raising stop_motion_cb must not prevent the halt from being sent."""
    transport = _FakeTransport()
    controller, logs, stream_btn = _make_controller(transport)
    controller.stop_motion_cb = lambda: (_ for _ in ()).throw(RuntimeError("boom"))

    controller.on_stop()

    assert transport.halt_calls == 1


def test_on_stop_raising_halt_logs_error_not_info() -> None:
    """A raised halt() failure must surface as a loud [ERROR] -- on_stop()
    must never log success on faith (128-003's own core safety property)."""

    class _RaisingHaltTransport(_FakeTransport):
        def halt(self) -> None:
            super().halt()
            raise ConnectionError("wire timeout")

    transport = _RaisingHaltTransport()
    controller, logs, stream_btn = _make_controller(transport)

    controller.on_stop()

    assert transport.halt_calls == 1
    assert any("HALT FAILED" in line and "[ERROR]" in line for line in logs)
    assert not any("[INFO] estop sent" in line for line in logs)
