"""tests/testgui/test_transport.py — headless unit tests for transport.py.

All tests run without hardware and without an aprilcam daemon.
SerialConnection is replaced with a fake; the aprilcam import path is
patched so the truth thread gracefully delivers None.

Run with:
    uv run python -m pytest tests/testgui -q

Requirements: PySide6 (uv sync --group gui).
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Any
from unittest.mock import MagicMock, patch

import pytest

# ---------------------------------------------------------------------------
# Fixtures and fakes
# ---------------------------------------------------------------------------


class FakeSerialConnection:
    """Minimal fake for robot_radio.io.serial_conn.SerialConnection.

    Simulates a connected port with a pre-filled TLM queue.
    """

    def __init__(
        self,
        port: str = "/dev/tty.test0",
        mode: str | None = None,
        on_send=None,
        on_recv=None,
        **_kwargs,
    ):
        self.port = port
        self._mode = mode
        self.on_send = on_send
        self.on_recv = on_recv
        self._open = False
        self._tlm_lines: list[str] = []

    # ---- lifecycle ----

    def connect(self, **_kw) -> dict[str, Any]:
        self._open = True
        return {"status": "connected", "port": self.port, "mode": self._mode}

    def disconnect(self) -> dict[str, Any]:
        self._open = False
        return {"status": "disconnected", "port": self.port}

    @property
    def is_open(self) -> bool:
        return self._open

    # ---- I/O ----

    def send(self, message: str, read_ms: int = 200, **_kw) -> dict[str, Any]:
        if self.on_send:
            self.on_send(message)
        return {"sent": message, "mode": self._mode, "responses": ["OK"]}

    def send_fast(self, message: str) -> None:
        if self.on_send:
            self.on_send(message)

    def read_pending_lines(self) -> list[str]:
        lines = list(self._tlm_lines)
        self._tlm_lines.clear()
        return lines

    # ---- helper used in tests ----

    def inject_tlm(self, line: str) -> None:
        """Inject a raw TLM line to be returned by the next read_pending_lines."""
        self._tlm_lines.append(line)


# ---------------------------------------------------------------------------
# Helper: patch SerialConnection in transport module
# ---------------------------------------------------------------------------


def _make_patched_transport(cls, port="/dev/tty.test0"):
    """Instantiate a transport with FakeSerialConnection injected."""
    fake = FakeSerialConnection(port=port, mode="direct")

    with patch(
        "robot_radio.testgui.transport.SerialConnection",
        return_value=fake,
    ):
        t = cls(port)

    # Keep a reference to the fake on the transport for test access.
    t._conn = fake  # type: ignore[attr-defined]
    return t, fake


# ---------------------------------------------------------------------------
# Tests: Transport ABC API surface
# ---------------------------------------------------------------------------


class TestTransportABC:
    """Verify the ABC structure and callback slots."""

    def test_transport_is_abstract(self):
        """Transport cannot be instantiated directly."""
        from robot_radio.testgui.transport import Transport

        with pytest.raises(TypeError):
            Transport()  # type: ignore[abstract]

    def test_serial_transport_is_transport(self):
        from robot_radio.testgui.transport import SerialTransport, Transport

        assert issubclass(SerialTransport, Transport)

    def test_relay_transport_is_transport(self):
        from robot_radio.testgui.transport import RelayTransport, Transport

        assert issubclass(RelayTransport, Transport)

    def test_callback_slots_exist(self):
        """Transport subclass instances expose on_telemetry, on_truth, on_log."""
        from robot_radio.testgui.transport import SerialTransport, Transport

        with patch("robot_radio.testgui.transport.SerialConnection"):
            t = SerialTransport("/dev/tty.test")

        assert hasattr(t, "on_telemetry")
        assert hasattr(t, "on_truth")
        assert hasattr(t, "on_log")
        assert t.on_telemetry is None
        assert t.on_truth is None
        assert t.on_log is None

    def test_send_command_abstract_methods_implemented(self):
        """SerialTransport has send() and command() methods."""
        from robot_radio.testgui.transport import SerialTransport

        assert callable(SerialTransport.send)
        assert callable(SerialTransport.command)


# ---------------------------------------------------------------------------
# Tests: list_ports
# ---------------------------------------------------------------------------


class TestListPorts:
    def test_list_ports_returns_list(self):
        from robot_radio.testgui.transport import list_ports

        ports = list_ports()
        assert isinstance(ports, list)
        # All entries should be strings.
        for p in ports:
            assert isinstance(p, str)


# ---------------------------------------------------------------------------
# Tests: SerialTransport connect / disconnect
# ---------------------------------------------------------------------------


class TestSerialTransportLifecycle:
    def test_connect_opens_connection(self):
        from robot_radio.testgui.transport import SerialTransport

        fake = FakeSerialConnection(port="/dev/tty.test0", mode="direct")
        with patch(
            "robot_radio.testgui.transport.SerialConnection",
            return_value=fake,
        ):
            t = SerialTransport("/dev/tty.test0")
            t.connect()

        assert fake.is_open

    def test_connect_is_idempotent(self):
        """Calling connect() twice does not crash."""
        from robot_radio.testgui.transport import SerialTransport

        fake = FakeSerialConnection(port="/dev/tty.test0", mode="direct")
        with patch(
            "robot_radio.testgui.transport.SerialConnection",
            return_value=fake,
        ):
            t = SerialTransport("/dev/tty.test0")
            t.connect()
            t.connect()  # should be a no-op

        assert fake.is_open

    def test_disconnect_closes_connection(self):
        from robot_radio.testgui.transport import SerialTransport

        fake = FakeSerialConnection(port="/dev/tty.test0", mode="direct")
        with patch(
            "robot_radio.testgui.transport.SerialConnection",
            return_value=fake,
        ):
            t = SerialTransport("/dev/tty.test0")
            t.connect()
            t.disconnect()

        assert not fake.is_open

    def test_disconnect_without_connect_does_not_raise(self):
        from robot_radio.testgui.transport import SerialTransport

        with patch("robot_radio.testgui.transport.SerialConnection"):
            t = SerialTransport("/dev/tty.test0")
            t.disconnect()  # must not raise

    def test_connect_raises_on_error(self):
        """If SerialConnection.connect returns an error dict, connect() raises ConnectionError."""
        from robot_radio.testgui.transport import SerialTransport

        bad_fake = FakeSerialConnection()
        bad_fake.connect = lambda **_kw: {"error": "port not found"}  # type: ignore

        with patch(
            "robot_radio.testgui.transport.SerialConnection",
            return_value=bad_fake,
        ):
            t = SerialTransport("/dev/tty.bad")
            with pytest.raises(ConnectionError, match="port not found"):
                t.connect()


# ---------------------------------------------------------------------------
# Tests: RelayTransport
# ---------------------------------------------------------------------------


class TestRelayTransportLifecycle:
    def test_relay_transport_uses_relay_mode(self):
        """RelayTransport passes mode='relay' to SerialConnection."""
        from robot_radio.testgui.transport import RelayTransport

        captured_modes: list[str] = []

        class CaptureModeConn(FakeSerialConnection):
            def __init__(self, port, mode=None, **kw):
                super().__init__(port=port, mode=mode, **kw)
                captured_modes.append(mode or "")

        with patch(
            "robot_radio.testgui.transport.SerialConnection",
            side_effect=CaptureModeConn,
        ):
            t = RelayTransport("/dev/tty.relay0")
            t.connect()
            t.disconnect()

        assert "relay" in captured_modes

    def test_relay_connect_and_disconnect(self):
        from robot_radio.testgui.transport import RelayTransport

        fake = FakeSerialConnection(port="/dev/tty.relay0", mode="relay")
        with patch(
            "robot_radio.testgui.transport.SerialConnection",
            return_value=fake,
        ):
            t = RelayTransport("/dev/tty.relay0")
            t.connect()
            assert fake.is_open
            t.disconnect()
            assert not fake.is_open


# ---------------------------------------------------------------------------
# Tests: send() and command()
# ---------------------------------------------------------------------------


class TestTransportCommands:
    def _connected_serial(self):
        from robot_radio.testgui.transport import SerialTransport

        fake = FakeSerialConnection(port="/dev/tty.test0", mode="direct")
        with patch(
            "robot_radio.testgui.transport.SerialConnection",
            return_value=fake,
        ):
            t = SerialTransport("/dev/tty.test0")
            t.connect()
        return t, fake

    def test_send_calls_send_fast(self):
        t, fake = self._connected_serial()
        sent_lines: list[str] = []
        fake.send_fast = lambda line: sent_lines.append(line)

        t.send("VW 100 0")
        t.disconnect()

        assert "VW 100 0" in sent_lines

    def test_command_returns_joined_responses(self):
        t, fake = self._connected_serial()

        result = t.command("PING")
        t.disconnect()

        # FakeSerialConnection.send() returns "OK"
        assert "OK" in result

    def test_command_when_disconnected_returns_empty(self):
        from robot_radio.testgui.transport import SerialTransport

        with patch("robot_radio.testgui.transport.SerialConnection"):
            t = SerialTransport("/dev/tty.test0")
            # Not connected — should return empty, not raise.
            result = t.command("PING")
        assert result == ""


# ---------------------------------------------------------------------------
# Tests: TLM reader thread — on_telemetry callback
# ---------------------------------------------------------------------------


class TestTLMReaderThread:
    def test_tlm_lines_delivered_to_callback(self):
        """TLM lines injected into the fake conn are parsed and delivered."""
        from robot_radio.testgui.transport import SerialTransport

        received_frames = []
        evt = threading.Event()

        def _on_tlm(frame):
            received_frames.append(frame)
            evt.set()

        fake = FakeSerialConnection(port="/dev/tty.test0", mode="direct")
        # Inject a TLM line before connect so the reader thread picks it up.
        fake.inject_tlm("TLM t=12345 enc=100,200 pose=300,400,900")

        with patch(
            "robot_radio.testgui.transport.SerialConnection",
            return_value=fake,
        ):
            t = SerialTransport("/dev/tty.test0")
            t.on_telemetry = _on_tlm
            t.connect()

        # Wait up to 1 s for the callback.
        delivered = evt.wait(timeout=1.0)
        t.disconnect()

        assert delivered, "on_telemetry was not called within 1 s"
        assert len(received_frames) >= 1
        frame = received_frames[0]
        assert frame.t == 12345
        assert frame.enc == (100, 200)

    def test_non_tlm_lines_do_not_invoke_callback(self):
        """Lines that are not TLM (e.g. EVT) do not invoke on_telemetry."""
        from robot_radio.testgui.transport import SerialTransport

        received_frames = []
        fake = FakeSerialConnection(port="/dev/tty.test0", mode="direct")
        # Inject an EVT line — parse_tlm should return None for it.
        fake.inject_tlm("EVT done T")

        with patch(
            "robot_radio.testgui.transport.SerialConnection",
            return_value=fake,
        ):
            t = SerialTransport("/dev/tty.test0")
            t.on_telemetry = lambda f: received_frames.append(f)
            t.connect()

        # Give the reader thread a moment.
        time.sleep(0.15)
        t.disconnect()

        assert received_frames == [], "EVT line should not produce a TLMFrame"


# ---------------------------------------------------------------------------
# Tests: camera truth thread — graceful degradation when no daemon
# ---------------------------------------------------------------------------


class TestCameraTruthThread:
    def test_truth_callback_receives_none_when_no_daemon(self):
        """When aprilcam is not available, on_truth is called with None."""
        from robot_radio.testgui.transport import SerialTransport

        received: list = []
        evt = threading.Event()

        def _on_truth(pose):
            received.append(pose)
            evt.set()

        fake = FakeSerialConnection(port="/dev/tty.test0", mode="direct")

        # Patch _open_playfield to always return None (no daemon).
        with patch(
            "robot_radio.testgui.transport.SerialConnection",
            return_value=fake,
        ), patch.object(
            # Patch the method on the class so instance gets it.
            SerialTransport,
            "_open_playfield",
            return_value=None,
        ):
            t = SerialTransport("/dev/tty.test0")
            t.on_truth = _on_truth
            t.connect()

        delivered = evt.wait(timeout=2.5)
        t.disconnect()

        assert delivered, "on_truth was not called within 2.5 s"
        # At least one None delivery expected when no daemon.
        assert None in received


# ---------------------------------------------------------------------------
# Tests: thread cleanup — no hanging threads after disconnect
# ---------------------------------------------------------------------------


class TestThreadCleanup:
    def test_no_dangling_threads_after_disconnect(self):
        """All transport threads must be dead after disconnect()."""
        from robot_radio.testgui.transport import SerialTransport

        fake = FakeSerialConnection(port="/dev/tty.test0", mode="direct")

        with patch(
            "robot_radio.testgui.transport.SerialConnection",
            return_value=fake,
        ), patch.object(SerialTransport, "_open_playfield", return_value=None):
            t = SerialTransport("/dev/tty.test0")
            t.connect()

            reader = t._reader_thread
            truth = t._truth_thread

            assert reader is not None and reader.is_alive()
            assert truth is not None and truth.is_alive()

            t.disconnect()

        # After disconnect, both threads must have exited.
        assert reader is not None
        assert truth is not None
        assert not reader.is_alive(), "reader thread still alive after disconnect"
        assert not truth.is_alive(), "truth thread still alive after disconnect"


# ---------------------------------------------------------------------------
# Tests: on_log callback
# ---------------------------------------------------------------------------


class TestLogCallback:
    def test_on_log_called_on_send(self):
        """Sending a command triggers the on_log callback.

        The on_send hook is wired from _HardwareTransport.connect() into the
        SerialConnection constructor.  We use side_effect so the fake captures
        the on_send keyword argument that connect() passes.
        """
        from robot_radio.testgui.transport import SerialTransport

        log_entries: list[str] = []
        created_fakes: list[FakeSerialConnection] = []

        def _make_fake(port, mode=None, on_send=None, on_recv=None, **_kw):
            f = FakeSerialConnection(port=port, mode=mode, on_send=on_send, on_recv=on_recv)
            created_fakes.append(f)
            return f

        with patch(
            "robot_radio.testgui.transport.SerialConnection",
            side_effect=_make_fake,
        ), patch.object(SerialTransport, "_open_playfield", return_value=None):
            t = SerialTransport("/dev/tty.test0")
            t.on_log = lambda msg: log_entries.append(msg)
            t.connect()
            t.send("STOP")
            time.sleep(0.05)
            t.disconnect()

        # The on_send hook fires from send_fast() and calls self._log() → on_log.
        assert any("STOP" in e for e in log_entries), (
            f"Expected 'STOP' in log entries but got: {log_entries}"
        )
