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


# ---------------------------------------------------------------------------
# Tests: SimTransport
# ---------------------------------------------------------------------------


class FakeSim:
    """Minimal fake for tests/_infra/sim/firmware.Sim.

    Tracks sent commands, provides canned TLM replies in get_async_evts(),
    and exposes get_true_pose() for ground-truth delivery tests.
    """

    def __init__(self) -> None:
        self.sent: list[str] = []
        self._async_evts: list[str] = []
        self._pose_x_mm: float = 100.0
        self._pose_y_mm: float = 200.0
        self._pose_h_rad: float = 0.5
        self._field_profile_applied: bool = False
        self._otos_noise_set: bool = False
        self._destroyed: bool = False
        # For context manager protocol
        self._t: int = 0
        # issue testgui-sim-error-profile-config: record the values passed
        # to each of the four error-profile knobs so tests can assert on
        # exactly what was applied (not just that some call happened).
        self.last_slip_turn_extra: float | None = None
        self.last_otos_linear_noise: float | None = None
        self.last_otos_yaw_noise: float | None = None
        self.last_encoder_noise: dict[int, float] = {}

    def __enter__(self) -> "FakeSim":
        return self

    def __exit__(self, *_) -> None:
        self._destroyed = True

    def send_command(self, line: str) -> str:
        self.sent.append(line)
        if line.startswith("STREAM"):
            return "OK stream"
        return "OK"

    def get_async_evts(self) -> str:
        evts = "\n".join(self._async_evts)
        self._async_evts.clear()
        return evts

    def tick_for(self, total_ms: int, step_ms: int = 24, **_kw) -> None:
        self._t += total_ms

    def get_true_pose(self) -> tuple[float, float, float]:
        return (self._pose_x_mm, self._pose_y_mm, self._pose_h_rad)

    def set_true_pose(self, x_mm: float, y_mm: float, h_rad: float) -> None:
        self._pose_x_mm = x_mm
        self._pose_y_mm = y_mm
        self._pose_h_rad = h_rad

    def set_true_wheel_travel(self, enc_l_mm: float, enc_r_mm: float) -> None:
        self._true_enc = (enc_l_mm, enc_r_mm)

    def set_true_velocity(self, vel_l_mms: float, vel_r_mms: float) -> None:
        self._true_vel = (vel_l_mms, vel_r_mms)

    def set_field_profile(self, slip_turn_extra: float = 0.26,
                          fuse_otos: bool = True) -> None:
        self._field_profile_applied = True
        self.last_slip_turn_extra = slip_turn_extra

    def set_otos_linear_noise(self, sigma_fraction: float) -> None:
        self._otos_noise_set = True
        self.last_otos_linear_noise = sigma_fraction

    def set_otos_yaw_noise(self, sigma_fraction: float) -> None:
        self.last_otos_yaw_noise = sigma_fraction

    def set_encoder_noise(self, side: int, sigma_mm: float) -> None:
        self.last_encoder_noise[side] = sigma_mm

    def inject_tlm(self, line: str) -> None:
        """Add a TLM line to the async event queue."""
        self._async_evts.append(line)


def _make_sim_transport_with_fake(fake_sim: FakeSim | None = None):
    """Construct a SimTransport whose tick-thread uses FakeSim instead of real Sim.

    Patches both the lib-path existence check and the Sim import so no real
    library is required.
    """
    from unittest.mock import patch, MagicMock
    from robot_radio.testgui.transport import SimTransport
    import pathlib

    if fake_sim is None:
        fake_sim = FakeSim()

    # Patch _sim_lib_path to return a path that "exists".
    fake_path = MagicMock(spec=pathlib.Path)
    fake_path.exists.return_value = True
    fake_path.parent.parent = pathlib.Path("/fake/tests/_infra/sim")

    # We patch at the tick_loop level by providing a fake Sim via sys.modules.
    import sys
    fake_firmware_module = MagicMock()
    fake_firmware_module.Sim.return_value = fake_sim
    fake_firmware_module.Sim.return_value.__enter__ = lambda s: s
    fake_firmware_module.Sim.return_value.__exit__ = lambda s, *a: None

    return fake_sim, fake_path, fake_firmware_module


class TestSimTransportABC:
    """SimTransport implements the Transport ABC."""

    def test_simtransport_is_transport(self):
        from robot_radio.testgui.transport import SimTransport, Transport

        assert issubclass(SimTransport, Transport)

    def test_simtransport_callback_slots(self):
        from robot_radio.testgui.transport import SimTransport

        t = SimTransport()
        assert hasattr(t, "on_telemetry")
        assert hasattr(t, "on_truth")
        assert hasattr(t, "on_log")
        assert t.on_telemetry is None
        assert t.on_truth is None
        assert t.on_log is None

    def test_simtransport_has_send_and_command(self):
        from robot_radio.testgui.transport import SimTransport

        assert callable(SimTransport.send)
        assert callable(SimTransport.command)


class TestSimTransportLibCheck:
    """connect() behaviour when the sim lib is missing or present."""

    def test_connect_returns_without_connecting_when_lib_missing(self):
        """If the lib path does not exist, connect() must NOT set _connected."""
        from unittest.mock import patch, MagicMock
        from robot_radio.testgui.transport import SimTransport
        import pathlib

        fake_path = MagicMock(spec=pathlib.Path)
        fake_path.exists.return_value = False

        t = SimTransport()
        log_entries: list[str] = []
        t.on_log = log_entries.append

        with patch(
            "robot_radio.testgui.transport._sim_lib_path",
            return_value=fake_path,
        ), patch.object(SimTransport, "_show_build_warning"):
            t.connect()

        assert not t._connected, "_connected must remain False when lib is missing"

    def test_connect_calls_show_build_warning_when_lib_missing(self):
        """When the lib is missing, _show_build_warning must be called."""
        from unittest.mock import patch, MagicMock, call
        from robot_radio.testgui.transport import SimTransport
        import pathlib

        fake_path = MagicMock(spec=pathlib.Path)
        fake_path.exists.return_value = False

        t = SimTransport()
        t.on_log = lambda _: None

        with patch(
            "robot_radio.testgui.transport._sim_lib_path",
            return_value=fake_path,
        ), patch.object(SimTransport, "_show_build_warning") as mock_warn:
            t.connect()

        mock_warn.assert_called_once()

    def test_connect_logs_error_when_lib_missing(self):
        """on_log must receive an error message mentioning build."""
        from unittest.mock import patch, MagicMock
        from robot_radio.testgui.transport import SimTransport
        import pathlib

        fake_path = MagicMock(spec=pathlib.Path)
        fake_path.exists.return_value = False

        t = SimTransport()
        log_entries: list[str] = []
        t.on_log = log_entries.append

        with patch(
            "robot_radio.testgui.transport._sim_lib_path",
            return_value=fake_path,
        ), patch.object(SimTransport, "_show_build_warning"):
            t.connect()

        assert any("build" in e.lower() or "Build" in e or "ERROR" in e
                   for e in log_entries), (
            f"Expected build-related error in log; got: {log_entries}"
        )


class TestSimTransportConnect:
    """connect() behaviour when the lib is present (using FakeSim)."""

    def _connected_sim(self) -> tuple:
        """Return (SimTransport, FakeSim) with a real connect() on fake Sim."""
        from unittest.mock import patch, MagicMock
        from robot_radio.testgui.transport import SimTransport
        import pathlib
        import sys

        fake_sim = FakeSim()

        fake_path = MagicMock(spec=pathlib.Path)
        fake_path.exists.return_value = True
        fake_path.parent.parent = str(pathlib.Path("/nonexistent/sim"))

        # Build a fake 'firmware' module so the import in _tick_loop succeeds.
        fake_fw_module = MagicMock()
        fake_fw_module.Sim.return_value = fake_sim

        with patch(
            "robot_radio.testgui.transport._sim_lib_path",
            return_value=fake_path,
        ), patch.dict(sys.modules, {"firmware": fake_fw_module}):
            t = SimTransport()
            log_entries: list[str] = []
            t.on_log = log_entries.append
            t.connect()

        # Give tick-thread a moment to start and send STREAM 50.
        time.sleep(0.15)

        return t, fake_sim, log_entries

    def test_connected_flag_is_set(self):
        t, _, _ = self._connected_sim()
        try:
            assert t._connected
        finally:
            t.disconnect()

    def test_stream_50_sent_on_connect(self):
        t, fake_sim, _ = self._connected_sim()
        try:
            # Give tick-thread time to run and send STREAM 50.
            time.sleep(0.1)
            assert any("STREAM" in s for s in fake_sim.sent), (
                f"STREAM 50 not found in sent commands: {fake_sim.sent}"
            )
        finally:
            t.disconnect()

    def test_field_profile_applied_on_connect(self):
        t, fake_sim, _ = self._connected_sim()
        try:
            time.sleep(0.1)
            assert fake_sim._field_profile_applied, (
                "set_field_profile was not called on connect"
            )
            assert fake_sim._otos_noise_set, (
                "set_otos_linear_noise was not called on connect"
            )
        finally:
            t.disconnect()

    def test_set_true_pose_teleports_plant(self):
        """set_true_pose() must move the plant ground truth (cm→mm), so the
        avatar does not snap back to a stale pose after Set Robot @ 0,0.

        Regression: SI/OZ only reset the firmware belief; without teleporting
        the plant, get_true_pose() (which drives the sim avatar) kept the
        robot's prior heading and the avatar jumped back.
        """
        t, fake_sim, _ = self._connected_sim()
        try:
            # Plant starts at the FakeSim default (100 mm, 200 mm, 0.5 rad).
            assert fake_sim.get_true_pose() != (0.0, 0.0, 0.0)
            t.set_true_pose(0.0, 0.0, 0.0)
            # Give the tick-thread a moment to drain the queued plant action.
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                if fake_sim.get_true_pose() == (0.0, 0.0, 0.0):
                    break
                time.sleep(0.02)
            assert fake_sim.get_true_pose() == (0.0, 0.0, 0.0), (
                f"plant not teleported to origin: {fake_sim.get_true_pose()}"
            )
        finally:
            t.disconnect()

    def test_set_true_pose_converts_cm_to_mm(self):
        """set_true_pose(x_cm, y_cm, yaw_rad) must scale position ×10 to mm."""
        t, fake_sim, _ = self._connected_sim()
        try:
            t.set_true_pose(12.5, -4.0, 1.0)
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                if fake_sim.get_true_pose() == (125.0, -40.0, 1.0):
                    break
                time.sleep(0.02)
            assert fake_sim.get_true_pose() == (125.0, -40.0, 1.0), (
                f"cm→mm conversion wrong: {fake_sim.get_true_pose()}"
            )
        finally:
            t.disconnect()

    def test_set_true_pose_noop_when_disconnected(self):
        from robot_radio.testgui.transport import SimTransport

        t = SimTransport()
        # Must not raise when not connected.
        t.set_true_pose(0.0, 0.0, 0.0)

    def test_connect_is_idempotent(self):
        from unittest.mock import patch, MagicMock
        from robot_radio.testgui.transport import SimTransport
        import pathlib
        import sys

        fake_sim = FakeSim()
        fake_path = MagicMock(spec=pathlib.Path)
        fake_path.exists.return_value = True
        fake_path.parent.parent = str(pathlib.Path("/nonexistent/sim"))
        fake_fw_module = MagicMock()
        fake_fw_module.Sim.return_value = fake_sim

        with patch(
            "robot_radio.testgui.transport._sim_lib_path",
            return_value=fake_path,
        ), patch.dict(sys.modules, {"firmware": fake_fw_module}):
            t = SimTransport()
            t.on_log = lambda _: None
            t.connect()
            t.connect()  # second call must be a no-op

        try:
            assert t._connected
            # tick_thread must be a single thread — count active sim threads.
            sim_threads = [
                th for th in threading.enumerate()
                if th.name == "sim-tick-thread"
            ]
            assert len(sim_threads) <= 1, (
                "Multiple tick-threads started by double connect()"
            )
        finally:
            t.disconnect()


class TestSimTransportConnectedFlagRace:
    """CR-15 item 4: _connected must not be set until the tick-thread
    confirms Sim() construction succeeded -- not merely after the
    tick-thread is started."""

    def test_connect_waits_for_sim_before_returning(self):
        """connect() must not return until Sim() construction is confirmed
        -- an early command()/send() must never race a not-yet-created Sim."""
        from unittest.mock import patch, MagicMock
        from robot_radio.testgui.transport import SimTransport
        import pathlib
        import sys

        fake_sim = FakeSim()
        fake_path = MagicMock(spec=pathlib.Path)
        fake_path.exists.return_value = True
        fake_path.parent.parent = str(pathlib.Path("/nonexistent/sim"))
        fake_fw_module = MagicMock()
        fake_fw_module.Sim.return_value = fake_sim

        with patch(
            "robot_radio.testgui.transport._sim_lib_path",
            return_value=fake_path,
        ), patch.dict(sys.modules, {"firmware": fake_fw_module}):
            t = SimTransport()
            t.on_log = lambda _: None
            t.connect()

        try:
            # No sleep here -- connect() itself must have waited for Sim()
            # construction to be confirmed before returning.
            assert t._connected
            assert t._sim is not None, (
                "connect() returned before Sim() construction was confirmed"
            )
        finally:
            t.disconnect()

    def test_sim_construction_failure_leaves_connected_false(self):
        """If Sim() itself raises, _connected must stay False (never raced
        true), and connect() must still return promptly (not hang)."""
        from unittest.mock import patch, MagicMock
        from robot_radio.testgui.transport import SimTransport
        import pathlib
        import sys

        fake_path = MagicMock(spec=pathlib.Path)
        fake_path.exists.return_value = True
        fake_path.parent.parent = str(pathlib.Path("/nonexistent/sim"))

        fake_fw_module = MagicMock()
        fake_fw_module.Sim.side_effect = RuntimeError("boom: construction failed")

        with patch(
            "robot_radio.testgui.transport._sim_lib_path",
            return_value=fake_path,
        ), patch.dict(sys.modules, {"firmware": fake_fw_module}):
            t = SimTransport()
            t.on_log = lambda _: None
            start = time.monotonic()
            t.connect()
            elapsed = time.monotonic() - start

        try:
            assert not t._connected, (
                "connect() must not report connected when Sim() raised"
            )
            assert t._sim is None
            assert elapsed < 2.0, (
                f"connect() took {elapsed:.2f}s -- should fail fast, not "
                f"wait out the full ready-timeout"
            )
        finally:
            t.disconnect()

    def test_import_failure_leaves_connected_false(self):
        """If importing the 'firmware' module fails, _connected must stay
        False (regression guard for the pre-existing import-failure path,
        which must also unblock connect()'s new wait).

        ``sys.modules["firmware"] = None`` is the standard sentinel Python's
        import system honors to force ``from firmware import Sim`` to raise
        ImportError, regardless of whether a real 'firmware' module is
        importable elsewhere on sys.path (tests/conftest.py adds
        tests/_infra/sim/ to sys.path for the whole session, so a bare
        missing-module approach would not actually fail here).
        """
        from unittest.mock import patch, MagicMock
        from robot_radio.testgui.transport import SimTransport
        import pathlib
        import sys

        fake_path = MagicMock(spec=pathlib.Path)
        fake_path.exists.return_value = True
        fake_path.parent.parent = str(pathlib.Path("/nonexistent/sim"))

        with patch(
            "robot_radio.testgui.transport._sim_lib_path",
            return_value=fake_path,
        ), patch.dict(sys.modules, {"firmware": None}):
            t = SimTransport()
            t.on_log = lambda _: None
            t.connect()

        try:
            assert not t._connected
        finally:
            t.disconnect()


class TestSimTransportErrorProfile:
    """Sim Errors panel backing: apply_error_profile() / turn_scrub_factor.

    issue testgui-sim-error-profile-config.
    """

    def _connected_sim(self, fake_sim: "FakeSim | None" = None) -> tuple:
        """Return (SimTransport, FakeSim) connected against a FakeSim.

        Mirrors ``TestSimTransportConnect._connected_sim`` (duplicated here
        to keep this test class self-contained and allow a caller-supplied
        fake_sim for the stale-lib degradation test).
        """
        from unittest.mock import patch, MagicMock
        from robot_radio.testgui.transport import SimTransport
        import pathlib
        import sys

        if fake_sim is None:
            fake_sim = FakeSim()

        fake_path = MagicMock(spec=pathlib.Path)
        fake_path.exists.return_value = True
        fake_path.parent.parent = str(pathlib.Path("/nonexistent/sim"))

        fake_fw_module = MagicMock()
        fake_fw_module.Sim.return_value = fake_sim

        with patch(
            "robot_radio.testgui.transport._sim_lib_path",
            return_value=fake_path,
        ), patch.dict(sys.modules, {"firmware": fake_fw_module}):
            t = SimTransport()
            t.on_log = lambda _: None
            t.connect()

        time.sleep(0.15)
        return t, fake_sim

    def test_apply_field_profile_uses_persisted_profile_on_connect(self, monkeypatch):
        """connect() must load sim_prefs and apply ALL four knobs, not just two."""
        from robot_radio.testgui import sim_prefs

        custom_profile = {
            "encoder_noise_mm": 3.0,
            "slip_turn_extra": 0.4,
            "otos_linear_noise": 0.2,
            "otos_yaw_noise": 0.05,
        }
        monkeypatch.setattr(
            sim_prefs, "load_sim_error_profile", lambda: dict(custom_profile)
        )

        t, fake_sim = self._connected_sim()
        try:
            time.sleep(0.1)
            assert fake_sim.last_slip_turn_extra == 0.4
            assert fake_sim.last_otos_linear_noise == 0.2
            assert fake_sim.last_otos_yaw_noise == 0.05
            assert fake_sim.last_encoder_noise == {0: 3.0, 1: 3.0}
        finally:
            t.disconnect()

    def test_turn_scrub_factor_reflects_applied_profile(self, monkeypatch):
        from robot_radio.testgui import sim_prefs

        monkeypatch.setattr(
            sim_prefs,
            "load_sim_error_profile",
            lambda: {**sim_prefs.DEFAULT_PROFILE, "slip_turn_extra": 0.77},
        )

        t, fake_sim = self._connected_sim()
        try:
            time.sleep(0.1)
            assert t.turn_scrub_factor == 0.77
        finally:
            t.disconnect()

    def test_turn_scrub_factor_reads_persisted_profile_before_connect(
        self, monkeypatch, tmp_path
    ):
        """Before connect(), turn_scrub_factor still reads sim_prefs off disk."""
        from robot_radio.testgui import sim_prefs
        from robot_radio.testgui.transport import SimTransport

        prefs_path = tmp_path / "sim_error_profile.json"
        prefs_path.write_text('{"slip_turn_extra": 0.5}')
        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", prefs_path)

        t = SimTransport()
        assert t.turn_scrub_factor == 0.5

    def test_turn_scrub_factor_default_when_no_persisted_file(
        self, monkeypatch, tmp_path
    ):
        from robot_radio.testgui import sim_prefs
        from robot_radio.testgui.transport import SimTransport

        monkeypatch.setattr(sim_prefs, "_PREFS_PATH", tmp_path / "missing.json")

        t = SimTransport()
        assert t.turn_scrub_factor == 0.26

    def test_apply_error_profile_updates_running_sim(self):
        t, fake_sim = self._connected_sim()
        try:
            time.sleep(0.1)
            new_profile = {
                "encoder_noise_mm": 7.0,
                "slip_turn_extra": 0.9,
                "otos_linear_noise": 0.15,
                "otos_yaw_noise": 0.03,
            }
            t.apply_error_profile(new_profile)

            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                if fake_sim.last_slip_turn_extra == 0.9:
                    break
                time.sleep(0.02)

            assert fake_sim.last_slip_turn_extra == 0.9
            assert fake_sim.last_otos_linear_noise == 0.15
            assert fake_sim.last_otos_yaw_noise == 0.03
            assert fake_sim.last_encoder_noise == {0: 7.0, 1: 7.0}
            assert t.turn_scrub_factor == 0.9
        finally:
            t.disconnect()

    def test_apply_error_profile_noop_when_not_connected(self):
        from robot_radio.testgui.transport import SimTransport

        t = SimTransport()
        log_entries: list[str] = []
        t.on_log = log_entries.append

        # Must not raise.
        t.apply_error_profile({"slip_turn_extra": 0.5})
        assert any("not connected" in e.lower() for e in log_entries)

    def test_apply_error_profile_tolerates_missing_encoder_noise_method(self):
        """A stale FakeSim (no set_encoder_noise) must not break the other knobs."""

        class _StaleFakeSim(FakeSim):
            def set_encoder_noise(self, side, sigma_mm):
                raise AttributeError("stale lib: sim_set_encoder_noise missing")

        t, fake_sim = self._connected_sim(fake_sim=_StaleFakeSim())
        try:
            time.sleep(0.1)
            new_profile = {
                "encoder_noise_mm": 1.0,
                "slip_turn_extra": 0.33,
                "otos_linear_noise": 0.11,
                "otos_yaw_noise": 0.01,
            }
            t.apply_error_profile(new_profile)

            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                if fake_sim.last_slip_turn_extra == 0.33:
                    break
                time.sleep(0.02)

            assert fake_sim.last_slip_turn_extra == 0.33
            assert fake_sim.last_otos_linear_noise == 0.11
            assert fake_sim.last_otos_yaw_noise == 0.01
            # Encoder noise silently failed but did not prevent the rest.
            assert fake_sim.last_encoder_noise == {}
        finally:
            t.disconnect()


class TestSimTransportDisconnect:
    """disconnect() behaviour."""

    def test_disconnect_stops_tick_thread(self):
        from unittest.mock import patch, MagicMock
        from robot_radio.testgui.transport import SimTransport
        import pathlib
        import sys

        fake_sim = FakeSim()
        fake_path = MagicMock(spec=pathlib.Path)
        fake_path.exists.return_value = True
        fake_path.parent.parent = str(pathlib.Path("/nonexistent/sim"))
        fake_fw_module = MagicMock()
        fake_fw_module.Sim.return_value = fake_sim

        with patch(
            "robot_radio.testgui.transport._sim_lib_path",
            return_value=fake_path,
        ), patch.dict(sys.modules, {"firmware": fake_fw_module}):
            t = SimTransport()
            t.on_log = lambda _: None
            t.connect()
            time.sleep(0.1)

            tick_thread = t._tick_thread
            assert tick_thread is not None and tick_thread.is_alive()

            t.disconnect()

        # After disconnect, tick-thread must exit.
        if tick_thread is not None:
            tick_thread.join(timeout=3.0)
        assert tick_thread is None or not tick_thread.is_alive(), (
            "Tick-thread still alive after disconnect()"
        )

    def test_disconnect_without_connect_does_not_raise(self):
        from robot_radio.testgui.transport import SimTransport

        t = SimTransport()
        t.disconnect()  # must not raise


class TestSimTransportCommands:
    """send() and command() route through the cmd queue to the tick-thread."""

    def _connected_sim(self) -> tuple:
        from unittest.mock import patch, MagicMock
        from robot_radio.testgui.transport import SimTransport
        import pathlib
        import sys

        fake_sim = FakeSim()
        fake_path = MagicMock(spec=pathlib.Path)
        fake_path.exists.return_value = True
        fake_path.parent.parent = str(pathlib.Path("/nonexistent/sim"))
        fake_fw_module = MagicMock()
        fake_fw_module.Sim.return_value = fake_sim

        with patch(
            "robot_radio.testgui.transport._sim_lib_path",
            return_value=fake_path,
        ), patch.dict(sys.modules, {"firmware": fake_fw_module}):
            t = SimTransport()
            t.on_log = lambda _: None
            t.connect()

        time.sleep(0.15)
        return t, fake_sim

    def test_send_queues_command(self):
        t, fake_sim = self._connected_sim()
        try:
            t.send("D 200 200 500")
            # Give tick-thread a chance to process the queue.
            time.sleep(0.1)
            assert any("D 200 200 500" in s for s in fake_sim.sent), (
                f"D command not found in sim.sent: {fake_sim.sent}"
            )
        finally:
            t.disconnect()

    def test_send_raises_when_not_connected(self):
        from robot_radio.testgui.transport import SimTransport

        t = SimTransport()
        with pytest.raises(ConnectionError):
            t.send("PING")

    def test_command_returns_reply(self):
        t, fake_sim = self._connected_sim()
        try:
            reply = t.command("PING", read_ms=500)
            assert reply == "OK", f"Expected 'OK' reply, got {reply!r}"
        finally:
            t.disconnect()

    def test_command_when_not_connected_returns_empty(self):
        from robot_radio.testgui.transport import SimTransport

        t = SimTransport()
        result = t.command("PING")
        assert result == ""


class TestSimTransportTLMDelivery:
    """TLM lines from get_async_evts() are parsed and delivered to on_telemetry."""

    def test_tlm_delivered_to_callback(self):
        """TLM lines injected into FakeSim's async queue are delivered."""
        from unittest.mock import patch, MagicMock
        from robot_radio.testgui.transport import SimTransport
        import pathlib
        import sys

        fake_sim = FakeSim()
        fake_path = MagicMock(spec=pathlib.Path)
        fake_path.exists.return_value = True
        fake_path.parent.parent = str(pathlib.Path("/nonexistent/sim"))
        fake_fw_module = MagicMock()
        fake_fw_module.Sim.return_value = fake_sim

        received_frames: list = []
        done_evt = threading.Event()

        def _on_tlm(frame):
            received_frames.append(frame)
            done_evt.set()

        with patch(
            "robot_radio.testgui.transport._sim_lib_path",
            return_value=fake_path,
        ), patch.dict(sys.modules, {"firmware": fake_fw_module}):
            t = SimTransport()
            t.on_telemetry = _on_tlm
            t.on_log = lambda _: None
            t.connect()

        # Inject a TLM line into the fake sim's async event queue.
        fake_sim.inject_tlm("TLM t=12345 enc=100,200 pose=300,400,900")

        # Wait for delivery.
        delivered = done_evt.wait(timeout=2.0)
        t.disconnect()

        assert delivered, "on_telemetry was not called within 2 s"
        assert len(received_frames) >= 1
        frame = received_frames[0]
        assert frame.t == 12345
        assert frame.enc == (100, 200)

    def test_non_tlm_lines_not_delivered(self):
        """EVT lines in get_async_evts() do not trigger on_telemetry."""
        from unittest.mock import patch, MagicMock
        from robot_radio.testgui.transport import SimTransport
        import pathlib
        import sys

        fake_sim = FakeSim()
        fake_path = MagicMock(spec=pathlib.Path)
        fake_path.exists.return_value = True
        fake_path.parent.parent = str(pathlib.Path("/nonexistent/sim"))
        fake_fw_module = MagicMock()
        fake_fw_module.Sim.return_value = fake_sim

        received_frames: list = []

        with patch(
            "robot_radio.testgui.transport._sim_lib_path",
            return_value=fake_path,
        ), patch.dict(sys.modules, {"firmware": fake_fw_module}):
            t = SimTransport()
            t.on_telemetry = lambda f: received_frames.append(f)
            t.on_log = lambda _: None
            t.connect()

        # Inject an EVT line — should not trigger on_telemetry.
        fake_sim.inject_tlm("EVT done T")
        time.sleep(0.2)
        t.disconnect()

        assert received_frames == [], "EVT line should not produce a TLMFrame"


class TestSimTransportTruthDelivery:
    """Ground-truth pose is delivered to on_truth with correct unit conversion."""

    def test_truth_delivered_to_callback(self):
        """on_truth is called with (x_cm, y_cm, yaw_rad) from the sim."""
        from unittest.mock import patch, MagicMock
        from robot_radio.testgui.transport import SimTransport, _SIM_TRUTH_EVERY_N_TICKS
        import pathlib
        import sys

        fake_sim = FakeSim()
        # Set a known true pose (mm units from sim).
        fake_sim._pose_x_mm = 1000.0   # 100.0 cm
        fake_sim._pose_y_mm = 500.0    # 50.0 cm
        fake_sim._pose_h_rad = 1.2

        fake_path = MagicMock(spec=pathlib.Path)
        fake_path.exists.return_value = True
        fake_path.parent.parent = str(pathlib.Path("/nonexistent/sim"))
        fake_fw_module = MagicMock()
        fake_fw_module.Sim.return_value = fake_sim

        received_poses: list = []
        done_evt = threading.Event()

        def _on_truth(pose):
            if pose is not None:
                received_poses.append(pose)
                done_evt.set()

        with patch(
            "robot_radio.testgui.transport._sim_lib_path",
            return_value=fake_path,
        ), patch.dict(sys.modules, {"firmware": fake_fw_module}):
            t = SimTransport()
            t.on_truth = _on_truth
            t.on_log = lambda _: None
            t.connect()

        # Wait long enough for at least _SIM_TRUTH_EVERY_N_TICKS ticks.
        wait_s = (_SIM_TRUTH_EVERY_N_TICKS + 2) * 0.025 + 0.5
        delivered = done_evt.wait(timeout=wait_s)
        t.disconnect()

        assert delivered, f"on_truth was not called within {wait_s:.1f} s"
        assert len(received_poses) >= 1

        x_cm, y_cm, yaw_rad = received_poses[0]
        # Conversion: mm → cm (divide by 10); heading passthrough.
        assert abs(x_cm - 100.0) < 1e-3, f"x_cm mismatch: {x_cm}"
        assert abs(y_cm - 50.0) < 1e-3, f"y_cm mismatch: {y_cm}"
        assert abs(yaw_rad - 1.2) < 1e-3, f"yaw_rad mismatch: {yaw_rad}"
