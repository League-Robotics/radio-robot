"""Unit tests for serial_conn.probe_devices() (CR-15 item 2, sprint 066).

probe_devices() used to send the retired '>PING' relay-control-plane prefix,
which the current relay firmware's data-plane pipe does not recognize on
either a direct or relay-fronted port -- it could never observe a live
device. The fix rewrites it to the plain HELLO-classify protocol
_banner_classify() already uses: send HELLO repeatedly, watch for a
DEVICE: banner line.

Return shape ({port, lines, responsive}) is unchanged so the one MCP tool
caller (robot_mcp.py) needs no update.

All tests mock at the ``serial.Serial`` boundary -- no hardware required.
"""

from __future__ import annotations

import queue
import threading
import time
from unittest.mock import patch

import pytest


# ---------------------------------------------------------------------------
# Fake serial device
# ---------------------------------------------------------------------------


class _FakeProbeSerial:
    """Minimal fake pyserial.Serial for probe_devices() tests.

    probe_devices() constructs ``serial.Serial(port, BAUD_RATE,
    timeout=READ_TIMEOUT_S)`` -- a positional/keyword mix -- so the fake
    accepts that exact call shape.
    """

    def __init__(self, port=None, baudrate=None, timeout=None, **kwargs) -> None:
        self.port = port
        self.written: list[bytes] = []
        self._q: "queue.Queue[bytes]" = queue.Queue()

    def write(self, data: bytes) -> int:
        self.written.append(data)
        return len(data)

    def flush(self) -> None:
        pass

    def reset_input_buffer(self) -> None:
        pass

    def readline(self) -> bytes:
        try:
            return self._q.get(timeout=0.05)
        except queue.Empty:
            return b""

    def close(self) -> None:
        pass

    def inject(self, line: str) -> None:
        self._q.put((line + "\n").encode("utf-8"))

    def written_text(self) -> list[str]:
        out = []
        for b in self.written:
            for part in b.decode("utf-8", "ignore").splitlines():
                s = part.strip()
                if s:
                    out.append(s)
        return out


def _respond_after(fake: _FakeProbeSerial, line: str, delay: float = 0.05) -> threading.Thread:
    def _work():
        time.sleep(delay)
        fake.inject(line)

    t = threading.Thread(target=_work, daemon=True)
    t.start()
    return t


def _patch_probe(fakes_by_port: dict) -> "patch":
    """Patch serial.Serial and list_serial_ports for probe_devices()."""

    def _make_serial(port, *args, **kwargs):
        return fakes_by_port[port]

    ports = list(fakes_by_port.keys())
    return (
        patch("robot_radio.io.serial_conn.serial.Serial", side_effect=_make_serial),
        patch("robot_radio.io.serial_conn.list_serial_ports", return_value=ports),
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestProbeDevicesHelloProtocol:
    def test_sends_plain_hello_not_ping(self):
        """probe_devices() must send HELLO, never the retired '>PING'."""
        from robot_radio.io import serial_conn

        fake = _FakeProbeSerial()
        _respond_after(fake, "DEVICE:NEZHA2:robot:tovez:AB:CD:EF:01")

        p_serial, p_ports = _patch_probe({"/dev/cu.usbmodemFAKE": fake})
        with p_serial, p_ports:
            serial_conn.probe_devices(read_ms=300)

        sent = fake.written_text()
        assert "HELLO" in sent, f"HELLO not sent: {sent}"
        assert not any(">PING" in s for s in sent), (
            f"Retired '>PING' relay-prefix protocol still in use: {sent}"
        )
        assert not any("PING" in s for s in sent if s != "HELLO"), (
            f"Unexpected PING variant sent: {sent}"
        )

    def test_responsive_true_on_device_banner(self):
        """A DEVICE: banner within read_ms marks the port responsive."""
        from robot_radio.io import serial_conn

        fake = _FakeProbeSerial()
        _respond_after(fake, "DEVICE:RADIOBRIDGE:relay:gozop:00:11:22:33")

        p_serial, p_ports = _patch_probe({"/dev/cu.usbmodemFAKE": fake})
        with p_serial, p_ports:
            results = serial_conn.probe_devices(read_ms=300)

        assert len(results) == 1
        assert results[0]["responsive"] is True
        assert any("DEVICE:" in ln for ln in results[0]["lines"])

    def test_responsive_false_on_timeout(self):
        """No DEVICE: banner within read_ms -> responsive=False, no error."""
        from robot_radio.io import serial_conn

        fake = _FakeProbeSerial()  # never injects anything

        p_serial, p_ports = _patch_probe({"/dev/cu.usbmodemFAKE": fake})
        with p_serial, p_ports:
            results = serial_conn.probe_devices(read_ms=150)

        assert len(results) == 1
        assert results[0]["responsive"] is False
        assert "error" not in results[0]

    def test_return_shape_unchanged(self):
        """Result dict keys stay {port, lines, responsive} (MCP caller contract)."""
        from robot_radio.io import serial_conn

        fake = _FakeProbeSerial()
        _respond_after(fake, "DEVICE:NEZHA2:robot:tovez:AB:CD:EF:01")

        p_serial, p_ports = _patch_probe({"/dev/cu.usbmodemFAKE": fake})
        with p_serial, p_ports:
            results = serial_conn.probe_devices(read_ms=300)

        assert len(results) == 1
        entry = results[0]
        assert entry["port"] == "/dev/cu.usbmodemFAKE"
        assert isinstance(entry["lines"], list)
        assert isinstance(entry["responsive"], bool)

    def test_multiple_ports_probed_independently(self):
        """One responsive relay port and one silent port, probed in order."""
        from robot_radio.io import serial_conn

        fake_relay = _FakeProbeSerial()
        _respond_after(fake_relay, "DEVICE:RADIOBRIDGE:relay:gozop:00:11:22:33")
        fake_silent = _FakeProbeSerial()

        p_serial, p_ports = _patch_probe({
            "/dev/cu.usbmodemA": fake_relay,
            "/dev/cu.usbmodemB": fake_silent,
        })
        with p_serial, p_ports:
            results = serial_conn.probe_devices(read_ms=250)

        by_port = {r["port"]: r for r in results}
        assert by_port["/dev/cu.usbmodemA"]["responsive"] is True
        assert by_port["/dev/cu.usbmodemB"]["responsive"] is False
