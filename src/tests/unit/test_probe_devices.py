"""src/tests/unit/test_probe_devices.py -- coverage for
``serial_conn.probe_devices()``, the second discovery probe touched by ticket
133-006 (the third, testgui's ``_relay_probe_banner()``, is covered by
``src/tests/testgui/test_relay_discovery.py``; ``SerialConnection.connect()``
itself by ``test_serial_conn_dtr_policy.py``).

Why this file exists at all: ``probe_devices()`` had NO test coverage, and it
shows. It is also exposed as an MCP tool, so "what is plugged in?" is a
question agents ask constantly, and it was carrying two independent defects:

1. It opened every port with DTR asserted, so enumerating the hub rebooted
   every micro:bit on it -- the robot included, discarding any live config
   push (``DRIVE`` is deliberately not flash-persisted). Fixed by 133-006.

2. It unpacked ``for kind, payload in demux.feed(chunk)``, but
   ``ByteStreamDemuxer.feed()`` returns a list of complete LINES, not
   ``(kind, payload)`` pairs. Every probe of a device that actually ANSWERED
   therefore died on ``ValueError: too many values to unpack (expected 2)``,
   was swallowed by the enclosing ``except Exception``, and was reported as a
   bare ``{"port", "error"}`` row. Silent devices, which never reached the
   unpack, looked fine. Also fixed by 133-006, found by hand while verifying
   defect 1.

Defect 2 is the reason defect 1 was hard to confirm -- the probe could not
demonstrate a successful classification either way. Both are pinned below,
because both are the kind a future tidy-up reintroduces without noticing:
nothing about either failure is loud.

Collected under ``src/tests/unit/`` -- ``pyproject.toml``'s ``testpaths``
includes ``src/tests/unit``, so ``uv run python -m pytest`` collects it.
"""

from __future__ import annotations

import pytest

from robot_radio.io import serial_conn as serial_conn_mod

_ROBOT_BANNER = b"DEVICE:NEZHA2:robot:tovez:2314287040\n"
_RELAY_BANNER = b"DEVICE:RADIOBRIDGE:relay:getez:1784514240\n"


class _ProbeSerial:
    """A ``pyserial.Serial`` stand-in for the ``probe_devices()`` loop.

    Records the DTR/open ordering, and answers ``HELLO`` with a banner -- on
    request only. Emits NOTHING on open: with the reset gone there is no boot
    announcement left to send, so a regression to waiting for one shows up
    here as an unresponsive probe.

    Optionally prefixes the banner with a binary telemetry line, which the
    real firmware interleaves continuously; the probe must skip it rather
    than report it as a device line.
    """

    #: every instance constructed during a test, in order
    instances: list["_ProbeSerial"] = []
    #: what every instance answers HELLO with (set per test)
    next_banner: bytes = _ROBOT_BANNER
    #: prepended to the banner when set, to model interleaved telemetry
    next_preamble: bytes = b""

    def __init__(self, *args, **kwargs) -> None:
        self.init_args = args
        self.init_kwargs = kwargs
        self.port = None
        self.is_open = False
        self.events: list[str] = []
        self.written: list[bytes] = []
        self._dtr = None
        self._dtr_at_open = None
        self._rx = bytearray()
        self.banner = _ProbeSerial.next_banner
        self.preamble = _ProbeSerial.next_preamble
        _ProbeSerial.instances.append(self)

    # -- the property under test ---------------------------------------
    @property
    def dtr(self):
        return self._dtr

    @dtr.setter
    def dtr(self, value) -> None:
        self._dtr = value
        self.events.append(f"dtr={value}")

    @property
    def dtr_at_open(self):
        """What ``dtr`` was when ``open()`` ran -- None if never set."""
        return self._dtr_at_open

    def open(self) -> None:
        self.is_open = True
        self._dtr_at_open = self._dtr
        self.events.append("open")

    def close(self) -> None:
        self.is_open = False
        self.events.append("close")

    # -- I/O -----------------------------------------------------------
    def reset_input_buffer(self) -> None:
        self._rx.clear()

    def flush(self) -> None:
        pass

    def write(self, data: bytes) -> int:
        self.written.append(data)
        if data.strip() == b"HELLO":
            self._rx += self.preamble
            self._rx += self.banner
        return len(data)

    @property
    def in_waiting(self) -> int:
        return len(self._rx)

    def read(self, size: int = 1) -> bytes:
        if not self._rx:
            return b""          # a read timeout, not a closed port
        n = max(1, min(size, len(self._rx)))
        chunk = bytes(self._rx[:n])
        del self._rx[:n]
        return chunk


class _SilentSerial(_ProbeSerial):
    """A port that opens fine and never answers anything."""

    def write(self, data: bytes) -> int:
        self.written.append(data)
        return len(data)


@pytest.fixture
def probe_serial(monkeypatch):
    _ProbeSerial.instances = []
    _ProbeSerial.next_banner = _ROBOT_BANNER
    _ProbeSerial.next_preamble = b""
    monkeypatch.setattr(serial_conn_mod, "list_serial_ports", lambda: ["/dev/fake-robot"])
    monkeypatch.setattr(serial_conn_mod.serial, "Serial", _ProbeSerial)
    yield _ProbeSerial
    _ProbeSerial.next_banner = _ROBOT_BANNER
    _ProbeSerial.next_preamble = b""


# ---------------------------------------------------------------------------
# 1. The DTR policy (133-006).
# ---------------------------------------------------------------------------


def test_probe_opens_every_port_with_dtr_deasserted(probe_serial):
    """Enumerating the hub must not reboot the devices on it."""
    serial_conn_mod.probe_devices(read_timeout=600)

    fake = probe_serial.instances[0]
    assert fake.dtr_at_open is False, (
        "probe_devices() opened the port with DTR asserted -- asking what is "
        "plugged in then reboots every micro:bit on the hub, discarding any "
        "live config the robot was holding")
    assert fake.events.index("dtr=False") < fake.events.index("open"), (
        f"dtr must be cleared BEFORE open(); event order was {fake.events}. "
        "Clearing it afterwards still pulses the line.")
    assert "dtr=True" not in fake.events, f"event order was {fake.events}"


def test_probe_closes_the_port_it_opened(probe_serial):
    """A discovery sweep must not leave ports held open behind it."""
    serial_conn_mod.probe_devices(read_timeout=600)

    fake = probe_serial.instances[0]
    assert "close" in fake.events, f"port was never closed; events {fake.events}"


# ---------------------------------------------------------------------------
# 2. Classification works, and does not depend on a boot banner.
# ---------------------------------------------------------------------------


def test_responsive_device_is_classified_from_its_hello_reply(probe_serial):
    """The regression test for the ``feed()`` unpacking bug.

    A device that ANSWERS is the case that used to fail: the bad unpack raised
    only once there was a line to iterate, so this exact path produced an
    ``{"port", "error"}`` row while silent ports looked healthy.
    """
    results = serial_conn_mod.probe_devices(read_timeout=600)

    assert len(results) == 1, results
    row = results[0]
    assert "error" not in row, (
        f"probing a device that answered produced an error row: {row}. "
        "That is the signature of the feed() unpacking bug -- feed() returns "
        "a list of lines, not (kind, payload) pairs.")
    assert row["port"] == "/dev/fake-robot"
    assert row["responsive"] is True, row
    assert any("DEVICE:NEZHA2:robot:tovez" in line for line in row["lines"]), row

    fake = probe_serial.instances[0]
    assert any(b"HELLO" in payload for payload in fake.written), (
        "no HELLO was sent; the banner cannot have been requested -- and with "
        "DTR deasserted there is no boot banner to wait for")


def test_relay_is_classified_the_same_way(probe_serial):
    """The relay answers the same probe; it needs no reset either."""
    probe_serial.next_banner = _RELAY_BANNER
    results = serial_conn_mod.probe_devices(read_timeout=600)

    row = results[0]
    assert row["responsive"] is True, row
    assert any("DEVICE:RADIOBRIDGE" in line for line in row["lines"]), row
    assert probe_serial.instances[0].dtr_at_open is False


def test_interleaved_binary_telemetry_is_skipped_not_reported(probe_serial):
    """Firmware pushes telemetry continuously, so a probe reply arrives with
    binary frames around it. Those must be demuxed away, not decoded into the
    reported line list."""
    probe_serial.next_preamble = b"TLM:\x01\x02\x03\x04\n"
    results = serial_conn_mod.probe_devices(read_timeout=600)

    row = results[0]
    assert row["responsive"] is True, row
    assert not any(line.startswith("TLM:") for line in row["lines"]), (
        f"a binary telemetry line was reported as a device line: {row['lines']}")


def test_silent_port_is_reported_unresponsive_not_as_an_error(monkeypatch):
    """A port that opens but never answers is a normal, non-error outcome."""
    _ProbeSerial.instances = []
    monkeypatch.setattr(serial_conn_mod, "list_serial_ports", lambda: ["/dev/fake-silent"])
    monkeypatch.setattr(serial_conn_mod.serial, "Serial", _SilentSerial)

    results = serial_conn_mod.probe_devices(read_timeout=400)

    row = results[0]
    assert "error" not in row, row
    assert row["responsive"] is False, row
    assert row["lines"] == [], row


def test_unopenable_port_is_reported_as_an_error_row(monkeypatch):
    """A port that cannot be opened yields an error row rather than raising --
    one bad port must not abort the whole sweep."""
    class _Unopenable(_ProbeSerial):
        def open(self) -> None:
            raise OSError("could not open port: busy")

    _ProbeSerial.instances = []
    monkeypatch.setattr(serial_conn_mod, "list_serial_ports", lambda: ["/dev/fake-busy"])
    monkeypatch.setattr(serial_conn_mod.serial, "Serial", _Unopenable)

    results = serial_conn_mod.probe_devices(read_timeout=400)

    assert len(results) == 1
    assert results[0]["port"] == "/dev/fake-busy"
    assert "error" in results[0], results[0]


if __name__ == "__main__":
    import sys

    sys.exit(pytest.main([__file__, "-v"]))
