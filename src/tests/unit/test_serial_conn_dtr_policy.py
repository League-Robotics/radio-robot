"""src/tests/unit/test_serial_conn_dtr_policy.py -- ticket 133-006 part 1
(A-live-config-push-is-wiped-by-the-next-reconnect.md): ``SerialConnection``
must NOT assert DTR when it opens a port.

Why this is a test and not just a one-line change: asserting DTR resets the
micro:bit through the DAPLink, so every host connection rebooted the robot,
which silently erased any live config push (``DRIVE`` is deliberately not
flash-persisted). 132-019 lost a bench measurement to exactly that. The
sprint-036 justification for asserting it -- "without a reset there is no boot
banner, and the banner is required for HELLO-classify" -- was stale:
``_banner_classify()`` sends ``HELLO`` and the device answers.

The two properties worth pinning, both of which a future "tidy up the connect
path" edit could quietly undo:

1. ``dtr`` is set False BEFORE ``open()``. Order matters -- clearing it after
   the fact still pulses the line, which is the entire thing being avoided.
2. Classification does not depend on an unsolicited boot banner. The fakes
   below emit a ``DEVICE:`` line ONLY in response to ``HELLO``, never on open,
   so a regression to "wait for the boot announcement" fails here rather than
   on the bench.

Hardware-level confirmation is out of scope for a host unit test and is
recorded on the ticket instead: measured on the ``getez`` relay 2026-08-04
(handshake reaches ``# entering data plane`` with DTR deasserted, three
consecutive opens, no power cycle), and on ``tovez`` over direct USB for the
clock-does-not-reset half.

Collected under ``src/tests/unit/`` -- ``pyproject.toml``'s ``testpaths``
includes ``tests/unit``, so ``uv run python -m pytest`` collects it by default.
"""

from __future__ import annotations

import pytest

from robot_radio.io import serial_conn as serial_conn_mod
from robot_radio.io.serial_conn import SerialConnection

_ROBOT_BANNER = b"DEVICE:NEZHA2:robot:tovez:2314287040\n"
_RELAY_BANNER = b"DEVICE:RADIOBRIDGE:relay:getez:1784514240\n"


class _RecordingSerial:
    """A ``pyserial.Serial`` stand-in that records the DTR/open ordering and
    answers the handshake the way a real device does -- on request only.

    Deliberately emits NOTHING on open: a device that is already running (the
    normal case once connecting stops resetting it) has no boot announcement
    left to send.
    """

    #: every instance constructed during a test, in order
    instances: list["_RecordingSerial"] = []
    #: which banner every instance answers HELLO with (set per test)
    next_banner: bytes = _ROBOT_BANNER

    def __init__(self, *args, **kwargs) -> None:
        self.init_kwargs = kwargs
        self.port = None
        self.is_open = False
        self.events: list[str] = []
        self._dtr = None
        self._dtr_at_open = None
        self._rx = bytearray()
        self.written: list[bytes] = []
        self._pending = bytearray()
        self.banner = _RecordingSerial.next_banner
        _RecordingSerial.instances.append(self)

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

    def fileno(self) -> int:
        # _disable_hupcl() catches everything; there is no real fd here.
        raise OSError("no fd on a fake serial port")

    # -- I/O -----------------------------------------------------------
    def reset_input_buffer(self) -> None:
        self._rx.clear()

    def flush(self) -> None:
        pass

    def write(self, data: bytes) -> int:
        self.written.append(data)
        self._pending += data
        while b"\n" in self._pending:
            line, _, rest = self._pending.partition(b"\n")
            self._pending = bytearray(rest)
            self._respond(line.strip())
        return len(data)

    def _respond(self, line: bytes) -> None:
        if line == b"HELLO":
            self._rx += self.banner
        elif line == b"PING":
            # READY first: _poll_read_lines stops at the PONG: token, so a
            # READY emitted after it would never be seen.
            self._rx += b"READY\n"
            self._rx += b"PONG:t=40416\n"
        elif line == b"?":
            self._rx += b"# channel: 0 group: 10 mode: RAW250 power: 7\n"
        elif line == b"!ECHO OFF":
            self._rx += b"# echo: OFF\n"
        elif line == b"!MODE RAW250":
            self._rx += b"# mode: RAW250\n"
        elif line == b"!GO":
            self._rx += b"# entering data plane\n"

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


@pytest.fixture
def recording_serial(monkeypatch):
    _RecordingSerial.instances = []
    _RecordingSerial.next_banner = _ROBOT_BANNER
    monkeypatch.setattr(serial_conn_mod.serial, "Serial", _RecordingSerial)
    yield _RecordingSerial
    _RecordingSerial.next_banner = _ROBOT_BANNER


def _connect(port: str = "/dev/fake", mode: str | None = None):
    conn = SerialConnection(port=port, mode=mode) if mode else SerialConnection(port=port)
    info = conn.connect()
    return conn, info


# ---------------------------------------------------------------------------
# 1. The policy itself.
# ---------------------------------------------------------------------------


def test_connect_deasserts_dtr_before_opening_the_port(recording_serial):
    conn, info = _connect()

    assert info.get("status") == "connected", info
    fake = recording_serial.instances[0]
    assert fake.dtr_at_open is False, (
        "the port was opened with DTR asserted -- that reboots the robot and "
        "silently erases any live config push")
    assert fake.events.index("dtr=False") < fake.events.index("open"), (
        f"dtr must be cleared BEFORE open(); event order was {fake.events}. "
        "Clearing it afterwards still pulses the line.")
    conn.disconnect()


def test_connect_does_not_reenable_dtr_anywhere_in_the_handshake(recording_serial):
    """No later step may put DTR back -- a reset halfway through the
    handshake would be just as destructive as one at open."""
    conn, _ = _connect()

    fake = recording_serial.instances[0]
    assert fake.dtr is False, f"DTR ended the handshake as {fake.dtr!r}"
    assert "dtr=True" not in fake.events, f"event order was {fake.events}"
    conn.disconnect()


# ---------------------------------------------------------------------------
# 2. Classification still works without a boot banner.
# ---------------------------------------------------------------------------


def test_direct_robot_classifies_from_the_hello_reply_not_a_boot_banner(recording_serial):
    conn, info = _connect()

    announcement = info.get("announcement")
    assert announcement is not None, (
        "classification failed with no unsolicited boot banner -- HELLO is "
        f"supposed to elicit one. info={info}")
    assert announcement["role"] == "NEZHA2"
    assert announcement["device_name"] == "tovez"
    assert info["mode"] == "direct"

    fake = recording_serial.instances[0]
    assert any(b"HELLO" in payload for payload in fake.written), (
        "no HELLO was sent; the banner cannot have been requested")
    conn.disconnect()


def test_relay_handshake_completes_with_dtr_deasserted(recording_serial):
    """The relay was the open question -- its ``!GO`` data plane is stateful,
    so it was plausible the dongle needed the reset to be found in its control
    plane. It does not: measured against ``getez`` on 2026-08-04, and pinned
    here so the per-role exception is not reintroduced without evidence."""
    recording_serial.next_banner = _RELAY_BANNER
    conn, info = _connect(port="/dev/fake-relay")

    assert info.get("status") == "connected", info
    assert info["announcement"]["role"] == "RADIOBRIDGE"
    relay_info = info.get("relay_info")
    assert relay_info is not None, f"relay handshake never ran: {info}"
    assert relay_info["entered_data_plane"] is True, (
        "the relay did not reach its data plane with DTR deasserted")

    fake = recording_serial.instances[0]
    assert fake.dtr_at_open is False
    conn.disconnect()


# ---------------------------------------------------------------------------
# 3. The stale comment is gone.
# ---------------------------------------------------------------------------


def test_the_stale_dtr_required_comment_is_corrected():
    """``serial_conn.py`` used to carry a comment asserting the DTR pulse was
    required because the banner is boot-only. Code and comment must not
    disagree -- a reader who believes the comment will restore the reset."""
    from pathlib import Path

    source = Path(serial_conn_mod.__file__).read_text()

    assert "Do NOT force dtr=False here" not in source, (
        "the stale sprint-036 comment instructing the reader NOT to clear DTR "
        "is still present, and now contradicts the code")
    assert "DTR assertion is the correct default" not in source, (
        "the stale 'DTR assertion is the correct default for the relay path' "
        "claim is still present; the relay was measured not to need it")
    assert "DTR policy" in source, (
        "the corrected DTR policy note is missing -- the change needs to carry "
        "its own reasoning, not just flip a flag")


if __name__ == "__main__":
    import sys

    sys.exit(pytest.main([__file__, "-v"]))
