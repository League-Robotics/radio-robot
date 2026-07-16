"""src/tests/testgui/test_relay_discovery.py -- ticket 085-008: camera/relay
selection test port. Ported from ``tests_old/testgui/test_relay_discovery.py``.

Unit tests for relay auto-discovery. All tests are Qt-free and headless --
no QApplication is required. ``_relay_probe_banner`` is tested with a
non-existent port to verify defensive error handling, and with a fake
``serial.Serial`` to verify the HELLO-classify handshake (send HELLO, read
the DEVICE: reply -- see
``.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md``).
``find_relay_port`` is tested entirely with injectable fake probe functions.

No production code change: pure verification pass.

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest src/tests/testgui/test_relay_discovery.py -q
"""

from __future__ import annotations

import serial  # type: ignore[import]

from robot_radio.testgui.transport import find_relay_port, _relay_probe_banner

# ---------------------------------------------------------------------------
# Tests: find_relay_port
# ---------------------------------------------------------------------------


class TestFindRelayPort:
    """find_relay_port -- pure relay port discovery logic."""

    def test_match_on_first_port(self):
        """Returns the first matching port when its banner contains RADIOBRIDGE."""
        def probe(port: str) -> str | None:
            return "DEVICE:RADIOBRIDGE:relay:gozop:abc123" if port == "/dev/relay" else None

        result = find_relay_port(["/dev/other", "/dev/relay"], probe)
        assert result == "/dev/relay"

    def test_empty_list_returns_none(self):
        """Returns None immediately when port_list is empty."""
        result = find_relay_port([], lambda p: "DEVICE:RADIOBRIDGE:relay:gozop:x")
        assert result is None

    def test_no_match_returns_none(self):
        """Returns None when no port's banner contains RADIOBRIDGE."""
        result = find_relay_port(["/dev/portA", "/dev/portB"], lambda p: None)
        assert result is None

    def test_stops_early_after_first_match(self):
        """probe_fn is not called for ports after the first match."""
        calls: list[str] = []

        def probe(port: str) -> str | None:
            calls.append(port)
            return "DEVICE:RADIOBRIDGE:relay:gozop:x" if port == "/dev/first" else None

        find_relay_port(["/dev/first", "/dev/second"], probe)
        assert "/dev/second" not in calls, (
            f"probe was called for /dev/second even though /dev/first already matched; "
            f"calls={calls}"
        )

    def test_skips_port_on_probe_exception(self):
        """An exception from probe_fn is silently caught; remaining ports are tried."""
        def probe(port: str) -> str | None:
            if port == "/dev/bad":
                raise IOError("port exploded")
            return "DEVICE:RADIOBRIDGE:relay:gozop:y"

        result = find_relay_port(["/dev/bad", "/dev/good"], probe)
        assert result == "/dev/good"

    def test_no_radiobridge_in_banner(self):
        """Returns None when the banner does not contain 'RADIOBRIDGE'."""
        result = find_relay_port(
            ["/dev/robot"],
            lambda p: "DEVICE:NEZHA2:robot:tovez:1",
        )
        assert result is None

    def test_none_banner_is_skipped(self):
        """Returns None when probe_fn returns None for all ports."""
        result = find_relay_port(["/dev/portA"], lambda p: None)
        assert result is None

    def test_returns_first_of_multiple_matches(self):
        """When multiple ports match, the first one in list order is returned."""
        def probe(port: str) -> str | None:
            return "DEVICE:RADIOBRIDGE:relay:gozop:x"

        result = find_relay_port(["/dev/alpha", "/dev/beta"], probe)
        assert result == "/dev/alpha"

    def test_partial_banner_match(self):
        """RADIOBRIDGE token anywhere in the banner is sufficient."""
        result = find_relay_port(
            ["/dev/p1"],
            lambda p: "prefix RADIOBRIDGE suffix",
        )
        assert result == "/dev/p1"


# ---------------------------------------------------------------------------
# Tests: _relay_probe_banner
# ---------------------------------------------------------------------------


class TestRelayProbeBanner:
    """_relay_probe_banner -- real I/O probe defensive error handling."""

    def test_returns_none_for_nonexistent_port(self):
        """Opening a non-existent port returns None without raising."""
        result = _relay_probe_banner("/dev/nonexistent_port_xyz_does_not_exist_12345")
        assert result is None

    def test_returns_none_for_garbage_port_name(self):
        """Garbage port names return None, not an exception."""
        result = _relay_probe_banner("not_a_port_at_all")
        assert result is None


# ---------------------------------------------------------------------------
# Fake serial.Serial for HELLO-classify handshake tests.
# ---------------------------------------------------------------------------


class _FakeSerial:
    """Minimal fake pyserial ``Serial`` for headless probe tests.

    Only replies with ``reply_line`` (if any) AFTER a ``b"HELLO\\n"`` write is
    observed -- this is what makes the tests prove the probe actively sends
    HELLO rather than relying on a spontaneous boot banner.  If
    ``raise_on_init`` is set, the constructor raises immediately, simulating
    an ``open()`` failure (port busy, permission denied, etc).
    """

    #: Instances created, in order -- lets tests inspect/close-assert them.
    created: list["_FakeSerial"] = []

    def __init__(self, port, baud, timeout=None, reply_line=None, raise_on_init=False):
        if raise_on_init:
            raise OSError(f"could not open port {port!r}")
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.closed = False
        self.hello_received = False
        self._reply_line = reply_line
        self._replied = False
        _FakeSerial.created.append(self)

    def reset_input_buffer(self) -> None:
        pass

    def write(self, data: bytes) -> None:
        if data == b"HELLO\n":
            self.hello_received = True

    def flush(self) -> None:
        pass

    def readline(self) -> bytes:
        # Never reply until HELLO has been sent -- proves the probe does not
        # depend on a spontaneous boot banner.
        if self.hello_received and self._reply_line and not self._replied:
            self._replied = True
            return (self._reply_line + "\n").encode("ascii")
        return b""

    def close(self) -> None:
        self.closed = True


def _make_fake_serial_factory(**kwargs):
    """Return a callable usable as a ``serial.Serial`` replacement.

    Extra kwargs are forwarded to each ``_FakeSerial`` instance created.
    """
    def _factory(port, baud, timeout=None):
        return _FakeSerial(port, baud, timeout=timeout, **kwargs)
    return _factory


class TestRelayProbeBannerHelloClassify:
    """_relay_probe_banner -- HELLO-classify handshake (fake serial, no hardware)."""

    def setup_method(self) -> None:
        _FakeSerial.created.clear()

    def test_sends_hello_and_reads_relay_banner(self, monkeypatch):
        """Replies only after HELLO is sent; probe returns the DEVICE: banner."""
        monkeypatch.setattr(
            serial,
            "Serial",
            _make_fake_serial_factory(
                reply_line="DEVICE:RADIOBRIDGE:relay:zavaz:4076631795"
            ),
        )
        result = _relay_probe_banner("/dev/fake-relay", timeout_s=0.5)
        assert result == "DEVICE:RADIOBRIDGE:relay:zavaz:4076631795"
        assert len(_FakeSerial.created) == 1
        assert _FakeSerial.created[0].hello_received is True

    def test_port_closed_after_successful_probe(self, monkeypatch):
        """The port is closed after a successful HELLO-classify probe."""
        monkeypatch.setattr(
            serial,
            "Serial",
            _make_fake_serial_factory(
                reply_line="DEVICE:RADIOBRIDGE:relay:zavaz:4076631795"
            ),
        )
        _relay_probe_banner("/dev/fake-relay", timeout_s=0.5)
        assert _FakeSerial.created[0].closed is True

    def test_never_replies_returns_none_within_timeout(self, monkeypatch):
        """A device that never replies causes the probe to return None."""
        monkeypatch.setattr(serial, "Serial", _make_fake_serial_factory(reply_line=None))
        result = _relay_probe_banner("/dev/fake-silent", timeout_s=0.5)
        assert result is None

    def test_port_closed_after_no_reply(self, monkeypatch):
        """The port is closed even when no DEVICE: reply ever arrives."""
        monkeypatch.setattr(serial, "Serial", _make_fake_serial_factory(reply_line=None))
        _relay_probe_banner("/dev/fake-silent", timeout_s=0.5)
        assert _FakeSerial.created[0].closed is True

    def test_robot_banner_is_returned_but_not_classified_as_relay(self, monkeypatch):
        """A robot answering HELLO with its own banner is returned as-is;

        find_relay_port then skips the port because the banner lacks
        RADIOBRIDGE.
        """
        monkeypatch.setattr(
            serial,
            "Serial",
            _make_fake_serial_factory(reply_line="DEVICE:NEZHA2:robot:tovez:1"),
        )
        result = _relay_probe_banner("/dev/fake-robot", timeout_s=0.5)
        assert result == "DEVICE:NEZHA2:robot:tovez:1"

        found = find_relay_port(["/dev/fake-robot"], lambda p: result)
        assert found is None

    def test_raises_on_open_returns_none(self, monkeypatch):
        """An exception from serial.Serial(...) (e.g. port busy) yields None."""
        monkeypatch.setattr(
            serial, "Serial", _make_fake_serial_factory(raise_on_init=True)
        )
        result = _relay_probe_banner("/dev/fake-busy", timeout_s=0.5)
        assert result is None

    def test_port_skipped_by_find_relay_port_on_open_failure(self, monkeypatch):
        """find_relay_port skips a port whose probe raises during open."""
        monkeypatch.setattr(
            serial, "Serial", _make_fake_serial_factory(raise_on_init=True)
        )

        def probe(port: str) -> str | None:
            return _relay_probe_banner(port, timeout_s=0.5)

        result = find_relay_port(["/dev/fake-busy"], probe)
        assert result is None

    def test_resends_hello_within_timeout_when_device_mid_boot(self, monkeypatch):
        """HELLO is retried if the device is still booting on the first attempt.

        Simulates a device that ignores the first N HELLO writes (as if it
        were mid-boot) and only replies once a later HELLO arrives.
        """

        class _MidBootSerial(_FakeSerial):
            def __init__(self, port, baud, timeout=None):
                super().__init__(
                    port,
                    baud,
                    timeout=timeout,
                    reply_line="DEVICE:RADIOBRIDGE:relay:zavaz:1",
                )
                self._hellos_seen = 0

            def write(self, data: bytes) -> None:
                if data == b"HELLO\n":
                    self._hellos_seen += 1
                    # Only "wake up" and start replying after the 2nd HELLO.
                    if self._hellos_seen >= 2:
                        self.hello_received = True

        monkeypatch.setattr(serial, "Serial", _MidBootSerial)
        result = _relay_probe_banner("/dev/fake-midboot", timeout_s=2.0)
        assert result == "DEVICE:RADIOBRIDGE:relay:zavaz:1"
