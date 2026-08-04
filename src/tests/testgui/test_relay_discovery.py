"""src/tests/testgui/test_relay_discovery.py -- ticket 085-008: camera/relay
selection test port. Ported from ``tests_old/testgui/test_relay_discovery.py``.

Unit tests for relay auto-discovery. All tests are Qt-free and headless --
no QApplication is required. ``_relay_probe_banner`` is tested with a
non-existent port to verify defensive error handling, and with a fake
``serial.Serial`` to verify the HELLO-classify handshake (send HELLO, read
the DEVICE: reply -- see
``.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md``).
``find_relay_port`` is tested entirely with injectable fake probe functions.

DTR policy (133-006 -- these tests were UPDATED, not relaxed)
-------------------------------------------------------------
``_relay_probe_banner()`` used to open the port pyserial's one-step way,
``serial.Serial(port, baud, timeout=...)``, which asserts DTR and therefore
REBOOTED every device it walked past -- a discovery probe that resets the
robot it is merely looking for, discarding any live config it was holding.
It now constructs the port **unopened** (keyword args only), assigns
``.port``, clears ``.dtr``, and only then calls ``.open()``; the clear has to
happen before ``open()`` or the line is pulsed anyway.

``_FakeSerial`` below models that new sequence. The old fake demanded the old
positional ``(port, baud, timeout=...)`` signature, so after the change it was
not constructible at all -- and the probe's defensive ``except Exception``
turned the resulting ``TypeError`` into a silent ``None``, failing five tests
with an assertion about a missing banner rather than about the real cause.
That is worth spelling out because it is the trap in testing this function:
**a fake whose constructor signature has drifted looks exactly like a probe
that timed out.**

What the updated tests pin is the NEW contract, explicitly:

1. ``dtr`` is False, and set BEFORE ``open()`` -- discovery must not reboot
   the devices it enumerates (``test_probe_opens_with_dtr_deasserted``).
2. The banner is ELICITED, never awaited. ``_FakeSerial`` emits nothing on
   open and replies only after it observes a ``HELLO`` write, so a regression
   to "wait for the boot announcement" fails here rather than on the bench.
   This property is what makes point 1 safe: with no reset there is no boot
   banner to wait for.

The same policy, on the ``SerialConnection.connect()`` path, is pinned by
``src/tests/unit/test_serial_conn_dtr_policy.py``; the sibling discovery
probe ``serial_conn.probe_devices()`` by
``src/tests/unit/test_probe_devices.py``.

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

    Models the sequence ``_relay_probe_banner()`` actually performs since
    133-006: construct **unopened** with keyword args only (no positional
    port), assign ``.port``, clear ``.dtr``, then ``.open()``.  ``dtr`` is a
    recording property so a test can assert both its value and its ORDER
    relative to ``open()`` -- clearing DTR after opening still pulses the
    line, so the ordering is the property that matters, not the final value.

    Only replies with ``reply_line`` (if any) AFTER a ``b"HELLO\\n"`` write is
    observed, and emits nothing whatsoever on open.  That is what makes the
    tests prove the probe actively ELICITS the banner rather than waiting for
    a spontaneous boot one -- load bearing now that the probe no longer
    resets the device, because an un-reset device has no boot banner left to
    send.

    ``raise_on_open`` simulates a port that cannot be opened (busy, permission
    denied).  It raises from ``open()``, not from ``__init__``: constructing a
    portless ``serial.Serial`` never fails in pyserial, so a fake that raised
    from its constructor would be exercising a path the probe cannot take.
    """

    #: Instances created, in order -- lets tests inspect/close-assert them.
    created: list["_FakeSerial"] = []

    def __init__(self, *, baudrate=None, timeout=None, reply_line=None,
                 raise_on_open=False, **_ignored):
        self.baudrate = baudrate
        self.timeout = timeout
        self.port = None
        self.is_open = False
        self.closed = False
        self.hello_received = False
        #: ordered log of "dtr=<v>" / "open" / "close" -- the ordering assert
        self.events: list[str] = []
        self._dtr = None
        self._dtr_at_open = None
        self._reply_line = reply_line
        self._raise_on_open = raise_on_open
        self._replied = False
        _FakeSerial.created.append(self)

    # -- DTR, the property under test ----------------------------------
    @property
    def dtr(self):
        return self._dtr

    @dtr.setter
    def dtr(self, value) -> None:
        self._dtr = value
        self.events.append(f"dtr={value}")

    @property
    def dtr_at_open(self):
        """What ``dtr`` was at the moment ``open()`` ran -- None if never set."""
        return self._dtr_at_open

    def open(self) -> None:
        if self._raise_on_open:
            raise OSError(f"could not open port {self.port!r}")
        self.is_open = True
        self._dtr_at_open = self._dtr
        self.events.append("open")

    # -- I/O -----------------------------------------------------------
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
        self.is_open = False
        self.events.append("close")


def _make_fake_serial_factory(**fake_kwargs):
    """Return a callable usable as a ``serial.Serial`` replacement.

    Accepts exactly what the probe passes -- keyword ``baudrate``/``timeout``,
    no positional port -- and forwards the test-only kwargs (``reply_line``,
    ``raise_on_open``) to each ``_FakeSerial`` instance created.
    """
    def _factory(**kwargs):
        return _FakeSerial(**kwargs, **fake_kwargs)
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

    def test_probe_opens_with_dtr_deasserted(self, monkeypatch):
        """The probe clears DTR, and clears it BEFORE open() (133-006).

        Discovery walks every candidate port in turn, so asserting DTR made
        merely asking "what is plugged in?" reboot the robot, silently
        discarding whatever live config it was holding.  The ordering is the
        real assertion: clearing DTR after open() would already have pulsed
        the line.
        """
        monkeypatch.setattr(
            serial,
            "Serial",
            _make_fake_serial_factory(
                reply_line="DEVICE:RADIOBRIDGE:relay:zavaz:4076631795"
            ),
        )
        _relay_probe_banner("/dev/fake-relay", timeout_s=0.5)

        fake = _FakeSerial.created[0]
        assert fake.dtr_at_open is False, (
            "the probe opened the port with DTR asserted -- that reboots every "
            "device it enumerates, which is the whole defect 133-006 removed")
        assert fake.events.index("dtr=False") < fake.events.index("open"), (
            f"dtr must be cleared BEFORE open(); event order was {fake.events}. "
            "Clearing it afterwards still pulses the line.")
        assert "dtr=True" not in fake.events, (
            f"DTR was re-asserted during the probe; event order was {fake.events}")

    def test_banner_is_elicited_by_hello_not_awaited_on_open(self, monkeypatch):
        """No reset means no boot banner -- HELLO is what produces one.

        The counterpart to the DTR test above: it is only safe to stop
        resetting the device because this probe asks for the banner.  The fake
        emits nothing on open, so a regression to a passive boot-banner wait
        times out here instead of on the bench.
        """
        monkeypatch.setattr(
            serial,
            "Serial",
            _make_fake_serial_factory(
                reply_line="DEVICE:RADIOBRIDGE:relay:zavaz:4076631795"
            ),
        )
        result = _relay_probe_banner("/dev/fake-relay", timeout_s=0.5)

        fake = _FakeSerial.created[0]
        assert fake.hello_received is True, (
            "the probe never sent HELLO, so the banner cannot have been "
            "requested -- with DTR deasserted there is nothing to wait for")
        assert result == "DEVICE:RADIOBRIDGE:relay:zavaz:4076631795"

    def test_raises_on_open_returns_none(self, monkeypatch):
        """An exception from ser.open() (e.g. port busy) yields None.

        Raised from ``open()``, not the constructor: since 133-006 the probe
        constructs a portless ``serial.Serial`` first (so DTR can be cleared
        before opening), and that construction never fails in pyserial.
        """
        monkeypatch.setattr(
            serial, "Serial", _make_fake_serial_factory(raise_on_open=True)
        )
        result = _relay_probe_banner("/dev/fake-busy", timeout_s=0.5)
        assert result is None

    def test_port_skipped_by_find_relay_port_on_open_failure(self, monkeypatch):
        """find_relay_port skips a port whose probe raises during open."""
        monkeypatch.setattr(
            serial, "Serial", _make_fake_serial_factory(raise_on_open=True)
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
            def __init__(self, **kwargs):
                super().__init__(
                    reply_line="DEVICE:RADIOBRIDGE:relay:zavaz:1",
                    **kwargs,
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
