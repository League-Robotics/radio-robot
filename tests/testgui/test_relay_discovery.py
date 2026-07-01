"""tests/testgui/test_relay_discovery.py — Unit tests for relay auto-discovery.

All tests are Qt-free and headless — no QApplication is required.
``_relay_probe_banner`` is tested with a non-existent port to verify
defensive error handling; ``find_relay_port`` is tested entirely with
injectable fake probe functions.

Run with:
    QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui -q
"""

from __future__ import annotations

from robot_radio.testgui.transport import find_relay_port, _relay_probe_banner

# ---------------------------------------------------------------------------
# Tests: find_relay_port
# ---------------------------------------------------------------------------


class TestFindRelayPort:
    """find_relay_port — pure relay port discovery logic."""

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
    """_relay_probe_banner — real I/O probe defensive error handling."""

    def test_returns_none_for_nonexistent_port(self):
        """Opening a non-existent port returns None without raising."""
        result = _relay_probe_banner("/dev/nonexistent_port_xyz_does_not_exist_12345")
        assert result is None

    def test_returns_none_for_garbage_port_name(self):
        """Garbage port names return None, not an exception."""
        result = _relay_probe_banner("not_a_port_at_all")
        assert result is None
