"""tests/testgui/test_drive.py — headless tests for KeyboardDriver (ticket 083-002).

Covers the rewrite from dead top-level ``VW``/``STOP`` wire verbs to the
firmware's actual ``DEV DT VW``/``DEV DT STOP``/``DEV DT PORTS`` verbs
(``docs/protocol-v2.md`` §"DEV DT ... — Drivetrain Control"):

- ``vw_line_for_key`` / ``vw_line_for_key_set`` are pure, Qt-free (beyond a
  bare ``PySide6.QtCore.Qt`` import to resolve key constants) and return the
  exact ``DEV DT VW ...`` strings -- no ``QApplication`` required for these.
- ``KeyboardDriver.attach()`` sends ``DEV DT PORTS <left> <right>`` exactly
  once per attach, before any drive command, regardless of how many key
  presses/releases follow -- this needs a real ``QTimer``, so these tests use
  a session ``QApplication`` (``QT_QPA_PLATFORM=offscreen``).
- The bounded STOP deadman-resend sequence sends ``DEV DT STOP``
  ``STOP_RESEND_COUNT`` times on both key release and focus-loss.

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui/test_drive.py -q

This module is not yet wired into ``pyproject.toml``'s ``testpaths`` (ticket
083-004's job, which also adds this directory's own fixtures/conftest) — run
it directly, per ticket 083-002's Testing section.
"""
from __future__ import annotations

import pytest

from robot_radio.testgui.drive import (
    DEFAULT_PORTS,
    FWD_SPEED,
    ROTATE_OMEGA,
    STOP_RESEND_COUNT,
    KeyboardDriver,
    vw_line_for_key,
    vw_line_for_key_set,
)
from robot_radio.testgui.transport import Transport

# ---------------------------------------------------------------------------
# QApplication fixture (module-scoped) -- needed only for KeyboardDriver.attach()
# (it constructs a real QTimer). The pure vw_line_for_key* tests below do NOT
# use this fixture, demonstrating they need no QApplication.
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def qapp():
    # 107-004: turn a missing `gui` dependency group into a clean skip, not
    # a hard collection/run error -- see test_tour1_geometry.py's module
    # docstring for the full rationale.
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication

    app = QApplication.instance()
    if app is None:
        app = QApplication([])
    yield app


# ---------------------------------------------------------------------------
# Fakes
# ---------------------------------------------------------------------------


class _FakeWindow:
    """Minimal stand-in for a QMainWindow: just needs assignable event slots."""

    def __init__(self) -> None:
        self.keyPressEvent = lambda event: None
        self.keyReleaseEvent = lambda event: None
        self.focusOutEvent = lambda event: None


class _FakeKeyEvent:
    """Minimal stand-in for a QKeyEvent."""

    def __init__(self, key: int, auto_repeat: bool = False) -> None:
        self._key = key
        self._auto_repeat = auto_repeat

    def isAutoRepeat(self) -> bool:
        return self._auto_repeat

    def key(self) -> int:
        return self._key


class _FakeTransport(Transport):
    """Records every line sent/commanded; no real IO."""

    def __init__(self) -> None:
        super().__init__()
        self.sent: list[str] = []

    def connect(self) -> None:
        pass

    def disconnect(self) -> None:
        pass

    def send(self, line: str) -> None:
        self.sent.append(line)

    def command(self, line: str, read_timeout: int = 200) -> str:  # [ms]
        self.sent.append(line)
        return ""


# ---------------------------------------------------------------------------
# Pure mapping helpers -- no QApplication used in this section.
# ---------------------------------------------------------------------------


def test_named_constants_match_ticket_values() -> None:
    """Pin the exact values the wire strings below are built from."""
    assert FWD_SPEED == 200
    assert ROTATE_OMEGA == 0.5
    assert DEFAULT_PORTS == (1, 2)


def test_vw_line_for_key_arrow_keys() -> None:
    from PySide6.QtCore import Qt

    assert vw_line_for_key(int(Qt.Key.Key_Up)) == "DEV DT VW 200 0 0"
    assert vw_line_for_key(int(Qt.Key.Key_Down)) == "DEV DT VW -200 0 0"
    assert vw_line_for_key(int(Qt.Key.Key_Left)) == "DEV DT VW 0 0 0.5"
    assert vw_line_for_key(int(Qt.Key.Key_Right)) == "DEV DT VW 0 0 -0.5"


def test_vw_line_for_key_non_arrow_key_returns_none() -> None:
    from PySide6.QtCore import Qt

    assert vw_line_for_key(int(Qt.Key.Key_A)) is None


def test_vw_line_for_key_set_empty_returns_none() -> None:
    assert vw_line_for_key_set(frozenset()) is None


def test_vw_line_for_key_set_single_key() -> None:
    from PySide6.QtCore import Qt

    assert (
        vw_line_for_key_set(frozenset({int(Qt.Key.Key_Up)}))
        == "DEV DT VW 200 0 0"
    )
    assert (
        vw_line_for_key_set(frozenset({int(Qt.Key.Key_Right)}))
        == "DEV DT VW 0 0 -0.5"
    )


def test_vw_line_for_key_set_combo_returns_one_valid_command() -> None:
    """Multi-key combos have undefined priority but must return SOME
    recognised arrow key's exact wire string, never a malformed one."""
    from PySide6.QtCore import Qt

    combo = frozenset({int(Qt.Key.Key_Up), int(Qt.Key.Key_Left)})
    result = vw_line_for_key_set(combo)
    assert result in ("DEV DT VW 200 0 0", "DEV DT VW 0 0 0.5")


# ---------------------------------------------------------------------------
# KeyboardDriver.attach() -- DEV DT PORTS bind exactly once per attach.
# ---------------------------------------------------------------------------


def test_attach_sends_ports_bind_exactly_once(qapp) -> None:
    from PySide6.QtCore import Qt

    driver = KeyboardDriver()
    window = _FakeWindow()
    transport = _FakeTransport()

    driver.attach(window, transport)
    try:
        assert transport.sent == ["DEV DT PORTS 1 2"]

        # Multiple presses/releases across all four arrow keys must not
        # resend the PORTS bind.
        for key in (
            Qt.Key.Key_Up,
            Qt.Key.Key_Down,
            Qt.Key.Key_Left,
            Qt.Key.Key_Right,
        ):
            driver._on_key_press(_FakeKeyEvent(int(key)))
            driver._on_key_release(_FakeKeyEvent(int(key)))
            # Drain the bounded STOP deadman sequence so the timer settles
            # before the next key in this loop.
            for _ in range(STOP_RESEND_COUNT):
                driver._on_timer_tick()

        ports_sends = [l for l in transport.sent if l.startswith("DEV DT PORTS")]
        assert ports_sends == ["DEV DT PORTS 1 2"]
    finally:
        driver.detach()


def test_attach_ports_bind_precedes_any_vw(qapp) -> None:
    driver = KeyboardDriver()
    window = _FakeWindow()
    transport = _FakeTransport()

    driver.attach(window, transport)
    try:
        driver._on_key_press(_FakeKeyEvent(_up_key()))
        assert transport.sent[0] == "DEV DT PORTS 1 2"
        assert transport.sent[1] == "DEV DT VW 200 0 0"
    finally:
        driver.detach()


def _up_key() -> int:
    from PySide6.QtCore import Qt

    return int(Qt.Key.Key_Up)


# ---------------------------------------------------------------------------
# STOP deadman resend -- key release and focus-loss.
# ---------------------------------------------------------------------------


def test_key_release_sends_dev_dt_stop_resend_count_times(qapp) -> None:
    driver = KeyboardDriver()
    window = _FakeWindow()
    transport = _FakeTransport()

    driver.attach(window, transport)
    try:
        key = _up_key()
        driver._on_key_press(_FakeKeyEvent(key))
        transport.sent.clear()  # isolate the deadman sequence

        driver._on_key_release(_FakeKeyEvent(key))  # 1st send (on release)
        for _ in range(STOP_RESEND_COUNT - 1):
            driver._on_timer_tick()  # remaining resends via the timer

        assert transport.sent == ["DEV DT STOP"] * STOP_RESEND_COUNT
        assert driver._cmd is None
        assert not driver._timer.isActive()
    finally:
        driver.detach()


def test_focus_out_sends_dev_dt_stop_resend_count_times(qapp) -> None:
    driver = KeyboardDriver()
    window = _FakeWindow()
    transport = _FakeTransport()

    driver.attach(window, transport)
    try:
        key = _up_key()
        driver._on_key_press(_FakeKeyEvent(key))
        transport.sent.clear()  # isolate the deadman sequence

        driver._on_focus_out(_FakeKeyEvent(key))  # 1st send (implicit release)
        for _ in range(STOP_RESEND_COUNT - 1):
            driver._on_timer_tick()  # remaining resends via the timer

        assert transport.sent == ["DEV DT STOP"] * STOP_RESEND_COUNT
        assert driver._cmd is None
        assert not driver._timer.isActive()
    finally:
        driver.detach()
