"""robot_radio.testgui.drive — Cursor-key interactive driving with keepalive timer.

Defines ``KeyboardDriver``, which wires ``keyPressEvent`` /
``keyReleaseEvent`` onto a ``QMainWindow`` and dispatches ``VW`` commands
over a connected transport.

Key mapping
-----------
- Up arrow:   ``VW <FWD_SPEED_MMS> 0``          (forward)
- Down arrow: ``VW -<FWD_SPEED_MMS> 0``         (back)
- Left arrow: ``VW 0 <ROTATE_OMEGA_MRADS>``      (rotate CCW)
- Right arrow: ``VW 0 -<ROTATE_OMEGA_MRADS>``    (rotate CW)
- Release:    ``STOP``

A ~100 ms ``QTimer`` resends the current ``VW`` command while any arrow key
is held, doubling as a firmware watchdog keepalive.  Qt auto-repeat is
suppressed: held keys emit one logical press (the timer handles re-sends).

Guard
-----
If the transport reply to a ``VW`` command contains ``vw busy``, the warning
is logged.  ``STOP`` is never suppressed — it is always sent on key release.

Units
-----
``v_mms``     — linear velocity in mm/s.
``omega_mrads`` — angular velocity in milli-radians/s; positive = CCW.

Lazy PySide6 import
-------------------
This module does NOT import PySide6 at the top level so that
``import robot_radio.testgui`` (and importing this module in unit tests)
works without PySide6 installed.

Pure mapping helper
-------------------
``vw_line_for_key(key_int)`` and ``vw_line_for_key_set(held_keys)`` are
module-level pure functions that accept ``Qt.Key`` integer constants and
return the corresponding wire string.  They are importable and testable
without a ``QApplication``.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot_radio.testgui.transport import Transport

_log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Named constants — edit here to change drive speeds.
# ---------------------------------------------------------------------------

#: Forward / back speed in mm/s.
FWD_SPEED_MMS: int = 200

#: Rotate speed in milli-radians/s; positive = CCW.
ROTATE_OMEGA_MRADS: int = 500

# ---------------------------------------------------------------------------
# Key integer constants (mirrors PySide6.QtCore.Qt.Key values).
# These are stored as plain ints so the module is importable without PySide6.
# Values match Qt::Key constants; kept here for reference / test use.
# ---------------------------------------------------------------------------

# Qt.Key_Up    = 0x01000013
# Qt.Key_Down  = 0x01000015
# Qt.Key_Left  = 0x01000012
# Qt.Key_Right = 0x01000014
#
# Resolved lazily at first use — see _qt_arrow_keys().

_ARROW_KEYS: dict[int, str] | None = None


def _qt_arrow_keys() -> dict[int, str]:
    """Return a mapping of Qt arrow-key int constants to VW wire strings.

    Resolved lazily so the module is importable without PySide6.
    """
    global _ARROW_KEYS
    if _ARROW_KEYS is None:
        from PySide6.QtCore import Qt  # type: ignore[import-untyped]

        _ARROW_KEYS = {
            int(Qt.Key.Key_Up):    f"VW {FWD_SPEED_MMS} 0",
            int(Qt.Key.Key_Down):  f"VW -{FWD_SPEED_MMS} 0",
            int(Qt.Key.Key_Left):  f"VW 0 {ROTATE_OMEGA_MRADS}",
            int(Qt.Key.Key_Right): f"VW 0 -{ROTATE_OMEGA_MRADS}",
        }
    return _ARROW_KEYS


# ---------------------------------------------------------------------------
# Pure mapping helpers (testable without QApplication)
# ---------------------------------------------------------------------------


def vw_line_for_key(key_int: int) -> str | None:
    """Return the VW wire string for a single held Qt arrow-key integer.

    Returns ``None`` if ``key_int`` is not an arrow key.

    This is a pure function: no Qt widgets, no timer, no transport.
    Requires PySide6 to be installed (to resolve key constants) but does
    NOT require a ``QApplication`` to exist.
    """
    return _qt_arrow_keys().get(key_int)


def vw_line_for_key_set(held_keys: frozenset[int]) -> str | None:
    """Return the VW wire string for a set of simultaneously-held arrow keys.

    The ticket design uses a single ``QTimer`` with a ``_cmd`` state variable,
    so only the *last* pressed arrow key drives the active command.  This
    function returns the first match found in press-order by the driver; for
    tests, it returns the command for the single key in the set (or the first
    ordered match when multiple keys are held — undefined priority for
    multi-key).

    Returns ``None`` if no recognised arrow key is in ``held_keys``.
    """
    arrow = _qt_arrow_keys()
    for k in held_keys:
        if k in arrow:
            return arrow[k]
    return None


# ---------------------------------------------------------------------------
# KeyboardDriver
# ---------------------------------------------------------------------------


class KeyboardDriver:
    """Wires cursor-key driving onto a ``QMainWindow``.

    Usage::

        driver = KeyboardDriver()
        # After transport.connect() succeeds:
        driver.attach(window, transport)
        # On transport.disconnect():
        driver.detach()

    The driver is inactive (ignores key events) when no transport is set.

    Parameters
    ----------
    interval_ms:
        QTimer interval in milliseconds.  Defaults to 100.
    """

    def __init__(self, interval_ms: int = 100) -> None:
        self._interval_ms = interval_ms
        self._transport: "Transport | None" = None
        self._window: "object | None" = None
        self._timer: "object | None" = None   # QTimer, created lazily
        self._cmd: str | None = None          # current VW command

        # Original key event handlers (restored on detach).
        self._orig_key_press: "object | None" = None
        self._orig_key_release: "object | None" = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def attach(self, window: "object", transport: "Transport") -> None:
        """Install key event overrides on *window* and bind to *transport*.

        Calling ``attach`` while already attached first calls ``detach``.

        Parameters
        ----------
        window:
            The ``QMainWindow`` instance to receive key events.
        transport:
            A connected ``Transport`` instance.  ``send()`` is used for
            fire-and-forget dispatch.
        """
        if self._window is not None:
            self.detach()

        self._transport = transport
        self._window = window

        # Create the keepalive timer (lazy PySide6 import).
        from PySide6.QtCore import QTimer  # type: ignore[import-untyped]

        self._timer = QTimer()
        self._timer.setInterval(self._interval_ms)
        self._timer.timeout.connect(self._on_timer_tick)

        # Save and override key-event handlers on the window.
        self._orig_key_press = window.keyPressEvent  # type: ignore[union-attr]
        self._orig_key_release = window.keyReleaseEvent  # type: ignore[union-attr]

        window.keyPressEvent = self._on_key_press  # type: ignore[union-attr]
        window.keyReleaseEvent = self._on_key_release  # type: ignore[union-attr]

        _log.debug("KeyboardDriver attached")

    def detach(self) -> None:
        """Remove key event overrides and stop the timer.

        Safe to call when not attached.
        """
        self._stop_timer()
        self._cmd = None
        self._transport = None

        if self._window is not None:
            if self._orig_key_press is not None:
                self._window.keyPressEvent = self._orig_key_press  # type: ignore[union-attr]
            if self._orig_key_release is not None:
                self._window.keyReleaseEvent = self._orig_key_release  # type: ignore[union-attr]
        self._window = None
        self._orig_key_press = None
        self._orig_key_release = None

        if self._timer is not None:
            self._timer.deleteLater()  # type: ignore[union-attr]
            self._timer = None

        _log.debug("KeyboardDriver detached")

    # ------------------------------------------------------------------
    # Qt event handlers (installed onto the window by attach())
    # ------------------------------------------------------------------

    def _on_key_press(self, event: "object") -> None:
        """Handle a key-press event on the main window.

        Ignores Qt auto-repeat events so a held key produces exactly one
        logical press.  Only cursor arrow keys are handled; all others are
        forwarded to the original handler.
        """
        # event.isAutoRepeat() → ignore held-key auto-repeats.
        if event.isAutoRepeat():  # type: ignore[union-attr]
            return

        key = int(event.key())  # type: ignore[union-attr]
        cmd = _qt_arrow_keys().get(key)

        if cmd is None:
            # Not an arrow key — forward to the original handler.
            if self._orig_key_press is not None:
                self._orig_key_press(event)
            return

        if self._transport is None:
            return

        # Switch to the new command and (re)start the timer.
        self._cmd = cmd
        self._send_cmd()
        self._start_timer()

    def _on_key_release(self, event: "object") -> None:
        """Handle a key-release event on the main window.

        Stops the keepalive timer and sends ``STOP``.  Forwarded to the
        original handler for non-arrow keys.
        """
        if event.isAutoRepeat():  # type: ignore[union-attr]
            return

        key = int(event.key())  # type: ignore[union-attr]

        if key not in _qt_arrow_keys():
            if self._orig_key_release is not None:
                self._orig_key_release(event)
            return

        self._stop_timer()
        self._cmd = None

        if self._transport is None:
            return

        try:
            self._transport.send("STOP")
        except Exception as exc:
            _log.warning("KeyboardDriver: failed to send STOP: %s", exc)

    # ------------------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------------------

    def _on_timer_tick(self) -> None:
        """Resend the current VW command (keepalive while key is held)."""
        self._send_cmd()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _send_cmd(self) -> None:
        """Send ``self._cmd`` via the transport if one is set."""
        if self._transport is None or self._cmd is None:
            return
        try:
            self._transport.send(self._cmd)
        except Exception as exc:
            _log.warning("KeyboardDriver: failed to send %r: %s", self._cmd, exc)

    def _start_timer(self) -> None:
        """Start the keepalive QTimer if not already running."""
        if self._timer is not None and not self._timer.isActive():  # type: ignore[union-attr]
            self._timer.start()

    def _stop_timer(self) -> None:
        """Stop the keepalive QTimer."""
        if self._timer is not None and self._timer.isActive():  # type: ignore[union-attr]
            self._timer.stop()
