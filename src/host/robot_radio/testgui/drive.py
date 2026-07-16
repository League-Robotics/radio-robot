"""robot_radio.testgui.drive — Cursor-key interactive driving with keepalive timer.

Defines ``KeyboardDriver``, which wires ``keyPressEvent`` /
``keyReleaseEvent`` onto a ``QMainWindow`` and dispatches ``DEV DT VW``
commands over a connected transport.

Key mapping
-----------
- Up arrow:   ``DEV DT VW <FWD_SPEED> 0 0``           (forward)
- Down arrow: ``DEV DT VW -<FWD_SPEED> 0 0``          (back)
- Left arrow: ``DEV DT VW 0 0 <ROTATE_OMEGA>``        (rotate CCW)
- Right arrow: ``DEV DT VW 0 0 -<ROTATE_OMEGA>``      (rotate CW)
- Release:    bounded ``DEV DT STOP`` deadman resend (see below)

``attach()`` additionally sends ``DEV DT PORTS <left> <right>`` exactly once
per drive session — before any ``DEV DT VW`` — using the module-level
``DEFAULT_PORTS`` pair, so the drivetrain is bound to the ports this driver
assumes it commands (083-002).

A ~100 ms ``QTimer`` resends the current command while any arrow key is
held, doubling as a firmware watchdog keepalive.  Qt auto-repeat is
suppressed: held keys emit one logical press (the timer handles re-sends).

Guard / STOP deadman resend
----------------------------
If the transport reply to a ``DEV DT VW`` command contains ``vw busy``, the
warning is logged.  ``DEV DT STOP`` delivery is not a single fire-and-forget
send: a direct-USB link intermittently drops 15-50% of lines, so a dropped
``DEV DT STOP`` would otherwise leave the robot coasting at the last
commanded velocity indefinitely.  Instead, on key release ``KeyboardDriver``
sets ``self._cmd = _STOP_CMD`` (``"DEV DT STOP"``) and reuses the *existing*
non-blocking timer/``_send_cmd`` machinery to resend it ``STOP_RESEND_COUNT``
times (counting the immediate send-on-release) before actually stopping the
timer — no new thread, no blocking ``command()`` retry loop on the Qt main
thread (a blocking acked-retry design was considered and rejected; see
``architecture-update.md`` Design Rationale Decision 4 in sprint 065). With a
15-50% per-line drop rate, the odds every resend is lost are bounded by
``(0.5) ** STOP_RESEND_COUNT`` — vanishingly small at the default count.

Separately, Qt never delivers ``keyReleaseEvent`` if the window loses focus
while an arrow key is physically held down.  ``KeyboardDriver`` also
overrides ``focusOutEvent`` and treats focus loss as an implicit release:
if a key is currently tracked as held, losing focus triggers the same
bounded STOP deadman-resend sequence a real key-release would.

Keepalive arm/disarm (sprint 065, ticket 005)
----------------------------------------------
``SerialConnection.connect()`` no longer arms the ambient ``+`` keepalive
daemon automatically — an ambient keepalive that keeps streaming for the
entire lifetime of an open port, independent of whether anything is
driving, silently defeats the firmware motion watchdog for any hung host
process.  ``KeyboardDriver`` is the layer that owns open-ended (``VW``)
motion sessions, so it calls ``self._transport.arm_keepalive()`` on the
first key press of a drive session (guarded so a held key / direction
change never re-arms an already-armed session) and
``self._transport.disarm_keepalive()`` once the bounded STOP deadman
sequence above completes, or on ``detach()`` as a safety net.  Both calls
are no-ops on ``SimTransport`` (inherited ``Transport`` default).

Units
-----
``v``     — linear velocity in mm/s.
``omega`` — angular velocity in rad/s; positive = CCW.

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

#: Forward / back speed.
FWD_SPEED: int = 200  # [mm/s]

#: Rotate speed; positive = CCW.
ROTATE_OMEGA: float = 0.5  # [rad/s]

#: Default drivetrain port pair (left, right) bound once per drive session
#: via ``DEV DT PORTS`` — matches the firmware boot default and the sim's
#: default plant binding (see src/sim/firmware.py's ``vel()``
#: docstring: "port 1=LEFT, port 2=RIGHT").
DEFAULT_PORTS: tuple[int, int] = (1, 2)

#: Wire command sent by the bounded STOP deadman-resend sequence (see
#: STOP_RESEND_COUNT below).  Also doubles as ``self._cmd``'s sentinel value
#: during that resend window (compared via ``_cmd == _STOP_CMD`` /
#: ``_cmd != _STOP_CMD``).
_STOP_CMD: str = "DEV DT STOP"

#: Number of times ``_STOP_CMD`` is (re)sent on key release / focus-loss,
#: counting the initial send-on-release itself.  Reuses the existing
#: ``QTimer``/``_send_cmd`` resend machinery (no new thread, no blocking
#: ``command()`` retry — see architecture-update.md Design Rationale
#: Decision 4, sprint 065).  At the default 100 ms timer interval this spans
#: ~400-500 ms of resends; with a 15-50% per-line drop rate the probability
#: every send is lost is bounded by ``(0.5) ** STOP_RESEND_COUNT``.
STOP_RESEND_COUNT: int = 5

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
            int(Qt.Key.Key_Up):    f"DEV DT VW {FWD_SPEED} 0 0",
            int(Qt.Key.Key_Down):  f"DEV DT VW -{FWD_SPEED} 0 0",
            int(Qt.Key.Key_Left):  f"DEV DT VW 0 0 {ROTATE_OMEGA}",
            int(Qt.Key.Key_Right): f"DEV DT VW 0 0 -{ROTATE_OMEGA}",
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
    interval:
        QTimer interval in milliseconds.  Defaults to 100.
    """

    def __init__(self, interval: int = 100) -> None:  # [ms]
        self._interval = interval
        self._transport: "Transport | None" = None
        self._window: "object | None" = None
        self._timer: "object | None" = None   # QTimer, created lazily
        self._cmd: str | None = None          # current VW command, or _STOP_CMD during the deadman resend window
        self._stop_resends_left: int = 0      # remaining bounded STOP resends (see STOP_RESEND_COUNT)
        self._keepalive_armed: bool = False   # tracks whether transport.arm_keepalive() is currently in effect
        # Currently-held arrow keys (CR-15 item 8). Releasing one key while
        # another is still held falls back to driving the remaining key
        # instead of starting the STOP deadman sequence -- see
        # _on_key_release / vw_line_for_key_set.
        self._held_keys: set[int] = set()

        # Original key event handlers (restored on detach).
        self._orig_key_press: "object | None" = None
        self._orig_key_release: "object | None" = None
        self._orig_focus_out: "object | None" = None

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

        # DEV DT PORTS bind DROPPED (2026-07-16, out-of-process): `DEV *` has no
        # binary arm on the current wire (real robot OR sim -- see
        # binary_bridge.py / RobotLoop dispatch), so this only produced "not
        # supported" noise on every connect. It was also redundant: DEFAULT_PORTS
        # (1=LEFT, 2=RIGHT) already matches the firmware boot default and the
        # sim's default plant binding, so the drivetrain is bound to the right
        # ports without it. Restore this bind if/when a DEV (or config) arm for
        # runtime drivetrain-port assignment is added to the protocol.

        # Create the keepalive timer (lazy PySide6 import).
        from PySide6.QtCore import QTimer  # type: ignore[import-untyped]

        self._timer = QTimer()
        self._timer.setInterval(self._interval)
        self._timer.timeout.connect(self._on_timer_tick)

        # Save and override key-event handlers on the window.
        self._orig_key_press = window.keyPressEvent  # type: ignore[union-attr]
        self._orig_key_release = window.keyReleaseEvent  # type: ignore[union-attr]
        self._orig_focus_out = window.focusOutEvent  # type: ignore[union-attr]

        window.keyPressEvent = self._on_key_press  # type: ignore[union-attr]
        window.keyReleaseEvent = self._on_key_release  # type: ignore[union-attr]
        window.focusOutEvent = self._on_focus_out  # type: ignore[union-attr]

        _log.debug("KeyboardDriver attached")

    def detach(self) -> None:
        """Remove key event overrides and stop the timer.

        Safe to call when not attached.  Disarms the keepalive first (a
        safety net for the case where detach() -- e.g. on disconnect() --
        happens mid-drive or mid-deadman, before the bounded STOP sequence
        would otherwise have disarmed it itself).
        """
        self._stop_timer()
        self._cmd = None
        self._stop_resends_left = 0
        self._held_keys.clear()
        self._disarm_keepalive_if_armed()
        self._transport = None

        if self._window is not None:
            if self._orig_key_press is not None:
                self._window.keyPressEvent = self._orig_key_press  # type: ignore[union-attr]
            if self._orig_key_release is not None:
                self._window.keyReleaseEvent = self._orig_key_release  # type: ignore[union-attr]
            if self._orig_focus_out is not None:
                self._window.focusOutEvent = self._orig_focus_out  # type: ignore[union-attr]
        self._window = None
        self._orig_key_press = None
        self._orig_key_release = None
        self._orig_focus_out = None

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

        # A drive session is starting (or continuing) -- arm the ambient
        # host keepalive.  Only the first press of a session actually arms
        # it (self._keepalive_armed guards re-arming on every subsequent
        # press/direction-change while already driving); disarm happens once
        # the bounded STOP deadman sequence completes (see _send_cmd) or on
        # detach().
        self._arm_keepalive_if_needed()

        # Track this key as held (CR-15 item 8) so a later release of a
        # DIFFERENT key can fall back to whichever key(s) remain held.
        self._held_keys.add(key)

        # Switch to the new command and (re)start the timer.
        self._cmd = cmd
        self._send_cmd()
        self._start_timer()

    def _on_key_release(self, event: "object") -> None:
        """Handle a key-release event on the main window.

        If another arrow key is still held after removing the released key
        from ``self._held_keys``, driving continues with the remaining
        key's command (CR-15 item 8) -- releasing one arrow while another is
        held must not stop the robot.  Only when the LAST held key is
        released does this begin the bounded STOP deadman-resend sequence
        (see :meth:`_start_stop_deadman`) instead of stopping the timer and
        sending a single fire-and-forget ``STOP``.  Forwarded to the
        original handler for non-arrow keys.
        """
        if event.isAutoRepeat():  # type: ignore[union-attr]
            return

        key = int(event.key())  # type: ignore[union-attr]

        if key not in _qt_arrow_keys():
            if self._orig_key_release is not None:
                self._orig_key_release(event)
            return

        self._held_keys.discard(key)

        fallback_cmd = vw_line_for_key_set(frozenset(self._held_keys))
        if fallback_cmd is not None:
            # Another arrow key is still held -- keep driving with its
            # command instead of starting the STOP deadman sequence.
            self._cmd = fallback_cmd
            self._send_cmd()
            self._start_timer()
            return

        self._start_stop_deadman()

    def _on_focus_out(self, event: "object") -> None:
        """Handle the main window losing keyboard focus.

        Qt does not deliver ``keyReleaseEvent`` when the window loses focus
        while an arrow key is physically held down -- a real release would
        leave the robot driving indefinitely.  Focus loss is treated as an
        implicit release of EVERY currently-held key (``self._held_keys`` is
        cleared): if a key is currently tracked as held (``self._cmd`` is a
        VW line, not ``None``/``_STOP_CMD``), this triggers the same bounded
        ``DEV DT STOP`` deadman-resend sequence a real key-release would.
        Forwarded to the original ``focusOutEvent`` handler afterward.
        """
        if self._cmd is not None and self._cmd != _STOP_CMD:
            self._held_keys.clear()
            self._start_stop_deadman()

        if self._orig_focus_out is not None:
            self._orig_focus_out(event)

    # ------------------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------------------

    def _on_timer_tick(self) -> None:
        """Resend the current command.

        While a key is held, ``self._cmd`` is the VW line and this is an
        unbounded keepalive resend, exactly as before.  During the bounded
        STOP deadman window after a release or focus-loss, ``self._cmd`` is
        ``_STOP_CMD`` (``"DEV DT STOP"``) and this same unconditional resend
        serves as the next deadman tick -- :meth:`_send_cmd` (not this
        method) tracks the remaining count and stops the timer once the
        sequence completes, so no special-casing is needed here.
        """
        self._send_cmd()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _start_stop_deadman(self) -> None:
        """Begin (or restart) the bounded ``DEV DT STOP`` deadman-resend sequence.

        Sets ``self._cmd = _STOP_CMD`` and sends it immediately -- the first
        of ``STOP_RESEND_COUNT`` total sends -- then leaves the existing
        keepalive timer running so :meth:`_on_timer_tick` keeps resending
        ``DEV DT STOP`` for the remaining ``STOP_RESEND_COUNT - 1`` ticks
        (:meth:`_send_cmd` counts down and stops the timer once exhausted).
        Called from both a real key-release and a focus-loss event, since
        both represent the same "key is no longer held" transition. No new
        thread, no blocking ``command()`` retry (rejected alternative -- see
        architecture-update.md Design Rationale Decision 4, sprint 065).

        If no transport is attached there is nothing to protect; the timer
        is simply stopped and ``self._cmd`` cleared, mirroring the driver's
        "ignores key events when no transport" invariant.
        """
        if self._transport is None:
            self._stop_timer()
            self._cmd = None
            return

        self._cmd = _STOP_CMD
        self._stop_resends_left = STOP_RESEND_COUNT
        self._start_timer()
        self._send_cmd()

    def _send_cmd(self) -> None:
        """Send ``self._cmd`` via the transport if one is set.

        When ``self._cmd == _STOP_CMD`` (the bounded deadman-resend window),
        this additionally counts the send against ``STOP_RESEND_COUNT`` and,
        once exhausted, stops the keepalive timer and clears ``self._cmd``
        -- this is the single place that ends the deadman sequence, whether
        called from :meth:`_start_stop_deadman` (the first send) or
        :meth:`_on_timer_tick` (each subsequent resend). The countdown
        advances even if ``transport.send`` raises, so a dropped/failed send
        does not prevent the timer from stopping on schedule.
        """
        if self._transport is None or self._cmd is None:
            return
        try:
            self._transport.send(self._cmd)
        except Exception as exc:
            _log.warning("KeyboardDriver: failed to send %r: %s", self._cmd, exc)

        if self._cmd == _STOP_CMD:
            self._stop_resends_left -= 1
            if self._stop_resends_left <= 0:
                self._stop_timer()
                self._cmd = None
                # The bounded STOP deadman sequence has completed -- the
                # drive session is over, so disarm the ambient keepalive
                # (see architecture-update.md Step 4-5 item 5, sprint 065).
                self._disarm_keepalive_if_armed()

    def _start_timer(self) -> None:
        """Start the keepalive QTimer if not already running."""
        if self._timer is not None and not self._timer.isActive():  # type: ignore[union-attr]
            self._timer.start()

    def _stop_timer(self) -> None:
        """Stop the keepalive QTimer."""
        if self._timer is not None and self._timer.isActive():  # type: ignore[union-attr]
            self._timer.stop()

    # ------------------------------------------------------------------
    # Keepalive arm/disarm (sprint 065, ticket 005)
    # ------------------------------------------------------------------
    #
    # SerialConnection.connect() no longer arms the ambient "+" keepalive
    # daemon automatically -- an ambient keepalive that outlives whatever is
    # actually driving silently defeats the firmware motion watchdog for any
    # hung host process.  KeyboardDriver is the layer that owns open-ended
    # (VW) motion sessions, so it arms the keepalive when a session starts
    # and disarms it once the session is unambiguously over (bounded STOP
    # deadman sequence completed, or detach()).  Both helpers are guarded by
    # ``self._keepalive_armed`` so arm/disarm calls are only ever made once
    # per session, matching the "first key press while not already armed" /
    # "once the deadman sequence completes" bracketing in the ticket's
    # acceptance criteria.

    def _arm_keepalive_if_needed(self) -> None:
        """Arm the transport's keepalive if a drive session isn't already armed."""
        if self._keepalive_armed or self._transport is None:
            return
        try:
            self._transport.arm_keepalive()
        except Exception as exc:
            _log.warning("KeyboardDriver: failed to arm keepalive: %s", exc)
        self._keepalive_armed = True

    def _disarm_keepalive_if_armed(self) -> None:
        """Disarm the transport's keepalive if this driver armed it."""
        if not self._keepalive_armed:
            return
        if self._transport is not None:
            try:
                self._transport.disarm_keepalive()
            except Exception as exc:
                _log.warning("KeyboardDriver: failed to disarm keepalive: %s", exc)
        self._keepalive_armed = False
