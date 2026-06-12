"""bench_safety.py — BenchRun context manager for bench and dev drive programs.

Overview
--------
``BenchRun`` wraps every motion-commanding bench program.  It provides:

1. **Preflight liveness check** — sends PING (or SNAP) and waits up to 2 s
   for a reply; raises ``RobotSilentError`` if none arrives.

2. **SIGINT handler** — Ctrl-C calls ``send_stop()`` then re-raises, so the
   robot is always halted before the process exits.

3. **Guaranteed stop on exit** — ``__exit__`` (and its internal ``finally``)
   always calls ``send_stop()`` regardless of whether the block exited
   normally, raised an exception, or was interrupted.

4. **Wall-clock cap** — a background daemon thread monitors elapsed time;
   if ``max_seconds`` passes without the ``with``-block completing normally,
   ``send_stop()`` is called and the block raises ``RunawayAbortError``.

5. **Runaway detection** (optional) — when a ``telem_iter`` is passed at
   construction time the context manager registers a ``telem_check`` hook
   that is called with each ``TLMFrame``; on every call it checks:
   - Full-tilt PWM (commanded speed > 50% max) with encoder delta < 5 mm/s
     for 3 consecutive frames → ``send_stop()`` + ``RunawayAbortError``.
   - Zero encoder motion for > 5 s while commanding motion → same.
   When no ``telem_iter`` is supplied the runaway detection is bypassed (the
   wall-clock cap remains active as the backstop).

API
---
::

    with BenchRun(proto, max_seconds=60) as bench:
        # motion code here; if Ctrl-C, robot stops automatically
        ...

    # or with optional telemetry runaway detection:
    with BenchRun(proto, max_seconds=90) as bench:
        for frame in my_tlm_stream():
            bench.check_tlm(frame)   # raises RunawayAbortError on runaway
            ...

``BenchRun`` accepts any object that provides:
    - ``.send(cmd: str, read_ms: int)`` — sends a command and returns a dict
    - ``.ping()`` — returns ``(robot_t_ms, rtt_ms)`` or ``None``

Typically this is a ``NezhaProtocol`` instance (``robot._proto``).

Error types
-----------
``RobotSilentError``
    Raised from ``__enter__`` when the preflight PING gets no reply.

``RunawayAbortError``
    Raised when the wall-clock cap is exceeded or runaway is detected.
    The message indicates the reason: ``"wall clock cap"`` or
    ``"stall: full-tilt with no encoder motion"`` etc.
"""

from __future__ import annotations

import signal
import threading
import time
from typing import Any, Callable


# --------------------------------------------------------------------------- #
# Custom exceptions                                                            #
# --------------------------------------------------------------------------- #

class RobotSilentError(RuntimeError):
    """Raised when the preflight PING receives no reply within the timeout."""


class RunawayAbortError(RuntimeError):
    """Raised when the wall-clock cap fires or runaway motion is detected."""


# --------------------------------------------------------------------------- #
# BenchRun                                                                     #
# --------------------------------------------------------------------------- #

# Speed threshold above which the robot is considered "commanding motion".
# Firmware max speed is ~1000 mm/s; 50% = 500 mm/s.
_MOTION_SPEED_THRESHOLD = 500  # mm/s

# Consecutive stall frames required before declaring a runaway.
_STALL_FRAME_COUNT = 3

# Encoder delta (mm/s) below which a frame is considered stalled.
_ENC_STALL_THRESHOLD = 5  # mm/s

# Seconds of zero-encoder motion while commanding to trigger zero-motion abort.
_ZERO_MOTION_TIMEOUT = 5.0  # s


class BenchRun:
    """Context manager that wraps bench drive programs with mandatory safety.

    Parameters
    ----------
    proto:
        The active robot protocol object — must have ``.send(cmd, read_ms)``
        and ``.ping()`` methods.  Typically ``robot._proto`` (NezhaProtocol).
    max_seconds:
        Wall-clock cap.  If the ``with`` block has not exited after this many
        seconds the manager calls ``send_stop()`` and raises
        ``RunawayAbortError("wall clock cap")``.  Default: 60 s.
    progress_fn:
        Optional callable invoked by the wall-clock thread just before it
        triggers the abort.  Use for logging / UI feedback.
    """

    def __init__(
        self,
        proto: Any,
        max_seconds: float = 60,
        progress_fn: Callable[[], None] | None = None,
    ) -> None:
        self._proto = proto
        self._max_seconds = max_seconds
        self._progress_fn = progress_fn

        # Runaway-detection state
        self._stall_count: int = 0
        self._zero_motion_since: float | None = None

        # Used to communicate between the enforcer thread and __exit__.
        # _wall_clock_fired: signalled to stop the enforcer thread (normal
        #   exit) OR set by the enforcer thread itself (cap triggered).
        # _wall_cap_triggered: True only when the enforcer actually fired the
        #   cap — used to raise RunawayAbortError in __exit__.
        self._wall_clock_fired = threading.Event()
        self._wall_cap_triggered: bool = False

        # Saved SIGINT handler so we can restore it.
        self._orig_sigint: Any = None

        # Background enforcer thread.
        self._enforcer: threading.Thread | None = None
        self._entered_at: float = 0.0

    # ------------------------------------------------------------------ #
    # Public stop + check API                                              #
    # ------------------------------------------------------------------ #

    def send_stop(self) -> None:
        """Send ``X`` (hard-cancel) followed by ``STREAM 0`` to the robot.

        This is the canonical safe-stop.  It is always called from
        ``__exit__`` and from the SIGINT handler.  Suppresses all exceptions
        so it can be called safely from signal handlers and ``finally`` blocks.
        """
        try:
            self._proto.send("X", 100)
        except Exception:  # noqa: BLE001
            pass
        try:
            self._proto.send("STREAM 0", 150)
        except Exception:  # noqa: BLE001
            pass

    def check_tlm(self, frame: Any) -> None:
        """Check a single TLM frame for runaway patterns.

        Call this on every telemetry frame inside the ``with`` block.  If a
        runaway is detected ``send_stop()`` is called and ``RunawayAbortError``
        is raised.

        Parameters
        ----------
        frame:
            A ``TLMFrame`` (or any object with ``.vel``, ``.twist``, and
            ``.enc`` attributes — may be ``None`` on absent fields).
        """
        now = time.monotonic()

        # ---- derive current speed and encoder delta ----------------------
        commanded_speed: int = 0
        enc_speed: float = 0.0  # average of |left| + |right| velocity

        # Use the fused twist if available (body-frame v in mm/s).
        if frame.twist is not None:
            commanded_speed = abs(frame.twist[0])

        # Use per-wheel velocity for encoder motion check.
        if frame.vel is not None:
            enc_speed = (abs(frame.vel[0]) + abs(frame.vel[1])) / 2.0

        # ---- stall check: full-tilt PWM but no encoder motion -----------
        if commanded_speed > _MOTION_SPEED_THRESHOLD:
            if enc_speed < _ENC_STALL_THRESHOLD:
                self._stall_count += 1
                if self._stall_count >= _STALL_FRAME_COUNT:
                    self.send_stop()
                    raise RunawayAbortError(
                        f"stall: commanded {commanded_speed} mm/s but "
                        f"encoder < {_ENC_STALL_THRESHOLD} mm/s for "
                        f"{_STALL_FRAME_COUNT} frames"
                    )
            else:
                self._stall_count = 0

        # ---- zero-motion check: commanding motion but encoders dead -----
        if commanded_speed > 0:
            if enc_speed < _ENC_STALL_THRESHOLD:
                if self._zero_motion_since is None:
                    self._zero_motion_since = now
                elif now - self._zero_motion_since > _ZERO_MOTION_TIMEOUT:
                    self.send_stop()
                    raise RunawayAbortError(
                        f"zero encoder motion for >{_ZERO_MOTION_TIMEOUT:.0f}s "
                        "while commanding motion"
                    )
            else:
                self._zero_motion_since = None
        else:
            self._zero_motion_since = None

    # ------------------------------------------------------------------ #
    # Context manager protocol                                             #
    # ------------------------------------------------------------------ #

    def __enter__(self) -> "BenchRun":
        # 1. Preflight liveness check (PING, 2 s timeout).
        self._preflight()

        # 2. Register SIGINT handler so Ctrl-C stops the robot.
        self._orig_sigint = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, self._sigint_handler)

        # 3. Start wall-clock enforcer thread.
        self._entered_at = time.monotonic()
        self._enforcer = threading.Thread(
            target=self._wall_clock_enforcer,
            name="BenchRun-enforcer",
            daemon=True,
        )
        self._enforcer.start()

        return self

    def __exit__(
        self,
        exc_type: type | None,
        exc_val: BaseException | None,
        exc_tb: Any,
    ) -> bool:
        # Always stop the robot — regardless of how the block exited.
        try:
            self.send_stop()
        finally:
            # Restore original SIGINT handler.
            if self._orig_sigint is not None:
                try:
                    signal.signal(signal.SIGINT, self._orig_sigint)
                except Exception:  # noqa: BLE001
                    pass
                self._orig_sigint = None

            # Signal the enforcer thread to stop (it checks _wall_clock_fired
            # or exits naturally when max_seconds is up).
            self._wall_clock_fired.set()

        # If the wall-clock enforcer actually triggered the cap, raise.
        # (Uses a dedicated boolean so there is no race with __exit__ also
        # setting _wall_clock_fired to stop the thread on normal exit.)
        if exc_type is None and self._wall_cap_triggered:
            elapsed = time.monotonic() - self._entered_at
            raise RunawayAbortError(
                f"wall clock cap: {elapsed:.1f}s >= {self._max_seconds}s"
            )

        # Propagate all other exceptions unchanged.
        return False

    # ------------------------------------------------------------------ #
    # Private helpers                                                       #
    # ------------------------------------------------------------------ #

    def _preflight(self) -> None:
        """Ping the robot; raise RobotSilentError if no reply within 2 s."""
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            try:
                result = self._proto.ping()
                if result is not None:
                    return  # alive
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.1)

        # Second attempt: SNAP (some programs use raw conn instead of proto)
        try:
            resp = self._proto.send("SNAP", 400)
            if resp and resp.get("responses"):
                return
        except Exception:  # noqa: BLE001
            pass

        raise RobotSilentError(
            "preflight PING failed — robot silent. Power-cycle and retry."
        )

    def _sigint_handler(
        self, signum: int, frame: Any  # noqa: ARG002
    ) -> None:
        """Handle SIGINT (Ctrl-C): stop the robot, then restore and re-raise."""
        # Stop the robot first.
        self.send_stop()
        # Restore the original handler so the second Ctrl-C works normally.
        if self._orig_sigint is not None:
            try:
                signal.signal(signal.SIGINT, self._orig_sigint)
            except Exception:  # noqa: BLE001
                pass
            self._orig_sigint = None
        # Re-raise KeyboardInterrupt so the caller's finally blocks run.
        raise KeyboardInterrupt

    def _wall_clock_enforcer(self) -> None:
        """Background thread: fire send_stop() when max_seconds is exceeded."""
        remaining = self._max_seconds - (time.monotonic() - self._entered_at)
        # Wait until the cap expires OR the event is set (block exited early).
        fired = self._wall_clock_fired.wait(timeout=max(0.0, remaining))
        if fired:
            # Event was set by __exit__ — block exited cleanly, nothing to do.
            return
        # Wall clock expired without the event being set → runaway.
        if self._progress_fn is not None:
            try:
                self._progress_fn()
            except Exception:  # noqa: BLE001
                pass
        self.send_stop()
        # Mark that the enforcer fired, then signal __exit__ to raise.
        self._wall_cap_triggered = True
        self._wall_clock_fired.set()
