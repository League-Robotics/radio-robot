"""robot_radio.testkit.safety — SafeRun context manager.

Generalization of ``tests/bench/bench_safety.py::BenchRun`` for all
targets (sim, bench, production).

SafeRun provides:

1. **Preflight liveness check** — sends PING and waits up to 2 s for a
   reply.  For sim targets the preflight is a no-op (the sim is always
   "live" after connect()).

2. **SIGINT handler** — Ctrl-C calls ``robot.estop()`` then re-raises so
   the robot is always halted before the process exits.

3. **Guaranteed stop on exit** — ``__exit__`` always calls ``robot.estop()``
   regardless of whether the block exited normally, raised, or was
   interrupted.

4. **Wall-clock cap** — a background daemon thread monitors elapsed time;
   if ``max_seconds`` passes without the ``with``-block completing normally,
   ``robot.estop()`` is called and the block raises ``RunawayAbortError``.

Usage::

    from robot_radio.testkit import make_target, SafeRun

    tr = make_target("bench", port="/dev/cu.usbmodem...")
    with SafeRun(tr, max_seconds=30) as sr:
        # motion code here
        ...

    # Also accepts a bare Nezha:
    with SafeRun(robot, max_seconds=30) as sr:
        ...

``BenchRun`` (the original class from bench_safety.py) is re-exported
from this module as a convenience alias.
"""

from __future__ import annotations

import logging
import signal
import threading
import time
from typing import TYPE_CHECKING, Any

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from robot_radio.testkit.target import TestRobot
    from robot_radio.robot.nezha import Nezha


# --------------------------------------------------------------------------- #
# Custom exceptions                                                            #
# --------------------------------------------------------------------------- #

class RobotSilentError(RuntimeError):
    """Raised when the preflight PING receives no reply within the timeout."""


class RunawayAbortError(RuntimeError):
    """Raised when the wall-clock cap fires."""


# --------------------------------------------------------------------------- #
# _ProtoShim — wraps a bare NezhaProtocol for back-compat with BenchRun(proto) #
# --------------------------------------------------------------------------- #

class _ProtoShim:
    """Minimal robot interface wrapping a bare NezhaProtocol.

    Allows SafeRun to accept legacy ``BenchRun(proto, ...)`` call patterns.
    ``proto`` must provide ``.send(cmd, read_timeout)`` and ``.ping()``.

    The ``_proto`` attribute is the wrapped NezhaProtocol, exposed so that
    SafeRun can call ``self._robot._proto.ping()`` and ``.send()`` via the
    same code path used for real Nezha instances.
    """

    def __init__(self, proto: Any) -> None:
        # _proto is the legacy NezhaProtocol — exposed as a public-ish attribute
        # so SafeRun's generic helpers can delegate to it.
        self._proto = proto

    def stop(self) -> None:
        """Send X (hard-cancel) to the robot via proto."""
        try:
            self._proto.send("X", 100)
        except Exception:  # noqa: BLE001
            pass


# --------------------------------------------------------------------------- #
# SafeRun                                                                      #
# --------------------------------------------------------------------------- #

class SafeRun:
    """Context manager that wraps drive programs with mandatory safety.

    Parameters
    ----------
    testrobot_or_nezha:
        A ``TestRobot`` (from ``make_target``) or a bare ``Nezha`` instance.
        When a ``TestRobot`` is passed, the ``target`` attribute is used to
        decide whether preflight should be a no-op ("sim").
    max_seconds:
        Wall-clock cap in seconds.  Default: 60.
    runaway:
        Reserved for future runaway detection (not yet implemented beyond
        the wall-clock cap).
    progress_fn:
        Optional callable invoked by the wall-clock thread just before it
        triggers the abort.  Use for logging / UI feedback.
    """

    def __init__(
        self,
        testrobot_or_nezha: "TestRobot | Nezha | Any",
        max_seconds: float = 60,
        runaway: bool = True,
        progress_fn: Any | None = None,
    ) -> None:
        # Accept TestRobot, bare Nezha, or legacy NezhaProtocol.
        # NezhaProtocol is detected by the presence of a .send() method but
        # no .stop() method (which is the Nezha / Robot interface).
        from robot_radio.testkit.target import TestRobot as _TR  # local import

        if isinstance(testrobot_or_nezha, _TR):
            self._robot = testrobot_or_nezha.robot
            self._is_sim = testrobot_or_nezha.target == "sim"
            self._proto_compat = False
        elif hasattr(testrobot_or_nezha, "stop"):
            # Bare Nezha (or any Robot subclass) — assume hardware.
            self._robot = testrobot_or_nezha
            self._is_sim = False
            self._proto_compat = False
        else:
            # Legacy NezhaProtocol interface: .send(cmd, read_timeout) and .ping().
            # Wrap it in a thin shim that delegates stop/stream/ping.
            self._robot = _ProtoShim(testrobot_or_nezha)
            self._is_sim = False
            self._proto_compat = True

        self._max_seconds = max_seconds
        self._runaway = runaway
        self._progress_fn = progress_fn

        # Wall-clock enforcer state.
        self._wall_clock_fired = threading.Event()
        self._wall_cap_triggered: bool = False

        # Saved SIGINT handler.
        self._orig_sigint: Any = None

        # Background enforcer thread.
        self._enforcer: threading.Thread | None = None
        self._entered_at: float = 0.0

    # ------------------------------------------------------------------ #
    # Context manager protocol                                             #
    # ------------------------------------------------------------------ #

    def __enter__(self) -> "SafeRun":
        # 1. Preflight liveness check (no-op for sim).
        if not self._is_sim:
            self._preflight()

        # 2. Register SIGINT handler.
        self._orig_sigint = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, self._sigint_handler)

        # 3. Start wall-clock enforcer thread.
        self._entered_at = time.monotonic()
        self._enforcer = threading.Thread(
            target=self._wall_clock_enforcer,
            name="SafeRun-enforcer",
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
            self._stop()
        finally:
            # Restore original SIGINT handler.
            if self._orig_sigint is not None:
                try:
                    signal.signal(signal.SIGINT, self._orig_sigint)
                except Exception:  # noqa: BLE001
                    pass
                self._orig_sigint = None

            # Signal the enforcer thread to stop.
            self._wall_clock_fired.set()

        # If the wall-clock enforcer actually triggered the cap, raise.
        if exc_type is None and self._wall_cap_triggered:
            elapsed = time.monotonic() - self._entered_at
            raise RunawayAbortError(
                f"wall clock cap: {elapsed:.1f}s >= {self._max_seconds}s"
            )

        # Propagate all other exceptions unchanged.
        return False

    # ------------------------------------------------------------------ #
    # Public stop API                                                      #
    # ------------------------------------------------------------------ #

    def stop(self) -> None:
        """Send stop to the robot.  Safe to call from any context."""
        self._stop()

    # ------------------------------------------------------------------ #
    # Private helpers                                                      #
    # ------------------------------------------------------------------ #

    def _stop(self) -> None:
        """Send ESTOP (halt now), then STREAM 0, to the robot.

        Both the Ctrl-C handler and the guaranteed exit path need a
        halt-now, not a planned stop: ``robot.stop()`` (the ``Nezha``/
        ``Robot`` interface's queued STOP) waits behind whatever is already
        in flight and would let a runaway ride out its current move --
        exactly wrong for "something went wrong, kill it." ``estop()``
        zeroes wheel targets and clears the queue in one cycle.
        ``_ProtoShim`` (the legacy bare-``NezhaProtocol`` wrapper) has no
        ``estop()`` of its own -- its ``stop()`` already sends the
        hard-cancel ``X`` -- so this falls back to ``.stop()`` there.

        Retries up to 3 times. A halt that silently fails is
        indistinguishable from one that worked, which is exactly what let
        a planned stop masquerade as a real one -- so if every attempt
        fails, that is logged rather than swallowed.
        """
        halt_error: Exception | None = None
        for _ in range(3):
            try:
                if hasattr(self._robot, "estop"):
                    self._robot.estop()
                else:
                    self._robot.stop()
                halt_error = None
                break
            except Exception as exc:  # noqa: BLE001
                halt_error = exc
            time.sleep(0.05)
        if halt_error is not None:
            logger.error(
                "SafeRun._stop(): halt failed on all 3 attempts, last "
                "error: %r -- ROBOT MAY STILL BE MOVING", halt_error)
        # Disable streaming.  NezhaProtocol (or _ProtoShim._proto) has .stream(0).
        try:
            proto = self._robot._proto
            if hasattr(proto, "stream"):
                proto.stream(0)
            else:
                proto.send("STREAM 0", 150)
        except Exception:  # noqa: BLE001
            pass

    def _preflight(self) -> None:
        """Ping the robot; raise RobotSilentError if no reply within 2 s."""
        proto = self._robot._proto
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            try:
                result = proto.ping()
                if result is not None:
                    return  # alive
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.1)

        # Second attempt: SNAP (or raw SNAP command for legacy proto).
        try:
            if hasattr(proto, "snap"):
                resp = proto.snap()
                if resp is not None:
                    return
            else:
                resp = proto.send("SNAP", 400)
                if resp and resp.get("responses"):
                    return
        except Exception:  # noqa: BLE001
            pass

        raise RobotSilentError(
            "preflight PING failed — robot silent. Power-cycle and retry."
        )

    def _sigint_handler(self, signum: int, frame: Any) -> None:  # noqa: ARG002
        """Handle SIGINT (Ctrl-C): stop the robot, restore handler, re-raise."""
        self._stop()
        if self._orig_sigint is not None:
            try:
                signal.signal(signal.SIGINT, self._orig_sigint)
            except Exception:  # noqa: BLE001
                pass
            self._orig_sigint = None
        raise KeyboardInterrupt

    def _wall_clock_enforcer(self) -> None:
        """Background thread: fire stop() when max_seconds is exceeded."""
        remaining = self._max_seconds - (time.monotonic() - self._entered_at)
        fired = self._wall_clock_fired.wait(timeout=max(0.0, remaining))
        if fired:
            # Event was set by __exit__ — block exited cleanly.
            return
        # Wall clock expired → runaway.
        if self._progress_fn is not None:
            try:
                self._progress_fn()
            except Exception:  # noqa: BLE001
                pass
        self._stop()
        self._wall_cap_triggered = True
        self._wall_clock_fired.set()


# --------------------------------------------------------------------------- #
# Back-compat alias                                                            #
# --------------------------------------------------------------------------- #

#: Legacy alias used by bench scripts that ``from bench_safety import BenchRun``.
BenchRun = SafeRun
