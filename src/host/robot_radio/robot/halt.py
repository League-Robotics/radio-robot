"""robot_radio.robot.halt — the one shared "halt now" idiom every host call
site must use (128-001, halt-now-call-sites-must-use-estop-and-never-
swallow-failure.md).

``NezhaProtocol.stop()`` (and ``Robot.stop()``, its polymorphic wrapper) is a
PLANNED stop: it enqueues behind whatever ``Move`` is already in flight and
only takes effect once it is that stop's turn. Measured on hardware
2026-07-29: sent 0.5s into a 400mm leg, the robot travelled the ENTIRE leg
(39.8cm, 5.9s) before a mid-leg ``stop()`` took effect. ``estop()`` is the
halt-now verb — same repro, 2.9cm / 0.10s.

Every call site in this tree that means "halt the robot right now" (a
Ctrl-C handler, a cleanup ``finally``, a geofence breach, an MCP "stop" tool)
must go through ``halt_now()`` here rather than calling ``stop()`` directly
or rolling its own retry loop — one halt idiom means an auditor checks one
function instead of re-deriving the stop-vs-estop distinction, and the
divergent-swallowed-exception version of this bug is what let a geofence
"detect correctly and stop nothing" (`.claude/rules/playfield-testing.md`).

Modeled directly on the first hardened version of this idiom,
``field/geofence.py::Geofence._halt()`` (127-003), which now delegates here.
"""
from __future__ import annotations

import time
from typing import Any, Callable

_RETRY_ATTEMPTS = 3
_RETRY_DELAY = 0.05  # [s] between estop() retries


def halt_now(proto: Any, log: Callable[[str], None] = print) -> None:
    """Halt ``proto`` NOW — calls ``proto.estop()``, never the planned
    ``stop()``. Retries up to 3x on failure; if every attempt fails, logs a
    loud error via ``log`` and RAISES the last exception.

    A halt that silently failed is indistinguishable from one that worked —
    exactly what let the robot drive off the table before this idiom
    existed. Never swallow the exception this raises without first calling
    ``log`` (or accepting that ``halt_now`` already did) — a bare
    ``except Exception: pass`` around a halt call is the defect this
    function exists to eliminate.

    A caller in a cleanup path that must not raise (e.g. a ``finally``
    during interpreter shutdown) may still catch the raised exception, but
    only AFTER this function's own log line has already fired:

        finally:
            try:
                halt_now(proto)
            except Exception:
                # halt_now already logged ROBOT MAY STILL BE MOVING; the
                # operator has been told -- which is the entire point.
                pass
    """
    err: Exception | None = None
    for _ in range(_RETRY_ATTEMPTS):
        try:
            proto.estop()
            return
        except Exception as exc:
            err = exc
        time.sleep(_RETRY_DELAY)
    log(f"ERROR: estop() failed {_RETRY_ATTEMPTS}x -- ROBOT MAY STILL BE MOVING: {err!r}")
    raise err
