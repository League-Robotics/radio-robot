"""Regression test for the tour step-completion (idle) detection bug (OOP).

Root cause
----------
The tour advances between steps by waiting for the robot to return to idle
(``mode=I``).  The old ``_TourRunner._wait_for_idle`` did this with
``transport.command("SNAP")`` and parsed ``mode=`` from the reply.  But a SNAP
reply is a ``TLM`` frame with **no corr-id**, and the serial reader routes all
TLM frames to ``_tlm_queue`` — never to the corr-id reply queue that
``command()`` waits on.  Over the radio relay (STREAM off, SNAP-polled),
``command("SNAP")`` therefore always returned ``""``, ``mode=I`` was never
seen, and the tour hung forever on step 1 (observed live: the robot did its
first ``RT`` turn and then sat idle while the GUI spammed useless SNAPs).

Fix
---
``_wait_for_idle`` now requests a fresh frame with a fire-and-forget
``transport.send("SNAP")`` and reads the mode from ``state["last_tlm"]``,
which the transport's ``on_telemetry`` callback populates (the SNAP's TLM
reply arrives through that path even though ``command()`` never sees it).

This module's production logic lives in a closure inside
``_build_main_window`` with no import seam, so — per the established pattern in
``test_tour_stop.py`` / ``test_set_origin.py`` — the exact ``_wait_for_idle``
control flow is re-implemented inline and exercised against a fake transport
that models the relay: ``command("SNAP")`` returns ``""`` (the bug condition),
while ``send("SNAP")`` delivers a frame via the ``on_telemetry`` cache.  A test
that drove idle detection through ``command()`` would time out here.
"""
from __future__ import annotations

import time


class _FakeFrame:
    """Minimal stand-in for a parsed TLMFrame — only ``mode`` is read."""

    def __init__(self, mode: str) -> None:
        self.mode = mode


class _RelayLikeTransport:
    """Fake transport modelling the radio relay's SNAP behaviour.

    - ``command("SNAP")`` returns ``""`` — the exact bug condition (a TLM
      reply carries no corr-id, so ``command()`` never receives it).
    - ``send("SNAP")`` delivers the next scripted frame into ``state`` via an
      ``on_telemetry``-style callback, mirroring how the real transport reader
      thread caches ``state["last_tlm"]``.
    """

    def __init__(self, state: dict, frames: list[str]) -> None:
        self._state = state
        self._frames = list(frames)
        self.command_calls = 0
        self.send_calls = 0

    def command(self, line: str, read_timeout: int = 200) -> str:  # [ms]
        self.command_calls += 1
        return ""  # relay: SNAP's TLM reply never reaches command()

    def send(self, line: str) -> None:
        self.send_calls += 1
        if line.strip().upper() == "SNAP" and self._frames:
            mode = self._frames.pop(0)
            self._state["last_tlm"] = (_FakeFrame(mode), time.monotonic())


# Timing constants mirror _TourRunner (shrunk where it only speeds the test).
_SPINUP_S = 0.0
_POLL_S = 0.01
_SNAP_REPLY_TIMEOUT_S = 0.3
_MOVE_TIMEOUT_S = 2.0


def _wait_for_idle(transport, state: dict, stop_flag: dict) -> bool:
    """Inline re-implementation of _TourRunner._wait_for_idle (see docstring)."""
    time.sleep(_SPINUP_S)
    t_start = time.monotonic()
    deadline = t_start + _MOVE_TIMEOUT_S
    while time.monotonic() < deadline:
        if stop_flag.get("stop"):
            return True
        try:
            transport.send("SNAP")
        except Exception:
            return False
        reply_deadline = time.monotonic() + _SNAP_REPLY_TIMEOUT_S
        while time.monotonic() < reply_deadline:
            if stop_flag.get("stop"):
                return True
            cached = state.get("last_tlm")
            if cached is not None:
                frame, ts = cached
                mode = (getattr(frame, "mode", None) or "").upper()
                if ts >= t_start and mode == "I":
                    return True
            time.sleep(_POLL_S)
    return False


def test_idle_detected_from_on_telemetry_cache_not_command():
    """Idle is detected via state['last_tlm'], and command('SNAP') is unused."""
    state: dict = {}
    # Robot reports moving twice, then idle — as a bounded move completes.
    transport = _RelayLikeTransport(state, frames=["V", "V", "I"])

    ok = _wait_for_idle(transport, state, stop_flag={})

    assert ok, "idle was never detected — tour would hang on this step"
    assert transport.command_calls == 0, (
        "idle detection must NOT use command('SNAP') — over the relay it "
        "returns '' and the tour never advances"
    )
    assert transport.send_calls >= 1, "a fire-and-forget SNAP must be issued"


def test_stale_pre_move_idle_frame_does_not_end_wait_early():
    """A cached idle frame from before the wait began must be ignored."""
    state: dict = {}
    # Pre-existing stale idle frame (e.g. left over from before the move).
    state["last_tlm"] = (_FakeFrame("I"), time.monotonic())
    # After the move: moving, moving, then genuinely idle.
    transport = _RelayLikeTransport(state, frames=["V", "V", "I"])

    ok = _wait_for_idle(transport, state, stop_flag={})

    assert ok
    # All three scripted frames should have been consumed to reach the real
    # idle — proving the stale frame did not short-circuit the wait.
    assert transport.send_calls >= 3, (
        "stale pre-move idle frame ended the wait early — ts>=t_start guard "
        "is not working"
    )


def test_timeout_returns_false_when_never_idle():
    """If the robot never reports idle, _wait_for_idle times out (False)."""
    state: dict = {}
    transport = _RelayLikeTransport(state, frames=["V"] * 500)

    ok = _wait_for_idle(transport, state, stop_flag={})

    assert ok is False, "must return False (abort) when idle never arrives"
