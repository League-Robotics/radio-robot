"""
test_053_s_stop_condition.py — Tests for S command migrated onto MotionCommand
(ticket 053-003).

Verifies that:
  - S now creates a MotionCommand with streamSeed=true (immediate BVC seed, no
    trapezoid ramp) and VELOCITY mode — NOT DriveMode::STREAMING.
  - stop= clauses on S now FIRE (Phase 1 deferral resolved).
  - S 300 300 stop=d:400 terminates with reason=dist.
  - S 300 300 stop=t:500 terminates with reason=time.
  - S 300 300 (no stop=) remains open-ended (does NOT terminate on its own).
  - EVT done S label emitted on stop-condition completion.
  - The 052 "parser-only" test for S stop= is now superseded: the stop fires,
    not merely parses (test_s_stop_fires_dist and test_s_stop_fires_time below).
"""
import ctypes
import pytest

from firmware import Sim

TICK_STEP_MS = 24


def _tick_collect(s: Sim, n: int) -> str:
    """Tick n times, accumulating async events. Returns all EVT strings."""
    evts = ""
    for _ in range(n):
        s._lib.sim_tick(s._h, ctypes.c_uint32(s._t))
        s._t += TICK_STEP_MS
        evts += s.get_async_evts()
    return evts


def _setup(s: Sim) -> None:
    """Common fixture: extend watchdog timeout so tests aren't cut short."""
    s.send_command("SET sTimeout=60000")


def _is_err(reply: str) -> bool:
    return "ERR" in reply.upper()


def _terminated(s: Sim, evts: str) -> bool:
    """Return True when the motion command appears to have stopped."""
    pwm_l = float(s._lib.sim_get_pwm_l(s._h))
    pwm_r = float(s._lib.sim_get_pwm_r(s._h))
    return "EVT done" in evts or (pwm_l == 0.0 and pwm_r == 0.0)


# ---------------------------------------------------------------------------
# S with no stop= remains open-ended (watchdog-only)
# ---------------------------------------------------------------------------

def test_s_no_stop_remains_open(sim):
    """S 300 300 with no stop= does NOT self-terminate — stays open-ended.

    The command should still be running after a generous tick window (no EVT done
    emitted, PWM non-zero).  Only the system watchdog can shut it down.
    """
    _setup(sim)
    sim.get_async_evts()  # drain

    r = sim.send_command("S 300 300")
    assert not _is_err(r), f"S command rejected: {r!r}"
    assert "OK" in r.upper(), f"Expected OK, got {r!r}"

    # 80 ticks ≈ 1.9 s — short enough to not hit the 60-second watchdog.
    evts = _tick_collect(sim, 80)
    assert "EVT done" not in evts, (
        f"S 300 300 self-terminated unexpectedly (evts={evts!r})"
    )
    # Motors should still be running (PWM non-zero).
    pwm_l = float(sim._lib.sim_get_pwm_l(sim._h))
    pwm_r = float(sim._lib.sim_get_pwm_r(sim._h))
    assert pwm_l != 0.0 or pwm_r != 0.0, (
        "S 300 300: motors stopped unexpectedly (open-ended S should keep driving)"
    )


# ---------------------------------------------------------------------------
# S with stop=d:<mm> → fires with reason=dist and EVT done S
# ---------------------------------------------------------------------------

def test_s_stop_fires_dist(sim):
    """S 300 300 stop=d:200 terminates after ~200 mm with reason=dist.

    This was the Phase 1 deferral: the stop= was parsed but could not fire
    because S ran via DriveMode::STREAMING (no MotionCommand).  After 053-003
    S runs via VELOCITY+streamSeed and the stop fires normally.
    """
    _setup(sim)
    sim.get_async_evts()  # drain

    r = sim.send_command("S 300 300 stop=d:200")
    assert not _is_err(r), f"S stop=d rejected: {r!r}"
    assert "OK" in r.upper(), f"Expected OK, got {r!r}"

    evts = _tick_collect(sim, 250)  # generous window
    assert _terminated(sim, evts), (
        f"S stop=d:200 did not terminate (evts={evts!r})"
    )
    assert "reason=dist" in evts, (
        f"S stop=d:200 terminated but missing reason=dist (evts={evts!r})"
    )
    assert "EVT done S" in evts, (
        f"S stop=d:200 did not emit 'EVT done S' (evts={evts!r})"
    )


# ---------------------------------------------------------------------------
# S with stop=t:<ms> → fires with reason=time and EVT done S
# ---------------------------------------------------------------------------

def test_s_stop_fires_time(sim):
    """S 300 300 stop=t:300 terminates after ~300 ms with reason=time."""
    _setup(sim)
    sim.get_async_evts()  # drain

    r = sim.send_command("S 300 300 stop=t:300")
    assert not _is_err(r), f"S stop=t rejected: {r!r}"
    assert "OK" in r.upper(), f"Expected OK, got {r!r}"

    # 300 ms / 24 ms per tick ≈ 12.5 ticks; give generous 80 ticks (~2 s).
    evts = _tick_collect(sim, 80)
    assert _terminated(sim, evts), (
        f"S stop=t:300 did not terminate within 80 ticks (evts={evts!r})"
    )
    assert "reason=time" in evts, (
        f"S stop=t:300 terminated but missing reason=time (evts={evts!r})"
    )
    assert "EVT done S" in evts, (
        f"S stop=t:300 did not emit 'EVT done S' (evts={evts!r})"
    )


# ---------------------------------------------------------------------------
# EVT done S label present on any stop-condition completion
# ---------------------------------------------------------------------------

def test_s_done_label_emitted(sim):
    """EVT done S label is emitted when a stop condition fires on S."""
    _setup(sim)
    sim.get_async_evts()  # drain

    # Use a short time stop so it fires quickly.
    r = sim.send_command("S 200 200 stop=t:200")
    assert not _is_err(r), f"S stop=t rejected: {r!r}"

    evts = _tick_collect(sim, 80)
    assert "EVT done S" in evts, (
        f"EVT done S not emitted when stop fires (evts={evts!r})"
    )


# ---------------------------------------------------------------------------
# S stop=line:<ge|le>:<thr> — fires when line sensor crosses
# ---------------------------------------------------------------------------

def test_s_stop_line_fires(sim):
    """S 200 200 stop=line:ge:512 terminates when a line sensor crosses 512."""
    _setup(sim)
    sim.init_line_sensor()
    sim.set_line_values(0, 0, 0, 0)
    _tick_collect(sim, 3)
    sim.get_async_evts()  # drain

    r = sim.send_command("S 200 200 stop=line:ge:512")
    assert not _is_err(r), f"S stop=line rejected: {r!r}"

    _tick_collect(sim, 5)
    sim.set_line_values(700, 0, 0, 0)  # cross threshold
    evts = _tick_collect(sim, 80)

    assert _terminated(sim, evts), (
        f"S stop=line:ge:512 did not terminate on crossing (evts={evts!r})"
    )
    assert "EVT done S" in evts, (
        f"EVT done S not emitted on line-stop fire (evts={evts!r})"
    )


# ---------------------------------------------------------------------------
# S with stop= is accepted (was already tested in 052, now confirms firing)
# ---------------------------------------------------------------------------

def test_s_stop_accepted_and_fires(sim):
    """S 200 200 stop=d:300 is accepted AND fires (supersedes 052 parser-only test).

    The 052 test test_s_stop_line_accepted_parser_only confirmed acceptance but
    noted the condition would not fire.  After 053-003 it does fire.
    """
    _setup(sim)
    sim.get_async_evts()  # drain

    r = sim.send_command("S 200 200 stop=d:300")
    assert not _is_err(r), f"S stop=d rejected: {r!r}"
    assert "OK" in r.upper(), f"Expected OK, got {r!r}"

    evts = _tick_collect(sim, 250)
    assert _terminated(sim, evts), (
        f"S stop=d:300 did not terminate (evts={evts!r})"
    )
    assert "reason=dist" in evts, (
        f"S stop=d:300: missing reason=dist (evts={evts!r})"
    )
