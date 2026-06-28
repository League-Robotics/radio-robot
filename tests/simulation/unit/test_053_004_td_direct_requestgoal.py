"""
test_053_004_td_direct_requestgoal.py — Tests for T/D direct requestGoal path
(ticket 053-004).

Verifies that handleT and handleD now build GoalRequest and call requestGoal
directly (no stringify/inverse round-trip):

  - T 300 300 1000 emits EVT done T reason=time.
  - D 300 300 400 emits EVT done D reason=dist.
  - T 300 300 5000 stop=sensor:line0:ge:512 fires on line before timeout.
  - D encoder baseline is 0 at start (distanceDrive encoder reset preserved).

The EVT done label and reason= tests confirm doneLabel is applied correctly
via Superstructure's stops/doneLabel injection after requestGoal.

The encoder reset test confirms Goal::DISTANCE still routes through
robot->distanceDrive (which calls resetEncoders), preserving the atomic
encoder-reset guarantee that was the motivation for keeping Goal::DISTANCE.
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


# ---------------------------------------------------------------------------
# T 300 300 1000 → EVT done T reason=time
# ---------------------------------------------------------------------------

def test_t_emits_done_t_reason_time(build_lib):
    """T 300 300 1000 emits 'EVT done T reason=time' via direct requestGoal."""
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.get_async_evts()  # drain

        s.send_command("T 300 300 1000")
        evts = _tick_collect(s, 150)  # ~3.6 s, generous for 1 s TIME stop

        assert "EVT done T" in evts, (
            f"Expected 'EVT done T'; evts={evts!r}"
        )
        assert "reason=time" in evts, (
            f"T TIME stop did not emit reason=time; evts={evts!r}"
        )


# ---------------------------------------------------------------------------
# D 300 300 400 → EVT done D reason=dist
# ---------------------------------------------------------------------------

def test_d_emits_done_d_reason_dist(build_lib):
    """D 300 300 400 emits 'EVT done D reason=dist' via direct requestGoal."""
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.get_async_evts()  # drain

        s.send_command("D 300 300 400")
        evts = _tick_collect(s, 250)  # generous window for 400 mm at 300 mm/s

        assert "EVT done D" in evts, (
            f"Expected 'EVT done D'; evts={evts!r}"
        )
        assert "reason=dist" in evts, (
            f"D DISTANCE stop did not emit reason=dist; evts={evts!r}"
        )


# ---------------------------------------------------------------------------
# T 300 300 5000 stop=sensor:line0:ge:512 → fires on line before timeout
# ---------------------------------------------------------------------------

def test_t_sensor_stop_fires_before_timeout(build_lib):
    """T 300 300 5000 stop=sensor:line0:ge:512 fires on line0 before 5-second timeout."""
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.init_line_sensor()
        s.set_line_values(0, 0, 0, 0)
        _tick_collect(s, 3)
        s.get_async_evts()  # drain

        s.send_command("T 300 300 5000 stop=sensor:line0:ge:512")
        _tick_collect(s, 5)  # let the command start
        s.set_line_values(800, 0, 0, 0)  # trip line0 above 512
        evts = _tick_collect(s, 80)  # ~2 s — should fire long before 5 s timeout

        assert "EVT done T" in evts, (
            f"Expected 'EVT done T' (sensor fire); evts={evts!r}"
        )
        assert "reason=line0" in evts, (
            f"sensor:line0 stop did not emit reason=line0; evts={evts!r}"
        )


# ---------------------------------------------------------------------------
# D encoder reset: encoder baseline is 0 at command start
# ---------------------------------------------------------------------------

def test_d_encoder_baseline_is_zero_at_start(build_lib):
    """D command: encoder baseline is 0 at start (distanceDrive reset preserved).

    Run some motion first to accumulate encoder counts, then issue D.
    The DISTANCE stop evaluation must start from 0, not from the prior
    accumulated value.  This verifies that Goal::DISTANCE still routes
    through robot->distanceDrive (which calls resetEncoders), preserving
    the atomic encoder-reset guarantee.

    Method: start T 200 200 500 to accumulate encoder counts, wait for
    completion, then issue D 200 200 200.  If the encoder reset works,
    the D completes after 200 mm of travel starting from 0, not from the
    stale accumulated value.  Check that EVT done D reason=dist fires
    (not the TIME backstop), confirming the DISTANCE stop fires correctly.
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.get_async_evts()  # drain

        # Phase 1: accumulate encoder counts via T.
        s.send_command("T 200 200 500")
        evts_phase1 = _tick_collect(s, 100)  # ~2.4 s, enough for T 500 ms + ramp-down
        assert "EVT done T" in evts_phase1, (
            f"Phase 1 T did not complete; evts={evts_phase1!r}"
        )

        # Confirm encoders have accumulated (non-zero after T).
        enc_l_after_t = float(s._lib.sim_get_enc_l(s._h))
        enc_r_after_t = float(s._lib.sim_get_enc_r(s._h))
        assert enc_l_after_t > 0.0 or enc_r_after_t > 0.0, (
            f"Expected non-zero encoders after T; enc_l={enc_l_after_t:.1f} "
            f"enc_r={enc_r_after_t:.1f}"
        )

        s.get_async_evts()  # drain

        # Phase 2: issue D — encoder reset must happen at start.
        s.send_command("D 200 200 200")

        # Read encoders immediately after D starts (one tick to confirm reset).
        s._lib.sim_tick(s._h, ctypes.c_uint32(s._t))
        s._t += TICK_STEP_MS
        enc_l_after_d_start = float(s._lib.sim_get_enc_l(s._h))
        enc_r_after_d_start = float(s._lib.sim_get_enc_r(s._h))

        # Encoder mirror should be near 0 just after the reset.
        assert abs(enc_l_after_d_start) < 10.0, (
            f"Encoder L not reset at D start: enc_l={enc_l_after_d_start:.1f}"
        )
        assert abs(enc_r_after_d_start) < 10.0, (
            f"Encoder R not reset at D start: enc_r={enc_r_after_d_start:.1f}"
        )

        # Let D complete — should fire DISTANCE stop (not TIME backstop).
        evts_phase2 = _tick_collect(s, 200)
        all_evts = evts_phase2 + s.get_async_evts()

        assert "EVT done D" in all_evts, (
            f"Expected 'EVT done D' after D 200 200 200; evts={all_evts!r}"
        )
        assert "reason=dist" in all_evts, (
            f"D completed but missing reason=dist (TIME backstop fired instead?); "
            f"evts={all_evts!r}"
        )
