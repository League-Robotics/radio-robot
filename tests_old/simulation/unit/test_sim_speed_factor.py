"""test_sim_speed_factor.py — TestGUI sim fast-forward (speed factor).

The sim clock is purely virtual (``Sim.tick_for`` advances a millisecond
counter as fast as the CPU allows); real-time feel in the TestGUI comes
solely from ``SimTransport._tick_loop`` sleeping the remainder of a 20 ms
wall interval after each 20 sim-ms step.  ``set_speed_factor(N)`` advances
N such steps per wall tick — same integration granularity, so trajectories
are identical at every speed; only wall-clock pacing compresses.

These tests cover the clamping contract (no lib needed) and the actual
fast-forward behaviour against the real firmware sim via a connected
``SimTransport`` (the same code path the GUI uses).
"""
from __future__ import annotations

import time

from robot_radio.testgui.transport import (
    SimTransport,
    _SIM_SPEED_MAX,
    _SIM_SPEED_MIN,
)


def test_speed_factor_clamps_and_defaults():
    """Default is real time (1x); out-of-range requests clamp, never raise."""
    st = SimTransport()
    assert st._speed_factor == 1

    st.set_speed_factor(5)
    assert st._speed_factor == 5

    st.set_speed_factor(0)
    assert st._speed_factor == _SIM_SPEED_MIN

    st.set_speed_factor(-3)
    assert st._speed_factor == _SIM_SPEED_MIN

    st.set_speed_factor(10_000)
    assert st._speed_factor == _SIM_SPEED_MAX


def test_fast_forward_advances_sim_time_faster_than_wall(build_lib):
    """At 10x, sim time must run well ahead of wall time; at 1x it cannot.

    Timing margins are deliberately loose (ratio 2x against a nominal 10x,
    generous absolute bounds) so scheduler jitter on a loaded CI machine
    does not flake this test.
    """
    st = SimTransport()
    st.connect()
    assert st._connected, "SimTransport failed to connect (sim lib built?)"
    try:
        window_s = 0.6

        # Baseline at 1x: the tick-thread paces 20 sim-ms per >=20 wall-ms,
        # so sim time cannot outrun wall time.
        t0 = st._sim._t
        time.sleep(window_s)
        delta_1x = st._sim._t - t0
        assert delta_1x > 0, "tick-thread is not advancing the sim"
        assert delta_1x <= 1500, (
            f"1x advanced {delta_1x} sim-ms in {window_s}s wall — "
            "sim is outrunning wall time at real-time pacing"
        )

        st.set_speed_factor(10)
        t0 = st._sim._t
        time.sleep(window_s)
        delta_10x = st._sim._t - t0

        assert delta_10x >= 1000, (
            f"10x advanced only {delta_10x} sim-ms in {window_s}s wall"
        )
        assert delta_10x >= 2 * delta_1x, (
            f"10x ({delta_10x} sim-ms) not meaningfully faster than "
            f"1x ({delta_1x} sim-ms)"
        )
    finally:
        st.disconnect()
