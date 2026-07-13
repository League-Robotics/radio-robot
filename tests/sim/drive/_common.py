"""tests/sim/drive/_common.py -- shared fixtures for the tier-0 Drive:: test
suite (ticket 100-006). NOT a test module itself (no ``test_`` prefix --
pytest will not collect it).

``make_limits()`` mirrors tests/sim/unit/drive_step_harness.cpp's own
``makeLimits()`` numbers exactly, including the issue's own control-law
gains (k_theta=6.0, k_c=1.5e-5, k_s=2.0, k_d=0 -- no derivative gain field
exists on ``Limits`` at all, the P-only outer-loop rule) so tier-0's
closed-loop/purity/fuzz tests exercise the SAME configuration the C++
harnesses already proved converges.
"""
from __future__ import annotations

from drive import Limits, ProfileLimits

TRACKWIDTH = 128.0  # [mm]


def make_limits(trackwidth: float = TRACKWIDTH) -> Limits:
    return Limits(
        linear=ProfileLimits(velocity=400.0, accel=800.0, decel=800.0, jerk=0.0),
        rotational=ProfileLimits(velocity=3.0, accel=15.0, decel=15.0, jerk=0.0),
        v_wheel_max=600.0,  # [mm/s]
        trim_v_max=120.0,  # [mm/s] -- issue's control-law table
        trim_omega_max=1.0,  # [rad/s] -- issue's control-law table (arc value)
        wheel_step_max=200.0,  # [mm/s]
        track_k_s=2.0,  # [1/s] -- k_s
        track_k_theta=6.0,  # [1/s] -- k_theta
        track_k_cross=1.5e-5,  # [rad/mm^2] -- k_c
        min_speed=20.0,  # [mm/s] -- pivot-mode threshold
    )
