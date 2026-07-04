#!/usr/bin/env python3
"""test_velocity_controller.py — Unit tests for VelocityController math (010-003).

Pure Python implementation of the same equations defined in:
  source/control/VelocityController.h / .cpp

Tests verify:
  - FF-only: zero error → output = kFF * |setpoint| (signed by setpoint)
  - Proportional term: positive error → positive correction added
  - Integrator accumulates with positive error, frozen under anti-windup
  - Deadband: integrator frozen when |setpoint| < minWheelMms
  - Output clamped to [-100, +100]
  - Direction: negative setpoint → negative PWM output
  - reset() zeros integrator
"""

from __future__ import annotations

import math
import pytest


# ---------------------------------------------------------------------------
# Pure Python mirror of VelocityController C++ class
# ---------------------------------------------------------------------------

class VelocityController:
    """Python mirror of source/control/VelocityController.cpp."""

    def __init__(self, kFF: float, kP: float, kI: float,
                 iMax: float, minWheelMms: float):
        self.kFF = kFF
        self.kP = kP
        self.kI = kI
        self.iMax = iMax
        self.minWheelMms = minWheelMms
        self.integral = 0.0

    def update(self, setpoint: float, measured: float, dt_s: float) -> float:
        """Compute one velocity control tick, return PWM% in [-100, +100]."""
        if dt_s <= 0.0:
            return 0.0

        err = setpoint - measured

        sp_abs = abs(setpoint)
        sp_sign = 1.0 if setpoint >= 0.0 else -1.0
        ff = self.kFF * sp_abs

        # Compute raw output before integrator update (to detect saturation)
        raw_pwm = sp_sign * ff + self.kP * err + self.integral

        saturated = (raw_pwm >= 100.0) or (raw_pwm <= -100.0)
        in_deadband = sp_abs < self.minWheelMms

        if not saturated and not in_deadband:
            self.integral += self.kI * err * dt_s
            self.integral = max(-self.iMax, min(self.iMax, self.integral))

        output = sp_sign * ff + self.kP * err + self.integral
        return max(-100.0, min(100.0, output))

    def reset(self):
        self.integral = 0.0


# ---------------------------------------------------------------------------
# Shared fixture
# ---------------------------------------------------------------------------

def make_vc(**overrides) -> VelocityController:
    """Create a VelocityController with sensible test defaults."""
    defaults = dict(kFF=0.15, kP=0.3, kI=0.05, iMax=60.0, minWheelMms=20.0)
    defaults.update(overrides)
    return VelocityController(**defaults)


# ---------------------------------------------------------------------------
# Tests — feed-forward only (zero error)
# ---------------------------------------------------------------------------

class TestFeedForward:
    """When setpoint == measured, output comes purely from FF term."""

    def test_ff_positive_setpoint(self):
        """Positive setpoint, zero error: output = kFF * setpoint."""
        vc = make_vc(kFF=0.15, kP=0.0, kI=0.0)
        out = vc.update(setpoint=200.0, measured=200.0, dt_s=0.02)
        assert out == pytest.approx(0.15 * 200.0, rel=1e-5)  # 30.0 PWM%

    def test_ff_negative_setpoint(self):
        """Negative setpoint, zero error: output is negative."""
        vc = make_vc(kFF=0.15, kP=0.0, kI=0.0)
        out = vc.update(setpoint=-200.0, measured=-200.0, dt_s=0.02)
        assert out == pytest.approx(-0.15 * 200.0, rel=1e-5)  # -30.0 PWM%

    def test_ff_zero_setpoint(self):
        """Zero setpoint: FF term is zero."""
        vc = make_vc(kFF=0.15, kP=0.0, kI=0.0)
        out = vc.update(setpoint=0.0, measured=0.0, dt_s=0.02)
        assert out == pytest.approx(0.0)

    def test_ff_magnitude_proportional_to_setpoint(self):
        """FF scales linearly with |setpoint|."""
        vc = make_vc(kFF=0.2, kP=0.0, kI=0.0)
        out100 = vc.update(setpoint=100.0, measured=100.0, dt_s=0.02)
        out200 = vc.update(setpoint=200.0, measured=200.0, dt_s=0.02)
        assert out200 == pytest.approx(2.0 * out100, rel=1e-5)


# ---------------------------------------------------------------------------
# Tests — proportional term
# ---------------------------------------------------------------------------

class TestProportional:
    """kP*err is added to the output."""

    def test_positive_error_increases_output(self):
        """Setpoint > measured → positive error → output > FF alone."""
        vc = make_vc(kFF=0.15, kP=0.5, kI=0.0)
        ff_only = 0.15 * 100.0  # 15.0
        out = vc.update(setpoint=100.0, measured=80.0, dt_s=0.02)  # err = 20
        assert out == pytest.approx(ff_only + 0.5 * 20.0, rel=1e-5)  # 15 + 10 = 25

    def test_negative_error_decreases_output(self):
        """Setpoint < measured → negative error → output < FF alone."""
        vc = make_vc(kFF=0.15, kP=0.5, kI=0.0)
        ff_only = 0.15 * 100.0  # 15.0
        out = vc.update(setpoint=100.0, measured=120.0, dt_s=0.02)  # err = -20
        assert out == pytest.approx(ff_only + 0.5 * (-20.0), rel=1e-5)  # 15 - 10 = 5

    def test_error_with_negative_setpoint(self):
        """Negative setpoint, measured too slow (less negative): corrects output."""
        vc = make_vc(kFF=0.0, kP=0.5, kI=0.0)
        # setpoint=-100, measured=-80: err = -100-(-80) = -20
        out = vc.update(setpoint=-100.0, measured=-80.0, dt_s=0.02)
        # sp_sign=-1, ff=0, kP*err = 0.5*(-20)=-10
        # raw = 0 + (-10) + 0 = -10 → output = -10
        assert out == pytest.approx(-10.0, rel=1e-5)


# ---------------------------------------------------------------------------
# Tests — integrator accumulation
# ---------------------------------------------------------------------------

class TestIntegrator:
    """Integrator accumulates error over time."""

    def test_integrator_grows_with_positive_error(self):
        """Repeated ticks with positive error → growing integral → growing output."""
        vc = make_vc(kFF=0.0, kP=0.0, kI=0.1, minWheelMms=10.0)
        # setpoint=100, measured=80: err=20, each tick adds kI*err*dt = 0.1*20*0.02 = 0.04
        out1 = vc.update(100.0, 80.0, 0.02)
        out2 = vc.update(100.0, 80.0, 0.02)
        assert out2 > out1  # integral grew

    def test_integrator_accumulates_correctly(self):
        """After N ticks with constant error, integral = kI * err * N * dt."""
        vc = make_vc(kFF=0.0, kP=0.0, kI=0.1, iMax=1000.0, minWheelMms=10.0)
        err = 20.0
        dt = 0.02
        n = 5
        for _ in range(n):
            vc.update(100.0, 80.0, dt)
        expected_integral = 0.1 * err * n * dt  # 0.02
        assert vc.integral == pytest.approx(expected_integral, rel=1e-5)

    def test_integrator_clamped_at_imax(self):
        """Integrator does not exceed iMax."""
        vc = make_vc(kFF=0.0, kP=0.0, kI=10.0, iMax=5.0, minWheelMms=0.0)
        for _ in range(100):
            vc.update(100.0, 0.0, 0.02)  # large error, many ticks
        assert vc.integral <= 5.0

    def test_reset_zeros_integrator(self):
        """reset() clears integrator state."""
        vc = make_vc(kFF=0.0, kP=0.0, kI=0.1, minWheelMms=10.0)
        for _ in range(10):
            vc.update(100.0, 80.0, 0.02)
        assert vc.integral != 0.0
        vc.reset()
        assert vc.integral == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# Tests — anti-windup
# ---------------------------------------------------------------------------

class TestAntiWindup:
    """Integrator frozen when output is rail-limited."""

    def test_integrator_frozen_at_positive_rail(self):
        """When raw output >= 100, integrator does not grow."""
        # Use gains that force saturation immediately.
        vc = make_vc(kFF=0.5, kP=0.0, kI=1.0, iMax=200.0, minWheelMms=0.0)
        # setpoint=200 → FF = 0.5*200 = 100.0 → already at rail before first tick
        integral_before = vc.integral
        vc.update(200.0, 0.0, 0.02)  # raw = 100 + kP*200 + 0 = 100 (saturated)
        assert vc.integral == pytest.approx(integral_before)  # must not have changed

    def test_integrator_frozen_at_negative_rail(self):
        """When raw output <= -100, integrator does not shrink further."""
        vc = make_vc(kFF=0.5, kP=0.0, kI=1.0, iMax=200.0, minWheelMms=0.0)
        # setpoint=-200: FF term causes saturation at -100
        integral_before = vc.integral
        vc.update(-200.0, 0.0, 0.02)
        assert vc.integral == pytest.approx(integral_before)

    def test_integrator_accumulates_below_rail(self):
        """When output is not rail-limited, integrator grows normally."""
        vc = make_vc(kFF=0.0, kP=0.0, kI=0.1, iMax=200.0, minWheelMms=0.0)
        # Small setpoint and error → output well below rail
        vc.update(50.0, 45.0, 0.02)  # err=5, integral+=0.1*5*0.02=0.01
        assert vc.integral == pytest.approx(0.01, rel=1e-4)


# ---------------------------------------------------------------------------
# Tests — deadband
# ---------------------------------------------------------------------------

class TestDeadband:
    """Integrator frozen when |setpoint| < minWheelMms."""

    def test_integrator_frozen_in_deadband(self):
        """setpoint below minWheelMms → integrator unchanged."""
        vc = make_vc(kFF=0.0, kP=0.0, kI=1.0, iMax=200.0, minWheelMms=20.0)
        vc.update(10.0, 0.0, 0.02)  # |10| < 20 → deadband
        assert vc.integral == pytest.approx(0.0)

    def test_integrator_active_above_deadband(self):
        """setpoint above minWheelMms → integrator accumulates."""
        vc = make_vc(kFF=0.0, kP=0.0, kI=1.0, iMax=200.0, minWheelMms=20.0)
        vc.update(25.0, 0.0, 0.02)  # |25| >= 20 → not in deadband; err=25
        assert vc.integral > 0.0  # must have grown

    def test_deadband_applies_to_negative_setpoint(self):
        """Negative setpoint within |minWheelMms| → integrator frozen."""
        vc = make_vc(kFF=0.0, kP=0.0, kI=1.0, iMax=200.0, minWheelMms=20.0)
        vc.update(-10.0, 0.0, 0.02)  # |-10| = 10 < 20 → deadband
        assert vc.integral == pytest.approx(0.0)

    def test_deadband_boundary_exact(self):
        """setpoint == minWheelMms: in-deadband (strictly less than)."""
        vc = make_vc(kFF=0.0, kP=0.0, kI=1.0, iMax=200.0, minWheelMms=20.0)
        vc.update(19.9, 0.0, 0.02)   # |19.9| < 20 → deadband
        assert vc.integral == pytest.approx(0.0)
        vc.update(20.0, 0.0, 0.02)   # |20.0| == 20 → NOT in deadband (>= check)
        # Note: C++ uses sp_abs < minWheelMms, so 20.0 is not in deadband
        assert vc.integral > 0.0


# ---------------------------------------------------------------------------
# Tests — output clamping
# ---------------------------------------------------------------------------

class TestOutputClamp:
    """Output is always in [-100, +100]."""

    def test_output_clamped_at_positive_100(self):
        """Very large setpoint: output clamped at 100."""
        vc = make_vc(kFF=1.0, kP=1.0, kI=0.0)
        out = vc.update(1000.0, 0.0, 0.02)
        assert out == pytest.approx(100.0)

    def test_output_clamped_at_negative_100(self):
        """Very large negative setpoint: output clamped at -100."""
        vc = make_vc(kFF=1.0, kP=1.0, kI=0.0)
        out = vc.update(-1000.0, 0.0, 0.02)
        assert out == pytest.approx(-100.0)

    def test_output_in_bounds_normal_operation(self):
        """Normal operating conditions stay within bounds."""
        vc = make_vc()
        for sp in [50.0, 100.0, 200.0, 300.0, -100.0, -200.0]:
            out = vc.update(sp, sp * 0.8, 0.02)
            assert -100.0 <= out <= 100.0, f"Out of bounds for sp={sp}: {out}"

    def test_dt_zero_returns_zero(self):
        """dt_s <= 0 is a no-op that returns 0."""
        vc = make_vc()
        out = vc.update(100.0, 50.0, 0.0)
        assert out == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# Tests — direction / sign handling
# ---------------------------------------------------------------------------

class TestDirection:
    """Output sign follows setpoint sign."""

    def test_positive_setpoint_positive_output(self):
        """Positive setpoint → positive PWM."""
        vc = make_vc(kFF=0.15, kP=0.0, kI=0.0)
        out = vc.update(100.0, 100.0, 0.02)
        assert out > 0.0

    def test_negative_setpoint_negative_output(self):
        """Negative setpoint → negative PWM."""
        vc = make_vc(kFF=0.15, kP=0.0, kI=0.0)
        out = vc.update(-100.0, -100.0, 0.02)
        assert out < 0.0

    def test_symmetry_forward_reverse(self):
        """Same magnitude: |forward output| ≈ |reverse output|."""
        vc_fwd = make_vc(kFF=0.15, kP=0.3, kI=0.0)
        vc_rev = make_vc(kFF=0.15, kP=0.3, kI=0.0)
        out_fwd = vc_fwd.update(150.0, 120.0, 0.02)
        out_rev = vc_rev.update(-150.0, -120.0, 0.02)
        assert out_fwd == pytest.approx(-out_rev, rel=1e-5)


# ---------------------------------------------------------------------------
# Tests — combined FF + PI
# ---------------------------------------------------------------------------

class TestCombined:
    """Full FF+PI scenario: output = kFF*|sp| + kP*err + I, all terms active."""

    def test_first_tick_ff_plus_pi(self):
        """On the first tick, output = kFF*|sp| + kP*err + kI*err*dt (integral accumulated within the tick)."""
        kFF, kP, kI = 0.15, 0.3, 0.05
        vc = make_vc(kFF=kFF, kP=kP, kI=kI, minWheelMms=10.0)
        sp, meas, dt = 200.0, 160.0, 0.02  # err = 40
        out = vc.update(sp, meas, dt)
        # The implementation updates the integrator within the first tick, then recomputes:
        # integral = kI * err * dt = 0.05 * 40 * 0.02 = 0.04
        # output = kFF*200 + kP*40 + 0.04 = 30 + 12 + 0.04 = 42.04
        err = sp - meas  # 40.0
        integral_after = kI * err * dt  # 0.04
        expected = kFF * abs(sp) + kP * err + integral_after
        assert out == pytest.approx(expected, rel=1e-4)

    def test_second_tick_with_integrator(self):
        """After one tick with error, second tick has non-zero integral contribution."""
        kFF, kP, kI = 0.0, 0.0, 0.1
        vc = make_vc(kFF=kFF, kP=kP, kI=kI, iMax=200.0, minWheelMms=10.0)
        sp, meas, dt = 100.0, 80.0, 0.02  # err=20
        vc.update(sp, meas, dt)  # tick 1: integral becomes kI*20*0.02 = 0.04
        out2 = vc.update(sp, meas, dt)  # tick 2: output includes integral
        assert out2 == pytest.approx(kI * 20.0 * dt * 2, rel=1e-4)
