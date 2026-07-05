"""Errored-observation split tests (sprint 081, ticket 005): nonzero error
knobs make the REPORTED encoder/OTOS observation diverge from ground truth
by ~the configured amount; zeroing every knob restores bit-for-bit
agreement -- re-exercising ticket 003's own zero-error determinism gate
(true encoder == reported encoder == OTOS accumulator, bit-for-bit) through
the Python ctypes wrapper rather than only at the C++ harness level.

The encoder equality (true == reported at zero error) is a genuine
bit-for-bit guarantee for ANY drive pattern -- physics_world.cpp's
"GOLDEN-TLM" sub-steps A/A' reduce to the identical float32 expression when
every error knob is at its no-op default (scale error 0, slip 0, noise 0).
The OTOS equality is bit-for-bit only for a STRAIGHT drive (heading stays
exactly 0.0 throughout, making the odometer's cos/sin round-trip reduce to
an exact identity) -- see sim_odometer.cpp's tick(); this is why the
straight-drive script below is reused for both checks.
"""
import pytest

from firmware import Sim

_WATCHDOG_WIDE_WINDOW = 60000   # [ms] -- see tests/sim/conftest.py


def _drive_straight(s: Sim, duty: int = 80, ms: int = 500) -> None:
    s.command(f"DEV M 1 DUTY {duty}")
    s.command(f"DEV M 2 DUTY {duty}")
    s.tick_for(ms)


def test_zero_error_knobs_give_bit_for_bit_agreement(sim):
    """Fixture default: every knob already at its zero/no-op value."""
    _drive_straight(sim)

    true_l, true_r = sim.true_wheel_travel()
    rep_l, rep_r = sim.enc()
    assert rep_l == true_l
    assert rep_r == true_r

    true_x, true_y, true_h = sim.true_pose()
    otos_x, otos_y, otos_h = sim.otos_pose()
    assert otos_x == pytest.approx(true_x, abs=1e-2)
    assert otos_y == pytest.approx(true_y, abs=1e-2)
    assert otos_h == pytest.approx(true_h, abs=1e-4)


def test_nonzero_error_knobs_cause_proportional_divergence(build_lib):
    with Sim() as s:
        s.command(f"DEV WD {_WATCHDOG_WIDE_WINDOW}")
        s.set_enc_scale_error(2, 0.05)             # both wheels over-report 5%
        s.set_otos_linear_scale_error(0.05)         # OTOS over-reports 5% too

        _drive_straight(s)

        true_l, true_r = s.true_wheel_travel()
        rep_l, rep_r = s.enc()
        assert rep_l != true_l
        assert rep_r != true_r
        assert rep_l == pytest.approx(true_l * 1.05, rel=0.01)
        assert rep_r == pytest.approx(true_r * 1.05, rel=0.01)

        true_x, _true_y, _true_h = s.true_pose()
        otos_x, _otos_y, _otos_h = s.otos_pose()
        assert otos_x != true_x
        assert otos_x == pytest.approx(true_x * 1.05, rel=0.02)


def test_zeroing_every_knob_restores_bit_for_bit_agreement(build_lib):
    """Explicitly re-zeroes every error knob this ABI exposes (rather than
    relying on a fresh instance's implicit defaults) -- proves "zeroed"
    behaves identically to "never touched", the actual determinism gate."""
    with Sim() as s:
        s.command(f"DEV WD {_WATCHDOG_WIDE_WINDOW}")

        s.set_enc_scale_error(2, 0.0)
        s.set_enc_slip(2, 0.0)
        s.set_enc_noise(2, 0.0)
        s.set_stiction(2, 0.0)
        s.set_motor_lag(2, 0.0)
        s.set_body_rotational_scrub(1.0)
        s.set_body_linear_scrub(1.0)
        s.set_otos_linear_noise(0.0)
        s.set_otos_yaw_noise(0.0)
        s.set_otos_linear_scale_error(0.0)
        s.set_otos_angular_scale_error(0.0)
        s.set_otos_linear_drift(0.0)
        s.set_otos_yaw_drift(0.0)

        _drive_straight(s)

        true_l, true_r = s.true_wheel_travel()
        rep_l, rep_r = s.enc()
        assert rep_l == true_l
        assert rep_r == true_r

        true_x, true_y, true_h = s.true_pose()
        otos_x, otos_y, otos_h = s.otos_pose()
        assert otos_x == pytest.approx(true_x, abs=1e-2)
        assert otos_y == pytest.approx(true_y, abs=1e-2)
        assert otos_h == pytest.approx(true_h, abs=1e-4)


def test_enc_noise_knob_causes_some_divergence(sim):
    """A nonzero noise sigma perturbs the reported encoder away from exact
    equality (statistical, not proportional -- just confirm it moves)."""
    sim.set_enc_noise(2, 2.0)   # [mm] per-tick sigma, both wheels
    _drive_straight(sim)

    true_l, true_r = sim.true_wheel_travel()
    rep_l, rep_r = sim.enc()
    assert (rep_l, rep_r) != (true_l, true_r)
