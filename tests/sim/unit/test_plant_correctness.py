"""Plant-correctness tests (sprint 081, ticket 005): commanded drive/turn
geometry matches Hal::PhysicsWorld's own true-pose integration within its
numerical tolerance.

Drives individual motors directly via ``DEV M <n> DUTY <duty>`` (open-loop,
no PID involved) rather than through the Drivetrain, so the expected
geometry is closed-form: true per-wheel velocity is exactly
``(duty/100) * PhysicsWorld::kNominalMaxSpeed`` (400 mm/s default, no slip/
noise/offset error at fixture defaults) and the pose integrator is the
midpoint-arc formula in source/hal/sim/physics_world.cpp -- see that file's
``update()``. Straight (equal-sign duty on both wheels) and in-place-turn
(opposite-sign duty) are the two canonical cases: straight leaves heading
at 0 and displacement purely along x; an in-place turn leaves (x, y)
unchanged (the (dL+dR) term is exactly zero every tick) and heading a pure
linear function of time.
"""
import pytest

_NOMINAL_MAX_SPEED = 400.0   # [mm/s] Hal::PhysicsWorld::kNominalMaxSpeed default
_TRACKWIDTH = 150.0          # [mm] Hal::PhysicsWorld::kDefaultTrackwidth default


def test_straight_drive_matches_expected_displacement(sim):
    """Equal DUTY on both wheels: pure straight-line travel, no heading change."""
    sim.command("DEV M 1 DUTY 100")
    sim.command("DEV M 2 DUTY 100")
    sim.tick_for(2400)   # [ms] = 100 * the default 24 ms step, no dropped remainder

    x, y, h = sim.true_pose()
    expected_x = _NOMINAL_MAX_SPEED * 2.4   # [mm] 400 mm/s * 2.4 s

    assert x == pytest.approx(expected_x, rel=0.01)
    assert abs(y) < 1.0
    assert abs(h) < 1e-3


def test_reverse_drive_matches_expected_displacement(sim):
    """Equal, NEGATIVE DUTY on both wheels: straight-line travel backwards."""
    sim.command("DEV M 1 DUTY -60")
    sim.command("DEV M 2 DUTY -60")
    sim.tick_for(1200)   # [ms] = 50 * 24 ms, no dropped remainder

    x, y, h = sim.true_pose()
    expected_x = -0.6 * _NOMINAL_MAX_SPEED * 1.2   # [mm] -240 mm/s * 1.2 s

    assert x == pytest.approx(expected_x, rel=0.01)
    assert abs(y) < 1.0
    assert abs(h) < 1e-3


def test_in_place_turn_matches_expected_heading(sim):
    """Opposite-sign DUTY: in-place rotation -- x/y stay at the origin,
    heading grows linearly at the commanded angular rate."""
    sim.command("DEV M 1 DUTY 50")
    sim.command("DEV M 2 DUTY -50")
    sim.tick_for(240)   # [ms] = 10 * 24 ms, no dropped remainder

    x, y, h = sim.true_pose()
    vel_l = 0.5 * _NOMINAL_MAX_SPEED
    vel_r = -0.5 * _NOMINAL_MAX_SPEED
    expected_h = ((vel_r - vel_l) / _TRACKWIDTH) * 0.24   # [rad] angular rate * 0.24 s

    assert h == pytest.approx(expected_h, rel=0.01)
    assert abs(x) < 0.5
    assert abs(y) < 0.5


def test_true_wheel_travel_matches_true_velocity_times_time(sim):
    """True per-wheel encoder travel accumulates true velocity * elapsed
    time exactly (the golden-TLM sub-step A expression, physics_world.cpp)."""
    sim.command("DEV M 1 DUTY 80")
    sim.command("DEV M 2 DUTY 80")
    sim.tick_for(1200)   # [ms] = 50 * 24 ms, no dropped remainder

    enc_l, enc_r = sim.true_wheel_travel()
    vel_l, vel_r = sim.true_velocity()
    expected = 0.8 * _NOMINAL_MAX_SPEED * 1.2   # [mm] 320 mm/s * 1.2 s

    assert enc_l == pytest.approx(expected, rel=0.01)
    assert enc_r == pytest.approx(expected, rel=0.01)
    assert vel_l == pytest.approx(0.8 * _NOMINAL_MAX_SPEED, rel=0.01)
    assert vel_r == pytest.approx(0.8 * _NOMINAL_MAX_SPEED, rel=0.01)
