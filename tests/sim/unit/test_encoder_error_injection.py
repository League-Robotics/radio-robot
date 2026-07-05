"""Encoder-error injection tests (sprint 081, ticket 006): per-wheel scale
error, slip, and Gaussian noise on ``Hal::PhysicsWorld``'s REPORTED encoder
accumulator (``source/hal/sim/physics_world.h``'s ``setEncoderScaleError``/
``setEncoderSlip``/``setEncoderNoise``, ``sim_setters.h``'s
``setSimMotorScaleError``/``setSimMotorSlip``/``setSimMotorNoise``) --
ported from ``tests_old/simulation/unit/test_physics_world_basic.py`` (the
bit-exact sub-step isolation these assertions rely on) and
``tests_old/simulation/system/test_069_004_encoder_otos_knobs.py`` /
``test_069_knob_telemetry_sweep.py``'s ``ENCODER_REPORT_ERROR`` group (the
"reported diverges from true by ~the configured amount" behavior).

Adapted onto this sprint's much smaller command surface and the ``Sim``
wrapper (ticket 005), not copied: the legacy suites drove ``T``/``RT``
(closed-loop motion-planner verbs) and configured knobs via ``SIMSET`` --
neither exists in the new tree yet (``source/commands/dev_commands.cpp``
only has ``DEV M``/``DEV DT``/``DEV WD``; see this file's sibling
``test_otos_error_injection.py`` for the full legacy-suite disposition
note). Driving happens entirely via ``DEV M <port> DUTY <duty>`` --
open-loop, straight into ``PhysicsWorld::setActuator()`` -- so, unlike the
legacy closed-loop ``T``/``RT`` tests (whose commanded velocity servoed off
the very REPORTED encoder these knobs perturb, curving the plant's own TRUE
path as a side effect), the true chassis pose here is provably, not just
empirically, independent of every encoder-report-error knob:
``PhysicsWorld::update()``'s sub-step B (true pose integration) reads only
``velL``/``velR`` (sub-step A's un-erred algebraic velocity), never the
sub-step A' reported accumulator these knobs perturb. This lets
``test_encoder_error_does_not_perturb_true_pose_or_otos_pose`` below assert
a tighter isolation claim than the legacy suite's closed-loop version could.
"""
import pytest

_NOMINAL_MAX_SPEED = 400.0   # [mm/s] Hal::PhysicsWorld::kNominalMaxSpeed default


def _drive_straight(s, duty: int = 80, ms: int = 1000) -> None:
    s.command(f"DEV M 1 DUTY {duty}")
    s.command(f"DEV M 2 DUTY {duty}")
    s.tick_for(ms)


def test_scale_error_diverges_reported_encoder_left_only(sim):
    """``setEncoderScaleError(side=0, 0.05)`` (LEFT only) makes the
    reported LEFT travel diverge from true by ~5% (over-report) while
    RIGHT stays bit-for-bit agreed -- the per-wheel independence
    ``physics_world.h`` documents."""
    sim.set_enc_scale_error(0, 0.05)
    _drive_straight(sim)

    true_l, true_r = sim.true_wheel_travel()
    rep_l, rep_r = sim.enc()

    assert rep_l != true_l
    assert rep_l == pytest.approx(true_l * 1.05, rel=0.01)
    assert rep_r == true_r   # right side untouched -- bit-for-bit


def test_scale_error_diverges_reported_encoder_right_only_negative(sim):
    """Same knob, RIGHT side only, a NEGATIVE (under-report) error --
    confirms the sign, not just the magnitude, of the fractional error
    propagates correctly."""
    sim.set_enc_scale_error(1, -0.08)
    _drive_straight(sim)

    true_l, true_r = sim.true_wheel_travel()
    rep_l, rep_r = sim.enc()

    assert rep_l == true_l   # left side untouched -- bit-for-bit
    assert rep_r != true_r
    assert rep_r == pytest.approx(true_r * 0.92, rel=0.01)


def test_slip_diverges_reported_encoder_by_underreport_fraction(sim):
    """``setEncoderSlip`` (both wheels) under-reports true travel by the
    configured fraction -- a distinct error mode from scale error: slip is
    applied as ``(1 - encSlipSide)``, scale error as
    ``(1 + encScaleErrSide)`` (``physics_world.cpp`` sub-step A'), both
    multiplicative but opposite in sign convention."""
    sim.set_enc_slip(2, 0.10)   # both wheels, 10% of motion not registered
    _drive_straight(sim)

    true_l, true_r = sim.true_wheel_travel()
    rep_l, rep_r = sim.enc()

    assert rep_l == pytest.approx(true_l * 0.90, rel=0.01)
    assert rep_r == pytest.approx(true_r * 0.90, rel=0.01)


def test_noise_causes_deterministic_nonzero_divergence_both_sides(sim):
    """A nonzero noise sigma perturbs both reported channels away from
    exact agreement. Statistical, not proportional -- but the RNG streams
    are fixed-seed (ticket 003/005's determinism gate: a fresh
    ``PhysicsWorld`` always seeds ``std::mt19937{42u}`` on both sides), so
    this is a deterministic, reproducible divergence, not a flaky one."""
    sim.set_enc_noise(2, 3.0)   # both wheels
    _drive_straight(sim)

    true_l, true_r = sim.true_wheel_travel()
    rep_l, rep_r = sim.enc()

    assert rep_l != true_l
    assert rep_r != true_r


def test_encoder_error_does_not_perturb_true_pose_or_otos_pose(sim):
    """A LARGE encoder-report error (scale + slip + noise, all three
    knobs, both wheels) leaves the plant's true pose -- and therefore the
    OTOS accumulator, which samples true pose directly -- untouched.
    Sub-step B (chassis pose integration) never reads the sub-step A'
    reported accumulator these knobs perturb (``physics_world.cpp``); this
    is the "two independent accumulators" design ticket 003's
    architecture-update.md documents, exercised from the encoder side."""
    sim.set_enc_scale_error(2, 0.20)
    sim.set_enc_slip(2, 0.15)
    sim.set_enc_noise(2, 5.0)
    _drive_straight(sim, duty=80, ms=1500)

    true_l, _true_r = sim.true_wheel_travel()
    rep_l, _rep_r = sim.enc()
    assert rep_l != pytest.approx(true_l, rel=0.01), (
        "the error knobs did not actually perturb the reported encoder -- "
        "this test's premise (a genuinely large error) did not hold"
    )

    true_x, true_y, true_h = sim.true_pose()
    otos_x, otos_y, otos_h = sim.otos_pose()
    assert otos_x == pytest.approx(true_x, abs=1e-2)
    assert otos_y == pytest.approx(true_y, abs=1e-2)
    assert otos_h == pytest.approx(true_h, abs=1e-4)


def test_zeroing_encoder_knobs_restores_bit_for_bit_agreement(sim):
    """Explicitly re-zeroes just the three encoder-report knobs this suite
    exercises -- a tighter, suite-scoped regression peg than
    ``test_errored_observation.py``'s all-13-knobs version -- proving
    "zeroed" behaves identically to "never touched" for this suite's own
    knobs specifically."""
    sim.set_enc_scale_error(2, 0.0)
    sim.set_enc_slip(2, 0.0)
    sim.set_enc_noise(2, 0.0)
    _drive_straight(sim)

    true_l, true_r = sim.true_wheel_travel()
    rep_l, rep_r = sim.enc()
    assert rep_l == true_l
    assert rep_r == true_r
