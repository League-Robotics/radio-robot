"""OTOS-error injection tests (sprint 081, ticket 006): noise/scale/drift
knobs on ``Hal::SimOdometer``'s own accumulator (``source/hal/sim/
sim_odometer.h``'s ``setLinearNoiseSigma``/``setYawNoiseSigma``/
``setLinearScaleError``/``setAngularScaleError``/``setLinearDriftPerTick``/
``setYawDriftPerTick``) -- confirming the accumulator diverges from true
pose, and does so INDEPENDENTLY of ``PhysicsWorld``'s encoder-report-error
model (the "two independent accumulators" design ``sim_odometer.h``'s own
file header documents: "this class's noise/scale/drift knobs are entirely
separate state from Hal::PhysicsWorld's encoder error model ... the two
error models NEVER share state").

Ported (adapted) from ``tests_old/simulation/system/
test_069_004_encoder_otos_knobs.py`` and ``test_069_knob_telemetry_sweep.py``'s
``OTOS_ERROR`` group -- re-derived against the ``Sim`` wrapper (ticket 005)
and ``DEV M DUTY`` open-loop driving rather than ``SIMSET``/``T``/``RT``
(this ticket's hard requirement forbids ``SIMSET``/``SIMGET``; ``T``/``RT``
have no equivalent in the new tree's dev-loop-only command surface).

---------------------------------------------------------------------------
Legacy suite disposition (this file + its two siblings,
test_encoder_error_injection.py / test_stiction_and_motor_lag.py cover the
full "port the high-value subset" scope of ticket 006):

PORTED (adapted onto the Sim wrapper / DEV M DUTY, not SIMSET/T/RT):
  - test_physics_world_basic.py           -> informs the bit-exact
    "does not perturb" assertions across all three new files (the
    golden-TLM sub-step separation).
  - test_physics_world_stiction.py        -> test_stiction_and_motor_lag.py
  - test_069_004_encoder_otos_knobs.py /
    test_069_knob_telemetry_sweep.py's
    ENCODER_REPORT_ERROR + OTOS_ERROR groups
                                           -> test_encoder_error_injection.py
                                              / this file

EXCLUDED -- EKF/fusion-dependent (no firmware consumer of OTOS exists yet in
the new tree; architecture-update.md's "OTOS gap" note). These legacy files
assert against `pose=` (the EKF-fused estimate) and/or `encpose=` (the
encoder-only dead-reckoning accumulator fed by `Odometry`), neither of which
has an equivalent in the new tree -- there is no `Odometry`/`Planner`/EKF
loop, and TLM has no `pose=`/`encpose=` field:
  - test_069_004_encoder_otos_knobs.py's `pose=`/`get_pose()` assertions
    (the file's own two tests -- both also asserted `encpose=` divergence).
  - test_068_004_zero_error_three_pose_agreement.py (encpose=/otos=/pose=
    three-way agreement across a whole tour).
  - test_070_004_sim_errors_from_cal.py (TestGUI "Sim Errors from
    Calibration" button, itself EKF-fusion-consuming).
  - test_otos_fusion.py (a pure-Python mirror of `Odometry::correct()`'s
    complementary filter -- the class itself does not exist in the new
    tree).
  - test_069_knob_telemetry_sweep.py's own `EKF_KEYS_OUT_OF_SCOPE` dict and
    every `pose=`-asserting line in its `_check_scrub_or_otos_error`/
    `_check_encoder_report_error` helpers.

EXCLUDED -- no equivalent command/feature exists in the new tree's
dev-loop-only surface (a command-surface gap, not an EKF gap):
  - test_rt_slip.py, test_073_002_setslip_decouple.py's SIMSET/RT-driven
    tests, test_069_rt_90deg_body_scrub.py, test_073_rt_angle_sweep.py,
    test_072_00{1,3,4}_*.py (D-drive/StopCondition/Planner terminal-
    completion) -- all drive `T`/`RT`/`D`/`VW`/`X`, none of which exist in
    `source/commands/` yet (only `DEV M`/`DEV DT`/`DEV WD`/liveness/SET/
    GET/TLM). `RobotConfig.rotationalSlip`/`SET rotSlip` (the firmware
    calibration constant these tests compensate for) also has no
    equivalent -- there is no `Planner::beginRotation()` arc-inflation to
    compensate.
  - test_069_004_encoder_otos_knobs.py / test_069_knob_telemetry_sweep.py's
    `GROUND_TRUTH_SCRUB` (bodyRotScrub/bodyLinScrub/trackwidthMm) and
    `PHYSICAL_ASYMMETRY` (motorOffsetL/R) groups -- driven via `RT`/`T` and
    read back via `SIMSET`/`SIMGET`, both unavailable; `bodyRotScrub`/
    `bodyLinScrub` themselves DO have new-tree equivalents
    (`set_body_rotational_scrub`/`set_body_linear_scrub`) and are already
    exercised by `test_physics_world_body_scrub.py`'s spiritual successor
    coverage in `test_errored_observation.py`'s zero-knob reset list; a
    dedicated behavioral port was judged lower-value than the encoder/OTOS/
    stiction trio this ticket's acceptance criteria name explicitly.
  - test_sim_otos_lever_arm.py, test_sim_otos_heading_reset.py,
    test_sim_hardware_bench_otos.py, test_bench_otos.py -- lever-arm/
    bench-OTOS/lift machinery `sim_odometer.h`'s own file header says was
    deliberately NOT ported forward from source_old this sprint ("none of
    that has an equivalent wire surface in the new tree yet ... porting it
    forward here would be unvalidatable scope creep, not a faithful minimal
    port") -- there is nothing left in `source/` for a test of it to
    exercise.
  - test_fit_sim_error_model.py -- a calibration-fitting TOOL test
    (`scipy`-based JSONL record/fit/replay), not an error-model behavior
    test; out of this ticket's "error-injection test suite" scope.

Incidental scaffolding NOT ported (never test-worthy on its own): `rogo.py`
(a CLI helper), and every registry-sweep meta-test keyed off `SIMGET`'s wire
enumeration (`test_no_unmapped_simset_keys`/`test_no_stale_mapped_keys`) --
`SIMSET`/`SIMGET` do not exist in the new tree at all (this ticket's own
hard requirement forbids introducing them), so there is no registry to
sweep.
---------------------------------------------------------------------------
"""
import pytest

_NOMINAL_MAX_SPEED = 400.0   # [mm/s] Hal::PhysicsWorld::kNominalMaxSpeed default


def _drive_straight(s, duty: int = 70, ms: int = 1000) -> None:
    s.command(f"DEV M 1 DUTY {duty}")
    s.command(f"DEV M 2 DUTY {duty}")
    s.tick_for(ms)


def _spin_in_place(s, duty: int = 30, ms: int = 240) -> None:
    """In-place rotation: opposite-sign duty on both wheels. Kept short
    (240 ms ~= 22 degrees at duty=30) so the true heading never approaches
    the +-pi wrap boundary either PhysicsWorld or SimOdometer applies to
    their own accumulator."""
    s.command(f"DEV M 1 DUTY {duty}")
    s.command(f"DEV M 2 DUTY {-duty}")
    s.tick_for(ms)


def test_linear_scale_error_diverges_otos_from_true_pose_proportionally(sim):
    """``setLinearScaleError`` inflates OTOS's reported straight-line
    distance relative to the plant's true pose by ~the configured
    fraction, while the reported encoder (a disjoint channel) stays
    bit-for-bit agreed with true."""
    sim.set_otos_linear_scale_error(0.10)
    _drive_straight(sim)

    true_x, true_y, _true_h = sim.true_pose()
    otos_x, otos_y, _otos_h = sim.otos_pose()

    assert otos_x == pytest.approx(true_x * 1.10, rel=0.02)
    assert abs(otos_y) < 1.0 and abs(true_y) < 1.0

    true_l, true_r = sim.true_wheel_travel()
    rep_l, rep_r = sim.enc()
    assert rep_l == true_l and rep_r == true_r


def test_angular_scale_error_diverges_otos_heading_from_true_heading(sim):
    """``setAngularScaleError`` inflates OTOS's reported heading change
    relative to the plant's true heading by ~the configured fraction,
    isolated via an in-place spin (dL+dR == 0 exactly every tick, so the
    linear term never contributes)."""
    sim.set_otos_angular_scale_error(0.10)
    _spin_in_place(sim)

    _true_x, _true_y, true_h = sim.true_pose()
    _otos_x, _otos_y, otos_h = sim.otos_pose()

    assert true_h != 0.0, "the spin did not actually rotate the plant"
    assert otos_h == pytest.approx(true_h * 1.10, rel=0.02)


def test_linear_noise_causes_nonzero_divergence(sim):
    """A nonzero linear-noise sigma perturbs OTOS's reported straight-line
    position away from exact agreement with true. Deterministic (fixed
    ``std::mt19937{43u}`` seed, ticket 003/005's determinism gate) rather
    than flaky, even though the perturbation itself is statistical."""
    sim.set_otos_linear_noise(0.05)
    _drive_straight(sim)

    true_x, true_y, _true_h = sim.true_pose()
    otos_x, otos_y, _otos_h = sim.otos_pose()
    assert (otos_x, otos_y) != (true_x, true_y)


def test_yaw_noise_causes_nonzero_divergence(sim):
    """A nonzero yaw-noise sigma perturbs OTOS's reported heading away
    from exact agreement with true, isolated via an in-place spin."""
    sim.set_otos_yaw_noise(0.05)
    _spin_in_place(sim)

    _true_x, _true_y, true_h = sim.true_pose()
    _otos_x, _otos_y, otos_h = sim.otos_pose()
    assert otos_h != true_h


def test_linear_drift_accumulates_while_stationary(sim):
    """``Hal::SimOdometer::tick()`` adds ``linearDriftPerTick_`` to its own
    accumulator EVERY tick, unconditionally -- even with the chassis
    completely at rest (no ``DEV M`` command ever issued in this test, so
    the true pose stays pinned at the origin throughout). This is the
    clearest possible demonstration of the two-independent-accumulators
    design: nothing here ever writes to a motor, so PhysicsWorld's true
    AND reported encoder accumulators are both untouched, while OTOS
    visibly drifts."""
    sim.set_otos_linear_drift(2.0)   # additive per tick

    sim.tick_for(240)   # 10 ticks @ 24 ms (the default step)
    x_after_10, _y, _h = sim.otos_pose()

    sim.tick_for(240)   # 10 more ticks
    x_after_20, _y, _h = sim.otos_pose()

    true_x, true_y, true_h = sim.true_pose()
    enc_l, enc_r = sim.enc()

    assert x_after_10 > 0.0
    assert x_after_20 > x_after_10   # accumulates monotonically with elapsed ticks

    assert true_x == 0.0 and true_y == 0.0 and true_h == 0.0
    assert enc_l == 0.0 and enc_r == 0.0


def test_yaw_drift_accumulates_while_stationary(sim):
    """Same as the linear-drift case above, for ``setYawDriftPerTick`` --
    OTOS's reported heading walks away from zero purely from elapsed
    ticks, with true heading and the reported encoder both pinned at zero
    the entire test."""
    sim.set_otos_yaw_drift(0.01)   # [rad] additive per tick

    sim.tick_for(240)
    h_after_10 = sim.otos_pose()[2]

    sim.tick_for(240)
    h_after_20 = sim.otos_pose()[2]

    true_x, true_y, true_h = sim.true_pose()
    enc_l, enc_r = sim.enc()

    assert h_after_10 > 0.0
    assert h_after_20 > h_after_10

    assert true_x == 0.0 and true_y == 0.0 and true_h == 0.0
    assert enc_l == 0.0 and enc_r == 0.0


def test_otos_error_does_not_perturb_reported_encoder(sim):
    """A LARGE OTOS error (noise + scale + drift, every one of the six
    knobs this ABI exposes) leaves ``PhysicsWorld``'s reported encoder
    accumulator bit-for-bit equal to true -- the encoder-side half of the
    two-independent-accumulators design (the encoder-side half of the
    isolation lives in test_encoder_error_injection.py's own
    ``test_encoder_error_does_not_perturb_true_pose_or_otos_pose``)."""
    sim.set_otos_linear_noise(0.05)
    sim.set_otos_yaw_noise(0.05)
    sim.set_otos_linear_scale_error(0.15)
    sim.set_otos_angular_scale_error(0.15)
    sim.set_otos_linear_drift(3.0)
    sim.set_otos_yaw_drift(0.02)

    _drive_straight(sim, duty=70, ms=1500)

    true_x, true_y, _true_h = sim.true_pose()
    otos_x, otos_y, _otos_h = sim.otos_pose()
    assert (otos_x, otos_y) != pytest.approx((true_x, true_y), abs=1e-2), (
        "the OTOS error knobs did not actually perturb otos_pose() -- this "
        "test's premise (a genuinely large error) did not hold"
    )

    true_l, true_r = sim.true_wheel_travel()
    rep_l, rep_r = sim.enc()
    assert rep_l == true_l
    assert rep_r == true_r


def test_zeroing_otos_knobs_restores_bit_for_bit_agreement(sim):
    """Explicitly re-zeroes all six OTOS-error knobs this suite exercises
    -- a suite-scoped regression peg distinct from
    test_errored_observation.py's all-13-knobs version -- proving
    "zeroed" behaves identically to "never touched" for this suite's own
    knobs specifically."""
    sim.set_otos_linear_noise(0.0)
    sim.set_otos_yaw_noise(0.0)
    sim.set_otos_linear_scale_error(0.0)
    sim.set_otos_angular_scale_error(0.0)
    sim.set_otos_linear_drift(0.0)
    sim.set_otos_yaw_drift(0.0)

    _drive_straight(sim)

    true_x, true_y, true_h = sim.true_pose()
    otos_x, otos_y, otos_h = sim.otos_pose()
    assert otos_x == pytest.approx(true_x, abs=1e-2)
    assert otos_y == pytest.approx(true_y, abs=1e-2)
    assert otos_h == pytest.approx(true_h, abs=1e-4)
