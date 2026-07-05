"""Stiction/motor-lag tests (sprint 081, ticket 006): ``Hal::PhysicsWorld``'s
per-tick PWM dead-zone gate (``setStictionPwm``/``sim_set_stiction``) and
optional first-order motor-response filter (``setMotorLag``/
``sim_set_motor_lag``) -- ported from ``tests_old/simulation/unit/
test_physics_world_stiction.py`` (ticket 072-001 there), re-derived against
the ``Sim`` wrapper (ticket 005) and ``DEV M <port> DUTY <duty>`` open-loop
driving rather than a standalone compiled ``PhysicsWorld`` harness. See
``test_otos_error_injection.py``'s module docstring for the full legacy-
suite disposition note (what else from tickets 069-073 was excluded and
why) -- the legacy stiction suite's OWN system-level sibling
(``test_072_001_stiction_d_drive_repro.py`` and its 003/004 successors) is
excluded there: it drives the `D` motion-planner verb and `Planner`/
`VelocityController`/`StopCondition` machinery that does not exist yet in
this tree's dev-loop-only command surface. Only the plant-level gate/lag
behavior this file covers -- true, not command-surface-dependent -- ports.

Both knobs act on ``PhysicsWorld``'s TRUE velocity/encoder accumulators
(sub-step A, ahead of the reported-encoder-error model
test_encoder_error_injection.py covers), so every assertion below reads
``true_velocity()``/``true_wheel_travel()``, not ``enc()``/``vel()``.
"""
import pytest

from firmware import Sim

_NOMINAL_MAX_SPEED = 400.0   # [mm/s] Hal::PhysicsWorld::kNominalMaxSpeed default
_WATCHDOG_WIDE_WINDOW = 60000   # [ms] -- see tests/sim/conftest.py


def test_stiction_gate_boundary_positive_duty(build_lib):
    """``|pwm|`` exactly AT the configured ``stictionPwm`` threshold does
    NOT gate (the algebraic formula applies in full); one unit BELOW DOES
    gate (true velocity forced to exactly 0). Ported from
    test_physics_world_stiction.py's positive-duty gate-boundary case."""
    with Sim() as at_thresh, Sim() as below_thresh:
        for s in (at_thresh, below_thresh):
            s.command(f"DEV WD {_WATCHDOG_WIDE_WINDOW}")
            s.set_stiction(2, 20.0)

        at_thresh.command("DEV M 1 DUTY 20")
        at_thresh.command("DEV M 2 DUTY 20")
        at_thresh.tick_for(240)

        below_thresh.command("DEV M 1 DUTY 19")
        below_thresh.command("DEV M 2 DUTY 19")
        below_thresh.tick_for(240)

        expected_vel = (20 / 100.0) * _NOMINAL_MAX_SPEED   # 80 mm/s

        vel_at_l, vel_at_r = at_thresh.true_velocity()
        vel_below_l, vel_below_r = below_thresh.true_velocity()

        assert vel_at_l == pytest.approx(expected_vel, rel=0.01)
        assert vel_at_r == pytest.approx(expected_vel, rel=0.01)
        assert vel_below_l == 0.0
        assert vel_below_r == 0.0


def test_stiction_gate_boundary_negative_duty(build_lib):
    """Same boundary behavior for negative duty -- ``|pwm|`` gates, not the
    sign of pwm."""
    with Sim() as at_thresh, Sim() as below_thresh:
        for s in (at_thresh, below_thresh):
            s.command(f"DEV WD {_WATCHDOG_WIDE_WINDOW}")
            s.set_stiction(2, 20.0)

        at_thresh.command("DEV M 1 DUTY -20")
        at_thresh.command("DEV M 2 DUTY -20")
        at_thresh.tick_for(240)

        below_thresh.command("DEV M 1 DUTY -19")
        below_thresh.command("DEV M 2 DUTY -19")
        below_thresh.tick_for(240)

        expected_vel = (-20 / 100.0) * _NOMINAL_MAX_SPEED   # -80 mm/s

        vel_at_l, vel_at_r = at_thresh.true_velocity()
        vel_below_l, vel_below_r = below_thresh.true_velocity()

        assert vel_at_l == pytest.approx(expected_vel, rel=0.01)
        assert vel_at_r == pytest.approx(expected_vel, rel=0.01)
        assert vel_below_l == 0.0
        assert vel_below_r == 0.0


def test_stiction_gate_is_stateless(sim):
    """A wheel gated to 0 on the CURRENT tick has no memory of a previous
    tick's nonzero motion -- the gate looks only at THIS tick's ``|pwm|``
    (a stateless per-tick dead-zone, not a persistent "stuck" latch)."""
    sim.set_stiction(2, 50.0)

    sim.command("DEV M 1 DUTY 80")
    sim.command("DEV M 2 DUTY 80")
    sim.tick_for(240)   # above threshold -- both wheels moving
    moving_vel_l, moving_vel_r = sim.true_velocity()
    assert moving_vel_l > 0.0 and moving_vel_r > 0.0
    enc_before_gate, _enc_before_r = sim.true_wheel_travel()

    sim.command("DEV M 1 DUTY 10")
    sim.command("DEV M 2 DUTY 10")
    sim.tick_for(24)   # exactly one more tick, below threshold
    gated_vel_l, gated_vel_r = sim.true_velocity()
    enc_after_gate, _enc_after_r = sim.true_wheel_travel()

    assert gated_vel_l == 0.0 and gated_vel_r == 0.0
    # No residual travel this tick -- the gate zeroed velocity outright, it
    # did not merely reduce it or retain a "still coasting" memory.
    assert enc_after_gate == enc_before_gate


def test_stiction_gate_is_per_wheel_independent(sim):
    """A stiction threshold configured on ONE side only gates that wheel
    but leaves the other side's response completely unaffected."""
    sim.set_stiction(0, 50.0)   # LEFT only; RIGHT stays at the default 0

    sim.command("DEV M 1 DUTY 10")
    sim.command("DEV M 2 DUTY 10")
    sim.tick_for(240)

    vel_l, vel_r = sim.true_velocity()
    expected_r = (10 / 100.0) * _NOMINAL_MAX_SPEED   # 40 mm/s, ungated

    assert vel_l == 0.0
    assert vel_r == pytest.approx(expected_r, rel=0.01)


def test_motor_lag_delays_convergence_toward_target(build_lib):
    """``motorLag`` tau > 0 converges true velocity toward the commanded
    target exponentially, tick by tick -- unlike the fixture default
    (tau <= 0), which reaches the full target on the very first tick
    (``physics_world.cpp``: tau <= 0 skips the ``expf()`` call entirely,
    ``vel == velTarget`` bit-for-bit, no ramp at all)."""
    with Sim() as no_lag, Sim() as with_lag:
        for s in (no_lag, with_lag):
            s.command(f"DEV WD {_WATCHDOG_WIDE_WINDOW}")
        with_lag.set_motor_lag(2, 200.0)   # tau, both wheels

        for s in (no_lag, with_lag):
            s.command("DEV M 1 DUTY 100")
            s.command("DEV M 2 DUTY 100")

        target = _NOMINAL_MAX_SPEED   # duty 100 -> full nominal speed

        no_lag.tick_for(24)
        assert no_lag.true_velocity()[0] == pytest.approx(target, rel=0.01)

        samples = []
        for _ in range(20):   # 20 * 24 ms = 480 ms ~= 2.4 * tau
            with_lag.tick_for(24)
            samples.append(with_lag.true_velocity()[0])

        # Starts well below target (a genuine lag, not an instant jump)...
        assert samples[0] < target * 0.5
        # ...rises monotonically...
        assert all(b >= a for a, b in zip(samples, samples[1:]))
        # ...and approaches, but a first-order lag toward a CONSTANT target
        # never overshoots, the target.
        assert all(v <= target + 1e-3 for v in samples)
        assert samples[-1] > target * 0.85   # 1 - e^-2.4 ~= 0.909


def test_motor_lag_tau_zero_is_explicit_noop(build_lib):
    """Setting ``motorLag`` explicitly to 0.0 behaves identically to never
    touching the knob -- both take the ``tau <= 0`` bit-for-bit no-op
    path (physics_world.cpp)."""
    with Sim() as default_sim, Sim() as explicit_zero:
        for s in (default_sim, explicit_zero):
            s.command(f"DEV WD {_WATCHDOG_WIDE_WINDOW}")
        explicit_zero.set_motor_lag(2, 0.0)

        for s in (default_sim, explicit_zero):
            s.command("DEV M 1 DUTY 65")
            s.command("DEV M 2 DUTY 65")
            s.tick_for(240)

        assert default_sim.true_velocity() == explicit_zero.true_velocity()
        assert default_sim.true_wheel_travel() == explicit_zero.true_wheel_travel()
