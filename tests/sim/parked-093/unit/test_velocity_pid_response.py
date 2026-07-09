"""Velocity-PID response test (sprint 081, ticket 005): a commanded velocity
step's rise/overshoot/settle land in the same envelope ticket 001's bench
pass observed for a VEL-120 step on the default plant (settle ~128-142
mm/s) -- meaningful now that sim (Hal::SimMotor) and hardware
(Hal::NezhaMotor) share the exact same Hal::MotorVelocityPid class, per
ticket 081-001's extraction.

The bracket below is intentionally wide: only the PID LAW is shared between
sim and hardware, not the plant dynamics (the sim plant has no motor lag or
load by default), so this asserts convergence and "no oscillation blow-up"
in the SAME neighborhood the bench pass found, not the bench's exact
numbers (which would be over-fitting a test to one specific hardware run).
"""
import pytest

_TARGET = 120.0   # [mm/s] matches ticket 001's bench VEL-120 step


def test_velocity_step_settles_in_bench_envelope(sim):
    sim.command("DEV M 1 VEL 120")

    samples = []
    for _ in range(25):   # 25 * 120 ms = 3000 ms -- matches dev_exercise.py's
                           # default --settle-time window
        sim.tick_for(120)
        samples.append(sim.vel()[0])

    peak = max(samples)
    settle = samples[-1]

    # No oscillation blow-up: the transient peak stays well under 2x target
    # and velocity never reverses sign (a real PID blow-up would either
    # runaway upward without bound or oscillate through zero).
    assert 0.0 < peak < _TARGET * 2.0
    assert all(v >= 0.0 for v in samples)

    # Settle value lands in the same envelope ticket 001's bench pass
    # observed for a VEL-120 step (settle ~128-142 mm/s) -- generously
    # bracketed since only the PID law is shared, not the plant dynamics.
    assert 90.0 <= settle <= 170.0

    # Convergence: given enough additional time the PID reaches the
    # commanded target with no steady-state error -- proves the response
    # above is settling toward the setpoint, not oscillating around some
    # other value indefinitely.
    sim.tick_for(20000)
    assert sim.vel()[0] == pytest.approx(_TARGET, abs=15.0)


def test_velocity_step_anti_windup_holds_under_saturating_target(sim):
    """A target far beyond the plant's own max speed must still converge
    (clamped by the plant, not by an unbounded, wound-up integrator)."""
    sim.command("DEV M 1 VEL 900")   # > PhysicsWorld::kNominalMaxSpeed (400 mm/s)

    sim.tick_for(10000)
    vel_l, _ = sim.vel()

    # Cannot exceed the plant's own physical max speed; must not diverge.
    assert 0.0 < vel_l <= 420.0
