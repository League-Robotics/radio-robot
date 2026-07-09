"""093/094 teardown: `S`/`STOP` wheel-motion assertions, parked.

This file is the drive-motion half of `tests/sim/unit/test_bare_loop_commands.py`
(093-003's focused four-verb suite), split out and parked here because the
motor/hardware inbound message queues (`Rt::Blackboard`'s `motorIn[]`/
`motorResetIn[]`/`hardwareBroadcastIn`) were removed from the Blackboard
(see `source/runtime/blackboard.h`'s file header) as prep work toward
"Drivetrain owns its motors" (sprint 094). `Rt::MainLoop::tick()` no longer
has a `routeOutputs()` step at all -- `Subsystems::Drivetrain`'s own held
output currently goes nowhere, so `S` still replies `OK drive l=... r=...`
(the command-parsing/reply half of the surface is untouched) but the plant
never actually moves: `Subsystems::Hardware::tick()` no longer receives any
command from the Blackboard, real or routed.

The three tests below assert on `sim.vel()`/`sim.true_velocity()`/
`sim.pwm()` after `S`/`STOP` -- i.e. they need the severed drive path to
actually reach the plant. They are PARKED, not deleted (mirrors
`tests/sim/parked-093/`'s own precedent) -- restore them (move back to
`tests/sim/unit/`) once the Drivetrain writes its own motors directly
(sprint 094's own tickets) and `S` can once again be proven to move the
plant, not just parse and reply.

The command-reply-only tests (`PING`/`HELLO`/`ERR unknown` for an
unregistered verb) from the original file are UNAFFECTED by this and stay
live at `tests/sim/unit/test_bare_loop_commands.py`.
"""
from __future__ import annotations

import pytest

_DRIVE_TARGET = 150.0  # [mm/s] -- comfortably inside the plant's ~400 mm/s
                       # nominal max speed (test_velocity_pid_response.py's
                       # own convention), well clear of PID saturation.


def test_s_drives_both_wheels_to_commanded_targets_and_direction(sim):
    """`S <l> <r>` (same-sign) drives both wheels to the commanded speed, in
    the commanded (forward) direction -- checked via the plant's own
    reported (`vel()`) AND true (`true_velocity()`) per-wheel velocity, not
    just the `OK` reply."""
    l = r = int(_DRIVE_TARGET)
    reply = sim.command(f"S {l} {r}")
    assert reply.strip() == f"OK drive l={l} r={r}"

    sim.tick_for(3000)   # settle -- see test_velocity_pid_response.py's
                         # own 3 s bracket for a comparable step target.

    vel_l, vel_r = sim.vel()
    true_vel_l, true_vel_r = sim.true_velocity()

    assert vel_l == pytest.approx(_DRIVE_TARGET, abs=30.0)
    assert vel_r == pytest.approx(_DRIVE_TARGET, abs=30.0)
    assert true_vel_l == pytest.approx(_DRIVE_TARGET, abs=30.0)
    assert true_vel_r == pytest.approx(_DRIVE_TARGET, abs=30.0)
    assert vel_l > 0.0 and vel_r > 0.0


def test_s_with_differing_sign_wheels_spins_them_opposite_directions(sim):
    """`S <l> <r>` with `l` and `r` of opposite sign spins the two wheels in
    opposite directions (an in-place-turn command shape), proving `S`
    applies each wheel's target independently rather than clamping/mirroring
    them to a common sign."""
    l = int(_DRIVE_TARGET)
    r = -int(_DRIVE_TARGET)
    reply = sim.command(f"S {l} {r}")
    assert reply.strip() == f"OK drive l={l} r={r}"

    sim.tick_for(3000)

    vel_l, vel_r = sim.vel()
    true_vel_l, true_vel_r = sim.true_velocity()

    assert vel_l > 50.0
    assert vel_r < -50.0
    assert true_vel_l > 50.0
    assert true_vel_r < -50.0


def test_stop_neutralizes_both_wheels_regardless_of_prior_drive_state(sim):
    """`STOP` neutralizes both wheels even after an active `S` drive -- the
    exact regression 093-001 fixed (motion_commands.cpp's `handleStop`
    header comment): a NEUTRAL command posted with `standby=true` was
    silently dropped by `routeOutputs()`'s `drivetrain_.active()` gate,
    leaving the wheels spinning at their last commanded speed. Asserted
    hard: velocity AND pwm must land near zero and STAY near zero across
    several subsequent ticks, not just dip momentarily."""
    reply = sim.command(f"S {int(_DRIVE_TARGET)} {int(_DRIVE_TARGET)}")
    assert reply.strip() == f"OK drive l={int(_DRIVE_TARGET)} r={int(_DRIVE_TARGET)}"

    sim.tick_for(2000)
    vel_l, vel_r = sim.vel()
    assert vel_l > 50.0 and vel_r > 50.0   # confirm it was genuinely driving
                                           # before STOP -- otherwise a
                                           # neutralize assertion below would
                                           # be vacuous.

    reply = sim.command("STOP")
    assert reply.strip() == "OK stop"

    # Neutralize within a reasonable settle window...
    sim.tick_for(1000)

    # ...and STAY neutralized for several more passes (not a momentary dip
    # that creeps back up, e.g. from a stale, un-cleared drivetrain target).
    for _ in range(5):
        sim.tick_for(200)
        vel_l, vel_r = sim.vel()
        pwm_l, pwm_r = sim.pwm()
        assert vel_l == pytest.approx(0.0, abs=5.0)
        assert vel_r == pytest.approx(0.0, abs=5.0)
        assert pwm_l == pytest.approx(0.0, abs=1.0)
        assert pwm_r == pytest.approx(0.0, abs=1.0)
