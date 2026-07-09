"""093-003/094-005: focused four-verb suite for the bare wheel-driving
executive.

Sprint 093 gutted `Rt::MainLoop`/`Rt::CommandRouter::buildTable()` down to a
liveness family (`systemCommands()`: `PING`/`VER`/`HELP`/`ECHO`/`ID`/
`HELLO`) plus a two-verb motion family (`motionCommands()`: `S`/`STOP` --
see `source/commands/motion_commands.cpp`'s own 093-001 header comments).
Every other command family (`dev`/`telemetry`/`config`/`pose`/`otos`, and
`T`/`D`/`R`/`TURN`/`RT`/`G` within `motion_commands.cpp` itself) is
unregistered -- their handlers still exist on disk but nothing in
`buildTable()` calls them, so any of those wire verbs now replies
`ERR unknown` (see `tests/sim/parked-093/README.md` for the full inventory
of tests this obsoleted).

This file is the small, currently-green replacement for that lost coverage,
scoped to exactly what ticket 093-003's acceptance criteria ask for:
`PING`/`HELLO` still work, and a wire verb outside the four-verb-plus-
liveness surface is rejected with `ERR unknown`, proving the table
reduction rather than merely that the survivors work.

094-005 UN-PARKS `S`'s/`STOP`'s own PLANT-MOTION assertions (previously
split out to `tests/sim/parked-094/unit/test_bare_loop_drive_severed.py`
because the 093/094 teardown removed the motor/hardware inbound message
queues from the Blackboard before `Subsystems::Drivetrain` owned its own
motors). `Subsystems::Drivetrain` now holds a `Hardware&` (ticket 094-004)
and the bare loop wires `hardware.tick(now)` -> `drivetrain.tick(now,
bb.segmentIn, bb.driveIn)` -> commit (ticket 094-005) -- `S`/`STOP` reach
the plant again, so those assertions move back here.
"""
from __future__ import annotations

import pytest

_DRIVE_TARGET = 150.0  # [mm/s] -- comfortably inside the plant's ~400 mm/s
                       # nominal max speed (test_velocity_pid_response.py's
                       # own convention), well clear of PID saturation.


def test_ping_replies_ok(sim):
    """`PING` (systemCommands(), untouched by the gut) still replies `OK
    pong ...` -- part of the surviving liveness family, not the removed
    surface."""
    reply = sim.command("PING")
    assert reply.strip().startswith("OK pong")


def test_hello_replies_device_shaped(sim):
    """`HELLO` still replies the `DEVICE:...` identity banner (its own bare
    reply taxonomy, docs/protocol-v2.md section 3) -- the second of the two
    verbs architecture-update.md's Decision 3 names as the always-kept
    surface alongside `S`/`STOP`."""
    reply = sim.command("HELLO")
    assert reply.strip().startswith("DEVICE:")


@pytest.mark.parametrize("line", ["DEV WD 100", "GET drivetrainConfig"])
def test_verb_outside_the_live_surface_replies_err_unknown(sim, line):
    """A wire verb belonging to an un-wired command family (`dev`, `config`)
    replies exactly `ERR unknown` -- proving `buildTable()`'s family
    reduction (ticket 093-001), not merely that `S`/`STOP`/`PING`/`HELLO`
    themselves happen to work."""
    reply = sim.command(line)
    assert reply.strip() == "ERR unknown"


def test_s_drives_both_wheels_to_commanded_targets_and_direction(sim):
    """`S <l> <r>` (same-sign) drives both wheels to the commanded speed, in
    the commanded (forward) direction -- checked via the plant's own
    reported (`vel()`) AND true (`true_velocity()`) per-wheel velocity, not
    just the `OK` reply. 094-005: un-parked -- `S` posts to `bb.driveIn`,
    Drivetrain::tick() drains it (escape-hatch precedence) and stages the
    wheel targets directly through `hardware.motor(port).apply()`, flushed
    the following pass by `hardware.tick()`."""
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
    several subsequent ticks, not just dip momentarily.

    094-005: since no segment is ever queued in this test, Drivetrain stays
    in DIRECT mode throughout -- `STOP`'s NEUTRAL finds nothing in-flight to
    gracefully decelerate (the SEGMENT-mode graceful decel-to-zero path is
    exercised separately, see test_drivetrain.py's segment-mode scenarios),
    so it falls straight through to the pre-094 instant-neutral behavior
    this test asserts."""
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
