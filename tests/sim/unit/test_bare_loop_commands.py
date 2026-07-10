"""093-003/094-005/097-006/097-008: focused STOP-plus-binary-drive suite for
the bare wheel-driving executive.

Sprint 093 gutted `Rt::MainLoop`/`Rt::CommandRouter::buildTable()` down to a
liveness family plus a small motion family. 097-006 (architecture-update-r2.md
Decision 9, pure-binary firmware) went further: the text `S` verb itself is
now DELETED (not merely unregistered the way 093-001 left T/D/R/TURN/RT/G) --
its binary parity is the `drive` arm (`source/commands/binary_channel.cpp`,
095, hardware-bench-smoke-tested), exercised below via
`_binary_envelope.send_drive()`. `systemCommands()` now registers only
`PING`/`HELLO` (VER/HELP/ECHO/ID deleted); `motionCommands()` now registers
only `STOP` (S/D/T/RT/MOVE/MOVER/QLEN deleted by 097-006, TLM deleted by
097-008); `telemetryCommands()` now registers nothing at all (STREAM/SNAP
deleted by 097-008). Every other command family (`dev`/`config`/`pose`/
`otos`) stays unregistered as before -- see `tests/sim/parked-093/README.md`
for the full inventory of tests this obsoleted originally.

This file is the small, currently-green replacement for that lost coverage:
`PING`/`HELLO` still work as text, `STOP` still works as text, `S`'s own
plant-motion assertions now go through the binary `drive` arm instead, and
a wire verb outside the live text surface is rejected with `ERR unknown`,
proving the table reduction rather than merely that the survivors work.

094-005 UN-PARKS `S`'s/`STOP`'s own PLANT-MOTION assertions (previously
split out to `tests/sim/parked-094/unit/test_bare_loop_drive_severed.py`
because the 093/094 teardown removed the motor/hardware inbound message
queues from the Blackboard before `Subsystems::Drivetrain` owned its own
motors). `Subsystems::Drivetrain` now holds a `Hardware&` (ticket 094-004)
and the bare loop wires `hardware.tick(now)` -> `drivetrain.tick(now,
bb.segmentIn, bb.driveIn)` -> commit (ticket 094-005) -- `S`/`STOP` reach
the plant again, so those assertions move back here; 097-006 re-points the
drive half of them at the binary `drive` arm, `STOP` unchanged.
"""
from __future__ import annotations

import pytest

from _binary_envelope import send_drive

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


@pytest.mark.parametrize("line", [
    "DEV WD 100", "GET drivetrainConfig",
    # 097-008: STREAM/SNAP/TLM are DELETED (not merely left unregistered
    # the way DEV/GET always were) -- proving acceptance criterion 1
    # ("STREAM/SNAP are no longer registered as text verbs") at the wire
    # level, alongside the C++-level proof that their handler functions no
    # longer exist at all (telemetry_commands.cpp/motion_commands.cpp).
    "STREAM 50", "SNAP", "TLM",
])
def test_verb_outside_the_live_surface_replies_err_unknown(sim, line):
    """A wire verb belonging to an un-wired or deleted command family
    (`dev`, `config`, the retired text telemetry family) replies exactly
    `ERR unknown` -- proving `buildTable()`'s family reduction (ticket
    093-001, extended by 097-008), not merely that `S`/`STOP`/`PING`/`HELLO`
    themselves happen to work."""
    reply = sim.command(line)
    assert reply.strip() == "ERR unknown"


def test_binary_drive_drives_both_wheels_to_commanded_targets_and_direction(sim):
    """097-006: binary parity for the deleted text `S <l> <r>` -- (same-sign)
    drives both wheels to the commanded speed, in the commanded (forward)
    direction -- checked via the plant's own reported (`vel()`) AND true
    (`true_velocity()`) per-wheel velocity, not just the `OK` reply.
    094-005: un-parked -- `drive{wheels}` posts to `bb.driveIn`,
    Drivetrain::tick() drains it (escape-hatch precedence) and stages the
    wheel targets directly through `hardware.motor(port).apply()`, flushed
    the following pass by `hardware.tick()` -- the SAME path text `S` used
    to reach through `handleS()`."""
    l = r = int(_DRIVE_TARGET)
    reply = send_drive(sim, l, r)
    assert reply.WhichOneof("body") == "ok"

    sim.tick_for(3000)   # settle -- see test_velocity_pid_response.py's
                         # own 3 s bracket for a comparable step target.

    vel_l, vel_r = sim.vel()
    true_vel_l, true_vel_r = sim.true_velocity()

    assert vel_l == pytest.approx(_DRIVE_TARGET, abs=30.0)
    assert vel_r == pytest.approx(_DRIVE_TARGET, abs=30.0)
    assert true_vel_l == pytest.approx(_DRIVE_TARGET, abs=30.0)
    assert true_vel_r == pytest.approx(_DRIVE_TARGET, abs=30.0)
    assert vel_l > 0.0 and vel_r > 0.0


def test_binary_drive_with_differing_sign_wheels_spins_them_opposite_directions(sim):
    """097-006: binary parity for the deleted text `S <l> <r>` with `l` and
    `r` of opposite sign -- spins the two wheels in opposite directions (an
    in-place-turn command shape), proving `drive{wheels}` applies each
    wheel's target independently rather than clamping/mirroring them to a
    common sign."""
    l = int(_DRIVE_TARGET)
    r = -int(_DRIVE_TARGET)
    reply = send_drive(sim, l, r)
    assert reply.WhichOneof("body") == "ok"

    sim.tick_for(3000)

    vel_l, vel_r = sim.vel()
    true_vel_l, true_vel_r = sim.true_velocity()

    assert vel_l > 50.0
    assert vel_r < -50.0
    assert true_vel_l > 50.0
    assert true_vel_r < -50.0


def test_stop_neutralizes_both_wheels_regardless_of_prior_drive_state(sim):
    """`STOP` neutralizes both wheels even after an active binary `drive`
    -- the exact regression 093-001 fixed (motion_commands.cpp's
    `handleStop` header comment): a NEUTRAL command posted with
    `standby=true` was silently dropped by `routeOutputs()`'s
    `drivetrain_.active()` gate, leaving the wheels spinning at their last
    commanded speed. Asserted hard: velocity AND pwm must land near zero
    and STAY near zero across several subsequent ticks, not just dip
    momentarily.

    094-005: since no segment is ever queued in this test, Drivetrain stays
    in DIRECT mode throughout -- `STOP`'s NEUTRAL finds nothing in-flight to
    gracefully decelerate (the SEGMENT-mode graceful decel-to-zero path is
    exercised separately, see test_drivetrain.py's segment-mode scenarios),
    so it falls straight through to the pre-094 instant-neutral behavior
    this test asserts. 097-006: the precondition drive is now the binary
    `drive` arm (text `S` is deleted); `STOP` itself is untouched, still
    text."""
    reply = send_drive(sim, int(_DRIVE_TARGET), int(_DRIVE_TARGET))
    assert reply.WhichOneof("body") == "ok"

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


def test_deleted_text_verbs_reply_err_unknown(sim):
    """097-006 (Decision 9) DELETED S/D/T/RT/MOVE/MOVER/QLEN/R/TURN/G
    (motion_commands.cpp) and ECHO/VER/HELP/ID (system_commands.cpp)
    outright -- not merely unregistered them. Sending any of them over the
    text plane now hits the exact same `ERR unknown` path an always-
    unregistered family (`DEV`/`GET`, above) already does, proving the
    deletion took effect at the wire, not just at the source level (the
    grep-clean acceptance criteria already prove the source level)."""
    for line in ("S 100 100", "D 100 100 300", "T 100 100 1000", "RT 9000",
                 "MOVE 300 0 0", "MOVER 0 0 0 t=800 v=250", "QLEN",
                 "R 100 500", "TURN 9000", "G 100 100 200",
                 "ECHO hi", "VER", "HELP", "ID"):
        reply = sim.command(line)
        assert reply.strip() == "ERR unknown", f"{line!r} did not reply ERR unknown: {reply!r}"
