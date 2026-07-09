"""093-003: focused four-verb suite for the post-093 bare wheel-driving
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

(093/094 teardown) `S`'s/`STOP`'s own PLANT-MOTION assertions ("`S` actually
drives both wheels", "opposite-sign `l`/`r` spins the wheels opposite ways",
"`STOP` neutralizes regardless of prior `S` state") are PARKED at
`tests/sim/parked-094/unit/test_bare_loop_drive_severed.py` -- the motor/
hardware inbound message queues (`Rt::Blackboard`'s `motorIn[]`/
`motorResetIn[]`/`hardwareBroadcastIn`) were removed (see
`source/runtime/blackboard.h`'s file header) and `Rt::MainLoop::tick()` no
longer has a `routeOutputs()` step, so `S`/`STOP` still PARSE and REPLY
correctly (checked below is left to the parked file, which still checks the
`OK` replies too) but no longer reach the plant at all -- see that parked
file's own header for the full explanation and the restore condition.
"""
from __future__ import annotations

import pytest


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
