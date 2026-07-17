"""src/tests/testgui/test_planner_config_patch_live_tuning.py -- sprint 109
ticket 008: `PlannerConfigPatch` gain patches un-stubbed and settable from
the TestGUI without a reflash.

Before this ticket, `RobotLoop::handleConfig` replied `ERR_UNIMPLEMENTED`
unconditionally for every `PLANNER` patch (`src/firm/app/DESIGN.md` §3's own
former "scope boundary" note) -- `heading_kp`/`heading_kd` (the sprint-098
precedent, "Velocity + heading gains live-tunable on stand") had no live
wire path at all in this tree since the single-loop rebuild. This ticket
wires `Pilot::applyPlannerPatch()` into that arm (merge-then-write onto
Pilot's own live `msg::PlannerConfig` baseline, then re-applied to
`Executor::configure()`/`HeadingSource::configure()`/`Pilot::
configureHeading()`).

This test drives the SAME path a TestGUI "SET headingKp=..." action takes:
`SimTransport.command("SET headingKp=<value>")` (routes through
`NezhaProtocol.config()`'s typed `PlannerConfigPatch` builder, ticket 002's
direct-patch-send mechanism, injected via `SimLoop.inject_command()`) then
confirms the value is echoed back via `GET headingKp` (Architecture
Revision 1's host-side GET echo -- the honest substitute for a firmware
query arm this sprint declines to add).

Run with::

    uv run pytest src/tests/testgui/test_planner_config_patch_live_tuning.py -v
"""
from __future__ import annotations

import pytest

from robot_radio.testgui.transport import SimTransport, _sim_lib_path

pytestmark = pytest.mark.skipif(
    not _sim_lib_path().exists(),
    reason="sim lib not built -- cmake --build src/sim/build (or `python build.py`)",
)


@pytest.fixture
def sim_transport():
    transport = SimTransport()
    transport.connect()
    assert transport._connected, "SimTransport failed to connect to the sim lib"
    try:
        yield transport
    finally:
        transport.disconnect()


def test_heading_kp_is_settable_and_echoes_back(sim_transport):
    """`SET headingKp=<v>` no longer hits the PLANNER arm's former
    ERR_UNIMPLEMENTED stub -- it round-trips OK and `GET headingKp` echoes
    the pushed value back (host-side echo of the last value THIS session
    itself pushed, per Architecture Revision 1 -- not a firmware query)."""
    reply = sim_transport.command("SET headingKp=2.5", read_timeout=500)
    assert reply.startswith("OK set"), f"expected an OK set reply, got {reply!r}"
    assert "headingKp=2.5" in reply

    echoed = sim_transport.command("GET headingKp", read_timeout=500)
    assert echoed == "headingKp=2.5", f"GET headingKp did not echo the pushed value: {echoed!r}"


def test_heading_kd_is_settable_and_echoes_back(sim_transport):
    reply = sim_transport.command("SET headingKd=0.15", read_timeout=500)
    assert reply.startswith("OK set"), f"expected an OK set reply, got {reply!r}"

    echoed = sim_transport.command("GET headingKd", read_timeout=500)
    assert echoed == "headingKd=0.15", f"GET headingKd did not echo the pushed value: {echoed!r}"


def test_min_speed_is_settable_and_echoes_back(sim_transport):
    """`minSpeed` is PlannerConfigPatch's third originally-registered key
    (config_commands.cpp's own kAllKeys, pre-rebuild) -- same live path."""
    reply = sim_transport.command("SET minSpeed=12", read_timeout=500)
    assert reply.startswith("OK set"), f"expected an OK set reply, got {reply!r}"

    echoed = sim_transport.command("GET minSpeed", read_timeout=500)
    assert echoed == "minSpeed=12", f"GET minSpeed did not echo the pushed value: {echoed!r}"


def test_planner_gain_patch_actually_reaches_firmware_pilot_state(sim_transport):
    """Beyond the host-side echo (which only proves the HOST remembers what
    it sent): confirm the firmware ack itself is OK, not the old
    ERR_UNIMPLEMENTED -- constructs and sends the SAME ConfigDelta{planner}
    envelope `SET` uses, directly via `NezhaProtocol.config()`, and waits
    for the REAL ack-ring entry off the connected sim (not the host echo)."""
    protocol = sim_transport.protocol
    assert protocol is not None

    from robot_radio.robot.pb2 import telemetry_pb2

    corr_id = sim_transport._config_proto.config(headingKp=3.3)
    ack = sim_transport._config_conn.poll_ack(corr_id, timeout=1000)
    assert ack is not None, "no ack received for the PlannerConfigPatch{heading_kp} envelope"
    assert ack.ok, (
        f"PlannerConfigPatch{{heading_kp}} did not ack OK (still ERR_UNIMPLEMENTED?): "
        f"ok={ack.ok} err_code={ack.err_code}"
    )


def test_drivetrain_patch_stays_the_honest_unsupported_error(sim_transport):
    """Sanity check that this ticket's un-stub is SCOPED to PLANNER --
    DrivetrainConfigPatch keys (rotSlip/tw) still have no firmware consumer
    (Architecture Revision 1) and must still get the honest unsupported
    error, not silently start "working" as a side effect of this ticket."""
    reply = sim_transport.command("SET rotSlip=0.9", read_timeout=500)
    assert reply.startswith("ERR unsupported"), (
        f"rotSlip must still be the honest unsupported error: {reply!r}"
    )


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
