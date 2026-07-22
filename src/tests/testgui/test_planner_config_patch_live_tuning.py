"""src/tests/testgui/test_planner_config_patch_live_tuning.py -- sprint 109
ticket 008's own historical scope: `PlannerConfigPatch` gain patches
un-stubbed and settable from the TestGUI without a reflash.

115-003 (gut-to-minimal-firmware S1 motion-stack excision) DELETES
`PlannerConfigPatch`/`ConfigDelta.planner` wholesale, alongside the
`App::Pilot` that applied it (`Pilot::applyPlannerPatch()`) -- there is no
`headingKp`/`headingKd`/`minSpeed` wire target left at all; every test this
file used to carry for that live-tuning path (`test_heading_kp_is_settable_
and_echoes_back`, `test_heading_kd_is_settable_and_echoes_back`,
`test_min_speed_is_settable_and_echoes_back`,
`test_planner_gain_patch_actually_reaches_firmware_pilot_state`) is removed,
not ported -- there is nothing left to prove.

Only the ONE test this file always carried that is INDEPENDENT of
PlannerConfigPatch survives: `DrivetrainConfigPatch` keys (rotSlip/tw) have
no firmware consumer (Architecture Revision 1, sprint 109) and must still
get the honest `ERR unsupported` error over `SET` -- this remains true and
worth guarding even with PLANNER gone (`RobotLoop::handleConfig` now applies
MOTOR/OTOS only; DRIVETRAIN/WATCHDOG/NONE still reply the same honest
unsupported error).

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


def test_drivetrain_patch_stays_the_honest_unsupported_error(sim_transport):
    """DrivetrainConfigPatch keys (rotSlip/tw) still have no firmware
    consumer (Architecture Revision 1) and must still get the honest
    unsupported error -- unaffected by 115-003's PlannerConfigPatch
    deletion (RobotLoop::handleConfig applies MOTOR/OTOS only)."""
    reply = sim_transport.command("SET rotSlip=0.9", read_timeout=500)
    assert reply.startswith("ERR unsupported"), (
        f"rotSlip must still be the honest unsupported error: {reply!r}"
    )


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
