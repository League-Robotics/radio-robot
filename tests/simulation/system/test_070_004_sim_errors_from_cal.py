"""
test_070_004_sim_errors_from_cal.py — ticket 070-004 end-to-end acceptance
point (issue testgui-sim-errors-from-calibration-button).

Mirrors ``test_069_rt_90deg_body_scrub.py``'s ``sim`` fixture usage and the
same headline claim (``bodyRotScrub == rotationalSlip`` cancels
``PlannerBegin.cpp``'s ``beginRotation()`` arc-inflation), but drives the
scenario the way the TestGUI "Sim Errors" panel's new **From Calibration**
button would: read the active robot's calibration
(``robot_radio.config.robot_config.get_robot_config()``) and inject the
INVERSE of it into the plant via the full ``SIMSET`` knob set, not just the
two fields test_069 exercises directly.

Per the ticket, ``DefaultConfig.cpp`` already bakes ``rotationalSlip=0.92f``/
``trackwidthMm=128.0f`` to match ``data/robots/tovez.json`` (the active
robot), so this test does NOT send ``SET rotSlip=`` — only the plant
(``SIMSET``) side is configured, exactly as the button does. If the active
robot's calibration ever drifts from ``DefaultConfig.cpp``'s baked values,
this test's own arc-inflation vs. plant-scrub cancellation stops matching
and the assertion will (correctly) start failing — which is precisely the
cross-check the issue's "Caveat" section flags as worth watching for.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

# host/ is already added to sys.path by tests/conftest.py, but be defensive
# in case this file is ever collected standalone.
_HOST_DIR = Path(__file__).parent.parent.parent.parent / "host"
if str(_HOST_DIR) not in sys.path:
    sys.path.insert(0, str(_HOST_DIR))

import pytest  # noqa: E402

from robot_radio.config.robot_config import _reset_robot_config  # noqa: E402
from robot_radio.testgui import sim_prefs  # noqa: E402


@pytest.fixture(autouse=True)
def pin_calibrated_tovez(monkeypatch):
    """Pin the active robot to the CALIBRATED tovez config for this module.

    This test's premise is that the active robot's calibration matches what
    DefaultConfig.cpp bakes (rotationalSlip=0.92, trackwidth=128).  The
    repo's active_robot.json pointer is operator state (the GUI robot
    picker rewrites it — e.g. to tovez_nocal.json, which broke this test on
    2026-07-03), so pin the config explicitly via the ROBOT_CONFIG env var
    instead of depending on it.
    """
    monkeypatch.setenv(
        "ROBOT_CONFIG",
        str(Path(__file__).parent.parent.parent.parent
            / "data" / "robots" / "tovez.json"),
    )
    _reset_robot_config()
    yield
    _reset_robot_config()

# Wide enough to absorb PlannerBegin.cpp's pre-existing, out-of-scope RT
# coast-tuning residual (see test_069_rt_90deg_body_scrub.py's module
# docstring) — same tolerance the ticket specifies reusing.
_NEAR_90_TOL_DEG = 5.0

# Firmware's ArgList cap (source/types/CommandTypes.h MAX_ARGS=10) silently
# drops kv pairs past the 8th in a single SIMSET line (see transport.py's
# _SIMSET_MAX_PAIRS_PER_LINE) — chunk to stay safely under it.
_SIMSET_MAX_PAIRS_PER_LINE = 8


def _from_calibration_profile() -> dict:
    """Build the exact profile dict the "From Calibration" button's handler
    (``_on_sim_errors_from_cal`` in ``host/robot_radio/testgui/__main__.py``)
    produces: the 12 mapped knobs set to the inverse of the active robot's
    calibration (falling back to the neutral value per-field when the
    config or a field is missing, exactly like the handler), merged over
    ``sim_prefs.DEFAULT_PROFILE`` — the noise fields are left at their
    defaults, untouched, matching the button's contract.

    073-003: the lookup/fallback itself is now the SHARED
    ``sim_prefs.resolve_calibration_defaults()`` resolver (Design Rationale
    Decision 4) — this test no longer keeps its own third copy of that
    logic alongside ``__main__.py``'s button handler and
    ``load_sim_error_profile()``'s factory-default fallback.
    """
    profile = dict(sim_prefs.DEFAULT_PROFILE)

    rot_slip, trackwidth = sim_prefs.resolve_calibration_defaults()

    profile.update(
        {
            "slip_turn_extra": 0.0,
            "body_rot_scrub": rot_slip,
            "body_lin_scrub": 1.0,
            "motor_offset_l": 1.0,
            "motor_offset_r": 1.0,
            "trackwidth_mm": trackwidth,
            "enc_scale_err_l": 0.0,
            "enc_scale_err_r": 0.0,
            "otos_lin_scale_err": 0.0,
            "otos_ang_scale_err": 0.0,
            "otos_lin_drift_mms": 0.0,
            "otos_yaw_drift_degs": 0.0,
        }
    )
    return profile


def _apply_profile_via_simset(sim, profile: dict) -> None:
    """Apply ``profile`` to ``sim`` via chunked ``SIMSET`` lines plus the
    legacy ``slip_turn_extra`` channel — reproduces
    ``transport.py``'s ``SimTransport._apply_profile_to_sim()`` without
    going through the Qt GUI (this test drives the ctypes ``Sim`` directly,
    like every other ``tests/simulation/system/`` test).
    """
    pairs = [
        (wire_key, profile[key])
        for key, wire_key in sim_prefs.PROFILE_TO_SIMSET_KEY.items()
    ]
    pairs.append(("encNoiseL", profile["encoder_noise_mm"]))
    pairs.append(("encNoiseR", profile["encoder_noise_mm"]))

    for i in range(0, len(pairs), _SIMSET_MAX_PAIRS_PER_LINE):
        chunk = pairs[i:i + _SIMSET_MAX_PAIRS_PER_LINE]
        simset_line = "SIMSET " + " ".join(f"{k}={v}" for k, v in chunk)
        reply = sim.send_command(simset_line)
        assert "OK" in reply.upper(), f"{simset_line} -> unexpected reply {reply!r}"

    # set_field_profile() (the legacy slip_turn_extra channel) has no wire
    # reply — it calls ctypes hooks directly (firmware.py) — so there is
    # nothing to assert on here, unlike the SIMSET lines above.
    sim.set_field_profile(slip_turn_extra=profile["slip_turn_extra"], fuse_otos=True)


def test_rt_90deg_with_inverse_calibration_profile_from_active_robot_config(sim):
    """Headline acceptance point: applying the inverse-calibration mapping
    (bodyRotScrub = active robot's rotational_slip, trackwidth = active
    robot's geometry.trackwidth, every other knob neutral, noise at 0)
    closes the ideal-plant RT 9000 over-rotation gap — the sim robot lands
    within _NEAR_90_TOL_DEG of the commanded 90°, not the ~95° a zero-scrub
    plant produces against DefaultConfig.cpp's baked rotationalSlip=0.92
    (see test_069_rt_90deg_body_scrub.py).

    No ``SET rotSlip=`` is sent: DefaultConfig.cpp already bakes
    rotationalSlip=0.92/trackwidthMm=128.0 to match the active robot config
    (data/robots/tovez.json via active_robot.json) — only the plant
    (SIMSET) side is configured here, exactly as the "From Calibration"
    button does.
    """
    profile = _from_calibration_profile()

    # Sanity: this test's premise only holds if the active robot's
    # calibration is the non-trivial (not-already-neutral) value the ticket
    # describes -- otherwise this test would pass trivially even if the
    # mapping were broken.
    assert profile["body_rot_scrub"] != 1.0, (
        "expected the active robot config to have a non-neutral "
        "rotational_slip (e.g. 0.92) -- got the neutral fallback; check "
        "data/robots/active_robot.json"
    )

    sim.set_true_pose(0.0, 0.0, 0.0)
    reply = sim.send_command("ZERO enc")
    assert "OK" in reply.upper(), f"ZERO enc -> unexpected reply {reply!r}"

    _apply_profile_via_simset(sim, profile)

    reply = sim.send_command("RT 9000")
    assert "OK" in reply.upper(), f"RT 9000 -> unexpected reply {reply!r}"

    sim.tick_for(8000)

    _, _, true_h = sim.get_true_pose()
    true_h_deg = math.degrees(true_h)

    assert abs(true_h_deg - 90.0) < _NEAR_90_TOL_DEG, (
        f"RT 9000 with the inverse-calibration profile "
        f"(body_rot_scrub={profile['body_rot_scrub']}, "
        f"trackwidth_mm={profile['trackwidth_mm']}) should land near 90° "
        f"true (closing the ideal-plant over-rotation gap); got "
        f"{true_h_deg:.2f}°"
    )
