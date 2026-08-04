"""src/tests/unit/test_sim_boot_config.py -- ticket 113-004, narrowed by
115-003 (gut-to-minimal-firmware S1 motion-stack excision), retargeted by
132-014 (migrate host NezhaProtocol + calibration/push.py + TestGUI onto
the new config surface).

``sim_boot_config.py`` computes the Tier-2 (boot-only) ``Devices::
MotorConfig``/``App::Drive``/rotation-calibration scalar sets
``gen_boot_config.py`` bakes into a real robot's ``boot_config.cpp``, but at
sim-open time -- from EITHER a raw robot-JSON dict (still the OLD 13-section
shape pending ticket 017's reshape -- calls ``gen_boot_config.py``'s own
mapping functions, never re-deriving them, sprint 113's own Design
Rationale Decision 2) OR a ``RobotConfig``/grouped-shape object (132-020
adopted ``robot_config.proto``'s consumer-grouped shape into ``RobotConfig``
itself -- reads its already-typed ``.motors``/``.drive``/``.geometry``
groups DIRECTLY, no dict-path derivation needed at all).

115-003 deleted this module's ``msg::PlannerConfig`` half
(``planner_boot_config_for()``/``_heading_source_wire_value()``) wholesale
-- ``msg::PlannerConfig`` itself, and every ``gen_boot_config.py`` mapping
function it called, went with the deleted ``App::Pilot``/``Motion::Executor``
subsystems (ticket 003's proto surgery). ``motor_boot_config_for()`` below
is the sole survivor of that generation; ``drive_boot_config_for()``/
``drivetrain_boot_config_for()`` (125-007/113-005) round out this file's
coverage of the OTHER two ``*_boot_config_for()`` helpers.

132-014: ``RobotConfig``'s own ``.control``/``.calibration`` sub-models
(the OLD flat-shape source this file's tests used to build via
``ControlConfig``/``CalibrationConfig``) are gone -- ``RobotConfig`` adopted
``robot_config.proto``'s grouped shape (132-020). Every test below now
covers ONE of TWO paths explicitly:
  1. The raw-dict path (``gen_boot_config.py``'s own mapping functions,
     unchanged) -- covered against the two REAL shipped robot JSONs, and
     against a minimal/empty dict for the fail-closed case.
  2. The grouped-object path (132-014, NEW) -- covered against a synthetic
     ``RobotConfig``-shaped object built from the REAL generated
     ``robot_config_generated`` classes (``Motors``/``Drive``/``Geometry``),
     proving the direct-attribute read matches what was actually set on it
     (never a hardcoded expected-value table pinned against unrelated
     JSON content).

Covers, per the ticket's own Testing section:
  1. ``motor_boot_config_for()`` -- raw-dict parity against ``tovez.json``/
     ``tovez_nocal.json``'s own real JSON content (still the OLD shape),
     AND grouped-object direct-read against a synthetic ``Motors`` group.
  2. A fallback case: an empty dict (no "control"/"calibration" section)
     hard-fails (sprint 114 config-as-truth completion), not a silent
     fallback to old firmware defaults.
  3. The drive pair's mirror-mounted ``fwd_sign`` (088-002) genuinely reads
     through BOTH the raw-dict AND the grouped-object path.

This module is Qt-free and sim-lib-free (pure function coverage only) --
collected under ``src/tests/unit/`` per ``pyproject.toml``'s ``testpaths``,
mirroring ``src/tests/unit/test_calibration_kwargs.py``'s own placement
(the Tier-1 sibling of this Tier-2 helper).
"""
from __future__ import annotations

import json
import math
import sys
import types
from pathlib import Path

import pytest

from robot_radio.calibration.sim_boot_config import (
    drive_boot_config_for, drivetrain_boot_config_for, motor_boot_config_for)
from robot_radio.config.robot_config import load_robot_config
from robot_radio.config.robot_config_generated import Drive, Geometry, Motors

# src/tests/unit/test_sim_boot_config.py -> unit -> tests -> src -> repo root
_REPO_ROOT = Path(__file__).resolve().parents[3]
_ROBOTS_DIR = _REPO_ROOT / "data" / "robots"
_SCRIPTS_DIR = _REPO_ROOT / "src" / "scripts"

if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))

import gen_boot_config as gbc  # noqa: E402  (path must be set up before this import)


_ROBOT_JSON_NAMES = ["tovez.json", "tovez_nocal.json"]


def _raw_cfg(name: str) -> dict:
    return json.loads((_ROBOTS_DIR / name).read_text())


def _grouped_cfg(*, motors: Motors, drive: "Drive | None" = None,
                 geometry: "Geometry | None" = None) -> types.SimpleNamespace:
    """A minimal duck-typed grouped-shape object -- only ``.motors``/
    ``.drive`` are required for ``_is_grouped_robot_config()`` to select the
    direct-attribute-read path; ``.geometry`` is added when a test needs
    ``drivetrain_boot_config_for()`` too."""
    return types.SimpleNamespace(
        motors=motors, drive=drive if drive is not None else Drive(),
        geometry=geometry if geometry is not None else Geometry())


# ---------------------------------------------------------------------------
# motor_boot_config_for() -- raw-dict path (gen_boot_config.py parity).
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("name", _ROBOT_JSON_NAMES)
@pytest.mark.parametrize("port", [1, 2, 3, 4])
def test_motor_boot_config_for_raw_dict_matches_gen_boot_config(name, port):
    """A raw dict (json.loads() of the real, still-OLD-shape robot JSON)
    goes through gen_boot_config.py's own mapping functions -- proven here
    by an independent call-through comparison, never a hardcoded
    expected-value table."""
    raw = _raw_cfg(name)

    result = motor_boot_config_for(raw, port)

    *_gains, expected_filt = gbc.vel_gains_for_config(raw)
    expected_fwd_signs = gbc.fwd_sign_for_ports(raw)

    assert result["vel_filt_alpha"] == expected_filt
    assert result["fwd_sign"] == expected_fwd_signs[port - 1]


def test_motor_boot_config_for_raw_dict_raises_with_no_calibration_or_control_section():
    """motor_boot_config_for() unconditionally resolves vel_gains_for_config()
    first (for vel_filt_alpha) -- with no control.vel_* keys at all, it must
    raise the same MissingRobotConfigKeyError gen_boot_config.py itself
    raises (sprint 114 config-as-truth completion), not silently return the
    old VEL_FILT_ALPHA/FWD_SIGN placeholder pair."""
    with pytest.raises(gbc.MissingRobotConfigKeyError):
        motor_boot_config_for({}, port=1)


def test_as_cfg_dict_rejects_a_non_dict_non_grouped_object():
    """132-014: a non-dict object with no .motors/.drive attributes is
    neither a valid raw-dict input nor a grouped-shape one -- an honest
    TypeError, not the old function's silent {} fallback (which used to
    mask the 132-020 shape change as an empty, _require()-raising config)."""
    with pytest.raises(TypeError):
        motor_boot_config_for(types.SimpleNamespace(robot_name="bare"), port=1)


# ---------------------------------------------------------------------------
# motor_boot_config_for() -- grouped-object path (132-014, NEW): a
# RobotConfig (or duck-typed equivalent) reads config.motors DIRECTLY.
# ---------------------------------------------------------------------------


def test_motor_boot_config_for_grouped_object_reads_vel_filt_alpha_and_fwd_sign():
    """Proves the drive pair's mirror-mounted fwd_sign (088-002) genuinely
    reads through the grouped RobotConfig path -- left=-1/right=+1 (the
    OPPOSITE of both shipped fixtures' own left=+1/right=-1), so this
    cannot coincidentally match a hardcoded default."""
    cfg = _grouped_cfg(motors=Motors(
        vel_filt_alpha=0.85, fwd_sign_left=-1, fwd_sign_right=1))

    left = motor_boot_config_for(cfg, port=1)
    right = motor_boot_config_for(cfg, port=2)
    other = motor_boot_config_for(cfg, port=3)

    assert left == {"vel_filt_alpha": 0.85, "fwd_sign": -1}
    assert right == {"vel_filt_alpha": 0.85, "fwd_sign": 1}
    assert other == {"vel_filt_alpha": 0.85, "fwd_sign": gbc.FWD_SIGN}


def test_motor_boot_config_for_real_robot_config_takes_the_grouped_path_no_raise():
    """A REAL load_robot_config() result (RobotConfig, 132-020's grouped
    shape) must not raise -- even though the JSON on disk is still
    old-shape (pending ticket 017), _is_grouped_robot_config() selects the
    direct-attribute path, which reads .motors' proto3 zero defaults
    rather than calling gen_boot_config.py's _require()-guarded raw-dict
    path (which WOULD raise on a dict missing "control"). This is the
    exact regression this ticket closes: pre-132-014, this call raised
    MissingRobotConfigKeyError, cascading through SimLoop.
    configure_from_robot() and the shared TestGUI fixture."""
    cfg = load_robot_config(_ROBOTS_DIR / "tovez.json")

    result = motor_boot_config_for(cfg, port=1)

    assert result == {"vel_filt_alpha": cfg.motors.vel_filt_alpha,
                       "fwd_sign": cfg.motors.fwd_sign_left}


# ---------------------------------------------------------------------------
# drive_boot_config_for() / drivetrain_boot_config_for() -- grouped-object
# path direct-read coverage (132-014).
# ---------------------------------------------------------------------------


def test_drive_boot_config_for_grouped_object_reads_duty_and_crawl_only():
    """Reads config.drive.duty_per_speed_left/right/crawl_pulse directly --
    the eight wheel_gain_*/wheel_intercept_* Stage-A fields on the SAME
    Drive group are deliberately NOT read (TestSim::WheelPlant is linear;
    see this function's own docstring for the full linearization
    rationale)."""
    drive = Drive(duty_per_speed_left=0.0019, duty_per_speed_right=0.0021,
                  crawl_pulse=0.05,
                  wheel_gain_left_accel=1.47)  # must NOT leak into the result
    cfg = _grouped_cfg(motors=Motors(), drive=drive)

    result = drive_boot_config_for(cfg)

    assert result == {
        "duty_per_speed_left": pytest.approx(0.0019),
        "duty_per_speed_right": pytest.approx(0.0021),
        "crawl_pulse": pytest.approx(0.05),
    }


def test_drivetrain_boot_config_for_grouped_object_converts_degrees_to_radians():
    geometry = Geometry(rotation_gain_pos=1.02, rotation_offset=3.0,
                        rotation_gain_neg=0.98, rotation_offset_neg=-2.5)
    cfg = _grouped_cfg(motors=Motors(), geometry=geometry)

    result = drivetrain_boot_config_for(cfg)

    assert result["rot_gain_pos"] == pytest.approx(1.02)
    assert result["rot_offset_pos"] == pytest.approx(math.radians(3.0))
    assert result["rot_gain_neg"] == pytest.approx(0.98)
    assert result["rot_offset_neg"] == pytest.approx(math.radians(-2.5))


def test_drivetrain_boot_config_for_raw_dict_matches_gen_boot_config():
    raw = _raw_cfg("tovez.json")

    result = drivetrain_boot_config_for(raw)

    gain_pos, offset_pos_deg, gain_neg, offset_neg_deg = gbc.rotation_calibration_for_config(raw)
    assert result["rot_gain_pos"] == pytest.approx(gain_pos)
    assert result["rot_offset_pos"] == pytest.approx(math.radians(offset_pos_deg))
    assert result["rot_gain_neg"] == pytest.approx(gain_neg)
    assert result["rot_offset_neg"] == pytest.approx(math.radians(offset_neg_deg))
