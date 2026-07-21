"""src/tests/unit/test_gen_boot_config_otos.py -- generator-level proof for
ticket 086-005 (SUC-005/SUC-006): scripts/gen_boot_config.py's new
otos_boot_config_values() mapping and defaultOtosBootConfig() code
generation, additive to the existing trackwidth/travel_calib mappings (no
existing mapping touched -- see this ticket's acceptance criteria).

Invokes the generator in-process (mirrors src/tests/unit/
test_gen_messages_no_getters.py's own in-process pattern) against both a
real robot config (data/robots/tovez.json, which carries real
odometry_offset_mm/otos_linear_scale/otos_angular_scale values) and the
identity-default fallback (an empty dict, as when no robot config is
found), rather than shelling out to the script or depending on whatever
data/robots/active_robot.json happens to point at.

Collected under src/tests/unit/ (a generator/tooling-level check, not
sim/bench/playfield-scoped -- see tests/CLAUDE.md); pyproject.toml's
testpaths includes tests/unit.
"""

import json
import sys
from pathlib import Path

import pytest

# src/tests/unit/test_gen_boot_config_otos.py -> unit -> tests -> repo root
_REPO_ROOT = Path(__file__).resolve().parents[3]
_SCRIPTS_DIR = _REPO_ROOT / "src" / "scripts"
_TOVEZ_JSON = _REPO_ROOT / "data" / "robots" / "tovez.json"

if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))

import gen_boot_config as gbc  # noqa: E402  (path must be set up before this import)


def test_otos_boot_config_values_reads_tovez_json():
    """otos_boot_config_values() reads tovez.json's real geometry/calibration values."""
    cfg = json.loads(_TOVEZ_JSON.read_text())

    offset_x, offset_y, offset_yaw, linear_scale, angular_scale = (
        gbc.otos_boot_config_values(cfg)
    )

    assert offset_x == -47.7
    assert offset_y == 3.5
    assert offset_yaw == 0.0
    assert linear_scale == 1.067
    assert angular_scale == 0.987


def test_otos_boot_config_values_raises_with_no_robot_config():
    """Sprint 114 (config-as-truth completion): with no robot config at all,
    the generator hard-fails on the first required key
    (geometry.odometry_offset_mm.x) -- no more identity-default (zero
    offset, 1.0 scale) source-side fallback."""
    with pytest.raises(gbc.MissingRobotConfigKeyError,
                        match="geometry.odometry_offset_mm.x"):
        gbc.otos_boot_config_values({})


def test_generate_emits_default_otos_boot_config_additively():
    """generate()'s output gains defaultOtosBootConfig() without disturbing
    defaultMotorConfigs()/defaultDrivetrainConfig() (the pre-086-005 mappings)."""
    cfg = json.loads(_TOVEZ_JSON.read_text())
    content = gbc.generate(cfg, "data/robots/tovez.json")

    # Additive: the existing generated functions are still emitted, unchanged
    # in shape (this test does not re-assert their exact bodies -- that is
    # existing/unowned-by-this-ticket behavior -- only that they still exist).
    assert "void defaultMotorConfigs(msg::MotorConfig* out)" in content
    assert "msg::DrivetrainConfig defaultDrivetrainConfig()" in content

    # New (086-005): the OTOS boot struct generator function, carrying
    # tovez.json's real values through into the emitted C++ literals.
    assert "OtosBootConfig defaultOtosBootConfig()" in content
    assert "cfg.offsetX = -47.7f;" in content
    assert "cfg.offsetY = 3.5f;" in content
    assert "cfg.offsetYaw = 0.0f;" in content
    assert "cfg.linearScale = 1.067f;" in content
    assert "cfg.angularScale = 0.987f;" in content


if __name__ == "__main__":
    import pytest

    sys.exit(pytest.main([__file__, "-v"]))
