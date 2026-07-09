"""tests/sim/unit/test_gen_boot_config_fwd_sign.py -- generator-level proof
for ticket 088-002 (SUC-001): scripts/gen_boot_config.py's new
fwd_sign_for_ports() mapping and its wiring into defaultMotorConfigs()'s
code generation.

Bug fixed: the two drive-pair motors are mirror-mounted, but the generator
previously baked a uniform fwd_sign = +1 onto every port (FWD_SIGN=1 applied
blanket). A straight-drive command (equal L/R targets) then spun the two
wheels in OPPOSITE physical directions. clasi/issues/
tovez-drive-motor-reversed-fwd-sign.md and old-tree evidence
(source_old/robot/DefaultConfig.cpp fwdSignL=-1/fwdSignR=+1 combined with
source_old/robot/NezhaHAL.cpp's M2=LEFT/M1=RIGHT chip mapping) prove the
physically-correct PORT signs on this robot are port 1 = +1, port 2 = -1 --
which is what data/robots/tovez.json / tovez_nocal.json's new
calibration.fwd_sign_left=1 / fwd_sign_right=-1 (LEFT_PORT=1, RIGHT_PORT=2)
now bakes.

Mirrors tests/unit/test_gen_boot_config_otos.py's own in-process pattern
(invokes the generator module directly rather than shelling out, and against
a real robot config plus the identity-default empty-dict fallback) but is
placed under tests/sim/unit/ per this ticket's own testing plan so it is
collected -- and counted -- by `uv run python -m pytest tests/sim -q`, the
sprint's scoped no-hardware gate (pyproject.toml's testpaths). It is a pure
Python, generator-only test: it does not use the `sim`/`build_lib` fixtures
(tests/sim/conftest.py) and does not need libfirmware_host built.

NOTE (scope): the sim plant does not model physical wheel mounting, so this
file does not -- and must not -- try to assert "the robot drives straight"
against the simulator. It asserts only that the per-port fwd_sign VALUE the
robot JSON specifies reaches the generated boot config that main.cpp calls
(Config::defaultMotorConfigs()). msg::MotorConfig::fwd_sign itself and
NezhaMotor's consumption of it (source/hal/nezha/nezha_motor.cpp) are
already correct and out of this ticket's scope.
"""

import json
import sys
from pathlib import Path

# tests/sim/unit/test_gen_boot_config_fwd_sign.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = Path(__file__).resolve().parents[3]
_SCRIPTS_DIR = _REPO_ROOT / "scripts"
_TOVEZ_JSON = _REPO_ROOT / "data" / "robots" / "tovez.json"

if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))

import gen_boot_config as gbc  # noqa: E402  (path must be set up before this import)


def test_fwd_sign_for_ports_reads_tovez_json_mirror_mounted_signs():
    """fwd_sign_for_ports() reads tovez.json's real calibration.fwd_sign_left/
    right (1 / -1) onto LEFT_PORT/RIGHT_PORT (1/2); the two non-drive ports
    (3, 4) fall back to the FWD_SIGN placeholder, exactly matching
    travel_calib_for_ports()'s established fallback shape."""
    cfg = json.loads(_TOVEZ_JSON.read_text())

    signs = gbc.fwd_sign_for_ports(cfg)

    assert len(signs) == gbc.K_MOTOR_COUNT
    assert signs[gbc.LEFT_PORT - 1] == 1
    assert signs[gbc.RIGHT_PORT - 1] == -1
    # Non-drive ports keep the placeholder -- unaffected by the drive pair's
    # mirror-mount correction.
    for port in range(1, gbc.K_MOTOR_COUNT + 1):
        if port not in (gbc.LEFT_PORT, gbc.RIGHT_PORT):
            assert signs[port - 1] == gbc.FWD_SIGN


def test_fwd_sign_for_ports_falls_back_to_placeholder_for_every_port():
    """With no calibration.fwd_sign_left/right in the robot JSON (or no robot
    config at all), every port falls back to the FWD_SIGN=1 placeholder --
    matching every other mapping's fall-back-to-firmware-default behavior in
    this generator, and reproducing the pre-088-002 (buggy) uniform-sign
    output for a robot JSON that hasn't been calibrated yet."""
    signs = gbc.fwd_sign_for_ports({})

    assert signs == [gbc.FWD_SIGN] * gbc.K_MOTOR_COUNT


def test_generate_emits_per_port_fwd_sign_mirror_mounted_drive_pair():
    """generate()'s output bakes the mirror-mounted drive pair's opposite
    signs into defaultMotorConfigs() -- port 1 (LEFT_PORT) = +1, port 2
    (RIGHT_PORT) = -1 -- while the existing generated functions/mappings
    (velocity gains, travel calib, drivetrain config, OTOS boot config)
    remain present and undisturbed."""
    cfg = json.loads(_TOVEZ_JSON.read_text())
    content = gbc.generate(cfg, "data/robots/tovez.json")

    assert "void defaultMotorConfigs(msg::MotorConfig* out)" in content
    assert "msg::DrivetrainConfig defaultDrivetrainConfig()" in content
    assert "OtosBootConfig defaultOtosBootConfig()" in content

    # The mirror-mounted drive pair: opposite signs, matching the old-tree
    # evidence cited in clasi/issues/tovez-drive-motor-reversed-fwd-sign.md.
    assert "out[0].setFwdSign(1);" in content
    assert "out[1].setFwdSign(-1);" in content
    # Non-drive ports still carry the bench placeholder.
    assert "out[2].setFwdSign(1);" in content
    assert "out[3].setFwdSign(1);" in content


def test_generate_falls_back_to_uniform_fwd_sign_with_no_robot_config():
    """The identity-default fallback (no robot config found) reproduces the
    firmware-default uniform +1 on every port -- the build must still
    succeed and boot sanely with no robot JSON present."""
    content = gbc.generate({}, "(firmware defaults)")

    assert "out[0].setFwdSign(1);" in content
    assert "out[1].setFwdSign(1);" in content
    assert "out[2].setFwdSign(1);" in content
    assert "out[3].setFwdSign(1);" in content


if __name__ == "__main__":
    import pytest

    sys.exit(pytest.main([__file__, "-v"]))
