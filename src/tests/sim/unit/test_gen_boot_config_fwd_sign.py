"""src/tests/sim/unit/test_gen_boot_config_fwd_sign.py -- generator-level proof
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
physically-correct PORT signs on this robot are **port 1 = +1, port 2 = -1**.

PORT is the load-bearing word, and this file's assertions are written per
PORT for that reason (2026-08-13). The per-port signs above have never
changed; what changed is which WHEEL each port drives. tovez is wired
port 1 = RIGHT wheel, port 2 = LEFT wheel, and the old
`source_old/robot/NezhaHAL.cpp` mapping cited above says exactly that
("M2=LEFT/M1=RIGHT") -- but gen_boot_config.py hardcoded LEFT_PORT=1/
RIGHT_PORT=2, so the firmware called port 1's wheel "left". That negates
omega = (vR - vL) / b, and so every encoder-derived heading, while leaving
forward motion correct -- which is why it survived for months and got
patched three times downstream instead of once here.

tovez.json now carries `motors.left_port: 2` / `right_port: 1` (read by
`drive_ports()`) with its paired `fwd_sign_left: -1` / `fwd_sign_right: 1`
following the physical wheels. Net effect on the BAKED per-port array:
unchanged, port 1 = +1 and port 2 = -1, exactly as before -- only the
labels moved. These tests therefore assert the PORT array directly and
never via `gbc.LEFT_PORT`/`gbc.RIGHT_PORT`, which are now merely the
DEFAULTS for a robot that does not state its own binding.

Mirrors src/tests/unit/test_gen_boot_config_otos.py's own in-process pattern
(invokes the generator module directly rather than shelling out, and against
a real robot config plus the identity-default empty-dict fallback) but is
placed under src/tests/sim/unit/ per this ticket's own testing plan so it is
collected -- and counted -- by `uv run python -m pytest tests/sim -q`, the
sprint's scoped no-hardware gate (pyproject.toml's testpaths). It is a pure
Python, generator-only test: it does not use the `sim`/`build_lib` fixtures
(src/tests/sim/conftest.py) and does not need libfirmware_host built.

NOTE (scope): the sim plant does not model physical wheel mounting, so this
file does not -- and must not -- try to assert "the robot drives straight"
against the simulator. It asserts only that the per-port fwd_sign VALUE the
robot JSON specifies reaches the generated boot config that main.cpp calls
(Config::defaultMotorConfigs()). msg::MotorConfig::fwd_sign itself and
NezhaMotor's consumption of it (src/firm/hal/nezha/nezha_motor.cpp) are
already correct and out of this ticket's scope.
"""

import json
import sys
from pathlib import Path

import pytest

# src/tests/sim/unit/test_gen_boot_config_fwd_sign.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = Path(__file__).resolve().parents[4]
_SCRIPTS_DIR = _REPO_ROOT / "src" / "scripts"
_TOVEZ_JSON = _REPO_ROOT / "data" / "robots" / "tovez.json"

if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))

import gen_boot_config as gbc  # noqa: E402  (path must be set up before this import)


def test_fwd_sign_for_ports_reads_tovez_json_mirror_mounted_signs():
    """fwd_sign_for_ports() puts each wheel's fwd_sign on the PORT that
    wheel is actually wired to.

    tovez: `motors.left_port` is 2 and `right_port` is 1, so
    `fwd_sign_right: 1` lands on port 1 and `fwd_sign_left: -1` on port 2 --
    the physically-correct mirror-mounted pair, and byte-identical to what
    the generator baked before the port binding was introduced. The two
    non-drive ports (3, 4) fall back to the FWD_SIGN placeholder, matching
    travel_calib_for_ports()'s established fallback shape."""
    cfg = json.loads(_TOVEZ_JSON.read_text())
    left_port, right_port = gbc.drive_ports(cfg)
    assert (left_port, right_port) == (2, 1), "tovez is wired port 1 = RIGHT wheel"

    signs = gbc.fwd_sign_for_ports(cfg)

    assert len(signs) == gbc.K_MOTOR_COUNT
    # Per PORT -- the physical fact. Port 1 (this robot's RIGHT wheel) = +1,
    # port 2 (its LEFT wheel) = -1.
    assert signs[0] == 1
    assert signs[1] == -1
    # Equivalently, stated per WHEEL: each label follows its own port.
    assert signs[left_port - 1] == cfg["motors"]["fwd_sign_left"]
    assert signs[right_port - 1] == cfg["motors"]["fwd_sign_right"]
    # Non-drive ports keep the placeholder -- unaffected by the drive pair's
    # mirror-mount correction.
    for port in range(1, gbc.K_MOTOR_COUNT + 1):
        if port not in (left_port, right_port):
            assert signs[port - 1] == gbc.FWD_SIGN


def test_drive_ports_defaults_to_one_two_when_robot_json_is_silent():
    """A robot JSON with no motors.left_port/right_port keeps the historical
    binding (port 1 = left, port 2 = right). Only a robot that states
    otherwise -- tovez -- gets the swap, so no other robot's bake moves."""
    assert gbc.drive_ports({}) == (gbc.LEFT_PORT, gbc.RIGHT_PORT) == (1, 2)
    assert gbc.drive_ports({"motors": {}}) == (1, 2)
    assert gbc.drive_ports({"motors": {"left_port": 2, "right_port": 1}}) == (2, 1)


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
    signs into defaultMotorConfigs(), indexed by PORT -- `out[N]` is port
    N+1 -- while the existing generated functions/mappings (velocity gains,
    travel calib, drivetrain config, OTOS boot config) remain present and
    undisturbed.

    `defaultMotorConfigs()` is a per-port array and this is the only place
    the physical signs are asserted against it. The paired
    `defaultDrivetrainConfig()` binding below is what tells the firmware
    which of those ports is the left wheel -- boot_wiring.cpp indexes this
    array with `drivetrainConfig.left_port` to build `motorL_`. The two
    must be read together; either one alone says nothing about which wheel
    turns which way."""
    cfg = json.loads(_TOVEZ_JSON.read_text())
    content = gbc.generate(cfg, "data/robots/tovez.json")

    assert "void defaultMotorConfigs(msg::MotorConfig* out)" in content
    assert "msg::DrivetrainConfig defaultDrivetrainConfig()" in content
    assert "OtosBootConfig defaultOtosBootConfig()" in content

    # The mirror-mounted drive pair: opposite signs, matching the old-tree
    # evidence cited in clasi/issues/tovez-drive-motor-reversed-fwd-sign.md.
    # Port 1 is tovez's RIGHT wheel, port 2 its LEFT.
    assert "out[0].setFwdSign(1);" in content
    assert "out[1].setFwdSign(-1);" in content
    # Non-drive ports still carry the bench placeholder.
    assert "out[2].setFwdSign(1);" in content
    assert "out[3].setFwdSign(1);" in content

    # ...and the binding that gives those ports their wheel labels. Without
    # this pair the array above is ambiguous -- it was baking setLeftPort(1)
    # on a robot wired port 1 = right, which is the whole bug.
    assert "cfg.setLeftPort(2);" in content
    assert "cfg.setRightPort(1);" in content


def test_generate_raises_with_no_robot_config():
    """Sprint 114 (config-as-truth completion): generate() with no robot
    config found now hard-fails on the first required BEHAVIORAL key
    (geometry.trackwidth) before it ever reaches fwd_sign_for_ports() --
    "the build must still succeed with no robot JSON present" was this
    ticket's own explicit reversal (see gen_boot_config.py's module
    docstring); fwd_sign_for_ports() ITSELF is unaffected (a structural
    placeholder, out of this ticket's scope -- see
    test_fwd_sign_for_ports_falls_back_to_placeholder_for_every_port above,
    which still passes unchanged)."""
    with pytest.raises(gbc.MissingRobotConfigKeyError):
        gbc.generate({}, "(firmware defaults)")


if __name__ == "__main__":
    import pytest

    sys.exit(pytest.main([__file__, "-v"]))
