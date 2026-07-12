"""tests/sim/unit/test_gen_boot_config_planner.py -- generator-level
regression pin for ticket 098-001 (SUC-001/SUC-003): scripts/
gen_boot_config.py's new Config::defaultPlannerConfig() code path.

Before this ticket, main.cpp hand-wrote every msg::PlannerConfig field in a
local defaultMotionConfig() function OUTSIDE this generator -- the one boot
default that didn't go through the robot-JSON-driven path every other
per-robot tunable (velocity PID gains, trackwidth, fwd_sign, OTOS boot
config, ...) already used. This ticket moves the seven motion-limit fields
(a_max/a_decel/v_body_max/yaw_rate_max/yaw_acc_max/j_max/yaw_jerk_max) into
this generator VERBATIM (same numeric values, same units -- see
architecture-update.md M2) and adds two new per-robot heading-loop PD gains
(heading_kp/heading_kd, architecture-update.md M1).

Purpose of this file: prove the move introduced NO silent value change (the
seven motion-limit literals below are the exact values main.cpp's deleted
defaultMotionConfig() used to hardcode) and that heading_kp/heading_kd
resolve to the tovez.json starting values (3.0/0.0) for the active robot
config -- this ticket's own acceptance criteria.

Mirrors tests/sim/unit/test_gen_boot_config_fwd_sign.py's exact in-process
pattern (invokes the generator module directly rather than shelling out) and
is placed under tests/sim/unit/ for the same reason that file gives: this is
the sprint's scoped no-hardware gate (pyproject.toml's testpaths), and it is
a pure Python, generator-only test -- it does not use the `sim`/`build_lib`
fixtures (tests/sim/conftest.py) and does not need libfirmware_host built.

NOTE (scope): this file asserts only that gen_boot_config.py's generated C++
source text carries the right literal values through to
Config::defaultPlannerConfig() -- it does not compile/link/call the actual
generated function. Doing so would require a pybind-exposed sim harness,
which is unnecessary here: the C++ setters (msg::PlannerConfig::setAMax()
etc.) are generator (gen_messages.py) output already covered by other
tests, and Drivetrain::configureMotion()'s "pass the whole struct through"
behavior is unchanged by this ticket (main.cpp just calls a different
factory function returning the same msg::PlannerConfig type).
"""

import json
import sys
from pathlib import Path

# tests/sim/unit/test_gen_boot_config_planner.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = Path(__file__).resolve().parents[3]
_SCRIPTS_DIR = _REPO_ROOT / "scripts"
_TOVEZ_JSON = _REPO_ROOT / "data" / "robots" / "tovez.json"

if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))

import gen_boot_config as gbc  # noqa: E402  (path must be set up before this import)

# The seven motion-limit values main.cpp's deleted defaultMotionConfig()
# used to hardcode (source/main.cpp, pre-098-001) -- the regression pin.
_EXPECTED_MOTION_LIMITS = {
    "a_max": 800.0,          # [mm/s^2]
    "a_decel": 800.0,        # [mm/s^2]
    "v_body_max": 1000.0,    # [mm/s]
    "yaw_rate_max": 6.0,     # [rad/s]
    "yaw_acc_max": 20.0,     # [rad/s^2]
    "j_max": 5000.0,         # [mm/s^3]
    "yaw_jerk_max": 100.0,   # [rad/s^3]
}


def _motion_limit_setter_lines() -> list[str]:
    """The exact `cfg.set<Field>(<value>f);` lines defaultPlannerConfig()
    must emit for every motion-limit field, matching gen_messages.py's
    chainable-setter naming (setAMax, setADecel, ...)."""
    return [
        f"cfg.setAMax({gbc._f(_EXPECTED_MOTION_LIMITS['a_max'])});",
        f"cfg.setADecel({gbc._f(_EXPECTED_MOTION_LIMITS['a_decel'])});",
        f"cfg.setVBodyMax({gbc._f(_EXPECTED_MOTION_LIMITS['v_body_max'])});",
        f"cfg.setYawRateMax({gbc._f(_EXPECTED_MOTION_LIMITS['yaw_rate_max'])});",
        f"cfg.setYawAccMax({gbc._f(_EXPECTED_MOTION_LIMITS['yaw_acc_max'])});",
        f"cfg.setJMax({gbc._f(_EXPECTED_MOTION_LIMITS['j_max'])});",
        f"cfg.setYawJerkMax({gbc._f(_EXPECTED_MOTION_LIMITS['yaw_jerk_max'])});",
    ]


def test_heading_gains_for_config_reads_tovez_json():
    """heading_gains_for_config() reads tovez.json's real control.heading_kp/
    heading_kd -- the active robot's BENCH-TUNED values (6.0/0.0, sprint
    098-003: kp raised from the 3.0 starting value to overcome terminal motor
    stiction; full turn grid then landed 100% within +/-1 deg)."""
    cfg = json.loads(_TOVEZ_JSON.read_text())

    kp, kd = gbc.heading_gains_for_config(cfg)

    assert kp == 6.0
    assert kd == 0.0


def test_heading_gains_for_config_falls_back_to_firmware_defaults():
    """With no control.heading_kp/heading_kd in the robot JSON (or no robot
    config at all), both gains fall back to the conservative firmware
    defaults -- matching every other mapping's fall-back-to-firmware-default
    behavior in this generator (an unmigrated robot JSON simply inherits
    today's open-loop-equivalent Kp=Kd=0... except the firmware default here
    is intentionally nonzero, Kp=3.0, per Decision 2's starting-gain policy)."""
    kp, kd = gbc.heading_gains_for_config({})

    assert kp == gbc.HEADING_KP_DEFAULT == 3.0
    assert kd == gbc.HEADING_KD_DEFAULT == 0.0


def test_heading_gains_for_config_reads_arbitrary_json_values():
    """Proves the mapping genuinely reads from the JSON (not merely always
    returning the default, which would be indistinguishable from the
    fallback test above since tovez.json's committed starting values happen
    to equal the firmware defaults)."""
    cfg = {"control": {"heading_kp": 5.5, "heading_kd": 1.25}}

    kp, kd = gbc.heading_gains_for_config(cfg)

    assert kp == 5.5
    assert kd == 1.25


def test_generate_emits_default_planner_config_with_unchanged_motion_limits():
    """generate()'s output gains Config::defaultPlannerConfig(), carrying the
    seven motion-limit fields through with the EXACT pre-ticket hardcoded
    values (the regression pin against a silent value change during the
    move off main.cpp) plus tovez.json's real heading gains -- while the
    pre-existing generated functions (motor configs, drivetrain config, OTOS
    boot config) remain present and undisturbed."""
    cfg = json.loads(_TOVEZ_JSON.read_text())
    content = gbc.generate(cfg, "data/robots/tovez.json")

    # Additive: the existing generated functions are still emitted.
    assert "void defaultMotorConfigs(msg::MotorConfig* out)" in content
    assert "msg::DrivetrainConfig defaultDrivetrainConfig()" in content
    assert "OtosBootConfig defaultOtosBootConfig()" in content

    # New (098-001): the PlannerConfig boot-default generator function.
    assert "msg::PlannerConfig defaultPlannerConfig()" in content

    for line in _motion_limit_setter_lines():
        assert line in content, f"missing/changed motion-limit setter: {line}"

    # heading_kp/heading_kd resolve to tovez.json's real bench-tuned values
    # (6.0/0.0, sprint 098-003) for the active robot config.
    assert "cfg.setHeadingKp(6.0f);" in content
    assert "cfg.setHeadingKd(0.0f);" in content


def test_generate_motion_limits_unchanged_with_no_robot_config():
    """The seven motion-limit fields are firmware defaults, NOT robot-JSON-
    configurable (unlike heading_kp/heading_kd) -- they must be identical
    whether or not a robot config is found, exactly reproducing the
    pre-ticket main.cpp::defaultMotionConfig() behavior (which never read
    any robot JSON at all). heading_kp/heading_kd fall back to the firmware
    defaults (3.0/0.0) -- the conservative starting values for an
    uncharacterized robot; tovez.json overrides kp to its bench-tuned 6.0
    (098-003), so the firmware default deliberately no longer matches it."""
    content = gbc.generate({}, "(firmware defaults)")

    assert "msg::PlannerConfig defaultPlannerConfig()" in content
    for line in _motion_limit_setter_lines():
        assert line in content, f"missing/changed motion-limit setter: {line}"

    assert "cfg.setHeadingKp(3.0f);" in content
    assert "cfg.setHeadingKd(0.0f);" in content


if __name__ == "__main__":
    import pytest

    sys.exit(pytest.main([__file__, "-v"]))
