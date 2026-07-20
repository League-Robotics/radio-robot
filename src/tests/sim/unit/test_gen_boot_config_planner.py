"""src/tests/sim/unit/test_gen_boot_config_planner.py -- generator-level
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

Extended by ticket 100-001 (motion-stack-v2 M1): drive_limits_for_config()'s
17-field mapping for PlannerConfig's new fields 15-31 (Drive::Limits' wire/
config source, architecture-update.md M1/Decision 2) -- present-in-JSON
(tovez.json's real values, including the one bench-measured exception,
v_wheel_max) and absent-from-JSON/fallback-default cases, mirroring
test_heading_gains_for_config_reads_tovez_json()/
test_heading_gains_for_config_falls_back_to_firmware_defaults()'s exact
shape.

Reduced by ticket 111-004 (step 7 of the terminal-blips-close-the-loop fix
plan): 16 of the 17 Drive::Limits fields (v_wheel_max..arrive_vel_tol) were
never wired to any live consumer and were removed as dead wire fields --
drive_limits_for_config() no longer exists, replaced by the much smaller
arrive_dwell_for_config() (mirrors heading_dwell_for_config()'s shape). The
tests below now cover arrive_dwell alone, the one field from that original
span that IS live (Motion::Executor's dwell-completion gate). See
planner.proto's own PlannerConfig header comment for the full accounting.

Mirrors src/tests/sim/unit/test_gen_boot_config_fwd_sign.py's exact in-process
pattern (invokes the generator module directly rather than shelling out) and
is placed under src/tests/sim/unit/ for the same reason that file gives: this is
the sprint's scoped no-hardware gate (pyproject.toml's testpaths), and it is
a pure Python, generator-only test -- it does not use the `sim`/`build_lib`
fixtures (src/tests/sim/conftest.py) and does not need libfirmware_host built.

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
import math
import sys
from pathlib import Path

# src/tests/sim/unit/test_gen_boot_config_planner.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = Path(__file__).resolve().parents[4]
_SCRIPTS_DIR = _REPO_ROOT / "src" / "scripts"
_TOVEZ_JSON = _REPO_ROOT / "data" / "robots" / "tovez.json"

if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))

import gen_boot_config as gbc  # noqa: E402  (path must be set up before this import)

# The seven motion-limit values main.cpp's deleted defaultMotionConfig()
# used to hardcode (src/firm/main.cpp, pre-098-001) -- the regression pin.
_EXPECTED_MOTION_LIMITS = {
    "a_max": 800.0,          # [mm/s^2]
    "a_decel": 800.0,        # [mm/s^2]
    "v_body_max": 1000.0,    # [mm/s]
    "yaw_rate_max": 6.0,     # [rad/s]
    "yaw_acc_max": 20.0,     # [rad/s^2]
    "j_max": 5000.0,         # [mm/s^3]
    "yaw_jerk_max": 100.0,   # [rad/s^3]
}


# arrive_dwell (100-001) -- the sole survivor of the original 17-field
# Drive::Limits span (see module docstring's 111-004 note above): field
# name -> (setter, tovez value, firmware-default constant name). tovez.json's
# real value EQUALS the firmware default (both 0.15).
_ARRIVE_DWELL_FIELD = ("arrive_dwell", "setArriveDwell", 0.15, "ARRIVE_DWELL_DEFAULT")


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
    is intentionally nonzero, Kp=6.0 (112-004: bumped from the original
    3.0 starting value -- see HEADING_KP_DEFAULT's own comment for the
    deadband-inequality derivation this bump satisfies once App::Pilot's
    min-speed floor is deleted), per Decision 2's starting-gain policy)."""
    kp, kd = gbc.heading_gains_for_config({})

    assert kp == gbc.HEADING_KP_DEFAULT == 6.0
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


def test_generate_emits_default_planner_config_with_config_motion_limits():
    """generate()'s output gains Config::defaultPlannerConfig(). Five of the
    seven motion-limit fields (a_max/a_decel/v_body_max/j_max/yaw_jerk_max)
    stay firmware defaults; the two rotational-profile ceilings
    (yaw_rate_max/yaw_acc_max) now come from tovez.json's control block
    (ticket 100-014, deg->rad) -- control.yaw_rate_max=70 deg/s and
    control.max_rot_accel_dps2=600 deg/s^2 -- plus tovez.json's real heading
    gains, while the pre-existing generated functions (motor configs,
    drivetrain config, OTOS boot config) remain present and undisturbed."""
    cfg = json.loads(_TOVEZ_JSON.read_text())
    content = gbc.generate(cfg, "data/robots/tovez.json")

    # Additive: the existing generated functions are still emitted.
    assert "void defaultMotorConfigs(msg::MotorConfig* out)" in content
    assert "msg::DrivetrainConfig defaultDrivetrainConfig()" in content
    assert "OtosBootConfig defaultOtosBootConfig()" in content

    # New (098-001): the PlannerConfig boot-default generator function.
    assert "msg::PlannerConfig defaultPlannerConfig()" in content

    # Five firmware-default motion limits, unchanged.
    for field in ("a_max", "a_decel", "v_body_max", "j_max", "yaw_jerk_max"):
        setter = {"a_max": "setAMax", "a_decel": "setADecel",
                  "v_body_max": "setVBodyMax", "j_max": "setJMax",
                  "yaw_jerk_max": "setYawJerkMax"}[field]
        line = f"cfg.{setter}({gbc._f(_EXPECTED_MOTION_LIMITS[field])});"
        assert line in content, f"missing/changed motion-limit setter: {line}"

    # Two rotational-profile ceilings now read from tovez.json control (100-014).
    assert f"cfg.setYawRateMax({gbc._f(math.radians(70.0))});" in content
    assert f"cfg.setYawAccMax({gbc._f(math.radians(600.0))});" in content

    # heading_kp/heading_kd resolve to tovez.json's real bench-tuned values
    # (6.0/0.0, sprint 098-003) for the active robot config.
    assert "cfg.setHeadingKp(6.0f);" in content
    assert "cfg.setHeadingKd(0.0f);" in content


def test_generate_motion_limits_unchanged_with_no_robot_config():
    """With NO robot config, all seven motion-limit fields take their firmware
    defaults -- including yaw_rate_max/yaw_acc_max, which ARE robot-JSON-
    configurable as of ticket 100-014 (control.yaw_rate_max/max_rot_accel_dps2)
    but fall back to the 6.0 rad/s / 20.0 rad/s^2 firmware defaults when the
    keys are absent, exactly the same fall-back discipline as heading_kp/
    heading_kd (which fall back to 6.0/0.0 here -- 112-004 bumped
    HEADING_KP_DEFAULT from 3.0)."""
    content = gbc.generate({}, "(firmware defaults)")

    assert "msg::PlannerConfig defaultPlannerConfig()" in content
    for line in _motion_limit_setter_lines():
        assert line in content, f"missing/changed motion-limit setter: {line}"

    assert "cfg.setHeadingKp(6.0f);" in content
    assert "cfg.setHeadingKd(0.0f);" in content


# ---------------------------------------------------------------------------
# 100-014: profile_rot_limits_for_config() -- the rotational-profile ceiling
# (yaw_rate_max/yaw_acc_max) now read from control.* (deg->rad), not hardcoded.
# ---------------------------------------------------------------------------

def test_profile_rot_limits_for_config_reads_tovez_json():
    """profile_rot_limits_for_config() reads tovez.json's control.yaw_rate_max
    [deg/s] and control.max_rot_accel_dps2 [deg/s^2], converting to rad. Before
    100-014 these were silently hardcoded to 6.0 rad/s / 20.0 rad/s^2, ignoring
    the robot JSON and driving pivots at ~500 mm/s (unstable overshoot on the
    latent real plant); tovez's 70 deg/s -> ~78 mm/s at the wheels."""
    cfg = json.loads(_TOVEZ_JSON.read_text())

    yaw_rate_max, yaw_acc_max = gbc.profile_rot_limits_for_config(cfg)

    assert yaw_rate_max == math.radians(70.0)     # control.yaw_rate_max [deg/s]
    assert yaw_acc_max == math.radians(600.0)     # control.max_rot_accel_dps2 [deg/s^2]


def test_profile_rot_limits_for_config_falls_back_to_firmware_defaults():
    """With no control.yaw_rate_max/max_rot_accel_dps2 (or no robot config),
    both fall back to the rad-valued firmware defaults."""
    yaw_rate_max, yaw_acc_max = gbc.profile_rot_limits_for_config({})

    assert yaw_rate_max == gbc.YAW_RATE_MAX_DEFAULT == 6.0
    assert yaw_acc_max == gbc.YAW_ACC_MAX_DEFAULT == 20.0


def test_profile_rot_limits_for_config_reads_arbitrary_json_values():
    """Proves the mapping genuinely reads from the JSON (deg->rad), not merely
    returning the default."""
    cfg = {"control": {"yaw_rate_max": 90.0, "max_rot_accel_dps2": 360.0}}

    yaw_rate_max, yaw_acc_max = gbc.profile_rot_limits_for_config(cfg)

    assert yaw_rate_max == math.radians(90.0)
    assert yaw_acc_max == math.radians(360.0)


# ---------------------------------------------------------------------------
# 100-001 (motion-stack-v2 M1), reduced by 111-004: arrive_dwell_for_config()
# is the sole survivor of the original 17-field Drive::Limits mapping.
# ---------------------------------------------------------------------------

def test_arrive_dwell_for_config_reads_tovez_json():
    """arrive_dwell_for_config() reads tovez.json's real control.arrive_dwell
    starting value (0.15, which happens to equal the firmware default)."""
    cfg = json.loads(_TOVEZ_JSON.read_text())

    actual = gbc.arrive_dwell_for_config(cfg)

    assert actual == _ARRIVE_DWELL_FIELD[2]


def test_arrive_dwell_for_config_falls_back_to_firmware_default():
    """With no control.arrive_dwell key in the robot JSON (or no robot
    config at all), the field falls back to its firmware default constant --
    matching every other mapping's fall-back-to-firmware-default behavior in
    this generator."""
    actual = gbc.arrive_dwell_for_config({})

    expected_default = getattr(gbc, _ARRIVE_DWELL_FIELD[3])
    assert actual == expected_default


def test_arrive_dwell_for_config_reads_arbitrary_json_value():
    """Proves the mapping genuinely reads from the JSON (not merely always
    returning the default, which would be indistinguishable from the
    fallback test above since tovez.json's value happens to equal the
    firmware default)."""
    cfg = {"control": {"arrive_dwell": 0.33}}

    actual = gbc.arrive_dwell_for_config(cfg)

    assert actual == 0.33


def test_generate_emits_default_planner_config_with_arrive_dwell():
    """generate()'s output gains the arrive_dwell setter call inside
    defaultPlannerConfig(), resolving to tovez.json's real starting value --
    additive to the pre-existing motion-limit/heading-gain setters
    (test_generate_emits_default_planner_config_with_config_motion_limits
    above)."""
    cfg = json.loads(_TOVEZ_JSON.read_text())
    content = gbc.generate(cfg, "data/robots/tovez.json")

    _field, setter, expected, _default_name = _ARRIVE_DWELL_FIELD
    line = f"cfg.{setter}({gbc._f(expected)});"
    assert line in content, f"missing/changed arrive_dwell setter: {line}"


def test_generate_arrive_dwell_falls_back_with_no_robot_config():
    """With no robot config found, the arrive_dwell setter emits its
    firmware-default literal (mirrors
    test_generate_motion_limits_unchanged_with_no_robot_config's shape)."""
    content = gbc.generate({}, "(firmware defaults)")

    _field, setter, _tovez_value, default_name = _ARRIVE_DWELL_FIELD
    default_value = getattr(gbc, default_name)
    line = f"cfg.{setter}({gbc._f(default_value)});"
    assert line in content, f"missing/changed arrive_dwell default setter: {line}"


# ---------------------------------------------------------------------------
# 113-001: model_tau_for_config() -- App::Pilot's own two-stage model-
# reference feedback plant-lag time constants (msg::PlannerConfig fields
# 41/42), previously plain hardcoded pilot.h member initializers with no
# config path at all. Mirrors distance_gains_for_config()'s present/absent
# coverage style above.
# ---------------------------------------------------------------------------

def test_model_tau_for_config_reads_tovez_nocal_json():
    """model_tau_for_config() reads tovez_nocal.json's real
    control.model_tau_lin/control.model_tau_ang (0.1/0.08, added the same
    session that validated these SIM-VALIDATED motion values) -- which
    happen to equal the firmware defaults, so this alone doesn't prove the
    JSON path is read (see the arbitrary-value test below for that)."""
    nocal_json = _REPO_ROOT / "data" / "robots" / "tovez_nocal.json"
    cfg = json.loads(nocal_json.read_text())

    model_tau_lin, model_tau_ang = gbc.model_tau_for_config(cfg)

    assert model_tau_lin == 0.1
    assert model_tau_ang == 0.08


def test_model_tau_for_config_falls_back_to_firmware_defaults():
    """With no control.model_tau_lin/control.model_tau_ang in the robot JSON
    (or no robot config at all), both time constants fall back to the
    firmware defaults -- matching pilot.h's own prior hardcoded values
    exactly, so an unmigrated robot JSON boots byte-identical to before this
    ticket."""
    model_tau_lin, model_tau_ang = gbc.model_tau_for_config({})

    assert model_tau_lin == gbc.MODEL_TAU_LIN_DEFAULT == 0.10
    assert model_tau_ang == gbc.MODEL_TAU_ANG_DEFAULT == 0.08


def test_model_tau_for_config_reads_arbitrary_json_values():
    """Proves the mapping genuinely reads from the JSON (not merely always
    returning the default, which would be indistinguishable from the
    tovez_nocal.json test above since that file's committed values happen to
    equal the firmware defaults)."""
    cfg = {"control": {"model_tau_lin": 0.25, "model_tau_ang": 0.19}}

    model_tau_lin, model_tau_ang = gbc.model_tau_for_config(cfg)

    assert model_tau_lin == 0.25
    assert model_tau_ang == 0.19


def test_generate_emits_default_planner_config_with_model_tau():
    """generate()'s output gains the setModelTauLin()/setModelTauAng() setter
    calls inside defaultPlannerConfig(), resolving to tovez_nocal.json's real
    starting values -- additive to the pre-existing motion-limit/heading-
    gain/distance-gain setters covered above."""
    nocal_json = _REPO_ROOT / "data" / "robots" / "tovez_nocal.json"
    cfg = json.loads(nocal_json.read_text())
    content = gbc.generate(cfg, "data/robots/tovez_nocal.json")

    assert f"cfg.setModelTauLin({gbc._f(0.1)});" in content
    assert f"cfg.setModelTauAng({gbc._f(0.08)});" in content


def test_generate_model_tau_falls_back_with_no_robot_config():
    """With no robot config found, the model_tau setters emit their
    firmware-default literals (mirrors
    test_generate_arrive_dwell_falls_back_with_no_robot_config's shape)."""
    content = gbc.generate({}, "(firmware defaults)")

    assert f"cfg.setModelTauLin({gbc._f(gbc.MODEL_TAU_LIN_DEFAULT)});" in content
    assert f"cfg.setModelTauAng({gbc._f(gbc.MODEL_TAU_ANG_DEFAULT)});" in content


if __name__ == "__main__":
    import pytest

    sys.exit(pytest.main([__file__, "-v"]))
