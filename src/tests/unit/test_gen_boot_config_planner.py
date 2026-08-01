"""src/tests/unit/test_gen_boot_config_planner.py -- generator-level proof
for ticket 129-009 (config consolidation): scripts/gen_boot_config.py's new
planner_config_for_config() mapping and its wiring into
Config::defaultPlannerLimits()'s code generation.

Before this ticket, src/firm/main.cpp assembled the entire
Motion::PlannerLimits struct as C++ literals -- profile ceilings, loop
timing, settle/rest, plant model, duty-stage PID, trim loop -- while every
other per-robot boot default was already sourced from the active robot
JSON. This ticket moves that block into a `planner` section of
data/robots/*.json (robot_config.schema.json) and bakes it fail-closed via
gen_boot_config.py's planner_config_for_config() -> Config::
PlannerBootConfig / Config::defaultPlannerLimits().

This is the "host-side test comparing loader output to the recorded
pre-change literal values (byte-identical assertion)" the ticket's own
Testing section calls for -- mirrors src/tests/unit/test_gen_boot_config_otos.py's
in-process pattern (invokes the generator module directly, against the
real data/robots/tovez.json) rather than compiling firmware: since
boot_config.cpp is a pure function of gen_boot_config.py + the robot JSON,
proving the generator's output is unchanged (same values, new home) is
equivalent to proving Config::defaultPlannerLimits() itself.

Collected under src/tests/unit/ (a generator/tooling-level check, not
sim/bench/playfield-scoped -- see tests/CLAUDE.md); pyproject.toml's
testpaths includes tests/unit.
"""

import json
import sys
from pathlib import Path

import pytest

# src/tests/unit/test_gen_boot_config_planner.py -> unit -> tests -> repo root
_REPO_ROOT = Path(__file__).resolve().parents[3]
_SCRIPTS_DIR = _REPO_ROOT / "src" / "scripts"
_TOVEZ_JSON = _REPO_ROOT / "data" / "robots" / "tovez.json"

if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))

import gen_boot_config as gbc  # noqa: E402  (path must be set up before this import)


# The exact values main.cpp's deleted literal block assigned (see the
# ticket's own issue text and the pre-ticket git history of
# src/firm/main.cpp) -- tovez.json's new `planner` block carries these
# forward verbatim (value-preserving migration, no behavior change).
_EXPECTED_RAW = {
    "vMax": 400.0,
    "aMax": 300.0,
    "aDecel": 250.0,
    "omegaMax": 3.0,
    "alphaMax": 6.0,
    "alphaDecel": 5.0,
    "jerkMax": 1500.0,
    "yawJerkMax": 30.0,
    "controlPeriod": 47.0,
    "actuationDelay": 47.0,
    "requireSettle": False,
    "settleRestVelocity": 10.0,
    "settleRestOmega": 0.16,
    "settleWindow": 2500.0,
    "settleEpsilonLinear": 4.0,
    "settleEpsilonAngular": 0.035,
    "headingHoldGain": 2.0,
    "velKp": 0.0009,
    "velKi": 0.004,
    "velIMax": 0.25,
    "velIAccelGate": 50.0,
    "dutyFloor": 0.18,
    "trimKp": 0.15,
    "trimKi": 0.4,
    "trimIMax": 40.0,
    "trimMax": 80.0,
    "decelPlanFraction": 0.4,
}

# velKff/velKaff/trimKaff are DERIVED from the JSON's plant_gain=1370.0/
# plant_tau=0.23 -- exactly the arithmetic main.cpp's own deleted literal
# block used to do inline (kff = 1/gain, kaff = tau/gain, trimKaff = tau/2).
_PLANT_GAIN = 1370.0
_PLANT_TAU = 0.23
_EXPECTED_DERIVED = {
    "velKff": 1.0 / _PLANT_GAIN,
    "velKaff": _PLANT_TAU / _PLANT_GAIN,
    "trimKaff": _PLANT_TAU / 2.0,
}


def test_planner_config_for_config_reads_tovez_json():
    """planner_config_for_config() reads tovez.json's real planner block --
    every raw field byte-identical to the pre-ticket main.cpp literals, and
    every derived field computed from the JSON's plant_gain/plant_tau."""
    cfg = json.loads(_TOVEZ_JSON.read_text())

    result = gbc.planner_config_for_config(cfg)

    for field, expected in _EXPECTED_RAW.items():
        assert result[field] == expected, f"{field}: {result[field]!r} != {expected!r}"

    for field, expected in _EXPECTED_DERIVED.items():
        assert result[field] == pytest.approx(expected, rel=0, abs=0), (
            f"{field}: {result[field]!r} != {expected!r}"
        )


def test_planner_config_for_config_raises_with_no_robot_config():
    """Sprint 114 config-as-truth, extended to the planner block (129-009):
    with no robot config at all, the generator hard-fails on the first
    required key (planner.plant_gain, read before the raw profile-ceiling
    keys) -- no source-side fallback."""
    with pytest.raises(gbc.MissingRobotConfigKeyError, match="planner.plant_gain"):
        gbc.planner_config_for_config({})


def test_missing_whole_planner_block_fails_generator():
    """Booting with a `planner`-less JSON raises the configured boot fault
    (the ticket's own acceptance criterion, in its literal 'whole block
    missing' form, not just one field inside it) -- deleting the entire
    top-level `planner` key from an otherwise-complete robot JSON makes
    generate() raise MissingRobotConfigKeyError, never a silently-generated
    placeholder file."""
    cfg = json.loads(_TOVEZ_JSON.read_text())
    del cfg["planner"]

    with pytest.raises(gbc.MissingRobotConfigKeyError) as exc_info:
        gbc.generate(cfg, "data/robots/tovez.json")

    assert exc_info.value.key_path.startswith("planner.")


def test_generate_emits_default_planner_limits_byte_identical_to_pre_ticket_literals():
    """generate()'s output gains defaultPlannerLimits(), carrying tovez.json's
    planner block through into the emitted C++ literals -- additive (the
    existing generated functions are still emitted, unchanged), and
    byte-identical to the values main.cpp's deleted literal block used to
    assign directly (this ticket's own 'same values, new home' acceptance
    criterion)."""
    cfg = json.loads(_TOVEZ_JSON.read_text())
    content = gbc.generate(cfg, "data/robots/tovez.json")

    # Additive: pre-existing generated functions are still emitted.
    assert "void defaultMotorConfigs(msg::MotorConfig* out)" in content
    assert "msg::DrivetrainConfig defaultDrivetrainConfig()" in content
    assert "DriveBootConfig defaultDriveConfig()" in content

    # New (129-009): the planner boot-config generator function, carrying
    # tovez.json's real (pre-ticket-identical) values through as C++
    # literals -- see PlannerBootConfig's own doc comment
    # (src/firm/config/boot_config.h) for the field list.
    assert "PlannerBootConfig defaultPlannerLimits()" in content
    assert "cfg.vMax = 400.0f;" in content
    assert "cfg.aMax = 300.0f;" in content
    assert "cfg.aDecel = 250.0f;" in content
    assert "cfg.omegaMax = 3.0f;" in content
    assert "cfg.alphaMax = 6.0f;" in content
    assert "cfg.alphaDecel = 5.0f;" in content
    assert "cfg.jerkMax = 1500.0f;" in content
    assert "cfg.yawJerkMax = 30.0f;" in content
    assert "cfg.controlPeriod = 47.0f;" in content
    assert "cfg.actuationDelay = 47.0f;" in content
    assert "cfg.requireSettle = false;" in content
    assert "cfg.settleRestVelocity = 10.0f;" in content
    assert "cfg.settleRestOmega = 0.16f;" in content
    assert "cfg.settleWindow = 2500.0f;" in content
    assert "cfg.settleEpsilonLinear = 4.0f;" in content
    assert "cfg.settleEpsilonAngular = 0.035f;" in content
    assert "cfg.headingHoldGain = 2.0f;" in content
    # Derived (kff = 1/plant_gain, kaff = plant_tau/plant_gain,
    # trimKaff = plant_tau/2) -- these are the SAME float32-rounded values
    # `1.0f / 1370.0f` / `0.23f / 1370.0f` / `0.23f / 2` produced inline in
    # main.cpp's own deleted literal block (verified bit-identical via
    # gen_boot_config.py's existing _f() round-trip formatting).
    assert "cfg.velKff = 0.000729927f;" in content
    assert "cfg.velKp = 0.0009f;" in content
    assert "cfg.velKi = 0.004f;" in content
    assert "cfg.velIMax = 0.25f;" in content
    assert "cfg.velKaff = 0.0001678832f;" in content
    assert "cfg.velIAccelGate = 50.0f;" in content
    assert "cfg.dutyFloor = 0.18f;" in content
    assert "cfg.trimKp = 0.15f;" in content
    assert "cfg.trimKi = 0.4f;" in content
    assert "cfg.trimIMax = 40.0f;" in content
    assert "cfg.trimKaff = 0.115f;" in content
    assert "cfg.trimMax = 80.0f;" in content
    assert "cfg.decelPlanFraction = 0.4f;" in content


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
