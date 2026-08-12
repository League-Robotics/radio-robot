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

UPDATE (136-001, 2026-08-11): ``test_planner_config_for_config_reads_
tovez_json`` used to assert against ``_EXPECTED_RAW``, a hardcoded literal
snapshot of the values ``main.cpp``'s pre-129-009 block assigned. That
snapshot went stale the moment ``tovez.json`` was legitimately
re-measured/re-tuned after the migration -- ``control_period``/
``actuation_delay`` moved 50.0 -> 32.0 (2026-08-07, ``kCycle`` re-pace)
and ``heading_hold_gain`` was zeroed (130-011, a limit-cycle fix) -- the
exact failure mode ``clasi/issues/later/B-gen-boot-config-parity-tests-
encode-superseded-literals.md`` names for this file. The test now reads
``tovez.json`` at test time and asserts the generator faithfully carries
THOSE values through, so a legitimate re-measurement passes and a
generator defect (wrong key, dropped field) still fails.

``test_generate_emits_default_planner_limits_byte_identical_to_pre_
ticket_literals`` is RENAMED and TRIMMED to ``test_generate_emits_
default_planner_limits_with_130_009_removed_fields_absent``, not
re-pinned -- its own old name said what it was: a one-time refactor guard
proving 129-009's JSON migration moved main.cpp's literal block without
changing any value. That moment has long passed, and pinning the literal
values a second time would just reinstate the same failure at the next
legitimate re-measurement -- exactly what B-gen-boot-config-parity-tests-
encode-superseded-literals.md warns against. Every per-field literal
assertion is DROPPED (redundant with the rewritten dict-level test's own
exact-key-set check, which already fails if a removed field resurfaces
in ``planner_config_for_config()``'s return value); the one thing that
check alone could not catch -- a codegen TEMPLATE bug reintroducing a
removed field's literal into the generated C++ text without it ever
passing back through the dict -- is kept as this trimmed test's sole
remaining job. This resolves the first two of B-gen-boot-config-parity-
tests-encode-superseded-literals.md's three named tests (the third,
test_default_drive_group_matches_tovez_json, lives in test_gen_boot_
config_robot_groups.py and is fixed there the same way).
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


def test_planner_config_for_config_reads_tovez_json():
    """planner_config_for_config() reads tovez.json's real planner block --
    every field matches the JSON's OWN current value, read at test time
    (not a historical snapshot -- see this module's own dated note
    above)."""
    cfg = json.loads(_TOVEZ_JSON.read_text())

    result = gbc.planner_config_for_config(cfg)

    expected = {
        "vMax": float(cfg["planner"]["v_max"]),
        "aMax": float(cfg["planner_shaper"]["a_max"]),
        "aDecel": float(cfg["planner_shaper"]["a_decel"]),
        "omegaMax": float(cfg["planner"]["omega_max"]),
        "alphaMax": float(cfg["planner_shaper"]["alpha_max"]),
        "alphaDecel": float(cfg["planner_shaper"]["alpha_decel"]),
        "jerkMax": float(cfg["planner_shaper"]["jerk_max"]),
        "yawJerkMax": float(cfg["planner_shaper"]["yaw_jerk_max"]),
        # 130-007/2026-08-07: control_period/actuation_delay are
        # "= Core::RobotLoop::kCycle" by construction and move every time
        # the loop period does (they were 50.0, now 32.0) -- a hardcoded
        # literal here turns each legitimate re-pace into a spurious
        # failure, exactly what this module's dated note above describes.
        "controlPeriod": float(cfg["planner"]["control_period"]),
        "actuationDelay": float(cfg["planner"]["actuation_delay"]),
        "settleRestVelocity": float(cfg["planner"]["settle_rest_velocity"]),
        "settleRestOmega": float(cfg["planner"]["settle_rest_omega"]),
        "settleEpsilonLinear": float(cfg["planner"]["settle_epsilon_linear"]),
        "settleEpsilonAngular": float(cfg["planner"]["settle_epsilon_angular"]),
        # 130-011 zeroed this (limit-cycle fix) -- was 2.0 in the pre-ticket
        # main.cpp literal block this test used to pin against.
        "headingHoldGain": float(cfg["planner"]["heading_hold_gain"]),
        "decelPlanFraction": float(cfg["planner"]["decel_plan_fraction"]),
        # 134-003 terminal fine-align. alignTol is [rad]: 0.017453 rad IS
        # the report's 1.0 deg operating point, converted once in
        # tovez.json. If this assertion ever reads ~1.0 instead, someone
        # has stored degrees in a radian field -- a 57x error that nothing
        # downstream would catch.
        "alignTol": float(cfg["planner"]["align_tol"]),
        "alignMaxNudges": int(cfg["planner"]["align_max_nudges"]),
    }

    for field, exp in expected.items():
        assert result[field] == exp, f"{field}: {result[field]!r} != {exp!r}"
    assert set(result.keys()) == set(expected.keys()), (
        f"planner_config_for_config() field set changed: {sorted(result.keys())} != "
        f"{sorted(expected.keys())}"
    )


def test_planner_config_for_config_raises_with_no_robot_config():
    """Sprint 114 config-as-truth, extended to the planner block (129-009):
    with no robot config at all, the generator hard-fails on the first
    required key (planner.v_max, the first field this function's own
    returned dict lists) -- no source-side fallback.

    132-017 (JSON reshape ticket) found this assertion pre-existing-stale,
    unrelated to the reshape itself: `plant_gain` has not been read by
    `planner_config_for_config()` since 130-009 (it is no longer in the
    function's own required-key list at all, see that function's own
    docstring), so `planner.v_max` -- unchanged, first in the dict both
    before and after 130-009 -- was already the actual first-raised key
    pre-132-017 too. Fixed as a drive-by while retargeting this same
    function's JSON paths; not caused by the retarget."""
    with pytest.raises(gbc.MissingRobotConfigKeyError, match="planner.v_max"):
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


def test_generate_emits_default_planner_limits_with_130_009_removed_fields_absent():
    """generate()'s output gains defaultPlannerLimits(), carrying tovez.json's
    planner block through into the emitted C++ literals -- additive (the
    existing generated functions are still emitted, unchanged).

    RENAMED and TRIMMED, 136-001 (see this module's own dated note above):
    this used to be test_generate_emits_default_planner_limits_byte_
    identical_to_pre_ticket_literals, a one-time refactor guard pinning
    every emitted literal to 129-009's own pre-migration values. That
    guard's job ended the moment tovez.json was legitimately re-measured
    (see the dated note); what remains genuinely worth guarding at the
    STRING-output layer -- as opposed to the dict-output layer, already
    covered by test_planner_config_for_config_reads_tovez_json's own
    exact-key-set assertion -- is that 130-009's removed fields never
    resurface in the generated C++ text specifically (a template bug could
    reintroduce one without changing planner_config_for_config()'s own
    returned dict)."""
    cfg = json.loads(_TOVEZ_JSON.read_text())
    content = gbc.generate(cfg, "data/robots/tovez.json")

    # Additive: pre-existing generated functions are still emitted.
    assert "void defaultMotorConfigs(msg::MotorConfig* out)" in content
    assert "msg::DrivetrainConfig defaultDrivetrainConfig()" in content
    # "DriveBootConfig defaultDriveConfig()" -- DELETED, 132-015 (dead-code
    # sweep): Config::defaultDriveConfig() had zero remaining callers
    # (superseded by Config::defaultDriveGroup(), msg::Drive) and is no
    # longer emitted at all -- see config/boot_config.h's own note at that
    # struct's former declaration site.
    assert "PlannerBootConfig defaultPlannerLimits()" in content

    # 130-009: requireSettle/settleWindow, the M4 duty-stage gains (velKff/
    # velKp/velKi/velIMax/velKaff/velIAccelGate/dutyFloor), and the dead
    # planner-side trim gains (trimKp/trimKi/trimIMax/trimKaff/trimMax) no
    # longer appear anywhere in the generated PlannerBootConfig at all.
    for removed in ("requireSettle", "settleWindow", "velKff", "velKp",
                    "velKi", "velIMax", "velKaff", "velIAccelGate",
                    "dutyFloor", "trimKp", "trimKi", "trimIMax", "trimKaff",
                    "trimMax"):
        assert f"cfg.{removed} = " not in content, (
            f"cfg.{removed} still emitted -- 130-009 removed this field")


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
