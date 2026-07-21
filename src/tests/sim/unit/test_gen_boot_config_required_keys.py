"""src/tests/sim/unit/test_gen_boot_config_required_keys.py -- sprint 114
ticket 002's own core-guarantee test: "eliminate gen_boot_config.py
behavioral fallback defaults ... hard-fail build on missing required key."

Two things this file proves, per the ticket's own Testing section:

  1. **Fail-closed, one field at a time.** For every one of the ~29
     behavioral fields this ticket migrated off a Python-side ``*_DEFAULT``
     constant (velocity gains, trackwidth, OTOS offset/scale, motion
     limits, heading/distance gains and dwell, lead-compensation,
     actuation_lag, model_tau_lin/ang, arrive_dwell, min_speed -- see
     sprint.md's Architecture Boundary list), deleting that ONE key from an
     otherwise-complete robot JSON makes ``gen_boot_config.py`` exit/raise
     with a message naming the missing key and (via ``generate()``) the
     JSON path -- never a silently-generated placeholder file. Starts from
     ``data/robots/tovez_nocal.json`` (post-ticket, independently
     sufficient -- see the parametrized fixture below) and removes exactly
     one required key per case.

  2. **Value-preserving migration, verified by diff.** ``boot_config.cpp``
     generated from each of the three shipped profiles, AFTER this ticket's
     migration, is byte-identical to a golden snapshot captured from the
     SAME three profiles BEFORE this ticket's changes landed (this file's
     own ``fixtures/boot_config_golden_*.cpp``) -- proving no robot's
     compiled behavior changed on its next reflash, not merely asserting it.

Mirrors src/tests/sim/unit/test_gen_boot_config_planner.py's own in-process
pattern (invokes the generator module directly rather than shelling out)
and this project's placement convention: a pure-Python, generator-only test
belongs under src/tests/sim/unit/ (pyproject.toml's testpaths), not
gated on the compiled sim lib.

UPDATE (sprint 114 ticket 003, "eliminate nezha_motor.h's write-shaping ship
defaults"): ``control.output_deadband``/``control.reversal_dwell_ms`` joined
``_REQUIRED_KEY_PATHS`` below the same way -- Devices::NezhaMotor's own
kDefaultOutputDeadband/kDefaultReversalDwell ship-default substitution is
gone (folded into ``NezhaMotor::reconfigure()``, sprint 114 ticket 001's own
Revision 1), so these two fields now fail the build exactly like every
other required key here. The golden fixtures
(``fixtures/boot_config_golden_*.cpp``) were regenerated to include the new
``setOutputDeadband()``/``setReversalDwell()`` calls this ticket added to
``defaultMotorConfigs()`` -- still value-preserving (0.03 / 100.0, the same
numbers the deleted ship defaults used).
"""

import copy
import json
import sys
from pathlib import Path

import pytest

# src/tests/sim/unit/test_gen_boot_config_required_keys.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = Path(__file__).resolve().parents[4]
_SCRIPTS_DIR = _REPO_ROOT / "src" / "scripts"
_ROBOTS_DIR = _REPO_ROOT / "data" / "robots"
_FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures"

if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))

import gen_boot_config as gbc  # noqa: E402  (path must be set up before this import)


def _complete_cfg() -> dict:
    """A fresh, deep copy of tovez_nocal.json -- post-ticket, this file is
    independently sufficient (every field below present) -- so each
    parametrized case can delete exactly one key without disturbing any
    other test's own copy."""
    return copy.deepcopy(json.loads((_ROBOTS_DIR / "tovez_nocal.json").read_text()))


def _delete_key(cfg: dict, *path: str) -> None:
    """Delete the leaf key at *path* (e.g. ``_delete_key(cfg, "control",
    "vel_kp")`` deletes ``cfg["control"]["vel_kp"]``) in place."""
    cur = cfg
    for key in path[:-1]:
        cur = cur[key]
    del cur[path[-1]]


# Every field this ticket migrated off a *_DEFAULT Python fallback, as a
# dotted JSON key path -- one parametrized case per field, per the ticket's
# own Testing section ("New tests to write: one parametrized test case per
# migrated field"). Excludes the structural/documented-exception fields
# (K_MOTOR_COUNT, LEFT_PORT/RIGHT_PORT, TRAVEL_CALIB_PLACEHOLDER, FWD_SIGN)
# and the drive-pair calibration.mm_per_wheel_deg_*/fwd_sign_* keys (out of
# this ticket's explicit scope -- see sprint.md ticket 002's Approach step 1
# and the "documented exception" paragraph of the Architecture Boundary
# list).
_REQUIRED_KEY_PATHS = [
    ("geometry", "trackwidth"),
    ("control", "vel_kp"),
    ("control", "vel_ki"),
    ("control", "vel_kff"),
    ("control", "vel_imax"),
    ("control", "vel_kaw"),
    ("control", "vel_filt"),
    ("control", "output_deadband"),
    ("control", "reversal_dwell_ms"),
    ("geometry", "odometry_offset_mm", "x"),
    ("geometry", "odometry_offset_mm", "y"),
    ("geometry", "odometry_offset_mm", "yaw_rad"),
    ("calibration", "otos_linear_scale"),
    ("calibration", "otos_angular_scale"),
    ("control", "a_max"),
    ("control", "a_decel"),
    ("control", "v_body_max"),
    ("control", "yaw_rate_max"),
    ("control", "max_rot_accel_dps2"),
    ("control", "j_max"),
    ("control", "yaw_jerk_max"),
    ("control", "heading_kp"),
    ("control", "heading_kd"),
    ("control", "heading_source"),
    ("control", "heading_dwell_tol_deg"),
    ("control", "heading_dwell_rate_dps"),
    ("control", "heading_lead_bias"),
    ("control", "plan_lead"),
    ("control", "terminal_lead"),
    ("control", "actuation_lag"),
    ("control", "distance_kp"),
    ("control", "distance_tol"),
    ("control", "model_tau_lin"),
    ("control", "model_tau_ang"),
    ("control", "arrive_dwell"),
    ("control", "min_speed"),
]


@pytest.mark.parametrize(
    "key_path", _REQUIRED_KEY_PATHS, ids=[".".join(p) for p in _REQUIRED_KEY_PATHS]
)
def test_missing_required_key_fails_generator(key_path):
    """Deleting this ONE key from an otherwise-complete robot JSON makes
    generate() raise MissingRobotConfigKeyError naming exactly this key and
    the JSON source path -- never a silently-generated placeholder file."""
    cfg = _complete_cfg()
    _delete_key(cfg, *key_path)

    with pytest.raises(gbc.MissingRobotConfigKeyError) as exc_info:
        gbc.generate(cfg, "data/robots/tovez_nocal.json")

    assert exc_info.value.key_path == ".".join(key_path)
    assert exc_info.value.source_path == "data/robots/tovez_nocal.json"
    assert ".".join(key_path) in str(exc_info.value)
    assert "data/robots/tovez_nocal.json" in str(exc_info.value)


@pytest.mark.parametrize(
    "key_path", _REQUIRED_KEY_PATHS, ids=[".".join(p) for p in _REQUIRED_KEY_PATHS]
)
def test_null_required_key_fails_generator_the_same_as_missing(key_path):
    """A key present but explicitly ``null`` is treated identically to an
    absent key (``_require()``'s own ``cur[k] is None`` check) -- a robot
    JSON that carries the key with a JSON ``null`` value (e.g. an
    unmigrated template) must not silently pass a None through into
    ``float()``/``math.radians()``."""
    cfg = _complete_cfg()
    cur = cfg
    for key in key_path[:-1]:
        cur = cur[key]
    cur[key_path[-1]] = None

    with pytest.raises(gbc.MissingRobotConfigKeyError) as exc_info:
        gbc.generate(cfg, "data/robots/tovez_nocal.json")

    assert exc_info.value.key_path == ".".join(key_path)


def test_complete_cfg_fixture_is_independently_sufficient():
    """Sanity check for the fixture itself: an untouched _complete_cfg()
    copy must generate successfully (no missing key) -- otherwise every
    parametrized case above would trivially "pass" for the wrong reason
    (the fixture itself already broken, not the one deleted key)."""
    content = gbc.generate(_complete_cfg(), "data/robots/tovez_nocal.json")

    assert "msg::PlannerConfig defaultPlannerConfig()" in content


# ---------------------------------------------------------------------------
# Value-preserving migration -- byte-identical regression pin.
#
# fixtures/boot_config_golden_<profile>.cpp were captured by running THIS
# ticket's pre-migration gen_boot_config.py (the version with the *_DEFAULT
# Python fallback constants still in place, one commit before this ticket)
# against each of the three shipped profiles via
# ``ROBOT_CONFIG=data/robots/<profile>.json python3 src/scripts/gen_boot_config.py``.
# Regenerating the SAME three profiles against the post-migration generator
# and post-migration JSON (this ticket's own value-preserving key additions)
# must produce byte-identical output -- proving the migration changed WHERE
# each value lives, not WHAT it is.
# ---------------------------------------------------------------------------

_GOLDEN_PROFILES = ["tovez_nocal", "tovez", "togov"]


@pytest.mark.parametrize("profile", _GOLDEN_PROFILES)
def test_boot_config_cpp_byte_identical_to_pre_ticket_golden(profile):
    cfg = json.loads((_ROBOTS_DIR / f"{profile}.json").read_text())
    golden = (_FIXTURES_DIR / f"boot_config_golden_{profile}.cpp").read_text()

    actual = gbc.generate(cfg, f"data/robots/{profile}.json")

    # The golden snapshot's own "// Source: ..." header line carries
    # whatever REPO_ROOT-relative path _display_path() produced when the
    # snapshot was captured; regenerate golden's own header line here so
    # the comparison is scoped to every OTHER line -- the actual behavioral
    # content -- without being sensitive to which display-path string this
    # test happens to pass in versus what main() would resolve at build
    # time (both are cosmetic strings, not behavior).
    golden_lines = golden.splitlines()
    actual_lines = actual.splitlines()
    assert len(golden_lines) == len(actual_lines), (
        f"{profile}: line count changed ({len(golden_lines)} -> {len(actual_lines)}) "
        "-- migration was NOT value-preserving"
    )
    for i, (g, a) in enumerate(zip(golden_lines, actual_lines)):
        if i == 2:
            # "// Source: <path>" -- cosmetic, both point at the same file.
            assert g.startswith("// Source:") and a.startswith("// Source:")
            continue
        assert g == a, (
            f"{profile}: boot_config.cpp line {i + 1} differs from the "
            f"pre-ticket golden snapshot -- migration was NOT value-preserving\n"
            f"golden: {g!r}\nactual: {a!r}"
        )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
