"""src/tests/sim/unit/test_gen_boot_config_required_keys.py -- sprint 114
ticket 002's own core-guarantee test: "eliminate gen_boot_config.py
behavioral fallback defaults ... hard-fail build on missing required key."

**Fail-closed, one field at a time.** For every one of the behavioral
fields this ticket migrated off a Python-side ``*_DEFAULT`` constant
(velocity gains, trackwidth, OTOS offset/scale, output_deadband,
reversal_dwell_ms -- see sprint.md's Architecture Boundary list), deleting
that ONE key from an otherwise-complete robot JSON makes
``gen_boot_config.py`` exit/raise with a message naming the missing key and
(via ``generate()``) the JSON path -- never a silently-generated
placeholder file. Starts from ``data/robots/tovez_nocal.json`` (post-ticket,
independently sufficient -- see the parametrized fixture below) and removes
exactly one required key per case.

Mirrors src/tests/sim/unit/test_gen_boot_config_fwd_sign.py's own in-process
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
other required key here.

REDUCED (115-009, gut S1's own test-sweep/green-bar ticket): the ~21
``control.*`` PlannerConfig-only required-key cases (a_max/a_decel/
v_body_max/yaw_rate_max/max_rot_accel_dps2/j_max/yaw_jerk_max/heading_kp/
heading_kd/heading_source/heading_dwell_tol_deg/heading_dwell_rate_dps/
heading_lead_bias/plan_lead/terminal_lead/actuation_lag/distance_kp/
distance_tol/model_tau_lin/model_tau_ang/arrive_dwell/min_speed) are
REMOVED, not ported -- ``gen_boot_config.py``'s ``defaultPlannerConfig()``
emission (and every ``_require()`` call reading these keys) was deleted
wholesale (115-003, gut-to-minimal-firmware S1 motion-stack excision):
`msg::PlannerConfig` has no boot-config consumer left, so the generator no
longer reads (and therefore can no longer fail-closed on) any of these
keys. ``_REQUIRED_KEY_PATHS`` below now lists exactly the fields
``gen_boot_config.py``'s own ``_require()`` calls still read (verified
directly against that file -- ``otos_boot_config_values()``,
``vel_gains_for_config()``, ``output_deadband_for_config()``,
``reversal_dwell_for_config()``, ``trackwidth_for_config()``).

The "value-preserving migration, verified by diff against a pre-ticket
golden snapshot" section this file used to carry (``fixtures/
boot_config_golden_*.cpp`` + ``test_boot_config_cpp_byte_identical_to_pre_
ticket_golden``) is DELETED, not ported: that regression pin existed to
prove sprint 114's specific required-key migration changed WHERE each value
lived, not WHAT it was -- a "byte-identical output" claim. This sprint's
own change (removing `defaultPlannerConfig()` emission entirely) is a large,
intentional, DOCUMENTED behavior change to `boot_config.cpp`'s generated
content (99 lines vs. the golden snapshot's 178) -- the golden fixtures
would need a wholesale re-capture to mean anything post-gut, and re-pinning
against a NEW golden snapshot captured from the post-gut generator would
just prove the generator agrees with itself, not guard anything the
required-key tests above don't already cover.
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
    (the fixture itself already broken, not the one deleted key). Checks for
    ``defaultOtosBootConfig()`` (the generator's last emitted function,
    115-009) rather than the deleted ``defaultPlannerConfig()``."""
    content = gbc.generate(_complete_cfg(), "data/robots/tovez_nocal.json")

    assert "OtosBootConfig defaultOtosBootConfig()" in content


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
