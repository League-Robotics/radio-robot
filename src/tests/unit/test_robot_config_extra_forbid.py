"""src/tests/unit/test_robot_config_extra_forbid.py -- 132-016 (Host cleanup:
extra='forbid', drift assertions).

Covers this ticket's own Acceptance Criteria:

1. Every group model (generated, in ``robot_config_generated.py``, and
   hand-declared, in ``robot_config.py``) plus ``RobotConfig`` itself sets
   ``model_config = ConfigDict(extra="forbid")`` -- an unrecognized key
   anywhere in a robot JSON now raises loudly instead of pydantic's former
   default ``extra='ignore'`` silently dropping it.
2. The fields that were previously silently dropped by the OLD
   hand-written ``ControlConfig`` (36 fields, where the robot JSON's
   ``control`` section has 53 keys) are proven present on the generated
   model where they are load-bearing -- and, just as importantly, this
   ticket's own re-verification of "the other 16" (re-derived below, NOT
   assumed from the issue's prose) found they are genuinely dead
   (``robot_config.proto``'s own header checklist: no ``_require()``/
   ``_get()`` call site in ``gen_boot_config.py``) and correctly stay
   ABSENT from the new schema by design, not by oversight. Only 2 of the
   18 previously-dropped keys were load-bearing:
   ``output_deadband``/``reversal_dwell`` (JSON: ``reversal_dwell_ms``,
   renamed per naming-and-style.md's "no units in identifiers").
3. Loading a real, on-disk robot JSON (still in the OLD 13-section shape --
   ticket 017's reshape has not landed) is expected to FAIL extra='forbid'
   validation until then; see ``test_robot_config.py``'s own
   ``_XFAIL_UNTIL_017`` marker for the pre-existing tests this ticket's
   change blocks, and ``test_real_robot_json_rejected_until_017_reshape``
   below for a direct, affirmative demonstration of the new behavior
   itself (not xfail -- raising is the CORRECT, current, intended outcome,
   so asserting it is a normal passing test, not an expected failure).

Re-verification of the 18 previously-dropped ``control.*`` keys (not
assumed from memory, per this ticket's own instruction): computed as
``set(tovez.json's control.* keys, excluding leading-underscore comment
keys) - set(the OLD ControlConfig's 36 field names)``, the OLD model read
from git commit ``47ef2221^:src/host/robot_radio/config/robot_config.py``
(the last commit before ticket 020 deleted it). Both counts (53, 36) and
the resulting drop count (18) match the issue's own audit
(``the-configuration-object.md``: "the host pydantic model has 36 control
fields where the JSON has 53 and silently drops 18").

Collected under ``src/tests/unit/`` -- ``pyproject.toml``'s ``testpaths``
includes ``tests/unit``, so ``uv run python -m pytest`` collects it by
default.
"""

from pathlib import Path

import pytest
from pydantic import ValidationError

from robot_radio.config.robot_config import GeometryConfig, RobotConfig, load_robot_config
from robot_radio.config.robot_config_generated import (
    Connection,
    Drive,
    Estimator,
    Geometry,
    Identity,
    Motors,
    Otos,
    Planner,
    Vision,
    WheelControl,
)

_REPO_ROOT = Path(__file__).resolve().parents[3]
_ROBOTS_DIR = _REPO_ROOT / "data" / "robots"

_MINIMAL_IDENTITY = {"robot_name": "test-bot", "uid": "test-bot"}

_ALL_GENERATED_GROUPS = (
    Identity, Connection, Vision, Geometry, Motors, Drive, WheelControl,
    Planner, Otos, Estimator,
)

# ---------------------------------------------------------------------------
# The 18 previously-silently-dropped `control.*` keys, split by this
# ticket's own re-verification (see module docstring for how the 18 were
# re-derived) into the 2 that are load-bearing (gen_boot_config.py
# REQUIRES them -- carried into the new schema under a renamed, unit-free
# field) and the 16 that are genuinely dead (no live consumer -- correctly
# excluded from the new schema, not silently reconciled).
# ---------------------------------------------------------------------------

# old JSON key -> (new generated group class, new field name)
_RECOVERED_LOAD_BEARING_KEYS = {
    "output_deadband": (Motors, "output_deadband"),
    # "_ms" suffix dropped per naming-and-style.md ("no units in ANY
    # identifier") -- robot_config.proto's own header comment names this
    # exact rename explicitly.
    "reversal_dwell_ms": (Motors, "reversal_dwell"),
}

# The other 16 of the 18 -- confirmed absent from every one of the 10
# generated groups by full-text reading of robot_config.proto (2026-08):
# none of these names, or any obvious rename of them, appears anywhere in
# the schema. robot_config.proto's own header checklist independently
# confirms the reason: none has a `_require()`/`_get()` call site in
# gen_boot_config.py today.
_DEAD_DROPPED_KEYS = (
    "arrive_vel_tol",
    "handoff_tol_pos",
    "handoff_tol_v",
    "replan_err_pos",
    "replan_err_theta",
    "replan_hold",
    "replan_max",
    "replan_min_period",
    "steer_headroom",
    "track_k_cross",
    "track_k_s",
    "track_k_theta",
    "trim_omega_max",
    "trim_v_max",
    "v_wheel_max",
    "wheel_step_max",
)


def test_eighteen_previously_dropped_keys_partition_correctly():
    """Sanity check on the module-level tables themselves: together the
    recovered (2) and dead (16) buckets account for exactly the 18 keys
    the issue's audit measured -- guards against a copy/paste slip in
    either list drifting the count silently."""
    assert len(_RECOVERED_LOAD_BEARING_KEYS) == 2
    assert len(_DEAD_DROPPED_KEYS) == 16
    assert len(_RECOVERED_LOAD_BEARING_KEYS) + len(_DEAD_DROPPED_KEYS) == 18


@pytest.mark.parametrize(
    "old_key,expected", _RECOVERED_LOAD_BEARING_KEYS.items(),
    ids=list(_RECOVERED_LOAD_BEARING_KEYS),
)
def test_load_bearing_dropped_field_now_present(old_key, expected):
    """output_deadband/reversal_dwell_ms -- REQUIRED by gen_boot_config.py
    (it refuses to build without them) -- are structurally present on the
    generated model, under their new group and (for reversal_dwell_ms)
    their new unit-free name. This should already be true once ticket
    002's schema is generated; this test PROVES it rather than assuming
    it."""
    group_cls, new_field_name = expected
    assert new_field_name in group_cls.model_fields, (
        f"{old_key} (gen_boot_config.py REQUIRES it) has no field "
        f"{new_field_name!r} on {group_cls.__name__} -- would be silently "
        f"dropped again"
    )


@pytest.mark.parametrize("dead_key", _DEAD_DROPPED_KEYS)
def test_dead_dropped_key_correctly_stays_excluded(dead_key):
    """The other 16 of the 18 previously-dropped keys have NO live
    consumer (robot_config.proto's own header checklist) and were
    deliberately NOT carried into the new schema -- unlike
    output_deadband/reversal_dwell above. This is the ticket's own "prove
    it, don't assume it" instruction cutting the other way: the issue's
    premise that all 18 "are now present on the generated model" does not
    hold for these 16, and that is the CORRECT outcome (dead data should
    not be resurrected into a schema meant to mirror only what firmware
    actually reads), not a residual gap. Asserted absent so a future
    accidental re-add is a deliberate schema decision, not silent drift."""
    assert not any(dead_key in g.model_fields for g in _ALL_GENERATED_GROUPS), (
        f"{dead_key!r} reappeared in a generated group -- if this is "
        f"intentional (a live consumer was found), update this test and "
        f"its docstring; if not, it is drift back into the dead-key class "
        f"this ticket audited away"
    )


# ---------------------------------------------------------------------------
# AC 1 -- extra='forbid' is actually set, everywhere RobotConfig composes.
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "group_cls", _ALL_GENERATED_GROUPS, ids=[g.__name__ for g in _ALL_GENERATED_GROUPS]
)
def test_every_generated_group_forbids_extra(group_cls):
    assert group_cls.model_config.get("extra") == "forbid", (
        f"{group_cls.__name__} does not set extra='forbid'"
    )


def test_geometry_config_inherits_forbid_from_generated_geometry():
    assert issubclass(GeometryConfig, Geometry)
    assert GeometryConfig.model_config.get("extra") == "forbid"


def test_robot_config_root_forbids_extra():
    assert RobotConfig.model_config.get("extra") == "forbid"


def test_unrecognized_key_anywhere_raises_loudly():
    """The headline behavior this ticket exists for: an unrecognized key,
    anywhere in the object graph, fails LOUDLY instead of vanishing."""
    with pytest.raises(ValidationError):
        RobotConfig.model_validate({
            "identity": _MINIMAL_IDENTITY,
            "motors": {"not_a_real_field": 1.0},
        })


def test_unrecognized_top_level_section_raises_loudly():
    """A stray/mistyped top-level section name (not just a key inside a
    known group) is caught too -- RobotConfig itself forbids extra, not
    just its sub-groups."""
    with pytest.raises(ValidationError):
        RobotConfig.model_validate({
            "identity": _MINIMAL_IDENTITY,
            "not_a_real_section": {},
        })


# ---------------------------------------------------------------------------
# AC 3 -- current robot JSONs are expected to FAIL extra='forbid' until
# ticket 017's JSON reshape. Documented here as a direct, affirmative
# demonstration (raising IS the currently-correct behavior, so this is a
# normal passing test, not an xfail) -- see test_robot_config.py's
# _XFAIL_UNTIL_017 marker for the pre-existing tests this same fact
# blocks.
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("robot_json", ["tovez.json", "tovez_nocal.json", "togov.json"])
def test_real_robot_json_rejected_until_017_reshape(robot_json):
    """data/robots/*.json are still in the OLD 13-section shape (ticket
    017's JSON reshape has not landed): the old calibration/control
    top-level sections have no matching RobotConfig field at all, and
    several reshaped groups (connection/vision/geometry) carry old-shape
    keys the new schema doesn't recognize either. extra='forbid' (this
    ticket) makes that raise pydantic.ValidationError instead of silently
    dropping 18+ keys -- this is expected and accepted mid-sprint
    breakage (this ticket's own note), not a bug to work around here. Not
    marked xfail: raising IS the correct, intended outcome today, so
    asserting it is a normal test, not an expected failure -- see
    test_robot_config.py's _XFAIL_UNTIL_017 for tests whose ORIGINAL
    purpose (checking a value past a successful load) is what's blocked.
    """
    with pytest.raises(ValidationError):
        load_robot_config(_ROBOTS_DIR / robot_json)
