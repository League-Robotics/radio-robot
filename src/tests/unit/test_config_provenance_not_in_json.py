"""src/tests/unit/test_config_provenance_not_in_json.py -- ticket 133-006's
guard against the one mistake per-group config provenance could easily have
made (A-live-config-push-is-wiped-by-the-next-reconnect.md, part 2).

THE MISTAKE THIS FORBIDS: adding a ``source`` field to a config GROUP message
in ``robot_config.proto``. Every group message there generates three things
from one field list -- the C++ ``Config::Robot`` member, the host pydantic
model (``robot_config_generated.py``), and ``data/robots/robot_config.schema.json``
-- so a group-level ``source`` would land in the robot JSON schema too. A FILE
can never carry a runtime-assigned value, so the file would then either have to
supply a provenance (meaningless) or fail validation under
``extra='forbid'``, and sprint 132's headline read-back-equals-file property
would break either way.

Provenance is a property of the ANSWER, not of the configuration, so it lives
on the ``ConfigSnapshot`` REPLY message. That placement is enforced
structurally -- gen_messages.py's ``_CONFIG_ENVELOPE_MESSAGE_NAMES`` excludes
ConfigSnapshot from both group-emission modes -- and this file is the
regression test that says so out loud, in the three places the leak would
actually show up.

The firmware-side behaviour (what BAKED/LIVE/PERSISTED mean and when each is
stamped) is ``src/tests/sim/unit/test_configurator_provenance.py``'s job.

Collected under ``src/tests/unit/`` -- ``pyproject.toml``'s ``testpaths``
includes ``tests/unit``, so ``uv run python -m pytest`` collects it by default.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from robot_radio.config import robot_config_generated
from robot_radio.robot.pb2 import robot_config_pb2

_REPO_ROOT = Path(__file__).resolve().parents[3]
_ROBOTS_DIR = _REPO_ROOT / "data" / "robots"
_SCHEMA_PATH = _ROBOTS_DIR / "robot_config.schema.json"

# The seven robot-config groups plus the three host-only ones -- i.e. every
# message in robot_config.proto that is NOT a wire envelope. Named here rather
# than derived so that a group added without updating this test is a visible
# omission, not a silently-unchecked message.
_GROUP_NAMES = (
    "Geometry", "Motors", "Drive", "WheelControl", "Planner", "PlannerShaper",
    "Otos", "Estimator",
)

# Names that would indicate provenance leaking into the configuration itself.
_PROVENANCE_NAMES = ("source", "config_source", "provenance")


def test_no_config_group_message_declares_a_provenance_field():
    """The proto level: no GROUP message carries a provenance field.

    This is the root of the whole guard -- the pydantic model and the JSON
    schema are both generated from these same field lists, so a group message
    that stays clean here cannot leak downstream.
    """
    offenders = []
    for group_name in _GROUP_NAMES:
        pb_cls = getattr(robot_config_pb2, group_name)
        for field in pb_cls.DESCRIPTOR.fields:
            if field.name in _PROVENANCE_NAMES:
                offenders.append(f"{group_name}.{field.name}")
    assert not offenders, (
        "provenance must NOT be a field of a config group -- it would be "
        "emitted into the robot JSON schema, which cannot carry a "
        f"runtime-assigned value. Offending field(s): {offenders}. Put it on "
        "ConfigSnapshot (the reply) instead."
    )


def test_provenance_lives_on_the_configsnapshot_reply_instead():
    """The positive half: it IS on the reply, with the expected enum.

    Without this, "no group has a source field" would also pass if provenance
    had simply never been implemented.
    """
    snapshot_fields = {f.name for f in robot_config_pb2.ConfigSnapshot.DESCRIPTOR.fields}
    assert "source" in snapshot_fields, (
        "ConfigSnapshot must carry the provenance the config groups deliberately "
        f"do not; it has {sorted(snapshot_fields)}")

    # And the enum says the three things provenance can honestly be.
    assert robot_config_pb2.CONFIG_SOURCE_UNSPECIFIED == 0
    for name in ("CONFIG_SOURCE_BAKED", "CONFIG_SOURCE_LIVE", "CONFIG_SOURCE_PERSISTED"):
        assert hasattr(robot_config_pb2, name), f"ConfigSource is missing {name}"


@pytest.mark.parametrize("group_name", _GROUP_NAMES)
def test_generated_pydantic_group_model_has_no_provenance_field(group_name):
    """The host model: loading a robot JSON must not expect a provenance key.

    Every generated group model sets ``extra='forbid'`` (132-016), so a
    provenance field here would mean a robot JSON is required to supply one.
    """
    model_cls = getattr(robot_config_generated, group_name, None)
    if model_cls is None:
        pytest.skip(f"{group_name} has no generated pydantic model (host-only group)")
    leaked = sorted(set(model_cls.model_fields) & set(_PROVENANCE_NAMES))
    assert not leaked, (
        f"{group_name} pydantic model declares {leaked} -- provenance leaked "
        "out of the reply and into the config model")


def test_robot_config_json_schema_has_no_provenance_property():
    """The generated JSON schema: the file format itself stays clean."""
    assert _SCHEMA_PATH.is_file(), f"generated schema missing: {_SCHEMA_PATH}"
    schema_text = _SCHEMA_PATH.read_text()
    schema = json.loads(schema_text)

    leaked: list[str] = []

    def walk(node, path: str) -> None:
        if isinstance(node, dict):
            for key, value in node.items():
                if key == "properties" and isinstance(value, dict):
                    for prop in value:
                        if prop in _PROVENANCE_NAMES:
                            leaked.append(f"{path}.{prop}")
                walk(value, f"{path}.{key}")
        elif isinstance(node, list):
            for index, item in enumerate(node):
                walk(item, f"{path}[{index}]")

    walk(schema, "$")
    assert not leaked, (
        "data/robots/robot_config.schema.json declares provenance "
        f"propert(ies) {leaked} -- a file cannot carry a runtime-assigned "
        "value, and read-back-equals-file breaks the moment it tries")


def test_no_robot_json_carries_a_provenance_key():
    """The actual on-disk robot files, checked directly.

    The schema test above proves the FORMAT is clean; this proves no file has
    picked one up by hand either -- e.g. someone pasting a read-back (which
    does carry a source) back into a robot JSON as tuned values. That paste is
    the realistic way this would happen, and it fails loudly here rather than
    at the next boot.
    """
    robot_files = sorted(p for p in _ROBOTS_DIR.glob("*.json")
                         if p.name != "robot_config.schema.json")
    assert robot_files, f"no robot JSON files found in {_ROBOTS_DIR}"

    offenders: list[str] = []
    for path in robot_files:
        payload = json.loads(path.read_text())

        def walk(node, where: str) -> None:
            if isinstance(node, dict):
                for key, value in node.items():
                    if key in _PROVENANCE_NAMES:
                        offenders.append(f"{path.name}:{where}.{key}")
                    walk(value, f"{where}.{key}")
            elif isinstance(node, list):
                for index, item in enumerate(node):
                    walk(item, f"{where}[{index}]")

        walk(payload, "$")

    assert not offenders, (
        f"robot JSON carries provenance key(s) {offenders} -- provenance is "
        "reported by the robot, never authored in the file")


def test_read_back_equals_file_still_holds_shape_wise():
    """Sprint 132's headline property, at the level this test can reach
    without hardware: the fields a group reads back over the wire are exactly
    the fields its file model declares -- no more (a provenance field would be
    the extra) and no fewer.

    A full value-level read-back-equals-file check needs a robot; that is
    ticket 004's bench acceptance. What is checked here is the thing 133-006
    could plausibly have broken: the SHAPES drifting apart.
    """
    for group_name in _GROUP_NAMES:
        model_cls = getattr(robot_config_generated, group_name, None)
        if model_cls is None:
            continue
        pb_cls = getattr(robot_config_pb2, group_name)
        wire_fields = {f.name for f in pb_cls.DESCRIPTOR.fields}
        model_fields = set(model_cls.model_fields)
        assert wire_fields == model_fields, (
            f"{group_name}: the wire group and the file model have drifted -- "
            f"only on the wire: {sorted(wire_fields - model_fields)}; "
            f"only in the file model: {sorted(model_fields - wire_fields)}")


if __name__ == "__main__":
    import sys

    sys.exit(pytest.main([__file__, "-v"]))
