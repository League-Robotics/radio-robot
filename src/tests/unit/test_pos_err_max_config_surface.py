"""src/tests/unit/test_pos_err_max_config_surface.py -- 133-002.

``WheelControl.pos_err_max`` is Stage B's position-error clamp, in
millimetres. It is the INPUT-side bound on the I term; ``pid_i_max`` is
the OUTPUT-side bound, in mm/s. Confusing the two IS the original defect
(clasi/issues/B-wheel-controller-position-loop-and-tuning.md §3: because
the old accumulator's units were already millimetres, ``pid_i_max``
behaved as a DISGUISED position limit of ``pid_i_max/pid_ki`` mm, so
RAISING ki SHRANK the loop's position memory). Both are retained; neither
replaces the other. This file exists so that pairing cannot silently rot.

What it asserts, in the order .claude/rules/configuration-discipline.md
states its two invariants:

  invariant 2 -- every value in the file reaches the robot. The field has
      BOTH a bake path (``gen_boot_config.py`` -> ``defaultWheelControlGroup()``
      in ``src/firm/config/boot_config.cpp``) and a runtime path (the
      generated ``msg::WheelControl`` wire codec, reached by
      ``Configurator::applyGroup()``/``applyField()`` through the existing
      ``WHEEL_CONTROL`` machinery, plus the host's own ``pid.posErrMax``
      set-key). Both read THE SAME robot JSON.
  invariant 1 -- every value the robot uses comes from the file. Read-back
      equals file: for every ``data/robots/*.json``, the value the host
      model reports and the literal the generator bakes are both exactly
      the file's own number, with nothing substituted from a code-side
      default.

Deliberately NOT asserted here: any particular tuned VALUE. Every robot
JSON ships ``pos_err_max: 0.0`` (unclamped, and inert anyway while every
``pid_*`` key is 0). The measured ``vevov`` figure of 5 is a BARE-MOTOR
number on a rig whose ``v_min`` is a factor of five away from tovez's --
sprint 133 ticket 004 owns tovez's real value. A test that pinned a
specific number here would have to be rewritten by that ticket and would
assert nothing about the mechanism.

Collected under ``src/tests/unit/`` -- ``pyproject.toml``'s ``testpaths``
includes ``tests/unit``.

UPDATE (136-001, 2026-08-11): ``test_no_wheel_control_field_was_renumbered_
or_reused`` gained ``stall_speed``/``stall_demand``/``stall_window`` (wire
field numbers 13/14/15, the 2026-08-08 stall-detection directive) to its
expected dict -- a legitimate addition, not a renumbering defect; the
existing 12 entries are unchanged. Separately, ``togov.json``/
``tovez_nocal.json`` gained the same three ``wheel_control`` keys (held at
the documented inert ``0.0``, matching ``gopiv.json``'s 2026-08-09
precedent), which is what made ``test_bake_path_emits_the_files_own_pos_
err_max``/``test_read_back_equals_file_for_the_whole_wheel_control_group``
pass again for those two profiles -- no test code change was needed for
either.
"""

import json
import sys
from pathlib import Path

import pytest

# src/tests/unit/test_pos_err_max_config_surface.py -> unit -> tests -> src -> repo root
_REPO_ROOT = Path(__file__).resolve().parents[3]
_ROBOTS_DIR = _REPO_ROOT / "data" / "robots"
_PROTO = _REPO_ROOT / "src" / "protos" / "robot_config.proto"
_SCRIPTS = _REPO_ROOT / "src" / "scripts"

if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import gen_boot_config as gbc  # noqa: E402  (path must be set up before this import)

from robot_radio.config.robot_config import load_robot_config  # noqa: E402
from robot_radio.config.robot_config_generated import WheelControl  # noqa: E402
from robot_radio.robot import protocol as _protocol  # noqa: E402
from robot_radio.robot.pb2 import robot_config_pb2  # noqa: E402


def _robot_json_paths() -> "list[Path]":
    """Every real robot profile. ``active_robot.json`` is a POINTER file
    (it names which profile is active, it is not a profile), and
    ``robot_config.schema.json`` is generated schema, not a robot."""
    skip = {"active_robot.json", "robot_config.schema.json"}
    return sorted(p for p in _ROBOTS_DIR.glob("*.json") if p.name not in skip)


def test_robot_json_paths_is_not_empty():
    """Guard against the parametrized tests below silently collecting
    nothing if the directory layout ever moves."""
    paths = _robot_json_paths()
    assert paths, f"no robot JSON profiles found under {_ROBOTS_DIR}"
    assert {p.name for p in paths} >= {"tovez.json", "tovez_nocal.json", "togov.json"}


# ---------------------------------------------------------------------------
# Schema: field 12, nothing renumbered or reused.
# ---------------------------------------------------------------------------


def _wheel_control_field_numbers() -> "dict[str, int]":
    descriptor = robot_config_pb2.WheelControl.DESCRIPTOR
    return {field.name: field.number for field in descriptor.fields}


def test_pos_err_max_is_wheel_control_field_12():
    numbers = _wheel_control_field_numbers()
    assert numbers.get("pos_err_max") == 12


def test_no_wheel_control_field_was_renumbered_or_reused():
    """The eleven pre-133-002 fields plus 133-002's own pos_err_max keep
    their exact wire numbers, and no two fields share one. A renumber is a
    silent wire-format break: a host and a firmware built either side of
    it would write different fields with no error anywhere.

    UPDATED 136-001: stall_speed/stall_demand/stall_window (13/14/15) join
    this dict -- a legitimate ADDITION (the 2026-08-08 stall-detection
    directive), not a renumbering defect. The existing 12 entries are
    unchanged; this test was failing only because it did not yet know
    about the three new field numbers."""
    numbers = _wheel_control_field_numbers()
    assert numbers == {
        "v_min": 1,
        "bias_max": 2,
        "tau_adapt": 3,
        "a_steady": 4,
        "deficit_threshold": 5,
        "deficit_window": 6,
        "pid_kp": 7,
        "pid_ki": 8,
        "pid_i_max": 9,
        "pid_kaff": 10,
        "pid_max": 11,
        "pos_err_max": 12,
        "stall_speed": 13,
        "stall_demand": 14,
        "stall_window": 15,
    }
    assert len(set(numbers.values())) == len(numbers), "duplicate field number"


def test_proto_declares_pos_err_max_with_the_millimetre_unit_tag():
    """The proto is the one place the two clamps' DOMAINS are stated in
    the schema itself. ``pos_err_max`` is [mm]; ``pid_i_max`` is [mm/s].
    If a future edit drops either tag, the next reader has to re-derive
    the distinction that this whole ticket exists because someone could
    not."""
    text = _PROTO.read_text()
    assert "float pos_err_max        = 12 [(min) = 0.0];  // [mm]" in text
    assert "// [mm/s] I-term OUTPUT clamp" in text


def test_pos_err_max_survives_on_the_generated_host_model():
    """The generated pydantic group carries the field, so a robot JSON
    that sets it validates (``extra='forbid'`` would otherwise reject the
    key outright -- which is exactly how a half-wired field announces
    itself here)."""
    assert "pos_err_max" in WheelControl.model_fields
    assert WheelControl().pos_err_max == 0.0


# ---------------------------------------------------------------------------
# Invariant 2: every value in the file reaches the robot -- BOTH paths.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("path", _robot_json_paths(), ids=lambda p: p.name)
def test_every_robot_json_declares_pos_err_max(path: Path):
    """The generator ``_require()``s this key, so a profile missing it
    fails codegen loudly rather than booting on a code-side default."""
    cfg = json.loads(path.read_text())
    assert "pos_err_max" in cfg["wheel_control"], (
        f"{path.name} is missing wheel_control.pos_err_max -- the firmware "
        "build will fail closed on it (config-as-truth)"
    )


@pytest.mark.parametrize("path", _robot_json_paths(), ids=lambda p: p.name)
def test_bake_path_emits_the_files_own_pos_err_max(path: Path):
    """The BAKE half: ``gen_boot_config.generate()`` writes the file's own
    number into ``defaultWheelControlGroup()``, with no substitution."""
    cfg = json.loads(path.read_text())
    wanted = float(cfg["wheel_control"]["pos_err_max"])

    mapped = gbc.wheel_controller_config_for_config(cfg)
    assert mapped["posErrMax"] == wanted

    content = gbc.generate(cfg, str(path))
    assert f"cfg.pos_err_max = {gbc._f(wanted)};" in content


def test_bake_path_requires_pos_err_max_rather_than_defaulting_it():
    """Fail-closed, the same posture as every sibling: a profile that
    omits the key raises at codegen time. Silently defaulting is exactly
    the "value the robot uses that did not come from the file" this
    project's configuration discipline forbids."""
    cfg = json.loads((_ROBOTS_DIR / "tovez.json").read_text())
    del cfg["wheel_control"]["pos_err_max"]
    with pytest.raises(gbc.MissingRobotConfigKeyError) as excinfo:
        gbc.wheel_controller_config_for_config(cfg)
    assert "pos_err_max" in str(excinfo.value)


def test_runtime_path_reaches_pos_err_max_by_name_distinctly_from_i_max():
    """The RUNTIME half, host side: ``pid.posErrMax`` is its own live
    set-key targeting ``WHEEL_CONTROL.pos_err_max``. Asserted alongside
    ``pid.iMax`` on purpose -- the two must resolve to DIFFERENT fields,
    which is the property a reader who believes "posErrMax replaces iMax"
    would break."""
    targets = _protocol._SET_KEY_TARGETS
    assert targets["pid.posErrMax"] == (robot_config_pb2.WHEEL_CONTROL, "pos_err_max")
    assert targets["pid.iMax"] == (robot_config_pb2.WHEEL_CONTROL, "pid_i_max")
    assert targets["pid.posErrMax"] != targets["pid.iMax"]
    assert "pid.posErrMax" in _protocol._ALL_SET_KEYS


# ---------------------------------------------------------------------------
# Invariant 1: read-back equals file (sprint 132's headline property).
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("path", _robot_json_paths(), ids=lambda p: p.name)
def test_read_back_equals_file_for_the_whole_wheel_control_group(path: Path):
    """Loading a profile through the host model and reading every
    ``wheel_control`` key back reproduces the file EXACTLY -- pos_err_max
    included, and without disturbing any sibling. Runs over the whole
    group rather than the one new field so that adding a field can never
    quietly shadow an existing one."""
    raw = json.loads(path.read_text())["wheel_control"]
    loaded = load_robot_config(path).wheel_control

    for key, value in raw.items():
        if key.startswith("_"):
            continue  # provenance notes, not config values
        assert getattr(loaded, key) == pytest.approx(value), (
            f"{path.name}: wheel_control.{key} read back as "
            f"{getattr(loaded, key)!r}, file says {value!r}"
        )

    # And nothing the model knows about is absent from the file: a modeled
    # field with no file entry would boot from the model's own default,
    # which is the other direction of the same discipline violation.
    modeled = set(WheelControl.model_fields)
    in_file = {k for k in raw if not k.startswith("_")}
    assert modeled == in_file, (
        f"{path.name}: wheel_control keys disagree with the generated model "
        f"-- only in model: {sorted(modeled - in_file)}, "
        f"only in file: {sorted(in_file - modeled)}"
    )
