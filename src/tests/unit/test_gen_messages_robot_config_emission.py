"""src/tests/unit/test_gen_messages_robot_config_emission.py -- 132-002
(Extend gen_messages.py with pydantic + JSON Schema emission).

Generation smoke test: runs the extended gen_messages.py codegen pipeline
in-process (`gen_messages.generate_robot_config_artifacts()` — the SAME
path a real `python3 scripts/gen_messages.py` run takes; mirrors
`test_gen_messages_no_getters.py`'s own precedent for why in-process beats
grepping the checked-in output) and asserts ticket 002's own THREE
acceptance-criterion artifacts exist and are individually valid:

  1. ``src/firm/messages/robot_config.h`` — one C++ struct per
     robot-config group (Geometry/Motors/Drive/WheelControl/Planner/Otos/
     Estimator), field names/types matching the schema; Identity/
     Connection/Vision (host-only) absent; compiles standalone under
     HOST_BUILD.
  2. A pydantic ``BaseModel`` class per group, all 10 (host-only
     included) — the generated module imports cleanly and every class
     instantiates with its declared field set.
  3. A JSON Schema document declaring all 10 groups — valid JSON, and
     valid AS a JSON Schema document (draft-07 meta-schema check via the
     ``jsonschema`` package, already a ``dev``-group dependency).

Does not touch anything outside this generation path — no wiring into
Config::Robot (tickets 005/006), no wiring into
``src/host/robot_radio/config/robot_config.py``'s actual loader (deferred
to a later ticket — see ``gen_messages.py``'s own
``GENERATED_ROBOT_CONFIG_PYDANTIC_OUT`` comment).

Collected under ``src/tests/unit/`` — ``pyproject.toml``'s ``testpaths``
includes ``tests/unit``, so ``uv run python -m pytest`` collects it by
default.
"""

import importlib.util
import json
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

# src/tests/unit/test_gen_messages_robot_config_emission.py -> unit -> tests -> src -> repo root
_REPO_ROOT = Path(__file__).resolve().parents[3]
_SCRIPTS_DIR = _REPO_ROOT / "src" / "scripts"
_FIRM_DIR = _REPO_ROOT / "src" / "firm"

if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))

import gen_messages  # noqa: E402  (path must be set up before this import)

_ROBOT_CONFIG_GROUPS = (
    "Geometry", "Motors", "Drive", "WheelControl", "Planner", "PlannerShaper",
    "Navigator",  # 135-004: NavigatorLimits config group
    "Otos", "Estimator",
)
_HOST_ONLY_GROUPS = ("Identity", "Connection", "Vision")
_ALL_GROUPS = _HOST_ONLY_GROUPS + _ROBOT_CONFIG_GROUPS


def _find_cxx_compiler() -> str:
    """Locate a usable system C++ compiler, preferring c++ then clang++/g++
    (mirrors src/tests/sim/unit/test_wire_codec.py's own helper)."""
    for candidate in ("c++", "clang++", "g++"):
        found = shutil.which(candidate)
        if found:
            return found
    pytest.skip("no system C++ compiler (c++/clang++/g++) found on PATH")
    raise AssertionError("unreachable")  # pragma: no cover


@pytest.fixture(scope="module")
def robot_config_artifacts():
    """One in-process codegen run, shared by every test below."""
    return gen_messages.generate_robot_config_artifacts()


@pytest.fixture(scope="module")
def loaded_pydantic_module(robot_config_artifacts, tmp_path_factory):
    """Write the generated pydantic module text to a real file and import
    it via importlib (registered in sys.modules) rather than exec()-ing it
    into a bare namespace — pydantic v2 resolves `from __future__ import
    annotations` deferred annotations by looking the defining module up in
    sys.modules, which only a real, registered module satisfies."""
    _cpp, pydantic_text, _schema = robot_config_artifacts
    tmp_dir = tmp_path_factory.mktemp("robot_config_generated")
    mod_path = tmp_dir / "robot_config_generated_under_test.py"
    mod_path.write_text(pydantic_text)
    spec = importlib.util.spec_from_file_location(
        "robot_config_generated_under_test", mod_path
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_all_three_artifacts_are_nonempty(robot_config_artifacts):
    cpp_header, pydantic_module, json_schema_text = robot_config_artifacts
    assert cpp_header.strip()
    assert pydantic_module.strip()
    assert json_schema_text.strip()


# ---------------------------------------------------------------------------
# 1. C++ header (src/firm/messages/robot_config.h)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("name", _ROBOT_CONFIG_GROUPS)
def test_cpp_header_declares_one_struct_per_robot_config_group(robot_config_artifacts, name):
    cpp_header, _pydantic, _schema = robot_config_artifacts
    assert f"struct {name} {{" in cpp_header, (
        f"robot_config.h missing 'struct {name} {{' -- one C++ struct per "
        f"robot-config group is this ticket's own first acceptance criterion"
    )


@pytest.mark.parametrize("name", _HOST_ONLY_GROUPS)
def test_cpp_header_excludes_host_only_groups(robot_config_artifacts, name):
    cpp_header, _pydantic, _schema = robot_config_artifacts
    assert f"struct {name} {{" not in cpp_header, (
        f"robot_config.h must not declare a C++ struct for host-only group "
        f"{name!r} (options.proto's own (host_only) doc comment)"
    )


def test_cpp_header_field_names_and_types_match_schema(robot_config_artifacts):
    """Spot-check field name/type pairs straight off robot_config.proto's
    own declarations -- the acceptance criterion asks for 'field names/
    types matching the schema'."""
    cpp_header, _pydantic, _schema = robot_config_artifacts
    assert "float trackwidth = 0.0f;" in cpp_header
    assert "int32_t fwd_sign_left = 0;" in cpp_header
    assert "uint32_t staleness = 0;" in cpp_header
    assert "float wheel_gain_left_decel = 0.0f;" in cpp_header


def test_cpp_header_compiles_standalone_under_host_build(tmp_path, robot_config_artifacts):
    """HOST_BUILD compile check -- this ticket's own acceptance criterion.
    Compiles a throwaway translation unit that #includes the CHECKED-IN
    src/firm/messages/robot_config.h (the file a real build actually sees,
    with its sibling common.h etc already on disk) and constructs one
    instance of every group + envelope struct."""
    cxx = _find_cxx_compiler()
    header_path = _FIRM_DIR / "messages" / "robot_config.h"
    assert header_path.is_file(), (
        f"{header_path} missing -- run `python3 src/scripts/gen_messages.py` "
        f"and commit the generated output"
    )

    probe_src = tmp_path / "robot_config_probe.cpp"
    probe_src.write_text(
        '#include "messages/robot_config.h"\n'
        "int main() {\n"
        "    msg::Geometry g; msg::Motors mo; msg::Drive d; msg::WheelControl wc;\n"
        "    msg::Planner pl; msg::PlannerShaper ps; msg::Otos ot; msg::Estimator es;\n"
        "    msg::SetConfigGroup scg; msg::GetConfig gc; msg::ConfigSnapshot cs;\n"
        "    msg::SetConfigField scf;\n"
        "    (void)g; (void)mo; (void)d; (void)wc; (void)pl; (void)ps; (void)ot; (void)es;\n"
        "    (void)scg; (void)gc; (void)cs; (void)scf;\n"
        "    return 0;\n"
        "}\n"
    )
    result = subprocess.run(
        [cxx, "-std=c++20", "-Wall", "-Wextra", "-fsyntax-only",
         "-I", str(_FIRM_DIR), str(probe_src)],
        capture_output=True, text=True,
    )
    assert result.returncode == 0, (
        f"robot_config.h failed to compile under HOST_BUILD:\n"
        f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    )


# ---------------------------------------------------------------------------
# 2. pydantic BaseModel hierarchy
# ---------------------------------------------------------------------------

def test_pydantic_module_declares_all_ten_group_classes(loaded_pydantic_module):
    for name in _ALL_GROUPS:
        assert hasattr(loaded_pydantic_module, name), (
            f"generated pydantic module has no class {name!r}"
        )


@pytest.mark.parametrize("name", _ALL_GROUPS)
def test_pydantic_class_instantiates_with_zero_defaults(loaded_pydantic_module, name):
    from pydantic import BaseModel

    cls = getattr(loaded_pydantic_module, name)
    assert issubclass(cls, BaseModel)
    instance = cls()  # every field must carry a zero-value default
    assert instance is not None


def test_pydantic_field_types_match_schema(loaded_pydantic_module):
    m = loaded_pydantic_module
    assert m.Geometry.model_fields["trackwidth"].annotation is float
    assert m.Motors.model_fields["fwd_sign_left"].annotation is int
    assert m.Estimator.model_fields["staleness"].annotation is int
    assert m.Identity.model_fields["robot_name"].annotation is str


# ---------------------------------------------------------------------------
# 3. JSON Schema document
# ---------------------------------------------------------------------------

def test_json_schema_is_valid_json(robot_config_artifacts):
    _cpp, _pydantic, json_schema_text = robot_config_artifacts
    doc = json.loads(json_schema_text)  # raises on malformed JSON
    assert isinstance(doc, dict)


def test_json_schema_declares_all_ten_groups(robot_config_artifacts):
    _cpp, _pydantic, json_schema_text = robot_config_artifacts
    doc = json.loads(json_schema_text)
    assert set(doc["definitions"]) == set(_ALL_GROUPS)
    assert len(doc["properties"]) == len(_ALL_GROUPS)


def test_json_schema_on_disk_is_a_valid_json_schema_document():
    """'The JSON Schema file is valid JSON Schema' -- this ticket's own
    Testing criterion, checked against the actual on-disk, committed
    artifact (the file a real build writes), not just the in-process
    text."""
    jsonschema = pytest.importorskip("jsonschema")
    schema_path = _REPO_ROOT / "data" / "robots" / "robot_config.schema.json"
    assert schema_path.is_file()
    doc = json.loads(schema_path.read_text())
    jsonschema.Draft7Validator.check_schema(doc)  # raises SchemaError if invalid


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
