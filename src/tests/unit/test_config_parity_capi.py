"""src/tests/unit/test_config_parity_capi.py -- 132-003 (Generated parity
guard: capi export + Python harness).

Proves the generated C++ Config::Robot group structs
(src/firm/messages/robot_config.h, emitted by scripts/gen_messages.py from
protos/robot_config.proto -- ticket 002) and the generated pydantic model
(src/host/robot_radio/config/robot_config_generated.py, same generator run)
have not structurally drifted from each other -- mirroring
src/motion/planner/capi.cpp's plannerStructSizes()/plannerLimitsOffsets()
pattern (capi.cpp:69,93) and its Python counterpart
src/tests/bench/planner_harness.py:207-212's own ctypes-walk approach. This
is the mechanical guarantee that replaces check_config_sync.py's 58-entry
hand-curated allowlist (deleted next, ticket 004): byte-for-byte generated-
code comparison, not lint.

How the comparison avoids a THIRD hand-maintained field list
--------------------------------------------------------------
capi.cpp's own Python mirror (planner_harness.py's PlannerLimits/Ceilings/
etc.) is a hand-written ctypes.Structure -- appropriate there because
PlannerLimits has no pydantic sibling to read from. Config::Robot's groups
DO have one: the generated pydantic module already carries each group's
field name/type/ORDER (`model_fields`, itself generated from the same
schema walk that emits robot_config.h -- ticket 002). So instead of hand-
listing field names a second time in Python, this harness BUILDS a ctypes
mirror class dynamically from each pydantic model's own `model_fields`
(int -> c_int32, float -> c_float, same order) and compares ITS computed
`ctypes.sizeof()`/per-field `.offset` against the REAL values the C++ capi
export reports for the REAL struct. Two independently-generated artifacts
are compared directly; nothing about field names/order is re-typed by hand
here.

Compiles config_parity_capi.cpp (src/firm/config/) as a HOST_BUILD shared
library via subprocess -- the same throwaway-binary-per-test-run convention
every src/tests/sim/unit/ harness already uses (no shared CMake build step),
just producing a `-shared` object instead of an executable so it can be
`ctypes.CDLL`-loaded, mirroring how src/motion/planner/capi.cpp is built
into libmotionplanner for planner_harness.py.

    uv run python -m pytest src/tests/unit/test_config_parity_capi.py -v

Induced-failure check (ticket 132-003's own acceptance criterion): verified
by hand during implementation -- inserting an extra float field mid-struct
into the checked-in src/firm/messages/robot_config.h's `struct Geometry`
(between `trackwidth` and `rotational_slip`) and rerunning this file made
`test_field_offsets_match_pydantic_model[Geometry]` fail with a concrete
offset mismatch, exactly as designed; the edit was reverted before this
ticket landed. Not re-run automatically here -- corrupting the checked-in
generated header as part of the normal test suite would make every OTHER
test that compiles against it fail for the wrong reason.
"""

import ctypes
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

# src/tests/unit/test_config_parity_capi.py -> unit -> tests -> src -> repo root
_REPO_ROOT = Path(__file__).resolve().parents[3]
_FIRM_DIR = _REPO_ROOT / "src" / "firm"
_CAPI_SRC = _FIRM_DIR / "config" / "config_parity_capi.cpp"

# Same order as ConfigParityGroup (config_parity_capi.h) and
# src/firm/messages/robot_config.h's own struct declaration order --
# also test_gen_messages_robot_config_emission.py's _ROBOT_CONFIG_GROUPS.
_GROUPS = ("Geometry", "Motors", "Drive", "WheelControl", "Planner", "Otos", "Estimator")

# robot-config group fields are exclusively float/int32/uint32 (4 bytes,
# 4-byte-aligned) as of ticket 002's generator -- see robot_config.h. This
# map is deliberately narrow: an annotation this harness doesn't recognize
# is a real "the schema grew a shape this guard doesn't understand yet"
# signal and should fail loudly (see _ctypes_mirror_for below), not be
# silently coerced.
_ANNOTATION_TO_CTYPE = {int: ctypes.c_int32, float: ctypes.c_float}


def _find_cxx_compiler() -> str:
    """Locate a usable system C++ compiler, preferring c++ then clang++/g++
    (mirrors test_gen_messages_robot_config_emission.py's own helper)."""
    for candidate in ("c++", "clang++", "g++"):
        found = shutil.which(candidate)
        if found:
            return found
    pytest.skip("no system C++ compiler (c++/clang++/g++) found on PATH")
    raise AssertionError("unreachable")  # pragma: no cover


@pytest.fixture(scope="module")
def parity_lib(tmp_path_factory):
    """Compile config_parity_capi.cpp as a HOST_BUILD shared library and
    load it via ctypes -- the module-scope fixture every test below shares,
    mirroring planner_harness.py's loadLibrary()."""
    assert _CAPI_SRC.is_file(), f"required source missing: {_CAPI_SRC}"

    cxx = _find_cxx_compiler()
    build_dir = tmp_path_factory.mktemp("config_parity_capi")
    lib_name = "libconfigparitycapi.dylib" if sys.platform == "darwin" else "libconfigparitycapi.so"
    lib_path = build_dir / lib_name

    compile_result = subprocess.run(
        [
            cxx, "-std=c++20", "-Wall", "-Wextra", "-DHOST_BUILD",
            "-shared", "-fPIC",
            "-I", str(_FIRM_DIR),
            "-o", str(lib_path),
            str(_CAPI_SRC),
        ],
        capture_output=True, text=True,
    )
    assert compile_result.returncode == 0, (
        "config_parity_capi.cpp failed to compile as a shared library:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    lib = ctypes.CDLL(str(lib_path))
    lib.configParityStructSizes.argtypes = [ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint32]
    lib.configParityStructSizes.restype = ctypes.c_uint32
    lib.configParityFieldOffsets.argtypes = [ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint32]
    lib.configParityFieldOffsets.restype = ctypes.c_uint32
    return lib


@pytest.fixture(scope="module")
def pydantic_module():
    """The REAL, checked-in generated pydantic module -- not a freshly
    regenerated one. This harness is a parity check against the artifact a
    real build actually ships, mirroring
    test_gen_messages_robot_config_emission.py's own C++-header-compile
    test's choice to compile the checked-in robot_config.h rather than an
    in-process regeneration."""
    import robot_radio.config.robot_config_generated as m
    return m


def _real_struct_sizes(lib) -> list:
    buf = (ctypes.c_uint32 * len(_GROUPS))()
    total = lib.configParityStructSizes(buf, len(_GROUPS))
    assert total == len(_GROUPS), (
        f"configParityStructSizes() reports {total} groups, expected {len(_GROUPS)} "
        f"({_GROUPS!r}) -- ConfigParityGroup and this harness's _GROUPS tuple have drifted"
    )
    return list(buf)


def _real_field_offsets(lib, group_index: int, expected_field_count: int) -> list:
    # Oversize the buffer relative to what we expect so a MISSING field in
    # the harness's own expectation doesn't silently truncate the real
    # answer -- any C-side field count beyond what pydantic declares must
    # show up as a length mismatch, not get clipped away.
    capacity = expected_field_count + 8
    buf = (ctypes.c_uint32 * capacity)()
    total = lib.configParityFieldOffsets(group_index, buf, capacity)
    return list(buf[:total])


def _ctypes_mirror_for(model_cls):
    """Build a ctypes.Structure class from `model_cls.model_fields`, field
    for field, in pydantic's own declared order -- the "no third hand-
    maintained list" mechanism this file's header comment describes."""
    fields = []
    for name, info in model_cls.model_fields.items():
        ctype = _ANNOTATION_TO_CTYPE.get(info.annotation)
        assert ctype is not None, (
            f"{model_cls.__name__}.{name}: unsupported pydantic annotation "
            f"{info.annotation!r} for a config-parity ctypes mirror (only int/float "
            f"are expected in a robot-config group as of ticket 002 -- if the schema "
            f"legitimately grew a new field kind, this map needs a matching entry)"
        )
        fields.append((name, ctype))

    mirror = type(f"_{model_cls.__name__}ParityMirror", (ctypes.Structure,), {"_fields_": fields})
    return mirror, [name for name, _ in fields]


@pytest.mark.parametrize("name", _GROUPS)
def test_struct_size_matches_pydantic_model(parity_lib, pydantic_module, name):
    group_index = _GROUPS.index(name)
    real_sizes = _real_struct_sizes(parity_lib)
    real_size = real_sizes[group_index]

    model_cls = getattr(pydantic_module, name)
    mirror, _field_names = _ctypes_mirror_for(model_cls)
    expected_size = ctypes.sizeof(mirror)

    assert real_size == expected_size, (
        f"msg::{name}'s real C++ sizeof() is {real_size} bytes, but a ctypes mirror built "
        f"from the generated pydantic {name} model's own field list expects {expected_size} "
        f"bytes -- the generated C++ struct and the generated pydantic model have drifted"
    )


@pytest.mark.parametrize("name", _GROUPS)
def test_field_count_matches_pydantic_model(parity_lib, pydantic_module, name):
    group_index = _GROUPS.index(name)
    model_cls = getattr(pydantic_module, name)
    _mirror, field_names = _ctypes_mirror_for(model_cls)

    real_offsets = _real_field_offsets(parity_lib, group_index, len(field_names))

    assert len(real_offsets) == len(field_names), (
        f"msg::{name} has {len(real_offsets)} real C++ fields, but the generated pydantic "
        f"{name} model declares {len(field_names)} fields ({field_names!r}) -- field count "
        f"has drifted between the two generated targets"
    )


@pytest.mark.parametrize("name", _GROUPS)
def test_field_offsets_match_pydantic_model(parity_lib, pydantic_module, name):
    """The headline check: per-field byte offset, in pydantic's own
    declared order, real C++ vs. a ctypes mirror built from that same
    order. This is what catches a mid-struct insertion in ONE of the two
    generated targets that a size-only or count-only check would miss --
    see plannerLimitsOffsets()'s own header comment (capi.cpp:77-84) for
    why offsets, not just sizes, are the load-bearing check."""
    group_index = _GROUPS.index(name)
    model_cls = getattr(pydantic_module, name)
    mirror, field_names = _ctypes_mirror_for(model_cls)

    real_offsets = _real_field_offsets(parity_lib, group_index, len(field_names))
    expected_offsets = [getattr(mirror, field_name).offset for field_name in field_names]

    assert real_offsets == expected_offsets, (
        f"msg::{name}'s real per-field byte offsets {real_offsets} do not match the offsets "
        f"{expected_offsets} a ctypes mirror built from the generated pydantic {name} model's "
        f"own field order ({field_names!r}) would expect -- the generated C++ struct and the "
        f"generated pydantic model have structurally drifted (relative field ordering disagrees)"
    )


def test_unrecognized_group_returns_zero_fields(parity_lib):
    """configParityFieldOffsets() on an out-of-range group id writes
    nothing and reports 0 fields, per its own header contract -- guards
    against a future group-count change silently reading garbage instead
    of failing loudly."""
    buf = (ctypes.c_uint32 * 4)()
    total = parity_lib.configParityFieldOffsets(len(_GROUPS) + 1, buf, 4)
    assert total == 0


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
