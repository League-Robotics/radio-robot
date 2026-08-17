"""src/tests/sim/unit/test_configurator_getconfig.py -- ticket 132-011's own
acceptance test (GetConfig/ConfigSnapshot wire read-back + host
get_config(), sprint 132 "configuration discipline"):
Configurator::encodeSnapshot() -- the read-back half of the-configuration-
object.md's "GetConfig reads the object straight back out" story.

Compiles ``configurator_getconfig_harness.cpp`` together with
configurator.cpp, drive.cpp, boot_calibration.cpp, persisted_tuning.cpp,
config/boot_config.cpp, messages/wire.cpp, messages/wire_runtime.cpp,
hardware/generic/real_otos.cpp, and the standalone Motion::Planner sources
(planner.cpp/profile.cpp/estimation.cpp/shape.cpp) -- the SAME source list
configurator_applygroup_harness.cpp (132-008) already established, since
this harness reuses that fixture shape exactly. Runs the resulting binary
and asserts it exits 0, printing its own human-readable per-scenario
trace.

    uv run python -m pytest src/tests/sim/unit/test_configurator_getconfig.py -v -s
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_configurator_getconfig.py -> unit -> sim -> tests -> src -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_UNIT_DIR = pathlib.Path(__file__).resolve().parent

_HARNESS_SRC = _UNIT_DIR / "configurator_getconfig_harness.cpp"

_APP_SOURCES = [
    _SOURCE_DIR / "core" / "configurator.cpp",
    _SOURCE_DIR / "control" / "differential_drive.cpp",
    _SOURCE_DIR / "core" / "boot_calibration.cpp",
]
# _MOTION_SOURCES -- DELETED (exploratory-kernel rewrite, 2026-08-15):
# src/firm/motion/ no longer exists.
_MOTION_SOURCES = []
_CONFIG_SOURCES = [
    # Config::default*Group() -- this harness's own GEOMETRY scenario calls
    # Config::defaultGeometryGroup() directly to build its expectation, and
    # Configurator::loadBaked() reads every default*Group() function at
    # link time regardless of which scenario runs.
    _SOURCE_DIR / "config" / "boot_config.cpp",
    # Configurator::persistTuningIfChanged()/reapplyPersistedTuning()
    # reference Config::serializeSnapshot()/deserializeSnapshot() at link
    # time even though this harness always passes tuningStore=nullptr and
    # never exercises the persisted-tuning path.
    _SOURCE_DIR / "config" / "persisted_tuning.cpp",
]
_MESSAGE_SOURCES = [
    # The generated wire codec -- both directions: Configurator::
    # applyGroup() (132-008) decodes through msg::wire::decode(<Group>&,
    # ...), and encodeSnapshot() (132-011) encodes through
    # msg::wire::encode(<Group>&, ...) -- this harness's own scenarios
    # additionally call msg::wire::decode(<Group>&, ...) directly to
    # verify a snapshot's own body bytes.
    _SOURCE_DIR / "messages" / "wire.cpp",
    _SOURCE_DIR / "messages" / "wire_runtime.cpp",
]
_DEVICE_SOURCES = [
    # Hal::Otos's virtual destructor is declared out-of-line (otos.h)
    # and defined here -- RecordingOtos's own dtor chain needs the symbol
    # even though this harness never constructs a RealOtos.
    _SOURCE_DIR / "hardware" / "generic" / "real_otos.cpp",
]

_CXX_STANDARD = "c++20"


def _find_cxx_compiler() -> str:
    """Locate a usable system C++ compiler, preferring c++ then clang++/g++."""
    import shutil

    for candidate in ("c++", "clang++", "g++"):
        found = shutil.which(candidate)
        if found:
            return found
    pytest.skip("no system C++ compiler (c++/clang++/g++) found on PATH")
    raise AssertionError("unreachable")  # pragma: no cover


def _all_sources():
    return (
        [_HARNESS_SRC]
        + _APP_SOURCES
        + _MOTION_SOURCES
        + _CONFIG_SOURCES
        + _MESSAGE_SOURCES
        + _DEVICE_SOURCES
    )


def test_configurator_getconfig(tmp_path):
    """Compile configurator_getconfig_harness.cpp + its dependency graph;
    assert every read-back round-trip/boot-only/unspecified-target scenario
    passes."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "configurator_getconfig_harness"

    compile_result = subprocess.run(
        [
            cxx,
            f"-std={_CXX_STANDARD}",
            "-Wall",
            "-Wextra",
            "-DHOST_BUILD",
            "-I",
            str(_SOURCE_DIR),
            "-I",
            str(_REPO_ROOT / "src"),
            "-o",
            str(binary),
            *[str(src) for src in sources],
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "configurator_getconfig_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    print(run_result.stdout)
    assert run_result.returncode == 0, (
        "configurator_getconfig_harness reported a failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
