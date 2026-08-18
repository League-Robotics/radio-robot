"""src/tests/sim/unit/test_configurator_provenance.py -- ticket 133-006's own
acceptance test for PER-GROUP CONFIG PROVENANCE
(A-live-config-push-is-wiped-by-the-next-reconnect.md, part 2).

Compiles ``configurator_provenance_harness.cpp`` against the same dependency
graph ``configurator_getconfig_harness.cpp`` (132-011) established -- see that
harness's own header for the fixture shape -- runs the resulting binary, and
asserts it exits 0, printing its per-scenario trace.

What it holds in place: provenance is stamped at every ``config_`` mutation
site (never at a call site), is PER GROUP rather than one global flag, is left
alone by every rejection path, resets via ``loadBaked()``, distinguishes
flash-restored values from baked ones, and rides the ``ConfigSnapshot`` reply.

    uv run python -m pytest src/tests/sim/unit/test_configurator_provenance.py -v -s
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_configurator_provenance.py -> unit -> sim -> tests -> src -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_UNIT_DIR = pathlib.Path(__file__).resolve().parent

_HARNESS_SRC = _UNIT_DIR / "configurator_provenance_harness.cpp"

_APP_SOURCES = [
    _SOURCE_DIR / "core" / "configurator.cpp",
    _SOURCE_DIR / "diffdrive" / "differential_drive.cpp",
    _SOURCE_DIR / "core" / "boot_calibration.cpp",
]
# _MOTION_SOURCES -- DELETED (exploratory-kernel rewrite, 2026-08-15):
# src/firm/motion/ no longer exists.
_MOTION_SOURCES = []
_CONFIG_SOURCES = [
    # Config::default*Group() -- loadBaked() reads every one of them.
    _SOURCE_DIR / "config" / "boot_config.cpp",
    # Config::serializeSnapshot()/deserializeSnapshot() -- referenced by
    # persistTuningIfChanged()/reapplyPersistedTuning() at link time, and
    # scenario 9 drives reapplyPersistedTuning() for real.
    _SOURCE_DIR / "config" / "persisted_tuning.cpp",
]
_MESSAGE_SOURCES = [
    _SOURCE_DIR / "messages" / "wire.cpp",
    _SOURCE_DIR / "messages" / "wire_runtime.cpp",
]
_DEVICE_SOURCES = [
    # Hal::Otos's virtual destructor is defined out-of-line.
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


def test_configurator_provenance(tmp_path):
    """Compile configurator_provenance_harness.cpp + its dependency graph;
    assert every provenance scenario passes."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "configurator_provenance_harness"

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
            str(_SOURCE_DIR / "platform" / "host"),  # host_fiber.h
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
        "configurator_provenance_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    print(run_result.stdout)
    assert run_result.returncode == 0, (
        "configurator_provenance_harness reported a failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
