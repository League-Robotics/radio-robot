"""src/tests/sim/unit/test_bake_push_parity.py -- 132-018's own disposable
verification test (Full-system verification, sprint 132 "configuration
discipline"): sprint.md's "Bake/push parity" success criterion --

    "Build an image from one robot JSON, push that same JSON to a robot
    running a DIFFERENT baked config, confirm identical behaviour."

Compiles ``bake_push_parity_harness.cpp`` together with configurator.cpp,
drive.cpp, boot_calibration.cpp, persisted_tuning.cpp, config/boot_config.cpp,
messages/wire.cpp, messages/wire_runtime.cpp, hardware/generic/real_otos.cpp, and the
standalone Motion::Planner sources -- the SAME source list
configurator_applygroup_harness.cpp (132-008)/configurator_getconfig_harness.cpp
(132-011) already established, since this harness reuses that fixture shape
exactly. Runs the resulting binary and asserts it exits 0, printing its own
human-readable per-scenario trace.

    uv run python -m pytest src/tests/sim/unit/test_bake_push_parity.py -v -s
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_bake_push_parity.py -> unit -> sim -> tests -> src -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_UNIT_DIR = pathlib.Path(__file__).resolve().parent

_HARNESS_SRC = _UNIT_DIR / "bake_push_parity_harness.cpp"

_APP_SOURCES = [
    _SOURCE_DIR / "app" / "configurator.cpp",
    _SOURCE_DIR / "app" / "drive.cpp",
    _SOURCE_DIR / "app" / "boot_calibration.cpp",
]
_MOTION_SOURCES = [
    _REPO_ROOT / "src" / "motion" / "planner" / "profile.cpp",
    _REPO_ROOT / "src" / "motion" / "planner" / "estimation.cpp",
    _REPO_ROOT / "src" / "motion" / "planner" / "shape.cpp",
    _REPO_ROOT / "src" / "motion" / "planner" / "planner.cpp",
    _REPO_ROOT / "src" / "motion" / "navigator" / "arc_solver.cpp",  # 135-004
    _REPO_ROOT / "src" / "motion" / "navigator" / "navigator.cpp",  # 135-004
]
_CONFIG_SOURCES = [
    _SOURCE_DIR / "config" / "boot_config.cpp",
    _SOURCE_DIR / "config" / "persisted_tuning.cpp",
]
_MESSAGE_SOURCES = [
    _SOURCE_DIR / "messages" / "wire.cpp",
    _SOURCE_DIR / "messages" / "wire_runtime.cpp",
]
_DEVICE_SOURCES = [
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


def test_bake_push_parity(tmp_path):
    """Compile bake_push_parity_harness.cpp + its dependency graph; assert
    every bake-vs-push comparison scenario passes -- a robot baked from
    togov.json's own values, then pushed tovez.json's live-group values,
    ends up in the SAME config_ + real-subsystem state as a robot baked
    from tovez.json directly (for every field the live wire surface can
    actually reach)."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "bake_push_parity_harness"

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
            "-I",
            str(_REPO_ROOT / "src" / "motion" / "planner"),  # 135-004: navigator.h's bare #include "planner.h"
            "-o",
            str(binary),
            *[str(src) for src in sources],
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "bake_push_parity_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    print(run_result.stdout)
    assert run_result.returncode == 0, (
        "bake_push_parity_harness reported a failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
