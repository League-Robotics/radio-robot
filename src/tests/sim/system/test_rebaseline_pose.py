"""src/tests/sim/system/test_rebaseline_pose.py -- 131-004's own acceptance
(SUC-131-004, position-rebaseline-destroys-the-pose.md): pose-sanity across
App::RobotLoop::publishWheel()'s software position rebaseline, against the
REAL, live-responding TestSim::SimHarness/TestSim::SimPlant.

Compiles ``rebaseline_pose_harness.cpp`` together with ``sim_plant.cpp``
(``src/firm/platform/host/``), ``wire_test_codec.cpp``, the plant sources, and the same
full HOST_BUILD Devices/App/messages/kinematics dependency graph every
sibling ``test_*.py`` in this directory already compiles, runs the
resulting binary, and asserts it exits 0 -- printing its own
human-readable per-scenario trace.

Collected under ``src/tests/sim/system/`` -- already within
``pyproject.toml``'s ``testpaths = ["src/tests/sim"]``, no configuration
change needed:

    uv run python -m pytest src/tests/sim/system/test_rebaseline_pose.py -v -s
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/system/test_rebaseline_pose.py -> system -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_SYSTEM_DIR = pathlib.Path(__file__).resolve().parent
_SUPPORT_DIR = _SYSTEM_DIR.parent / "support"
_PLANT_DIR = _SYSTEM_DIR.parent / "plant"
_INFRA_SIM_DIR = _REPO_ROOT / "src" / "firm" / "platform" / "host"

_HARNESS_SRC = _SYSTEM_DIR / "rebaseline_pose_harness.cpp"
_SIM_PLANT_SRC = _INFRA_SIM_DIR / "sim_plant.cpp"
_WIRE_TEST_CODEC_SRC = _SUPPORT_DIR / "wire_test_codec.cpp"
_BENCH_TEST_CONFIG_SRC = _SUPPORT_DIR / "bench_test_config.cpp"
_WHEEL_PLANT_SRC = _PLANT_DIR / "wheel_plant.cpp"
_OTOS_PLANT_SRC = _PLANT_DIR / "otos_plant.cpp"

# Mirrors test_straight_twist.py's own dependency graph exactly (same
# composition root, TestSim::SimHarness) -- see that file's own comments for
# why each entry is here.
_APP_SOURCES = [
    _SOURCE_DIR / "app" / "robot_loop.cpp",
    _SOURCE_DIR / "app" / "comms.cpp",
    _SOURCE_DIR / "app" / "debug.cpp",
    _SOURCE_DIR / "app" / "configurator.cpp",
    _SOURCE_DIR / "app" / "telemetry.cpp",
    _REPO_ROOT / "src" / "firm" / "motion" / "planner" / "profile.cpp",
    _REPO_ROOT / "src" / "firm" / "motion" / "planner" / "estimation.cpp",
    _REPO_ROOT / "src" / "firm" / "motion" / "planner" / "shape.cpp",
    _REPO_ROOT / "src" / "firm" / "motion" / "planner" / "planner.cpp",
    _REPO_ROOT / "src" / "firm" / "motion" / "navigator" / "arc_solver.cpp",  # 135-004
    _REPO_ROOT / "src" / "firm" / "motion" / "navigator" / "navigator.cpp",  # 135-004
    _SOURCE_DIR / "app" / "drive.cpp",
    _REPO_ROOT / "src" / "firm" / "motion" / "odometry.cpp",
    _SOURCE_DIR / "app" / "preamble.cpp",
    _SOURCE_DIR / "app" / "boot_wiring.cpp",
    _SOURCE_DIR / "app" / "boot_calibration.cpp",
]
_MOTION_SOURCES = []
_DEVICE_SOURCES = [
    _INFRA_SIM_DIR / "sim_clock.cpp",
    _SOURCE_DIR / "hardware" / "nezha" / "nezha_motor.cpp",
    _SOURCE_DIR / "hardware" / "generic" / "real_otos.cpp",
    _SOURCE_DIR / "hardware" / "planetx" / "color_sensor.cpp",
    _SOURCE_DIR / "hardware" / "planetx" / "line_sensor.cpp",
]
_CONFIG_SOURCES = [
    _SOURCE_DIR / "config" / "persisted_tuning.cpp",
    _SOURCE_DIR / "config" / "boot_config.cpp",
]
_MESSAGE_SOURCES = [
    _SOURCE_DIR / "messages" / "wire.cpp",
    _SOURCE_DIR / "messages" / "wire_runtime.cpp",
]
_KINEMATICS_SOURCES = [
    _REPO_ROOT / "src" / "firm" / "kinematics" / "differential_kinematics.cpp",
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
        [_HARNESS_SRC, _SIM_PLANT_SRC, _WIRE_TEST_CODEC_SRC, _BENCH_TEST_CONFIG_SRC,
         _WHEEL_PLANT_SRC, _OTOS_PLANT_SRC]
        + _APP_SOURCES
        + _MOTION_SOURCES
        + _DEVICE_SOURCES
        + _CONFIG_SOURCES
        + _MESSAGE_SOURCES
        + _KINEMATICS_SOURCES
    )


def test_rebaseline_pose_sanity(tmp_path):
    """Compile rebaseline_pose_harness.cpp + its full dependency graph;
    assert both rebaseline pose-sanity scenarios pass and print their own
    trace."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "rebaseline_pose_harness"

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
            str(_REPO_ROOT / "src" / "firm" / "motion" / "planner"),  # 135-004: navigator.h's bare #include "planner.h"
            "-I",
            str(_SUPPORT_DIR),
            "-I",
            str(_PLANT_DIR),
            "-I",
            str(_INFRA_SIM_DIR),
            "-o",
            str(binary),
            *[str(src) for src in sources],
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "rebaseline_pose_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    print(run_result.stdout)
    assert run_result.returncode == 0, (
        "rebaseline_pose_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    # -s: don't capture stdout -- see this file's own header for the
    # standalone invocation.
    sys.exit(pytest.main([__file__, "-v", "-s"]))
