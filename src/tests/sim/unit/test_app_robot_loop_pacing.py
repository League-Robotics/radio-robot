"""Host-build unit test for ticket 131-005 (SUC-131-005): compiles
``app_robot_loop_pacing_harness.cpp`` together with every HOST_BUILD source
the real ``Core::composeRobot()`` graph needs (mirrors
``src/tests/sim/system/test_sim_api.py``'s own ``_all_sources()`` list, the
current, composeRobot()-based recipe -- NOT ``test_app_robot_loop.py``'s
older hand-wired-graph recipe, whose harness file is independently xfailed),
runs the resulting binary, and asserts it exits 0.

See ``app_robot_loop_pacing_harness.cpp``'s own file header for what the
harness actually proves: that ``Core::RobotLoop::cycle()``'s trailing pacing
block, now an absolute end-of-cycle deadline (131-005), converges the mean
measured inter-cycle-start period to ``kCycle`` under injected per-block
jitter/rounding, rather than drifting to ``kCycle`` plus a fixed structural
offset (the defect 130-011 measured on hardware: a rock-stable 54ms
delivered period against a 50ms nominal).

Collected under ``src/tests/sim/unit/`` -- already within ``pyproject.toml``'s
``testpaths = ["src/tests/sim"]``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_app_robot_loop_pacing.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_UNIT_DIR = pathlib.Path(__file__).resolve().parent
_TESTS_SIM_DIR = _REPO_ROOT / "src" / "tests" / "sim"
_SUPPORT_DIR = _TESTS_SIM_DIR / "support"
_PLANT_DIR = _TESTS_SIM_DIR / "plant"
_INFRA_SIM_DIR = _REPO_ROOT / "src" / "firm" / "platform" / "host"

_HARNESS_SRC = _UNIT_DIR / "app_robot_loop_pacing_harness.cpp"
_SIM_PLANT_SRC = _INFRA_SIM_DIR / "sim_plant.cpp"
_WHEEL_PLANT_SRC = _PLANT_DIR / "wheel_plant.cpp"
_OTOS_PLANT_SRC = _PLANT_DIR / "otos_plant.cpp"

# Mirrors test_sim_api.py's own _APP_SOURCES/_DEVICE_SOURCES/_CONFIG_SOURCES/
# _MESSAGE_SOURCES/_KINEMATICS_SOURCES lists exactly -- this harness composes
# the SAME Core::composeRobot() graph sim_api_harness.cpp does (via
# Core::RobotGraph/Core::composeRobot(), app/boot_wiring.h), just with a
# custom Platform::Sleeper (JitterySleeper) instead of TestSim::SimHarness's
# fixed internal one. Keep this list in sync with test_sim_api.py's own if
# composeRobot()'s own dependency graph ever grows -- see this codebase's
# established per-harness source-list duplication convention
# (coding-standards.md's "grep-ability" rationale).
_APP_SOURCES = [
    _SOURCE_DIR / "core" / "robot_loop.cpp",
    _SOURCE_DIR / "core" / "comms.cpp",
    _SOURCE_DIR / "core" / "debug.cpp",
    _SOURCE_DIR / "core" / "configurator.cpp",
    _SOURCE_DIR / "core" / "telemetry.cpp",
    _REPO_ROOT / "src" / "firm" / "motion" / "planner" / "profile.cpp",
    _REPO_ROOT / "src" / "firm" / "motion" / "planner" / "estimation.cpp",
    _REPO_ROOT / "src" / "firm" / "motion" / "planner" / "shape.cpp",
    _REPO_ROOT / "src" / "firm" / "motion" / "planner" / "planner.cpp",
    _REPO_ROOT / "src" / "firm" / "motion" / "navigator" / "arc_solver.cpp",  # 135-004
    _REPO_ROOT / "src" / "firm" / "motion" / "navigator" / "navigator.cpp",  # 135-004
    _SOURCE_DIR / "control" / "differential_drive.cpp",
    _REPO_ROOT / "src" / "firm" / "motion" / "odometry.cpp",
    _SOURCE_DIR / "core" / "preamble.cpp",
    _SOURCE_DIR / "core" / "boot_wiring.cpp",
    _SOURCE_DIR / "core" / "boot_calibration.cpp",
]
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
        [_HARNESS_SRC, _SIM_PLANT_SRC, _WHEEL_PLANT_SRC, _OTOS_PLANT_SRC]
        + _APP_SOURCES
        + _DEVICE_SOURCES
        + _CONFIG_SOURCES
        + _MESSAGE_SOURCES
        + _KINEMATICS_SOURCES
    )


def test_app_robot_loop_pacing_harness_compiles_and_passes(tmp_path):
    """Compile the composeRobot() graph + JitterySleeper harness; assert the
    mean measured inter-cycle period converges to kCycle under injected
    jitter (131-005)."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "app_robot_loop_pacing_harness"

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
            str(_TESTS_SIM_DIR),
            "-I",
            str(_INFRA_SIM_DIR),
            "-I",
            str(_PLANT_DIR),
            "-I",
            str(_SUPPORT_DIR),
            "-o",
            str(binary),
            *[str(src) for src in sources],
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "app_robot_loop_pacing_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run_result.returncode == 0, (
        "app_robot_loop_pacing_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
