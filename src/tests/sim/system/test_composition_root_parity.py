"""src/tests/sim/system/test_composition_root_parity.py -- 130-002's own
acceptance proof (unify-sim-and-robot-composition-roots.md, SUC-003): sim
and hardware construct IDENTICAL Motion::PlannerLimits values by default,
except the three explicitly documented BootOverrides (trackWidth,
controlPeriod/actuationDelay, otosConfig -- see app/boot_wiring.h's own
header for the full rationale on each).

Compiles ``composition_root_parity_harness.cpp`` together with the same
full HOST_BUILD dependency graph every sibling ``test_*.py`` in this
directory already compiles (SimHarness composes the real App::RobotLoop
graph through App::composeRobot(), see sim_harness.h's own header), runs the
resulting binary, and asserts it exits 0 -- printing its own human-readable
field-by-field comparison.

    uv run python -m pytest src/tests/sim/system/test_composition_root_parity.py -v -s
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/system/test_composition_root_parity.py -> system -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_SYSTEM_DIR = pathlib.Path(__file__).resolve().parent
_SUPPORT_DIR = _SYSTEM_DIR.parent / "support"
_PLANT_DIR = _SYSTEM_DIR.parent / "plant"
_INFRA_SIM_DIR = _REPO_ROOT / "src" / "firm" / "platform" / "host"

_HARNESS_SRC = _SYSTEM_DIR / "composition_root_parity_harness.cpp"
_SIM_PLANT_SRC = _INFRA_SIM_DIR / "sim_plant.cpp"
_WIRE_TEST_CODEC_SRC = _SUPPORT_DIR / "wire_test_codec.cpp"
_WHEEL_PLANT_SRC = _PLANT_DIR / "wheel_plant.cpp"
_OTOS_PLANT_SRC = _PLANT_DIR / "otos_plant.cpp"

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
    # 130-002 -- the shared composition root (App::composeRobot()/
    # RobotGraph) sim_harness.h boots through, plus its Config::boot_config-
    # reading calibration helpers (App::bootPlannerLimits()/
    # effectiveTrackWidth(), which this harness ALSO calls directly to
    # compute the hardware-equivalent value to diff against).
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
    # 130-002 -- both composition roots now bake the SAME robot-JSON
    # calibration by default (unify-sim-and-robot-composition-roots.md
    # work item 2) -- this harness's own "hardware-equivalent" computation
    # reads this SAME generated bake.
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
        [_HARNESS_SRC, _SIM_PLANT_SRC, _WIRE_TEST_CODEC_SRC, _WHEEL_PLANT_SRC, _OTOS_PLANT_SRC]
        + _APP_SOURCES
        + _MOTION_SOURCES
        + _DEVICE_SOURCES
        + _CONFIG_SOURCES
        + _MESSAGE_SOURCES
        + _KINEMATICS_SOURCES
    )


def test_composition_root_parity(tmp_path):
    """Compile composition_root_parity_harness.cpp + its full dependency
    graph; assert every non-overridden PlannerLimits field matches the
    hardware-equivalent computation, and every override is exactly the
    documented one."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "composition_root_parity_harness"

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
        "composition_root_parity_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    print(run_result.stdout)
    assert run_result.returncode == 0, (
        "composition_root_parity_harness reported a mismatch "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
