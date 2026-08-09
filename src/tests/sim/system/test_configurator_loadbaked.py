"""src/tests/sim/system/test_configurator_loadbaked.py -- ticket 132-006's
own smoke test (the-configuration-object.md, sprint 132 "configuration
discipline"): App::Configurator now owns the one Config::Robot instance --
RobotGraph's constructor calls configurator_.loadBaked() +
configurator_.install() in place of the old RobotGraph::Resolved +
install*Calibration() sequence.

Compiles ``configurator_loadbaked_harness.cpp`` together with
``sim_plant.cpp`` (``src/firm/platform/host/``), ``bench_test_config.cpp``, the plant
sources, and the same full HOST_BUILD Devices/App/messages/kinematics
dependency graph every sibling ``test_*.py`` in this directory already
compiles, runs the resulting binary, and asserts it exits 0 -- printing its
own human-readable per-scenario trace.

This is deliberately lighter than test_composition_root_parity.py -- that
harness (composition_root_parity_harness.cpp) is not required to pass yet
at this ticket (byte-identical boot is a ticket-018 concern, see 132-006's
own ticket file). This test only proves the sim composition root still
constructs/boots/drives, and that Configurator::config() reflects the same
baked values the active robot JSON produces, across all 7 groups.

    uv run python -m pytest src/tests/sim/system/test_configurator_loadbaked.py -v -s
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/system/test_configurator_loadbaked.py -> system -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_SYSTEM_DIR = pathlib.Path(__file__).resolve().parent
_SUPPORT_DIR = _SYSTEM_DIR.parent / "support"
_PLANT_DIR = _SYSTEM_DIR.parent / "plant"
_INFRA_SIM_DIR = _REPO_ROOT / "src" / "firm" / "platform" / "host"

_HARNESS_SRC = _SYSTEM_DIR / "configurator_loadbaked_harness.cpp"
_SIM_PLANT_SRC = _INFRA_SIM_DIR / "sim_plant.cpp"
_WIRE_TEST_CODEC_SRC = _SUPPORT_DIR / "wire_test_codec.cpp"
_BENCH_TEST_CONFIG_SRC = _SUPPORT_DIR / "bench_test_config.cpp"
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
    # The shared composition root (App::composeRobot()/RobotGraph)
    # sim_harness.h boots through, plus its Config::boot_config-reading
    # calibration helpers.
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
    # 132-005 -- Config::default*Group() (the generated, robot-JSON-baked
    # group defaults Configurator::loadBaked() reads, and this harness ALSO
    # calls directly to compute the "same baked values" comparison).
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


def test_configurator_loadbaked_smoke(tmp_path):
    """Compile configurator_loadbaked_harness.cpp + its full dependency
    graph; assert the sim composition root constructs/boots/drives, and
    Configurator::config() reflects the baked robot config."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "configurator_loadbaked_harness"

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
        "configurator_loadbaked_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    print(run_result.stdout)
    assert run_result.returncode == 0, (
        "configurator_loadbaked_harness reported a failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
