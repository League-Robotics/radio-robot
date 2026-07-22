"""src/tests/sim/system/test_state_estimator_tracking.py -- ticket 117-005's
own acceptance proof (SUC-060): the wired-in ``App::StateEstimator``
(``RobotLoop``'s own trailing kPace-block ``update()`` call, ticket 004)
tracks ``TestSim::SimPlant``'s own ground-truth wheel/body state across a
varied MOVE-pattern set, via a genuine one-cycle-ahead predict-to-now check
every cycle of every phase.

Compiles ``state_estimator_tracking_harness.cpp`` together with
``sim_plant.cpp`` (``src/sim/``), ``wire_test_codec.cpp``, the plant sources,
and the same full HOST_BUILD Devices/App/messages/kinematics dependency graph
every sibling ``test_*.py`` in this directory already compiles, runs the
resulting binary, and asserts it exits 0 -- printing its own human-readable
per-phase trace plus the AC #3 "largest tracking error" report.

Collected under ``src/tests/sim/system/`` -- already within ``pyproject.toml``'s
``testpaths``, no configuration change needed:

    uv run python -m pytest src/tests/sim/system/test_state_estimator_tracking.py -v -s
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/system/test_state_estimator_tracking.py -> system -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_SYSTEM_DIR = pathlib.Path(__file__).resolve().parent
_SUPPORT_DIR = _SYSTEM_DIR.parent / "support"
_PLANT_DIR = _SYSTEM_DIR.parent / "plant"
_INFRA_SIM_DIR = _REPO_ROOT / "src" / "sim"

_HARNESS_SRC = _SYSTEM_DIR / "state_estimator_tracking_harness.cpp"
_SIM_PLANT_SRC = _INFRA_SIM_DIR / "sim_plant.cpp"
_WIRE_TEST_CODEC_SRC = _SUPPORT_DIR / "wire_test_codec.cpp"
_BENCH_TEST_CONFIG_SRC = _SUPPORT_DIR / "bench_test_config.cpp"
_WHEEL_PLANT_SRC = _PLANT_DIR / "wheel_plant.cpp"
_OTOS_PLANT_SRC = _PLANT_DIR / "otos_plant.cpp"

# Mirrors test_move_protocol.py's own dependency lists exactly -- same
# HOST_BUILD App::/Devices::/Motion::/Config::/messages/kinematics graph
# every sibling sim/system harness in this directory compiles.
_APP_SOURCES = [
    _SOURCE_DIR / "app" / "robot_loop.cpp",
    _SOURCE_DIR / "app" / "comms.cpp",
    _SOURCE_DIR / "app" / "telemetry.cpp",
    _SOURCE_DIR / "app" / "move_queue.cpp",
    _SOURCE_DIR / "app" / "drive.cpp",
    _SOURCE_DIR / "app" / "odometry.cpp",
    _SOURCE_DIR / "app" / "preamble.cpp",
    # 117 ticket 002/004: App::StateEstimator -- this harness's own subject
    # under test, threaded through RobotLoop/SimHarness alongside
    # moveQueue/preamble.
    _SOURCE_DIR / "app" / "state_estimator.cpp",
]
_MOTION_SOURCES = [
    _SOURCE_DIR / "motion" / "stop_condition.cpp",
]
_DEVICE_SOURCES = [
    _INFRA_SIM_DIR / "sim_clock.cpp",
    _SOURCE_DIR / "devices" / "velocity_pid.cpp",
    _SOURCE_DIR / "devices" / "nezha_motor.cpp",
    _SOURCE_DIR / "devices" / "otos.cpp",
    _SOURCE_DIR / "devices" / "color_sensor.cpp",
    _SOURCE_DIR / "devices" / "line_sensor.cpp",
]
_CONFIG_SOURCES = [
    _SOURCE_DIR / "config" / "persisted_tuning.cpp",
]
_MESSAGE_SOURCES = [
    _SOURCE_DIR / "messages" / "wire.cpp",
    _SOURCE_DIR / "messages" / "wire_runtime.cpp",
]
_KINEMATICS_SOURCES = [
    _SOURCE_DIR / "kinematics" / "body_kinematics.cpp",
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


def test_state_estimator_tracking_scenarios_pass(tmp_path):
    """Compile state_estimator_tracking_harness.cpp + its full dependency
    graph; assert every tracking phase passes, and print its own trace
    (including the AC #3 largest-error report)."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "state_estimator_tracking_harness"

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
        "state_estimator_tracking_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    print(run_result.stdout)
    assert run_result.returncode == 0, (
        "state_estimator_tracking_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    # -s: don't capture stdout -- see this file's own docstring for the
    # standalone invocation.
    sys.exit(pytest.main([__file__, "-v", "-s"]))
