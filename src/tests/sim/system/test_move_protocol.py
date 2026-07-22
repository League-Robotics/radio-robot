"""src/tests/sim/system/test_move_protocol.py -- ticket 116-008's own
acceptance proof: the sim-executable half of the protocol set-point issue's
own Verification section (clasi/sprints/116-move-protocol-cutover/issues/
protocol-set-point-the-minimal-firmware-s-complete-command-surface.md) --
TIME/DISTANCE/ANGLE stop conditions, chaining, replace preemption,
ERR_FULL, the no-deadman empty-queue drain, and a CONFIG patch's
non-interference with an in-flight MOVE.

Compiles ``move_protocol_harness.cpp`` together with ``sim_plant.cpp``
(``src/sim/``), ``wire_test_codec.cpp``, the plant sources, and the same full
HOST_BUILD Devices/App/messages/kinematics dependency graph every sibling
``test_*.py`` in this directory already compiles, runs the resulting binary,
and asserts it exits 0 -- printing its own human-readable per-scenario
trace.

Collected under ``src/tests/sim/system/`` -- already within ``pyproject.toml``'s
``testpaths = ["src/tests/sim"]``, no configuration change needed:

    uv run python -m pytest src/tests/sim/system/test_move_protocol.py -v -s
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/system/test_move_protocol.py -> system -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_SYSTEM_DIR = pathlib.Path(__file__).resolve().parent
_SUPPORT_DIR = _SYSTEM_DIR.parent / "support"
_PLANT_DIR = _SYSTEM_DIR.parent / "plant"
_INFRA_SIM_DIR = _REPO_ROOT / "src" / "sim"

_HARNESS_SRC = _SYSTEM_DIR / "move_protocol_harness.cpp"
_SIM_PLANT_SRC = _INFRA_SIM_DIR / "sim_plant.cpp"
_WIRE_TEST_CODEC_SRC = _SUPPORT_DIR / "wire_test_codec.cpp"
_BENCH_TEST_CONFIG_SRC = _SUPPORT_DIR / "bench_test_config.cpp"
_WHEEL_PLANT_SRC = _PLANT_DIR / "wheel_plant.cpp"
_OTOS_PLANT_SRC = _PLANT_DIR / "otos_plant.cpp"

# 115-006 (gut S1): heading_source.cpp/pilot.cpp/motion/executor.cpp/
# motion/jerk_trajectory.cpp/vendor/ruckig are all DELETED along with the
# rest of the motion stack -- sim_harness.h no longer includes app/pilot.h
# (or transitively motion/executor.h -> vendor/ruckig) at all, so none of
# those sources are compiled into this harness (mirrors every sibling
# sim/system harness's own identical note).
_APP_SOURCES = [
    _SOURCE_DIR / "app" / "robot_loop.cpp",
    _SOURCE_DIR / "app" / "comms.cpp",
    _SOURCE_DIR / "app" / "telemetry.cpp",
    # 116-006 (MOVE protocol cutover): App::MoveQueue replaces the deleted
    # App::Deadman -- this harness's own subject under test.
    _SOURCE_DIR / "app" / "move_queue.cpp",
    _SOURCE_DIR / "app" / "drive.cpp",
    _SOURCE_DIR / "app" / "odometry.cpp",
    _SOURCE_DIR / "app" / "preamble.cpp",
    # 117 ticket 003: App::StateEstimator, threaded through RobotLoop's/
    # SimHarness's own constructors alongside moveQueue/preamble.
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
# 114-004: robot_loop.cpp now #includes config/persisted_tuning.h and calls
# its pure serializeSnapshot()/Config::TuningStore seam directly.
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


def test_move_protocol_scenarios_pass(tmp_path):
    """Compile move_protocol_harness.cpp + its full dependency graph;
    assert every MOVE-protocol scenario passes, and print its own trace."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "move_protocol_harness"

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
        "move_protocol_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    print(run_result.stdout)
    assert run_result.returncode == 0, (
        "move_protocol_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    # -s: don't capture stdout -- see this file's own docstring for the
    # standalone invocation.
    sys.exit(pytest.main([__file__, "-v", "-s"]))
