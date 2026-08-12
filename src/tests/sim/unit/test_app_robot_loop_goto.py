"""Off-hardware acceptance proof for sprint 135 ticket 004 (SUC-001/SUC-002/
SUC-003): Core::RobotLoop::handleGoto()/routeCommand()'s GOTO case/cycle()'s
Navigator-vs-Planner ownership dispatch, exercised end to end against the
REAL Core::RobotLoop graph (TestSim::SimHarness -- the same composeRobot()
composition root main.cpp uses, baking data/robots/tovez.json's real
navigator block).

This ticket's own "just proves the routing itself is correct" scope --
ticket 005 owns the fuller system-level wire-codec test. Covers:

  1. Basic end-to-end: GO_TO accepted, Navigator drives, exactly one
     completion ack, ZERO spurious ack(0) entries (Landmine 1).
  2. MOVE cancels an active goto -- no completion ack (preempted).
  3. WHEELS cancels an active goto -- no completion ack.
  4. GO_TO cancels active WHEELS teleop via drive_.takeover().
  5. ESTOP clears the Navigator's target the SAME cycle it clears the
     Planner's queue.

Landmine 2 (Aligning-phase stall) and Landmine 4 (omega sign) are covered
by src/firm/motion/navigator/tests/navigator_test.cpp's own ctest scenarios
(testPivotThenCruiseNotBlockedBehindAligning/
testYawSignMatchesGotoOtosConvention) -- cheaper to extend there, per this
ticket's own Testing section, since neither needs the full RobotLoop wire
graph this file's harness carries.

Compiles test_app_robot_loop_goto_harness.cpp against the same full
HOST_BUILD dependency graph test_sim_harness_configure.py/
test_app_robot_loop_replace.py compile (SimHarness composes the real
Core::RobotLoop graph -- see sim_harness.h's own header).

    uv run python -m pytest src/tests/sim/unit/test_app_robot_loop_goto.py -v -s
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_app_robot_loop_goto.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_UNIT_DIR = pathlib.Path(__file__).resolve().parent
_SUPPORT_DIR = _UNIT_DIR.parent / "support"
_PLANT_DIR = _UNIT_DIR.parent / "plant"
_TESTS_SIM_DIR = _UNIT_DIR.parent  # src/tests/sim -- resolves "support/..."-qualified includes
_INFRA_SIM_DIR = _REPO_ROOT / "src" / "firm" / "platform" / "host"
_MOTION_PLANNER_DIR = _REPO_ROOT / "src" / "firm" / "motion" / "planner"  # resolves navigator.h's own "planner.h"

_HARNESS_SRC = _UNIT_DIR / "test_app_robot_loop_goto_harness.cpp"
_SIM_PLANT_SRC = _INFRA_SIM_DIR / "sim_plant.cpp"
_WIRE_TEST_CODEC_SRC = _SUPPORT_DIR / "wire_test_codec.cpp"
_WHEEL_PLANT_SRC = _PLANT_DIR / "wheel_plant.cpp"
_OTOS_PLANT_SRC = _PLANT_DIR / "otos_plant.cpp"
_BENCH_TEST_CONFIG_SRC = _SUPPORT_DIR / "bench_test_config.cpp"

# Same full HOST_BUILD RobotLoop dependency graph test_sim_harness_configure.py/
# test_app_robot_loop_replace.py compile (SimHarness composes the real
# Core::RobotLoop graph -- see sim_harness.h's own header).
_APP_SOURCES = [
    _SOURCE_DIR / "core" / "robot_loop.cpp",
    _SOURCE_DIR / "core" / "comms.cpp",
    _SOURCE_DIR / "core" / "debug.cpp",
    _SOURCE_DIR / "core" / "configurator.cpp",
    _SOURCE_DIR / "core" / "telemetry.cpp",
    _MOTION_PLANNER_DIR / "profile.cpp",
    _MOTION_PLANNER_DIR / "estimation.cpp",
    _MOTION_PLANNER_DIR / "shape.cpp",
    _MOTION_PLANNER_DIR / "planner.cpp",
    _REPO_ROOT / "src" / "firm" / "motion" / "navigator" / "arc_solver.cpp",  # 135-004
    _REPO_ROOT / "src" / "firm" / "motion" / "navigator" / "navigator.cpp",  # 135-004
    _SOURCE_DIR / "control" / "differential_drive.cpp",
    _REPO_ROOT / "src" / "firm" / "motion" / "odometry.cpp",
    _SOURCE_DIR / "core" / "preamble.cpp",
    _SOURCE_DIR / "core" / "boot_wiring.cpp",
    _SOURCE_DIR / "core" / "boot_calibration.cpp",
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
    _REPO_ROOT / "src" / "firm" / "kinematics" / "differential.cpp",
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
        [_HARNESS_SRC, _SIM_PLANT_SRC, _WIRE_TEST_CODEC_SRC, _WHEEL_PLANT_SRC, _OTOS_PLANT_SRC,
         _BENCH_TEST_CONFIG_SRC]
        + _APP_SOURCES
        + _MOTION_SOURCES
        + _DEVICE_SOURCES
        + _CONFIG_SOURCES
        + _MESSAGE_SOURCES
        + _KINEMATICS_SOURCES
    )


def test_app_robot_loop_goto_harness_compiles_and_passes(tmp_path):
    """Compile the 135-004 GO_TO/Navigator RobotLoop harness + its dependency
    graph; assert every scenario passes (basic end-to-end + Landmine 1 +
    the four ownership-cancellation scenarios)."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "test_app_robot_loop_goto_harness"

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
            str(_MOTION_PLANNER_DIR),  # 135-004: navigator.h's bare #include "planner.h"
            "-I",
            str(_SUPPORT_DIR),
            "-I",
            str(_PLANT_DIR),
            "-I",
            str(_TESTS_SIM_DIR),
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
        "test_app_robot_loop_goto_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    print(run_result.stdout)
    assert run_result.returncode == 0, (
        "test_app_robot_loop_goto_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
