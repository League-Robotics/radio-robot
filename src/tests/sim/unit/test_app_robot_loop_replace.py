"""Off-hardware acceptance proof for sprint 127 ticket 001 (SUC-001):
characterizes the firmware's ``Move`` ``replace=True`` preemption path
against the cases the sprint's design issue investigation identified
(clasi/sprints/127-host-side-path-planner-goto-and-path-following/issues/
sprint-127-host-side-path-planner-goto-path-following.md, "Finding 1"),
plus a duplicate-id sanity smoke check. Originally five cases; cases 3/4
(the WheelTrim-integrator-survives-a-replace pair) are REMOVED as of
130-005 -- ``Motion::WheelTrim`` is deleted outright and Motion::Planner
carries no wheel-actuation state left for a replace to disturb (see
``test_app_robot_loop_replace_harness.cpp``'s own file header note).

Compiles ``test_app_robot_loop_replace_harness.cpp`` against TWO source
sets, matched to what each half of the harness needs:

  - Cases 1-5 drive a bare ``Motion::Planner`` directly
    (``src/motion/planner/planner.cpp`` + its own leaf modules --
    profile/shape/estimation (130-005: wheel_trim deleted; 130-007:
    wheel_pid/the parked M4 duty stage deleted too -- the wheel-speed
    controller now lives entirely in App::Drive) -- plus the test-only
    zero-Python scaffolding at ``src/motion/planner/tests/test_support.h``,
    the SAME machinery ``src/motion/planner/tests/
    planner_scenarios_test.cpp``'s own ``testReplacePreempts()`` already
    uses). No RobotLoop, no wire codec, no App::/Devices:: graph.
  - The duplicate-id sanity check drives the real
    ``TestSim::SimHarness`` (``src/firm/platform/host/sim_harness.h``), because that
    dedup short-circuit lives one layer up, in
    ``App::RobotLoop::handleMove()`` -- so this one scenario needs the
    full HOST_BUILD RobotLoop dependency graph every other post-Planner-
    integration sim/unit harness compiles (mirrors
    ``test_sim_harness_configure.py``'s own source list).

Mirrors every other sim/unit harness's shape: compile with the system
C++ compiler, run the resulting binary, assert it exits 0. Collected under
``src/tests/sim/unit/`` -- already within ``pyproject.toml``'s
``testpaths = ["src/tests/sim"]``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_app_robot_loop_replace.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_UNIT_DIR = pathlib.Path(__file__).resolve().parent
_SUPPORT_DIR = _UNIT_DIR.parent / "support"
_PLANT_DIR = _UNIT_DIR.parent / "plant"
_TESTS_SIM_DIR = _UNIT_DIR.parent  # src/tests/sim -- resolves "support/..."-qualified includes
_INFRA_SIM_DIR = _REPO_ROOT / "src" / "firm" / "platform" / "host"
_MOTION_PLANNER_DIR = _REPO_ROOT / "src" / "motion" / "planner"  # resolves test_support.h's own "planner.h"

_HARNESS_SRC = _UNIT_DIR / "test_app_robot_loop_replace_harness.cpp"
_SIM_PLANT_SRC = _INFRA_SIM_DIR / "sim_plant.cpp"
_WIRE_TEST_CODEC_SRC = _SUPPORT_DIR / "wire_test_codec.cpp"
_WHEEL_PLANT_SRC = _PLANT_DIR / "wheel_plant.cpp"
_OTOS_PLANT_SRC = _PLANT_DIR / "otos_plant.cpp"
_BENCH_TEST_CONFIG_SRC = _SUPPORT_DIR / "bench_test_config.cpp"

# Same full HOST_BUILD RobotLoop dependency graph test_sim_harness_configure.py
# compiles (SimHarness composes the real App::RobotLoop graph -- see
# sim_harness.h's own header) -- needed by this file's own
# scenarioDuplicateIdSanityNoOp(). Cases 1-5's own planner.cpp/shape.cpp/
# profile.cpp/estimation.cpp are ALREADY part of this same list (Planner
# integration folded them into every RobotLoop-linking graph; 130-005:
# wheel_trim.cpp deleted; 130-007: wheel_pid.cpp/the parked M4 duty stage
# deleted too -- the wheel-speed controller now lives entirely in
# App::Drive) -- one combined source list serves both halves of the
# harness with no duplication.
_APP_SOURCES = [
    _SOURCE_DIR / "app" / "robot_loop.cpp",
    _SOURCE_DIR / "app" / "comms.cpp",
    # debug.cpp (129-003): App::debugf()'s only implementation --
    # TestSim::SimHarness's constructor always calls
    # App::setDebugSink(&comms_) now (HOST_BUILD is defined below,
    # so the real, non-stub setDebugSink()/debugf() are what this
    # graph links), mirroring src/firm/platform/host/CMakeLists.txt's own
    # APP_SOURCES entry.
    _SOURCE_DIR / "app" / "debug.cpp",
    _SOURCE_DIR / "app" / "configurator.cpp",
    _SOURCE_DIR / "app" / "telemetry.cpp",
    _MOTION_PLANNER_DIR / "profile.cpp",
    _MOTION_PLANNER_DIR / "estimation.cpp",
    _MOTION_PLANNER_DIR / "shape.cpp",
    _MOTION_PLANNER_DIR / "planner.cpp",
    _REPO_ROOT / "src" / "motion" / "navigator" / "arc_solver.cpp",  # 135-004
    _REPO_ROOT / "src" / "motion" / "navigator" / "navigator.cpp",  # 135-004
    _SOURCE_DIR / "app" / "drive.cpp",
    _REPO_ROOT / "src" / "motion" / "odometry.cpp",
    _SOURCE_DIR / "app" / "preamble.cpp",
    # 130-002 -- the shared composition root (App::composeRobot()/
    # RobotGraph) sim_harness.h now boots through, plus its
    # Config::boot_config-reading calibration helpers (the "four-source-
    # list trap" this ticket's own note calls out).
    _SOURCE_DIR / "app" / "boot_wiring.cpp",
    _SOURCE_DIR / "app" / "boot_calibration.cpp",
]
# 128-015: the deleted closed-loop wheel-velocity PID (formerly the sole
# entry here) is gone outright -- zero instantiations; App::Drive holds no
# controller of its own (open-loop duty from calibrated speed, drive.h's
# own header). See src/motion/DESIGN.md's "wheel control generations" note.
_MOTION_SOURCES = []
_DEVICE_SOURCES = [
    _INFRA_SIM_DIR / "sim_clock.cpp",
    _SOURCE_DIR / "devices" / "nezha_motor.cpp",
    _SOURCE_DIR / "devices" / "otos.cpp",
    _SOURCE_DIR / "devices" / "color_sensor.cpp",
    _SOURCE_DIR / "devices" / "line_sensor.cpp",
]
_CONFIG_SOURCES = [
    _SOURCE_DIR / "config" / "persisted_tuning.cpp",
    # 130-002 -- both composition roots now bake the SAME robot-JSON
    # calibration by default (unify-sim-and-robot-composition-roots.md
    # work item 2).
    _SOURCE_DIR / "config" / "boot_config.cpp",
]
_MESSAGE_SOURCES = [
    _SOURCE_DIR / "messages" / "wire.cpp",
    _SOURCE_DIR / "messages" / "wire_runtime.cpp",
]
_KINEMATICS_SOURCES = [
    _REPO_ROOT / "src" / "motion" / "body_kinematics.cpp",
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


def test_app_robot_loop_replace_harness_compiles_and_passes(tmp_path):
    """Compile the 127-001 replace-preemption harness + its dependency graph;
    assert every scenario (cases 1, 2, 5 + the duplicate-id sanity check)
    passes, printing every measured number (Case 2's Edge B discontinuity,
    Case 5's max step/queue depth, ...). Cases 3/4 (WheelTrim-integrator-
    survives-a-replace) are REMOVED as of 130-005 -- see the harness's own
    file header note."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "test_app_robot_loop_replace_harness"

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
            "-I",
            str(_SUPPORT_DIR),
            "-I",
            str(_PLANT_DIR),
            "-I",
            str(_TESTS_SIM_DIR),
            "-I",
            str(_INFRA_SIM_DIR),
            "-I",
            str(_MOTION_PLANNER_DIR),
            "-o",
            str(binary),
            *[str(src) for src in sources],
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "test_app_robot_loop_replace_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    print(run_result.stdout)
    assert run_result.returncode == 0, (
        "test_app_robot_loop_replace_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
