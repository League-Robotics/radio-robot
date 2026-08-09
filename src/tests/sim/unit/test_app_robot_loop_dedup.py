"""Off-hardware acceptance proof for sprint 127 ticket 002 (SUC-002):
verifies the ``Move.id`` dedup short-circuit ``Core::RobotLoop::handleMove()``
already ships (``alreadyAccepted()``/``recordAccepted()``/
``acceptedMoveIds_``, ``src/firm/core/robot_loop.cpp`` ~216-258) against the
four rules the source issue's own Verification section names
(``clasi/sprints/127-host-side-path-planner-goto-and-path-following/issues/
duplicate-move-enqueue-on-ack-loss-retry.md``): ordinary duplicate
suppression, the ``Move.id == 0`` exemption, the window outliving
completion, and ``ERR_FULL`` rejections not being recorded.

CHARACTERIZATION, NOT A FIX: ``test_app_robot_loop_dedup_harness.cpp``
writes tests against EXISTING firmware behavior. It does not modify
``src/firm/core/robot_loop.{h,cpp}``, any wire message, or any ``.proto``
definition.

Compiles the harness against the SAME full ``HOST_BUILD`` ``RobotLoop``
dependency graph ``test_app_robot_loop_replace.py`` compiles for its own
``scenarioDuplicateIdSanityNoOp()`` (``TestSim::SimHarness`` composes the
real ``Core::RobotLoop`` graph -- see ``sim_harness.h``'s own header) --
this file's dedup short-circuit lives one layer above
``Motion::Planner``, in ``Core::RobotLoop::handleMove()`` itself, so it
needs the whole graph, not a bare ``Motion::Planner``.

Mirrors ``test_app_robot_loop_replace.py``'s shape: compile with the
system C++ compiler, run the resulting binary, assert it exits 0.
Collected under ``src/tests/sim/unit/`` -- already within
``pyproject.toml``'s ``testpaths = ["src/tests/sim"]``, no configuration
change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_app_robot_loop_dedup.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_UNIT_DIR = pathlib.Path(__file__).resolve().parent
_SUPPORT_DIR = _UNIT_DIR.parent / "support"
_PLANT_DIR = _UNIT_DIR.parent / "plant"
_TESTS_SIM_DIR = _UNIT_DIR.parent  # src/tests/sim -- resolves "support/..."-qualified includes
_INFRA_SIM_DIR = _REPO_ROOT / "src" / "firm" / "platform" / "host"
_MOTION_PLANNER_DIR = _REPO_ROOT / "src" / "firm" / "motion" / "planner"  # resolves planner.h's own includes

_HARNESS_SRC = _UNIT_DIR / "test_app_robot_loop_dedup_harness.cpp"
_SIM_PLANT_SRC = _INFRA_SIM_DIR / "sim_plant.cpp"
_WIRE_TEST_CODEC_SRC = _SUPPORT_DIR / "wire_test_codec.cpp"
_WHEEL_PLANT_SRC = _PLANT_DIR / "wheel_plant.cpp"
_OTOS_PLANT_SRC = _PLANT_DIR / "otos_plant.cpp"
_BENCH_TEST_CONFIG_SRC = _SUPPORT_DIR / "bench_test_config.cpp"

# Same full HOST_BUILD RobotLoop dependency graph test_app_robot_loop_replace.py
# compiles for its own scenarioDuplicateIdSanityNoOp() -- see that file's
# own comment for the source-by-source rationale. This whole harness needs
# it (every scenario here drives TestSim::SimHarness, none drive a bare
# Motion::Planner).
_APP_SOURCES = [
    _SOURCE_DIR / "core" / "robot_loop.cpp",
    _SOURCE_DIR / "core" / "comms.cpp",
    # debug.cpp (129-003): Core::debugf()'s only implementation --
    # TestSim::SimHarness's constructor always calls
    # Core::setDebugSink(&comms_) now (HOST_BUILD is defined below,
    # so the real, non-stub setDebugSink()/debugf() are what this
    # graph links), mirroring src/firm/platform/host/CMakeLists.txt's own
    # APP_SOURCES entry.
    _SOURCE_DIR / "core" / "debug.cpp",
    _SOURCE_DIR / "core" / "configurator.cpp",
    _SOURCE_DIR / "core" / "telemetry.cpp",
    _MOTION_PLANNER_DIR / "profile.cpp",
    _MOTION_PLANNER_DIR / "estimation.cpp",
    _MOTION_PLANNER_DIR / "shape.cpp",
    _MOTION_PLANNER_DIR / "planner.cpp",
    _REPO_ROOT / "src" / "firm" / "motion" / "navigator" / "arc_solver.cpp",  # 135-004
    _REPO_ROOT / "src" / "firm" / "motion" / "navigator" / "navigator.cpp",  # 135-004
    _SOURCE_DIR / "core" / "differential_drive.cpp",
    _REPO_ROOT / "src" / "firm" / "motion" / "odometry.cpp",
    _SOURCE_DIR / "core" / "preamble.cpp",
    # 130-002 -- the shared composition root (Core::composeRobot()/
    # RobotGraph) sim_harness.h now boots through, plus its
    # Config::boot_config-reading calibration helpers (the "four-source-
    # list trap" this ticket's own note calls out).
    _SOURCE_DIR / "core" / "boot_wiring.cpp",
    _SOURCE_DIR / "core" / "boot_calibration.cpp",
]
# 128-015: the deleted closed-loop wheel-velocity PID (formerly the sole
# entry here) is gone outright -- zero instantiations; Core::DifferentialDrive holds no
# controller of its own (open-loop duty from calibrated speed, drive.h's
# own header). See src/firm/motion/DESIGN.md's "wheel control generations" note.
# Kept as an empty list (rather than removed) so the `+ _MOTION_SOURCES`
# concatenation below needs no edit if a future motion module belongs here.
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
    # work item 2).
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
        [_HARNESS_SRC, _SIM_PLANT_SRC, _WIRE_TEST_CODEC_SRC, _WHEEL_PLANT_SRC, _OTOS_PLANT_SRC,
         _BENCH_TEST_CONFIG_SRC]
        + _APP_SOURCES
        + _MOTION_SOURCES
        + _DEVICE_SOURCES
        + _CONFIG_SOURCES
        + _MESSAGE_SOURCES
        + _KINEMATICS_SOURCES
    )


def test_app_robot_loop_dedup_harness_compiles_and_passes(tmp_path):
    """Compile the 127-002 Move.id dedup verification harness + its
    dependency graph; assert every scenario (the four dedup rules) passes,
    printing every measured queue-depth/ack number along the way."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "test_app_robot_loop_dedup_harness"

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
        "test_app_robot_loop_dedup_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    print(run_result.stdout)
    assert run_result.returncode == 0, (
        "test_app_robot_loop_dedup_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
