"""PARKED (sprint 094, ticket 094-002): this harness hand-drives a FOUR-
subsystem pipeline (Hardware, Drivetrain, PoseEstimator, Planner) that
already predates sprint 093's MainLoop gut (Hardware+Drivetrain only) --
Subsystems::Planner is central to the very property under test (tick-order
independence INCLUDING Planner's own DISTANCE-goal stop anticipation), so it
cannot be salvaged by simply dropping Planner from the pipeline without
rewriting the scenario from scratch. Parked alongside Planner's relocation
(source_parked/094/subsystems/); excluded from pytest collection via
pyproject.toml's norecursedirs ("parked-094"). A revival needs BOTH Planner
restored AND a decision on whether order-independence should be re-proven
against today's real (2-subsystem) Rt::MainLoop::tick() shape instead of
this file's own stale 4-subsystem hand-rolled pipeline -- see
clasi/issues/restore-goto-pursuit-with-pose-estimator.md.

Off-hardware acceptance proof for ticket 087-009 (SUC-001, re-confirmed
against the FULL rebuilt loop -- see main_loop_order_independence_harness.
cpp's own file header for exactly why the existing ticket 002/007 proofs are
narrower and why this one is needed).

Compiles ``main_loop_order_independence_harness.cpp`` together with the
real ``source/subsystems/{drivetrain,sim_hardware,pose_estimator,
planner}.cpp``, ``source/estimation/ekf_tiny.cpp``, ``source/kinematics/
body_kinematics.cpp``, ``source/motion/{velocity_ramp,stop_condition}.cpp``,
``source/hal/sim/{physics_world,sim_motor,sim_odometer}.cpp``, and
``source/hal/velocity_pid.cpp`` -- with ``-DHOST_BUILD`` only (this harness
drives the four subsystems directly, never ``Rt::MainLoop``/
``CommandRouter``/``Configurator``/the command-family ``.cpp`` files, so
none of ``ROBOT_DEV_BUILD``'s gated sources are needed).

Mirrors test_dev_loop_pose_estimator.py's shape exactly: compile with the
system C++ compiler, run the resulting binary, assert it exits 0.

Collected under ``tests/sim/unit/`` -- already within ``pyproject.toml``'s
``testpaths``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_main_loop_order_independence.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_TYPES_DIR = _SOURCE_DIR / "types"
_TINYEKF_DIR = _REPO_ROOT / "libraries" / "tinyekf"

_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "main_loop_order_independence_harness.cpp"
_DRIVETRAIN_SRC = _SOURCE_DIR / "subsystems" / "drivetrain.cpp"
_SIM_HARDWARE_SRC = _SOURCE_DIR / "subsystems" / "sim_hardware.cpp"
_POSE_ESTIMATOR_SRC = _SOURCE_DIR / "subsystems" / "pose_estimator.cpp"
_PLANNER_SRC = _SOURCE_DIR / "subsystems" / "planner.cpp"
_EKF_TINY_SRC = _SOURCE_DIR / "estimation" / "ekf_tiny.cpp"
_BODY_KINEMATICS_SRC = _SOURCE_DIR / "kinematics" / "body_kinematics.cpp"
_VELOCITY_RAMP_SRC = _SOURCE_DIR / "motion" / "velocity_ramp.cpp"
_STOP_CONDITION_SRC = _SOURCE_DIR / "motion" / "stop_condition.cpp"
# 089-003: planner.h now #includes motion/jerk_trajectory.h (DISTANCE's new
# linear channel), which in turn #includes the vendored Ruckig headers --
# mirrors test_planner.py's own identical addition.
_JERK_TRAJECTORY_SRC = _SOURCE_DIR / "motion" / "jerk_trajectory.cpp"
_RUCKIG_INCLUDE = _REPO_ROOT / "libraries" / "ruckig" / "include"
_RUCKIG_SRC_DIR = _REPO_ROOT / "libraries" / "ruckig" / "src"
_PHYSICS_WORLD_SRC = _SOURCE_DIR / "hal" / "sim" / "physics_world.cpp"
_SIM_MOTOR_SRC = _SOURCE_DIR / "hal" / "sim" / "sim_motor.cpp"
_SIM_ODOMETER_SRC = _SOURCE_DIR / "hal" / "sim" / "sim_odometer.cpp"
_VELOCITY_PID_SRC = _SOURCE_DIR / "hal" / "velocity_pid.cpp"

_SOURCES = [
    _HARNESS_SRC,
    _DRIVETRAIN_SRC,
    _SIM_HARDWARE_SRC,
    _POSE_ESTIMATOR_SRC,
    _PLANNER_SRC,
    _EKF_TINY_SRC,
    _BODY_KINEMATICS_SRC,
    _VELOCITY_RAMP_SRC,
    _STOP_CONDITION_SRC,
    _JERK_TRAJECTORY_SRC,
    _PHYSICS_WORLD_SRC,
    _SIM_MOTOR_SRC,
    _SIM_ODOMETER_SRC,
    _VELOCITY_PID_SRC,
]

# 089-003: gnu++20 (GNU extensions -- newlib exposes M_PI, which Ruckig's
# roots.hpp needs) plus -fno-exceptions/-fno-rtti, matching
# test_jerk_trajectory.py's/test_ruckig_smoke.py's own precedent -- Planner
# now transitively compiles Ruckig, so every harness that links planner.cpp
# must build under the SAME constraints the firmware itself imposes.
_CXX_STANDARD = "gnu++20"
_CONSTRAINT_FLAGS = ["-fno-exceptions", "-fno-rtti"]


def _find_cxx_compiler() -> str:
    """Locate a usable system C++ compiler, preferring c++ then clang++/g++."""
    import shutil

    for candidate in ("c++", "clang++", "g++"):
        found = shutil.which(candidate)
        if found:
            return found
    pytest.skip("no system C++ compiler (c++/clang++/g++) found on PATH")
    raise AssertionError("unreachable")  # pragma: no cover


def test_main_loop_order_independence_harness_compiles_and_passes(tmp_path):
    """Compile the FORWARD-vs-REVERSE tick-order harness and assert every
    scenario passes -- re-ordering the mandatory-tick call sequence must
    produce bit-identical committed state (SUC-001's own acceptance
    criterion, ticket 087-009)."""
    for src in _SOURCES:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"
    assert _TINYEKF_DIR.is_dir(), f"libraries/tinyekf missing: {_TINYEKF_DIR}"
    assert _RUCKIG_INCLUDE.is_dir(), f"ruckig include missing: {_RUCKIG_INCLUDE}"
    ruckig_srcs = sorted(_RUCKIG_SRC_DIR.glob("*.cpp"))
    assert ruckig_srcs, f"no vendored ruckig sources under {_RUCKIG_SRC_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "main_loop_order_independence_harness"

    compile_result = subprocess.run(
        [
            cxx,
            f"-std={_CXX_STANDARD}",
            *_CONSTRAINT_FLAGS,
            "-Wall",
            "-Wextra",
            "-DHOST_BUILD",
            "-I",
            str(_SOURCE_DIR),
            "-I",
            str(_TYPES_DIR),
            "-I",
            str(_TINYEKF_DIR),
            "-I",
            str(_RUCKIG_INCLUDE),
            "-o",
            str(binary),
        ]
        + [str(src) for src in _SOURCES]
        + [str(s) for s in ruckig_srcs],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "main_loop_order_independence_harness.cpp (or one of its real sources) "
        f"failed to compile:\nstdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "main_loop_order_independence_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
