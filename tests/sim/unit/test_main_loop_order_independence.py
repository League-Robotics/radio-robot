"""Off-hardware acceptance proof for ticket 087-009 (SUC-001, re-confirmed
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
    _PHYSICS_WORLD_SRC,
    _SIM_MOTOR_SRC,
    _SIM_ODOMETER_SRC,
    _VELOCITY_PID_SRC,
]

# messages/common.h documents its own target as "CODAL C++11" -- build the
# host harness to the same standard so it exercises exactly the language
# subset the firmware itself uses.
_CXX_STANDARD = "c++11"


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

    cxx = _find_cxx_compiler()
    binary = tmp_path / "main_loop_order_independence_harness"

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
            str(_TYPES_DIR),
            "-I",
            str(_TINYEKF_DIR),
            "-o",
            str(binary),
        ]
        + [str(src) for src in _SOURCES],
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
