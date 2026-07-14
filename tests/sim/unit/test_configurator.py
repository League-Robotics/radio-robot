"""Off-hardware acceptance proof for ticket 087-005 (SUC-002/SUC-003/
SUC-005): Rt::Configurator (source/runtime/configurator.{h,cpp}) -- the
single config-application authority, constructed with references to
Subsystems::Drivetrain/PoseEstimator/Hardware (the one deliberate
exception to "no subsystem pointers", architecture-update-r1.md Decision 4).

Ticket 094-002 update: Subsystems::Planner was relocated out of source/
entirely (see source_parked/094/subsystems/planner.h); Rt::Configurator no
longer takes a Planner& (source/runtime/configurator.h's own header note),
so at that point this compile list no longer needed planner.cpp/
velocity_ramp.cpp/stop_condition.cpp/jerk_trajectory.cpp or the vendored
Ruckig sources.

Ticket 094-004 update: the Ruckig-driven dependency is BACK, via a different
path -- Subsystems::Drivetrain (still one of Configurator's three live
subsystem references) now owns a Motion::SegmentExecutor
(subsystems/drivetrain.h -> motion/segment_executor.h ->
motion/jerk_trajectory.h -> "ruckig/ruckig.hpp"), so this harness once again
needs stop_condition.cpp/jerk_trajectory.cpp/segment_executor.cpp + the
vendored Ruckig sources + the gnu++20/-fno-exceptions/-fno-rtti/
-DHOST_BUILD=1 constraints test_segment_executor.py's own precedent
established -- Configurator itself did not grow a new dependency; Drivetrain
did, and Configurator holds a Drivetrain&.

Compiles ``configurator_harness.cpp`` together with the REAL
``source/runtime/configurator.cpp`` and every real subsystem it exercises
(Drivetrain, PoseEstimator, SimHardware and their own real dependencies)
using the system C++ compiler, runs the resulting binary, and asserts it
exits 0. Mirrors ``test_sim_hardware.py``'s ``-DHOST_BUILD`` compile-and-run
pattern (SimHardware/PhysicsWorld need it for their ``std::mt19937``
members) plus ``test_pose_estimator.py``'s own real-source list, combined --
Configurator's own test is the first harness in this sprint to need three
subsystems live at once.

Collected under ``tests/sim/unit/`` -- already within ``pyproject.toml``'s
``testpaths``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_configurator.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_TINYEKF_DIR = _REPO_ROOT / "libraries" / "tinyekf"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "configurator_harness.cpp"

_CONFIGURATOR_SRC = _SOURCE_DIR / "runtime" / "configurator.cpp"
_DRIVETRAIN_SRC = _SOURCE_DIR / "subsystems" / "drivetrain.cpp"
_BODY_KINEMATICS_SRC = _SOURCE_DIR / "kinematics" / "body_kinematics.cpp"
_POSE_ESTIMATOR_SRC = _SOURCE_DIR / "subsystems" / "pose_estimator.cpp"
_EKF_TINY_SRC = _SOURCE_DIR / "estimation" / "ekf_tiny.cpp"
_SIM_HARDWARE_SRC = _SOURCE_DIR / "subsystems" / "sim_hardware.cpp"
_PHYSICS_WORLD_SRC = _SOURCE_DIR / "hal" / "sim" / "physics_world.cpp"
_SIM_MOTOR_SRC = _SOURCE_DIR / "hal" / "sim" / "sim_motor.cpp"
_SIM_ODOMETER_SRC = _SOURCE_DIR / "hal" / "sim" / "sim_odometer.cpp"
_VELOCITY_PID_SRC = _SOURCE_DIR / "hal" / "velocity_pid.cpp"
# 100-007, THE CUTOVER: Subsystems::Drivetrain now holds a Drive::Drivetrain/
# Drive::MotionPlan -- source/drive/*.cpp must link too. The retired
# source/motion/ tree (segment_executor/jerk_trajectory/stop_condition) was
# deleted after bench/field sign-off; nothing in Drivetrain references it
# post-cutover, so it is no longer part of this harness's link set.
_DRIVE_SOURCES = sorted((_SOURCE_DIR / "drive").glob("*.cpp"))

_SOURCES = [
    _HARNESS_SRC,
    _CONFIGURATOR_SRC,
    _DRIVETRAIN_SRC,
    _BODY_KINEMATICS_SRC,
    _POSE_ESTIMATOR_SRC,
    _EKF_TINY_SRC,
    _SIM_HARDWARE_SRC,
    _PHYSICS_WORLD_SRC,
    _SIM_MOTOR_SRC,
    _SIM_ODOMETER_SRC,
    _VELOCITY_PID_SRC,
    *_DRIVE_SOURCES,
]

_RUCKIG_INCLUDE = _REPO_ROOT / "libraries" / "ruckig" / "include"
_RUCKIG_SRC_DIR = _REPO_ROOT / "libraries" / "ruckig" / "src"

# 094-004: gnu++20 (GNU extensions -- newlib exposes M_PI, which Ruckig's
# roots.hpp needs) plus -fno-exceptions/-fno-rtti, matching
# test_segment_executor.py's own precedent -- this harness transitively
# compiles Ruckig again via Drivetrain -> Motion::SegmentExecutor.
_CXX_STANDARD = "gnu++20"


def _find_cxx_compiler() -> str:
    """Locate a usable system C++ compiler, preferring c++ then clang++/g++."""
    import shutil

    for candidate in ("c++", "clang++", "g++"):
        found = shutil.which(candidate)
        if found:
            return found
    pytest.skip("no system C++ compiler (c++/clang++/g++) found on PATH")
    raise AssertionError("unreachable")  # pragma: no cover


def test_configurator_harness_compiles_and_passes(tmp_path):
    """Compile the Configurator harness and assert every scenario passes."""
    for src in _SOURCES:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"
    assert _TINYEKF_DIR.is_dir(), f"libraries/tinyekf missing: {_TINYEKF_DIR}"
    assert _RUCKIG_INCLUDE.is_dir(), f"ruckig include missing: {_RUCKIG_INCLUDE}"
    ruckig_srcs = sorted(_RUCKIG_SRC_DIR.glob("*.cpp"))
    assert ruckig_srcs, f"no vendored ruckig sources under {_RUCKIG_SRC_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "configurator_harness"

    compile_result = subprocess.run(
        [
            cxx,
            f"-std={_CXX_STANDARD}",
            "-fno-exceptions",
            "-fno-rtti",
            "-Wall",
            "-Wextra",
            "-DHOST_BUILD=1",
            "-I",
            str(_SOURCE_DIR),
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
        "configurator_harness.cpp (or one of its real sources) failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "configurator_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
