"""Off-hardware acceptance proof for ticket 082-003 (Subsystems::Hardware::
odometer() seam), rewired for sprint 087 ticket 007's real cyclic executive
(Rt::MainLoop, source/runtime/main_loop.{h,cpp} -- replaces ticket 006's
transitional dev_loop.{h,cpp}, deleted by ticket 007).

Compiles ``dev_loop_pose_estimator_harness.cpp`` together with the REAL
``source/runtime/main_loop.cpp`` (the cyclic executive this ticket adds a
pose-estimation step to), ``source/subsystems/{drivetrain,sim_hardware,
pose_estimator,planner}.cpp``, ``source/estimation/ekf_tiny.cpp`` (082
ticket 001), ``source/kinematics/body_kinematics.cpp``, ``source/motion/
{velocity_ramp,stop_condition}.cpp`` (Planner's own two real dependencies --
Rt::MainLoop::tick() dereferences its Planner reference unconditionally
every pass, the motion-executor step, even though this harness never stages
a motion command), ``source/commands/{arg_parse,command_processor,
dev_commands,telemetry_commands}.cpp`` (Rt::MainLoop::tick() calls
buildBroadcastNeutral()/buildDrivetrainStop() -- dev_commands.cpp -- and
telemetryEmit() -- telemetry_commands.cpp, its own periodic-TLM call site),
``source/telemetry/tlm_frame.cpp``, ``source/hal/sim/*.cpp``, and
``source/hal/velocity_pid.cpp`` -- with ``-DHOST_BUILD`` (sheds MicroBit.h/
CODAL dependencies) AND ``-DROBOT_DEV_BUILD=1`` (codal.json's value --
compiles in main_loop.cpp/dev_commands.cpp's DEV family, see main_loop.h's
file header), plus ``libraries/tinyekf/`` on the include path
(``estimation/ekf_tiny.h``'s ``tinyekf.h`` is header-only).

Notably SHORTER than the pre-007 source list: Rt::MainLoop (unlike ticket
006's transitional LoopContext) holds no Rt::CommandRouter/Rt::Configurator
reference at all -- those stay top-level objects only the SLACK phase
(main.cpp's/sim_api.cpp's own ingest step) calls directly (see main_loop.h's
class comment) -- so this harness's link no longer needs runtime/
{command_router,configurator}.cpp or the config/pose/otos/system/motion
command families (motion_commands.h's StreamingDriveWatchdog is a
header-only class -- no .cpp symbols needed either).

Mirrors test_dev_command_outbox.py/test_sim_hardware.py's shape exactly:
compile with the system C++ compiler, run the resulting binary, assert it
exits 0.

Collected under ``tests/sim/unit/`` alongside the other harness wrappers --
already within ``pyproject.toml``'s ``testpaths = ["tests/sim", "tests/unit"]``,
no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_dev_loop_pose_estimator.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
# source/commands/command_processor.h (and source/types/command_types.h)
# #include their types/ siblings by bare filename -- see
# test_dev_command_outbox.py's identical comment for why this second -I is
# needed.
_TYPES_DIR = _SOURCE_DIR / "types"
_TINYEKF_DIR = _REPO_ROOT / "libraries" / "tinyekf"

_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "dev_loop_pose_estimator_harness.cpp"
_MAIN_LOOP_SRC = _SOURCE_DIR / "runtime" / "main_loop.cpp"
_DRIVETRAIN_SRC = _SOURCE_DIR / "subsystems" / "drivetrain.cpp"
_SIM_HARDWARE_SRC = _SOURCE_DIR / "subsystems" / "sim_hardware.cpp"
_POSE_ESTIMATOR_SRC = _SOURCE_DIR / "subsystems" / "pose_estimator.cpp"
_EKF_TINY_SRC = _SOURCE_DIR / "estimation" / "ekf_tiny.cpp"
_BODY_KINEMATICS_SRC = _SOURCE_DIR / "kinematics" / "body_kinematics.cpp"
_ARG_PARSE_SRC = _SOURCE_DIR / "commands" / "arg_parse.cpp"
_COMMAND_PROCESSOR_SRC = _SOURCE_DIR / "commands" / "command_processor.cpp"
# Rt::MainLoop's emergency-neutralize bypass calls buildBroadcastNeutral()/
# buildDrivetrainStop() (dev_commands.cpp) -- see main_loop.cpp.
_DEV_COMMANDS_SRC = _SOURCE_DIR / "commands" / "dev_commands.cpp"
# Rt::MainLoop::tick()'s periodic-emission step calls telemetryEmit()
# (telemetry_commands.cpp) -- preserving that call site is this ticket's
# own scope; telemetry_commands.cpp needs tlm_frame.cpp in turn.
_TELEMETRY_COMMANDS_SRC = _SOURCE_DIR / "commands" / "telemetry_commands.cpp"
_TLM_FRAME_SRC = _SOURCE_DIR / "telemetry" / "tlm_frame.cpp"
# Rt::MainLoop::tick()'s motion-executor step dereferences its Planner
# reference unconditionally every pass (and, transitively, Planner's own two
# real dependencies) -- wired here only to satisfy that non-null contract;
# this harness never stages an S/T/D/STOP command.
_PLANNER_SRC = _SOURCE_DIR / "subsystems" / "planner.cpp"
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
# 082-004: telemetry_commands.cpp's SNAP handler reads Types::systemClockNow()
# (mirrors system_commands.cpp's PING handler) -- the HOST_BUILD fake-clock
# definition must link in, same as every other harness that pulls in a TU
# calling this seam.
_CLOCK_HOST_SRC = _SOURCE_DIR / "types" / "clock_host.cpp"

_SOURCES = [
    _HARNESS_SRC,
    _MAIN_LOOP_SRC,
    _DRIVETRAIN_SRC,
    _SIM_HARDWARE_SRC,
    _POSE_ESTIMATOR_SRC,
    _EKF_TINY_SRC,
    _BODY_KINEMATICS_SRC,
    _ARG_PARSE_SRC,
    _COMMAND_PROCESSOR_SRC,
    _DEV_COMMANDS_SRC,
    _PLANNER_SRC,
    _VELOCITY_RAMP_SRC,
    _STOP_CONDITION_SRC,
    _JERK_TRAJECTORY_SRC,
    _TELEMETRY_COMMANDS_SRC,
    _TLM_FRAME_SRC,
    _PHYSICS_WORLD_SRC,
    _SIM_MOTOR_SRC,
    _SIM_ODOMETER_SRC,
    _VELOCITY_PID_SRC,
    _CLOCK_HOST_SRC,
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


def test_dev_loop_pose_estimator_harness_compiles_and_passes(tmp_path):
    """Compile the Rt::MainLoop/PoseEstimator wiring harness and assert every scenario passes."""
    for src in _SOURCES:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"
    assert _TINYEKF_DIR.is_dir(), f"libraries/tinyekf missing: {_TINYEKF_DIR}"
    assert _RUCKIG_INCLUDE.is_dir(), f"ruckig include missing: {_RUCKIG_INCLUDE}"
    ruckig_srcs = sorted(_RUCKIG_SRC_DIR.glob("*.cpp"))
    assert ruckig_srcs, f"no vendored ruckig sources under {_RUCKIG_SRC_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "dev_loop_pose_estimator_harness"

    compile_result = subprocess.run(
        [
            cxx,
            f"-std={_CXX_STANDARD}",
            *_CONSTRAINT_FLAGS,
            "-Wall",
            "-Wextra",
            "-DHOST_BUILD",
            "-DROBOT_DEV_BUILD=1",
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
        "dev_loop_pose_estimator_harness.cpp (or one of its real sources) failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "dev_loop_pose_estimator_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
