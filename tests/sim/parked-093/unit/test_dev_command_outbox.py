"""Off-hardware acceptance proof for ticket 079-005 (SUC-004/SUC-005/SUC-006/
SUC-007).

Compiles ``dev_command_outbox_harness.cpp`` together with the REAL
``source/commands/dev_commands.cpp``, ``source/commands/command_processor.cpp``,
``source/commands/arg_parse.cpp``, ``source/subsystems/drivetrain.cpp``,
``source/kinematics/body_kinematics.cpp``, ``source/hal/nezha/nezha_motor.cpp``,
and ``source/subsystems/nezha_hardware.cpp``, plus ticket 001's HOST_BUILD scripted-fake
``source/com/i2c_bus_host.cpp``, against the SAME headers every ARM build
compiles, with ``-DHOST_BUILD`` (sheds nezha_motor.cpp's MicroBit.h dependency,
see nezha_flipflop_harness.cpp) AND ``-DROBOT_DEV_BUILD=1`` (codal.json's value --
compiles in the DEV command family, see dev_commands.h's file header). Mirrors
test_nezha_flipflop.py/test_drivetrain.py's shape exactly: compile with the
system C++ compiler, run the resulting binary, assert it exits 0.

Collected under ``tests/sim/unit/`` alongside the other harness wrappers --
already within ``pyproject.toml``'s ``testpaths = ["tests/sim"]``, no
configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_dev_command_outbox.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
# source/commands/command_processor.h (and source/types/command_types.h)
# #include their types/types.h siblings by bare filename ("protocol.h",
# "command_types.h", "arg_schema.h") rather than a path-qualified
# "types/protocol.h" -- the real CMake build's RECURSIVE_FIND_DIR adds every
# header-bearing directory individually (see CMakeLists.txt's comment near
# "finally, find sources and includes"); this second -I replicates that for
# the host compiler.
_TYPES_DIR = _SOURCE_DIR / "types"
# estimation/ekf_tiny.h's #include <tinyekf.h> needs this on the path (087-006:
# pulled in transitively now that Rt::CommandRouter's unified table drags in
# subsystems/pose_estimator.cpp -- see this file's own _SOURCES comment).
_TINYEKF_DIR = _REPO_ROOT / "libraries" / "tinyekf"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "dev_command_outbox_harness.cpp"
_HOST_FAKE_SRC = _SOURCE_DIR / "com" / "i2c_bus_host.cpp"
_NEZHA_MOTOR_SRC = _SOURCE_DIR / "hal" / "nezha" / "nezha_motor.cpp"
# 081-001: nezha_motor.cpp now calls into Hal::MotorVelocityPid::compute()
# (source/hal/velocity_pid.cpp) instead of its own former runVelocityPid()
# member — that translation unit must link in alongside it.
_VELOCITY_PID_SRC = _SOURCE_DIR / "hal" / "velocity_pid.cpp"
_NEZHA_HARDWARE_SRC = _SOURCE_DIR / "subsystems" / "nezha_hardware.cpp"
# 086-006: nezha_hardware.cpp now owns a Hal::OtosOdometer member (the real
# OTOS leaf) alongside its four NezhaMotors -- that translation unit must
# link in alongside it too.
_OTOS_ODOMETER_SRC = _SOURCE_DIR / "hal" / "otos" / "otos_odometer.cpp"
_DRIVETRAIN_SRC = _SOURCE_DIR / "subsystems" / "drivetrain.cpp"
_BODY_KINEMATICS_SRC = _SOURCE_DIR / "kinematics" / "body_kinematics.cpp"
_DEV_COMMANDS_SRC = _SOURCE_DIR / "commands" / "dev_commands.cpp"
_COMMAND_PROCESSOR_SRC = _SOURCE_DIR / "commands" / "command_processor.cpp"
_ARG_PARSE_SRC = _SOURCE_DIR / "commands" / "arg_parse.cpp"

# 087-006: Rt::CommandRouter's constructor unconditionally builds ONE unified
# table -- liveness + ALL SIX command families (command_router.cpp) --
# regardless of which family this harness actually dispatches through, so
# every family's own .cpp (and their transitive subsystem/kinematics/motion/
# estimation/telemetry dependencies) must link in too.
_SYSTEM_COMMANDS_SRC = _SOURCE_DIR / "commands" / "system_commands.cpp"
_OTOS_COMMANDS_SRC = _SOURCE_DIR / "commands" / "otos_commands.cpp"
_TELEMETRY_COMMANDS_SRC = _SOURCE_DIR / "commands" / "telemetry_commands.cpp"
_MOTION_COMMANDS_SRC = _SOURCE_DIR / "commands" / "motion_commands.cpp"
_CONFIG_COMMANDS_SRC = _SOURCE_DIR / "commands" / "config_commands.cpp"
_POSE_COMMANDS_SRC = _SOURCE_DIR / "commands" / "pose_commands.cpp"
_COMMAND_ROUTER_SRC = _SOURCE_DIR / "runtime" / "command_router.cpp"
_POSE_ESTIMATOR_SRC = _SOURCE_DIR / "subsystems" / "pose_estimator.cpp"
_PLANNER_SRC = _SOURCE_DIR / "subsystems" / "planner.cpp"
_VELOCITY_RAMP_SRC = _SOURCE_DIR / "motion" / "velocity_ramp.cpp"
_STOP_CONDITION_SRC = _SOURCE_DIR / "motion" / "stop_condition.cpp"
# 089-003: planner.h now #includes motion/jerk_trajectory.h (DISTANCE's new
# linear channel), which in turn #includes the vendored Ruckig headers --
# mirrors test_planner.py's own identical addition.
_JERK_TRAJECTORY_SRC = _SOURCE_DIR / "motion" / "jerk_trajectory.cpp"
_RUCKIG_INCLUDE = _REPO_ROOT / "libraries" / "ruckig" / "include"
_RUCKIG_SRC_DIR = _REPO_ROOT / "libraries" / "ruckig" / "src"
_EKF_TINY_SRC = _SOURCE_DIR / "estimation" / "ekf_tiny.cpp"
_TLM_FRAME_SRC = _SOURCE_DIR / "telemetry" / "tlm_frame.cpp"
_CLOCK_HOST_SRC = _SOURCE_DIR / "types" / "clock_host.cpp"

_SOURCES = [
    _HARNESS_SRC,
    _HOST_FAKE_SRC,
    _NEZHA_MOTOR_SRC,
    _VELOCITY_PID_SRC,
    _NEZHA_HARDWARE_SRC,
    _OTOS_ODOMETER_SRC,
    _DRIVETRAIN_SRC,
    _BODY_KINEMATICS_SRC,
    _DEV_COMMANDS_SRC,
    _COMMAND_PROCESSOR_SRC,
    _ARG_PARSE_SRC,
    _SYSTEM_COMMANDS_SRC,
    _OTOS_COMMANDS_SRC,
    _TELEMETRY_COMMANDS_SRC,
    _MOTION_COMMANDS_SRC,
    _CONFIG_COMMANDS_SRC,
    _POSE_COMMANDS_SRC,
    _COMMAND_ROUTER_SRC,
    _POSE_ESTIMATOR_SRC,
    _PLANNER_SRC,
    _VELOCITY_RAMP_SRC,
    _STOP_CONDITION_SRC,
    _JERK_TRAJECTORY_SRC,
    _EKF_TINY_SRC,
    _TLM_FRAME_SRC,
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


def test_dev_command_outbox_harness_compiles_and_passes(tmp_path):
    """Compile the DEV command outbox harness and assert every scenario passes."""
    for src in _SOURCES:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"
    assert _RUCKIG_INCLUDE.is_dir(), f"ruckig include missing: {_RUCKIG_INCLUDE}"
    ruckig_srcs = sorted(_RUCKIG_SRC_DIR.glob("*.cpp"))
    assert ruckig_srcs, f"no vendored ruckig sources under {_RUCKIG_SRC_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "dev_command_outbox_harness"

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
        "dev_command_outbox_harness.cpp / dev_commands.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "dev_command_outbox_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
