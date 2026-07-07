"""Off-hardware acceptance proof for ticket 082-003 (Subsystems::Hardware::
odometer() seam + dev_loop.{h,cpp}/main.cpp wiring).

Compiles ``dev_loop_pose_estimator_harness.cpp`` together with the REAL
``source/dev_loop.cpp`` (the shared loop body this ticket adds a step to),
``source/subsystems/{drivetrain,sim_hardware,pose_estimator}.cpp``,
``source/estimation/ekf_tiny.cpp`` (082 ticket 001), ``source/kinematics/
body_kinematics.cpp``, ``source/commands/{arg_parse,command_processor,
dev_commands,telemetry_commands}.cpp``, ``source/telemetry/tlm_frame.cpp``
(082 ticket 004 -- ``dev_loop.cpp`` gained a ``telemetryEmit()`` call site
for its periodic-TLM step, so this harness's link now needs both), ``source/
hal/sim/*.cpp``, and ``source/hal/velocity_pid.cpp`` -- plus, since 084-002,
``source/commands/motion_commands.cpp``, ``source/subsystems/planner.cpp``,
and its two real dependencies ``source/motion/{velocity_ramp,
stop_condition}.cpp`` (``dev_loop.cpp`` gained a new motion-executor step
that calls into ``Subsystems::Planner`` unconditionally every pass, so this
harness's link needs it too even though it never stages a motion command) --
with ``-DHOST_BUILD``
(sheds MicroBit.h/CODAL dependencies) AND ``-DROBOT_DEV_BUILD=1``
(codal.json's value -- compiles in dev_loop.cpp/dev_commands.cpp's DEV
family, see dev_loop.h's file header), plus ``libraries/tinyekf/`` on the
include path (``estimation/ekf_tiny.h``'s ``tinyekf.h`` is header-only).
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
_DEV_LOOP_SRC = _SOURCE_DIR / "dev_loop.cpp"
_DRIVETRAIN_SRC = _SOURCE_DIR / "subsystems" / "drivetrain.cpp"
_SIM_HARDWARE_SRC = _SOURCE_DIR / "subsystems" / "sim_hardware.cpp"
_POSE_ESTIMATOR_SRC = _SOURCE_DIR / "subsystems" / "pose_estimator.cpp"
_EKF_TINY_SRC = _SOURCE_DIR / "estimation" / "ekf_tiny.cpp"
_BODY_KINEMATICS_SRC = _SOURCE_DIR / "kinematics" / "body_kinematics.cpp"
_ARG_PARSE_SRC = _SOURCE_DIR / "commands" / "arg_parse.cpp"
_COMMAND_PROCESSOR_SRC = _SOURCE_DIR / "commands" / "command_processor.cpp"
_DEV_COMMANDS_SRC = _SOURCE_DIR / "commands" / "dev_commands.cpp"
_TELEMETRY_COMMANDS_SRC = _SOURCE_DIR / "commands" / "telemetry_commands.cpp"
# 084-002: dev_loop.cpp's new motion-executor step calls into
# Subsystems::Planner (and, transitively, its two real dependencies) --
# this harness links the real dev_loop.cpp, so it must link these too, even
# though the harness itself never stages an S/T/D/STOP command.
_MOTION_COMMANDS_SRC = _SOURCE_DIR / "commands" / "motion_commands.cpp"
_PLANNER_SRC = _SOURCE_DIR / "subsystems" / "planner.cpp"
_VELOCITY_RAMP_SRC = _SOURCE_DIR / "motion" / "velocity_ramp.cpp"
_STOP_CONDITION_SRC = _SOURCE_DIR / "motion" / "stop_condition.cpp"
_TLM_FRAME_SRC = _SOURCE_DIR / "telemetry" / "tlm_frame.cpp"
_PHYSICS_WORLD_SRC = _SOURCE_DIR / "hal" / "sim" / "physics_world.cpp"
_SIM_MOTOR_SRC = _SOURCE_DIR / "hal" / "sim" / "sim_motor.cpp"
_SIM_ODOMETER_SRC = _SOURCE_DIR / "hal" / "sim" / "sim_odometer.cpp"
_VELOCITY_PID_SRC = _SOURCE_DIR / "hal" / "velocity_pid.cpp"
# 082-004: telemetry_commands.cpp's SNAP handler reads Types::systemClockNow()
# (mirrors system_commands.cpp's PING handler) -- the HOST_BUILD fake-clock
# definition must link in, same as every other harness that pulls in a TU
# calling this seam.
_CLOCK_HOST_SRC = _SOURCE_DIR / "types" / "clock_host.cpp"
# 087-006: runLoopPass() (renamed from devLoopTick()) unconditionally
# dereferences LoopContext::configurator every pass (the config-plane
# drain) -- Rt::Configurator's own .cpp must link in too.
_CONFIGURATOR_SRC = _SOURCE_DIR / "runtime" / "configurator.cpp"
# 087-006: runLoopPass()'s `if (statement != nullptr) loop.router->route(...)`
# call site references Rt::CommandRouter::route() even though this harness's
# statement is always nullptr (never actually reached at runtime) -- the
# reference still needs linking. Rt::CommandRouter's constructor
# unconditionally builds ONE unified table (liveness + all six command
# families, command_router.cpp), so every family this harness didn't
# already need (config/pose/otos/system) must link in too.
_COMMAND_ROUTER_SRC = _SOURCE_DIR / "runtime" / "command_router.cpp"
_SYSTEM_COMMANDS_SRC = _SOURCE_DIR / "commands" / "system_commands.cpp"
_CONFIG_COMMANDS_SRC = _SOURCE_DIR / "commands" / "config_commands.cpp"
_POSE_COMMANDS_SRC = _SOURCE_DIR / "commands" / "pose_commands.cpp"
_OTOS_COMMANDS_SRC = _SOURCE_DIR / "commands" / "otos_commands.cpp"

_SOURCES = [
    _HARNESS_SRC,
    _DEV_LOOP_SRC,
    _DRIVETRAIN_SRC,
    _SIM_HARDWARE_SRC,
    _POSE_ESTIMATOR_SRC,
    _EKF_TINY_SRC,
    _BODY_KINEMATICS_SRC,
    _ARG_PARSE_SRC,
    _COMMAND_PROCESSOR_SRC,
    _DEV_COMMANDS_SRC,
    _MOTION_COMMANDS_SRC,
    _PLANNER_SRC,
    _VELOCITY_RAMP_SRC,
    _STOP_CONDITION_SRC,
    _TELEMETRY_COMMANDS_SRC,
    _TLM_FRAME_SRC,
    _PHYSICS_WORLD_SRC,
    _SIM_MOTOR_SRC,
    _SIM_ODOMETER_SRC,
    _VELOCITY_PID_SRC,
    _CLOCK_HOST_SRC,
    _CONFIGURATOR_SRC,
    _COMMAND_ROUTER_SRC,
    _SYSTEM_COMMANDS_SRC,
    _CONFIG_COMMANDS_SRC,
    _POSE_COMMANDS_SRC,
    _OTOS_COMMANDS_SRC,
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


def test_dev_loop_pose_estimator_harness_compiles_and_passes(tmp_path):
    """Compile the devLoopTick()/PoseEstimator wiring harness and assert every scenario passes."""
    for src in _SOURCES:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"
    assert _TINYEKF_DIR.is_dir(), f"libraries/tinyekf missing: {_TINYEKF_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "dev_loop_pose_estimator_harness"

    compile_result = subprocess.run(
        [
            cxx,
            f"-std={_CXX_STANDARD}",
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
            "-o",
            str(binary),
        ]
        + [str(src) for src in _SOURCES],
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
