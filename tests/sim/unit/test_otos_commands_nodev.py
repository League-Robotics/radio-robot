"""Off-hardware acceptance proof, originally for ticket 084-008 (SUC-007):
every one of the seven OTOS verbs (``OI``/``OZ``/``OR``/``OP``/``OV``/``OL``/
``OA``) replied ``ERR nodev <verb>`` against the REAL
``Subsystems::NezhaHardware`` (whose ``odometer()`` was ``nullptr`` — no
real-hardware OTOS driver existed then, ``clasi/issues/
nezha-hardware-otos-driver-for-new-source-tree.md``).

Ticket 086-006 gave ``NezhaHardware`` a real ``Hal::OtosOdometer`` member and
an ``odometer()`` override that always returns its address — this file's
module name is now historical (kept to avoid churn; nothing else in the repo
references it, see the ticket's own scope note), but the harness it compiles
(``otos_commands_harness.cpp``) now asserts the CURRENT invariant: all seven
verbs reach the real dispatch path and reply ``OK`` (that harness's own file
header has the full rationale, including why no I2C bus scripting is
involved). This file otherwise still proves what it always did: the SAME
compile-and-link recipe (``otos_commands.{h,cpp}`` is unchanged by 086-006 —
it already resolved ``hardware.odometer()`` live on every dispatch) still
builds and the harness binary still exits 0.

Compiles ``otos_commands_harness.cpp`` together with the REAL
``source/hal/nezha/nezha_motor.cpp``, ``source/hal/velocity_pid.cpp``,
``source/subsystems/nezha_hardware.cpp`` (the SAME trio
``test_hardware_seam.py`` compiles), 086-006's real
``source/hal/otos/otos_odometer.cpp`` (nezha_hardware.cpp now owns a
``Hal::OtosOdometer`` member, so this translation unit must link in too),
ticket 001's HOST_BUILD scripted-fake ``source/com/i2c_bus_host.cpp``, PLUS
this ticket's own ``source/commands/otos_commands.cpp``, ``source/commands/
command_processor.cpp``, and ``source/commands/arg_parse.cpp`` — the full
dispatch path from wire text to the reply, not just a bare
``odometer() != nullptr`` check. ``-DROBOT_DEV_BUILD=1`` is required in
addition to ``-DHOST_BUILD`` here (unlike ``test_hardware_seam.py``):
``commands/otos_commands.{h,cpp}`` are wrapped in ``#if ROBOT_DEV_BUILD``
(matching every other command-family file in this tree), while
``subsystems/hardware.h``/``nezha_hardware.cpp`` are not gated at all. A
second include dir (``source/types``) is also required, unlike
``test_hardware_seam.py``: ``commands/command_processor.h``/``types/
command_types.h`` include their ``types/`` siblings by bare filename
(``"protocol.h"``, ``"command_types.h"``) — mirrors ``test_dev_command_
outbox.py``'s own two ``-I`` flags for the identical reason (that file's own
doc comment / ``tests/_infra/sim/CMakeLists.txt``'s file header explain it).

``libfirmware_host`` cannot be reused for this proof: ``tests/_infra/sim/
CMakeLists.txt`` deliberately never compiles ``subsystems/nezha_hardware.cpp``
(its own file header's "Absent" list — it needs the REAL I2CBus/NezhaMotor,
not the sim's), so the ctypes-backed ``Sim`` wrapper (``firmware.py``) can
only ever exercise ``Subsystems::SimHardware``. This ad hoc harness (078's
"no CMake needed yet" precedent, reused by ``test_hardware_seam.py``/
``test_sim_hardware.py``) is this repo's only way to drive
``Subsystems::NezhaHardware`` at all.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_otos_commands_nodev.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_TYPES_DIR = _SOURCE_DIR / "types"
# estimation/ekf_tiny.h's #include <tinyekf.h> needs this on the path (087-006:
# pulled in transitively now that Rt::CommandRouter's unified table drags in
# subsystems/pose_estimator.cpp -- see this file's own _SOURCES comment).
_TINYEKF_DIR = _REPO_ROOT / "libraries" / "tinyekf"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "otos_commands_harness.cpp"
_HOST_FAKE_SRC = _SOURCE_DIR / "com" / "i2c_bus_host.cpp"
_NEZHA_MOTOR_SRC = _SOURCE_DIR / "hal" / "nezha" / "nezha_motor.cpp"
_VELOCITY_PID_SRC = _SOURCE_DIR / "hal" / "velocity_pid.cpp"
_NEZHA_HARDWARE_SRC = _SOURCE_DIR / "subsystems" / "nezha_hardware.cpp"
# 086-006: nezha_hardware.cpp now owns a Hal::OtosOdometer member (the real
# OTOS leaf) alongside its four NezhaMotors -- that translation unit must
# link in alongside it too.
_OTOS_ODOMETER_SRC = _SOURCE_DIR / "hal" / "otos" / "otos_odometer.cpp"
_OTOS_COMMANDS_SRC = _SOURCE_DIR / "commands" / "otos_commands.cpp"
_COMMAND_PROCESSOR_SRC = _SOURCE_DIR / "commands" / "command_processor.cpp"
_ARG_PARSE_SRC = _SOURCE_DIR / "commands" / "arg_parse.cpp"

# 087-006: Rt::CommandRouter's constructor unconditionally builds ONE unified
# table -- liveness + ALL SIX command families (command_router.cpp) --
# regardless of which family this harness actually dispatches through, so
# every family's own .cpp (and their transitive subsystem/kinematics/motion/
# estimation/telemetry dependencies) must link in too.
_SYSTEM_COMMANDS_SRC = _SOURCE_DIR / "commands" / "system_commands.cpp"
_DEV_COMMANDS_SRC = _SOURCE_DIR / "commands" / "dev_commands.cpp"
_TELEMETRY_COMMANDS_SRC = _SOURCE_DIR / "commands" / "telemetry_commands.cpp"
_MOTION_COMMANDS_SRC = _SOURCE_DIR / "commands" / "motion_commands.cpp"
_CONFIG_COMMANDS_SRC = _SOURCE_DIR / "commands" / "config_commands.cpp"
_POSE_COMMANDS_SRC = _SOURCE_DIR / "commands" / "pose_commands.cpp"
_COMMAND_ROUTER_SRC = _SOURCE_DIR / "runtime" / "command_router.cpp"
_DRIVETRAIN_SRC = _SOURCE_DIR / "subsystems" / "drivetrain.cpp"
_POSE_ESTIMATOR_SRC = _SOURCE_DIR / "subsystems" / "pose_estimator.cpp"
_PLANNER_SRC = _SOURCE_DIR / "subsystems" / "planner.cpp"
_BODY_KINEMATICS_SRC = _SOURCE_DIR / "kinematics" / "body_kinematics.cpp"
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
    _OTOS_COMMANDS_SRC,
    _COMMAND_PROCESSOR_SRC,
    _ARG_PARSE_SRC,
    _SYSTEM_COMMANDS_SRC,
    _DEV_COMMANDS_SRC,
    _TELEMETRY_COMMANDS_SRC,
    _MOTION_COMMANDS_SRC,
    _CONFIG_COMMANDS_SRC,
    _POSE_COMMANDS_SRC,
    _COMMAND_ROUTER_SRC,
    _DRIVETRAIN_SRC,
    _POSE_ESTIMATOR_SRC,
    _PLANNER_SRC,
    _BODY_KINEMATICS_SRC,
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


def test_otos_commands_nodev_harness_compiles_and_passes(tmp_path):
    """Compile the OTOS dispatch harness (086-006: now OK, not nodev) and assert
    every one-of-seven scenario passes."""
    for src in _SOURCES:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"
    assert _RUCKIG_INCLUDE.is_dir(), f"ruckig include missing: {_RUCKIG_INCLUDE}"
    ruckig_srcs = sorted(_RUCKIG_SRC_DIR.glob("*.cpp"))
    assert ruckig_srcs, f"no vendored ruckig sources under {_RUCKIG_SRC_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "otos_commands_harness"

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
        "otos_commands_harness.cpp / otos_commands.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "otos_commands_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
