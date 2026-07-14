"""Off-hardware acceptance proof for ticket 078-004 (SUC-005), extended by
ticket 099-003 (SUC-004).

Compiles ``motor_policy_harness.cpp`` (a dependency-free MockMotor leaf
exercising ``Hal::Motor``'s sprint-078 armor policy — zero-dwell reversal,
output deadband, standstill-guarded resets, motion-qualified wedge
reporting) with the system C++ compiler, runs the resulting binary, and
asserts it exits 0. Per architecture-update.md Decision 9, the ORIGINAL
078 scenarios deliberately skip the deferred new-tree simulator and any
CMake/ARM toolchain component: no hardware, no CODAL, just
``capability/motor.h`` and ``messages/*.h`` compiled standalone.

099-003 addition: ``Hal::Motor::trackAcceleration()`` (the generic
per-motor acceleration EMA) only ever runs from inside a REAL leaf's own
``tick()`` — a MockMotor never reaches it — so this ticket's two new
scenarios construct the REAL ``Hal::NezhaMotor``/``Hal::SimMotor`` leaves
directly (mirroring ``test_nezha_flipflop.py``'s own compile shape), which
means this file now also builds with ``-DHOST_BUILD`` and links
``com/i2c_bus_host.cpp`` (the scripted I2CBus fake), ``hal/nezha/
nezha_motor.cpp``, and ``hal/sim/sim_motor.cpp`` alongside the harness.
Still no CODAL, no wall clock, no real hardware — the scripted fake and
``Hal::SimMotor``'s standalone (plant-free) mode keep this a pure host
build. Runs in well under a second and needs nothing beyond a working
``c++``/``clang++`` on PATH.

Collected under ``tests/sim/unit/`` alongside the existing
``test_placeholder.py`` — already within ``pyproject.toml``'s
``testpaths = ["tests/sim"]``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_motor_policy.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "motor_policy_harness.cpp"

# 086-002: the harness's Invariant A/B scenarios drive the REAL
# Hal::MotorVelocityPid (not just capability/motor.h's header-only armor
# policy), so compute()'s own translation unit must be compiled in
# alongside the harness — mirrors test_velocity_pid.py's identical
# _PID_SRC precedent. Also needed by 099-003's real-leaf scenarios (both
# NezhaMotor and SimMotor embed a Hal::MotorVelocityPid member).
_PID_SRC = _SOURCE_DIR / "hal" / "velocity_pid.cpp"

# 099-003: the REAL SimMotor leaf translation unit exercises
# Hal::Motor::trackAcceleration() end-to-end. (The Nezha half of this
# scenario was retired with the legacy hal/nezha cluster; SimMotor alone
# covers the acceleration-EMA acceptance criterion.)
_SIM_MOTOR_SRC = _SOURCE_DIR / "hal" / "sim" / "sim_motor.cpp"

# messages/common.h documents its own target as "CODAL C++11" — build the
# host harness to the same standard so it exercises exactly the language
# subset the firmware itself uses.
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


def test_motor_policy_harness_compiles_and_passes(tmp_path):
    """Compile the MockMotor + real-leaf harness and assert every scenario passes."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _PID_SRC.is_file(), f"velocity_pid.cpp missing: {_PID_SRC}"
    assert _SIM_MOTOR_SRC.is_file(), f"sim_motor.cpp missing: {_SIM_MOTOR_SRC}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "motor_policy_harness"

    compile_result = subprocess.run(
        [
            cxx,
            f"-std={_CXX_STANDARD}",
            "-Wall",
            "-Wextra",
            "-DHOST_BUILD",
            "-I",
            str(_SOURCE_DIR),
            "-o",
            str(binary),
            str(_HARNESS_SRC),
            str(_PID_SRC),
            str(_SIM_MOTOR_SRC),
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "motor_policy_harness.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "motor_policy_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
