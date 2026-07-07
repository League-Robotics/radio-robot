"""Off-hardware acceptance proof for ticket 078-004 (SUC-005).

Compiles ``motor_policy_harness.cpp`` (a dependency-free MockMotor leaf
exercising ``Hal::Motor``'s sprint-078 armor policy — zero-dwell reversal,
output deadband, standstill-guarded resets, motion-qualified wedge
reporting) with the system C++ compiler, runs the resulting binary, and
asserts it exits 0. Per architecture-update.md Decision 9, this
deliberately skips the deferred new-tree simulator and any CMake/ARM
toolchain component: no hardware, no CODAL, just ``capability/motor.h`` and
``messages/*.h`` compiled standalone. Runs in well under a second and needs
nothing beyond a working ``c++``/``clang++`` on PATH.

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
# _PID_SRC precedent.
_PID_SRC = _SOURCE_DIR / "hal" / "velocity_pid.cpp"

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
    """Compile the MockMotor harness and assert every scenario passes."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _PID_SRC.is_file(), f"velocity_pid.cpp missing: {_PID_SRC}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "motor_policy_harness"

    compile_result = subprocess.run(
        [
            cxx,
            f"-std={_CXX_STANDARD}",
            "-Wall",
            "-Wextra",
            "-I",
            str(_SOURCE_DIR),
            "-o",
            str(binary),
            str(_HARNESS_SRC),
            str(_PID_SRC),
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
