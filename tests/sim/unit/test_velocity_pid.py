"""Off-hardware acceptance proof for ticket 081-001 (SUC-001).

Compiles ``velocity_pid_harness.cpp`` (a dependency-free harness exercising
``Hal::MotorVelocityPid::compute()`` — the control law extracted
byte-for-byte out of what used to be ``NezhaMotor::runVelocityPid()``)
together with ``source/hal/velocity_pid.cpp`` using the system C++
compiler, runs the resulting binary, and asserts it exits 0. Mirrors
``test_motor_policy.py``'s compile-and-run pattern (078-004): no hardware,
no CODAL, just ``hal/velocity_pid.{h,cpp}`` and ``messages/common.h``
compiled standalone. Unlike ``capability/motor.h`` (headers-only, inline),
``Hal::MotorVelocityPid::compute()`` is defined in ``velocity_pid.cpp``, so
that translation unit is compiled alongside the harness.

Collected under ``tests/sim/unit/`` alongside the existing
``test_motor_policy.py`` — already within ``pyproject.toml``'s
``testpaths = ["tests/sim"]``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_velocity_pid.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "velocity_pid_harness.cpp"
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


def test_velocity_pid_harness_compiles_and_passes(tmp_path):
    """Compile the MotorVelocityPid harness and assert every scenario passes."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _PID_SRC.is_file(), f"velocity_pid.cpp missing: {_PID_SRC}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "velocity_pid_harness"

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
        "velocity_pid_harness.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "velocity_pid_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
