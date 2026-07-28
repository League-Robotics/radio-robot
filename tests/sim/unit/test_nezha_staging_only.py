"""Off-hardware acceptance proof for ticket 094-004's second hard-constraint
verification: `Hal::NezhaMotor::setVelocity()`/`setDutyCycle()` remain
STAGING-ONLY -- no I2C bus write happens until an explicit `tick()` call.

Compiles ``nezha_staging_only_harness.cpp`` together with the REAL
``source/hal/nezha/nezha_motor.cpp``, ``source/hal/velocity_pid.cpp``, and
ticket 001's HOST_BUILD scripted-fake ``source/com/i2c_bus_host.cpp``, with
``-DHOST_BUILD`` so ``nezha_motor.cpp``'s own ``#ifndef HOST_BUILD`` guard
sheds its MicroBit.h dependency -- mirrors test_hardware_seam.py's/
test_nezha_flipflop.py's own precedent exactly. Compiles with the system
C++ compiler, runs the resulting binary, asserts it exits 0.

Collected under ``tests/sim/unit/`` -- already within ``pyproject.toml``'s
``testpaths``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_nezha_staging_only.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "nezha_staging_only_harness.cpp"
_HOST_FAKE_SRC = _SOURCE_DIR / "com" / "i2c_bus_host.cpp"
_NEZHA_MOTOR_SRC = _SOURCE_DIR / "hal" / "nezha" / "nezha_motor.cpp"
_VELOCITY_PID_SRC = _SOURCE_DIR / "hal" / "velocity_pid.cpp"

_SOURCES = [_HARNESS_SRC, _HOST_FAKE_SRC, _NEZHA_MOTOR_SRC, _VELOCITY_PID_SRC]

# messages/common.h documents its own target as "CODAL C++11" -- build the
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


def test_nezha_staging_only_harness_compiles_and_passes(tmp_path):
    """Compile the staging-only harness and assert every scenario passes."""
    for src in _SOURCES:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "nezha_staging_only_harness"

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
        ]
        + [str(src) for src in _SOURCES],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "nezha_staging_only_harness.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "nezha_staging_only_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
