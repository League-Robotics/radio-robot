"""Off-hardware acceptance proof for ticket 081-002 (Subsystems::Hardware).

Compiles ``hardware_seam_harness.cpp`` together with the REAL
``source/hal/nezha/nezha_motor.cpp``, ``source/hal/velocity_pid.cpp``, and
``source/subsystems/nezha_hardware.cpp`` plus ticket 001's HOST_BUILD
scripted-fake ``source/com/i2c_bus_host.cpp``, against the SAME
``source/subsystems/hardware.h`` every ARM build compiles, with
``-DHOST_BUILD`` so ``nezha_motor.cpp``'s own ``#ifndef HOST_BUILD`` guard
sheds its MicroBit.h dependency (see ``test_nezha_flipflop.py``'s docstring
for the identical precedent). Mirrors that file's shape exactly: compile
with the system C++ compiler, run the resulting binary, assert it exits 0.

This proves ``Subsystems::Hardware`` is a real abstract seam -- callable via
a base pointer, not just declared -- against the ONE concrete owner that
exists this ticket (``Subsystems::NezhaHardware``), ahead of ticket 003's
``Subsystems::SimHardware`` providing a second implementation.

Collected under ``tests/sim/unit/`` alongside the other harness wrappers --
already within ``pyproject.toml``'s ``testpaths = ["tests/sim"]``, no
configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_hardware_seam.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "hardware_seam_harness.cpp"
_HOST_FAKE_SRC = _SOURCE_DIR / "com" / "i2c_bus_host.cpp"
_NEZHA_MOTOR_SRC = _SOURCE_DIR / "hal" / "nezha" / "nezha_motor.cpp"
_VELOCITY_PID_SRC = _SOURCE_DIR / "hal" / "velocity_pid.cpp"
_NEZHA_HARDWARE_SRC = _SOURCE_DIR / "subsystems" / "nezha_hardware.cpp"
# 086-006: nezha_hardware.cpp now owns a Hal::OtosOdometer member (the real
# OTOS leaf) alongside its four NezhaMotors -- that translation unit must
# link in alongside it too.
_OTOS_ODOMETER_SRC = _SOURCE_DIR / "hal" / "otos" / "otos_odometer.cpp"

_SOURCES = [
    _HARNESS_SRC,
    _HOST_FAKE_SRC,
    _NEZHA_MOTOR_SRC,
    _VELOCITY_PID_SRC,
    _NEZHA_HARDWARE_SRC,
    _OTOS_ODOMETER_SRC,
]

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


def test_hardware_seam_harness_compiles_and_passes(tmp_path):
    """Compile the Subsystems::Hardware seam harness and assert every scenario passes."""
    for src in _SOURCES:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "hardware_seam_harness"

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
        "hardware_seam_harness.cpp / nezha_hardware.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "hardware_seam_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
