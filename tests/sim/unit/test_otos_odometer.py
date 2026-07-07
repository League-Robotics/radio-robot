"""Off-hardware acceptance proof for ticket 086-006 (SUC-005/SUC-006/SUC-007):
Hal::OtosOdometer (source/hal/otos/otos_odometer.{h,cpp}) -- the real-hardware
Hal::Odometer leaf for the SparkFun OTOS sensor.

Compiles ``otos_odometer_harness.cpp`` together with the REAL
``source/hal/otos/otos_odometer.cpp`` plus ticket 001's HOST_BUILD
scripted-fake ``source/com/i2c_bus_host.cpp``, against the SAME
``source/hal/otos/otos_odometer.h`` / ``source/hal/lever_arm.h`` every ARM
build compiles, with ``-DHOST_BUILD`` so otos_odometer.cpp's own
``#ifndef HOST_BUILD`` guard sheds its MicroBit.h dependency. Mirrors
``test_nezha_flipflop.py``'s shape exactly (that file's own docstring is
this ticket's explicit test precedent): compile with the system C++
compiler, run the resulting binary, assert it exits 0.

Collected under ``tests/sim/unit/`` -- already within ``pyproject.toml``'s
``testpaths``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_otos_odometer.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "otos_odometer_harness.cpp"
_HOST_FAKE_SRC = _SOURCE_DIR / "com" / "i2c_bus_host.cpp"
_OTOS_ODOMETER_SRC = _SOURCE_DIR / "hal" / "otos" / "otos_odometer.cpp"

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


def test_otos_odometer_harness_compiles_and_passes(tmp_path):
    """Compile the Hal::OtosOdometer harness and assert every scenario passes."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _HOST_FAKE_SRC.is_file(), f"HOST_BUILD fake missing: {_HOST_FAKE_SRC}"
    assert _OTOS_ODOMETER_SRC.is_file(), f"otos_odometer.cpp missing: {_OTOS_ODOMETER_SRC}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "otos_odometer_harness"

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
            str(_HOST_FAKE_SRC),
            str(_OTOS_ODOMETER_SRC),
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "otos_odometer_harness.cpp / otos_odometer.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "otos_odometer_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
