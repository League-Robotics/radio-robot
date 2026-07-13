"""Off-hardware acceptance proof for ticket DB-003 (device-bus-tickets.md).

Compiles ``devices_i2c_bus_harness.cpp`` together with the HOST_BUILD
scripted-fake implementation (``source/devices/i2c_bus_host.cpp``) against
the SAME ``source/devices/i2c_bus.h`` every ARM build compiles, with
``-DHOST_BUILD`` so the header's/the .cpp's HOST_BUILD fork is what gets
exercised — no MicroBitI2C, no CODAL, no wall clock, no real sleeps. Mirrors
``test_i2c_bus_clearance.py``'s shape exactly (the pre-port harness for
``source/com/i2c_bus.*``): compile with the system C++ compiler, run the
resulting binary, assert it exits 0.

Collected under ``tests/sim/unit/`` alongside the other harness wrappers —
already within ``pyproject.toml``'s ``testpaths = ["tests/sim"]``, no
configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_devices_i2c_bus.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "devices_i2c_bus_harness.cpp"
_HOST_FAKE_SRC = _SOURCE_DIR / "devices" / "i2c_bus_host.cpp"

# messages/common.h documents its own target as "CODAL C++11" — build the
# host harness to the same standard so it exercises exactly the language
# subset the firmware itself uses (matches every other tests/sim/unit
# harness's own _CXX_STANDARD).
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


def test_devices_i2c_bus_harness_compiles_and_passes(tmp_path):
    """Compile the Devices::I2CBus HOST_BUILD fake + harness and assert every scenario passes."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _HOST_FAKE_SRC.is_file(), f"HOST_BUILD fake missing: {_HOST_FAKE_SRC}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "devices_i2c_bus_harness"

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
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "devices_i2c_bus_harness.cpp / i2c_bus_host.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "devices_i2c_bus_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
