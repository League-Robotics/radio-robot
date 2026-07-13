"""Off-hardware acceptance proof for ticket DB-006 (device-bus-tickets.md).

Compiles ``devices_sensors_harness.cpp`` together with the HOST_BUILD
implementation it needs (``source/devices/i2c_bus_host.cpp`` — the scripted
I2CBus fake from DB-003) plus the real ``source/devices/color_sensor.cpp``
and ``source/devices/line_sensor.cpp`` against the SAME ``source/devices/``
headers every ARM build compiles, with ``-DHOST_BUILD`` so the HOST_BUILD
fork is what gets exercised — no MicroBitI2C, no CODAL, no wall clock, no
real sleeps. Mirrors ``test_devices_otos.py``'s shape exactly: compile with
the system C++ compiler, run the resulting binary, assert it exits 0.

Collected under ``tests/sim/unit/`` alongside the other harness wrappers —
already within ``pyproject.toml``'s ``testpaths = ["tests/sim"]``, no
configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_devices_sensors.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_DEVICES_DIR = _SOURCE_DIR / "devices"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "devices_sensors_harness.cpp"
_I2C_HOST_FAKE_SRC = _DEVICES_DIR / "i2c_bus_host.cpp"
_COLOR_SENSOR_SRC = _DEVICES_DIR / "color_sensor.cpp"
_LINE_SENSOR_SRC = _DEVICES_DIR / "line_sensor.cpp"

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


def test_devices_sensors_harness_compiles_and_passes(tmp_path):
    """Compile the Devices::ColorSensorLeaf/LineSensorLeaf leaves and the harness; assert every scenario passes."""
    sources = [_HARNESS_SRC, _I2C_HOST_FAKE_SRC, _COLOR_SENSOR_SRC, _LINE_SENSOR_SRC]
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "devices_sensors_harness"

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
            *[str(src) for src in sources],
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "devices_sensors_harness.cpp / its Devices sources failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "devices_sensors_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
