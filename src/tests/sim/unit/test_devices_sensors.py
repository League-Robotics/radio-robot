"""Off-hardware acceptance proof for ticket DB-006 (device-bus-tickets.md).

Compiles ``devices_sensors_harness.cpp`` together with ``TestSim::SimPlant``
(``src/sim/sim_plant.cpp`` — ticket 108-002's real Devices::I2CBus
implementation) plus its own ``src/tests/sim/plant/{wheel,otos}_plant.cpp``
physics dependencies, and the real ``src/firm/devices/color_sensor.cpp``
and ``src/firm/devices/line_sensor.cpp`` against the SAME ``src/firm/devices/``
headers every ARM build compiles, with ``-DHOST_BUILD``. Mirrors
``test_devices_otos.py``'s shape exactly: compile with the system C++
compiler, run the resulting binary, assert it exits 0.

Migrated by sprint 108 ticket 009 off the deleted ``src/firm/devices/
i2c_bus_host.cpp`` scripted-FIFO Devices::I2CBus fake — see
``devices_sensors_harness.cpp``'s own header and ``scripted_i2c_hook.h`` for
the migration rationale.

Collected under ``src/tests/sim/unit/`` alongside the other harness wrappers —
already within ``pyproject.toml``'s ``testpaths = ["src/tests/sim"]``, no
configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_devices_sensors.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_DEVICES_DIR = _SOURCE_DIR / "devices"
_INFRA_SIM_DIR = _REPO_ROOT / "src" / "sim"
_PLANT_DIR = _REPO_ROOT / "src" / "tests" / "sim" / "plant"

_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "devices_sensors_harness.cpp"
_SIM_PLANT_SRC = _INFRA_SIM_DIR / "sim_plant.cpp"
_WHEEL_PLANT_SRC = _PLANT_DIR / "wheel_plant.cpp"
_OTOS_PLANT_SRC = _PLANT_DIR / "otos_plant.cpp"
_BODY_KINEMATICS_SRC = _SOURCE_DIR / "kinematics" / "body_kinematics.cpp"
_COLOR_SENSOR_SRC = _DEVICES_DIR / "color_sensor.cpp"
_LINE_SENSOR_SRC = _DEVICES_DIR / "line_sensor.cpp"

# messages/common.h documents its own target as "CODAL C++11" — build the
# host harness to the same standard so it exercises exactly the language
# subset the firmware itself uses (matches every other src/tests/sim/unit
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


def _all_sources():
    return [
        _HARNESS_SRC,
        _SIM_PLANT_SRC,
        _WHEEL_PLANT_SRC,
        _OTOS_PLANT_SRC,
        _BODY_KINEMATICS_SRC,
        _COLOR_SENSOR_SRC,
        _LINE_SENSOR_SRC,
    ]


def test_devices_sensors_harness_compiles_and_passes(tmp_path):
    """Compile the Devices::ColorSensorLeaf/LineSensorLeaf leaves, SimPlant,
    and the harness; assert every scenario passes."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

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
            "-I",
            str(_INFRA_SIM_DIR),
            "-I",
            str(_PLANT_DIR),
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
