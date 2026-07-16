"""Off-hardware acceptance proof for ticket 108-008 (clasi/issues/
color-sensor-apds-probe-success-on-failure.md, 2026-07-13 code review
finding M4): Devices::ColorSensorLeaf::beginStep()'s APDS probe must not
latch present()==true on a NAK'd bus read.

Compiles ``devices_color_sensor_apds_probe_harness.cpp`` together with the
REAL ``src/firm/devices/color_sensor.cpp`` and ticket 108-002's real
``Devices::I2CBus`` implementation, ``TestSim::SimPlant``
(``src/sim/sim_plant.cpp`` -- itself named as this exact ticket's
consumer in its own header comment) plus SimPlant's own
``wheel_plant.cpp``/``otos_plant.cpp`` dependencies, with ``-DHOST_BUILD``,
against the SAME ``src/firm/devices/`` headers every ARM build compiles.
Mirrors ``test_plant.py``'s exact shape: compile with the system C++
compiler, run the resulting binary, assert it exits 0.

Collected under ``src/tests/sim/unit/`` -- already within ``pyproject.toml``'s
``testpaths = ["src/tests/sim"]``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_devices_color_sensor_apds_probe.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_DEVICES_DIR = _SOURCE_DIR / "devices"
_INFRA_SIM_DIR = _REPO_ROOT / "src" / "sim"
_PLANT_DIR = _REPO_ROOT / "src" / "tests" / "sim" / "plant"

_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "devices_color_sensor_apds_probe_harness.cpp"
_COLOR_SENSOR_SRC = _DEVICES_DIR / "color_sensor.cpp"
_SIM_PLANT_SRC = _INFRA_SIM_DIR / "sim_plant.cpp"
_WHEEL_PLANT_SRC = _PLANT_DIR / "wheel_plant.cpp"
_OTOS_PLANT_SRC = _PLANT_DIR / "otos_plant.cpp"
_BODY_KINEMATICS_SRC = _SOURCE_DIR / "kinematics" / "body_kinematics.cpp"

# Matches every other src/tests/sim/{unit,plant,system} harness's own compiled
# standard.
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
        _COLOR_SENSOR_SRC,
        _SIM_PLANT_SRC,
        _WHEEL_PLANT_SRC,
        _OTOS_PLANT_SRC,
        _BODY_KINEMATICS_SRC,
    ]


def test_devices_color_sensor_apds_probe_harness_compiles_and_passes(tmp_path):
    """Compile ColorSensorLeaf + SimPlant + the harness; assert both the
    NAK-latches-absent and OK-latches-present scenarios pass."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "devices_color_sensor_apds_probe_harness"

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
        "devices_color_sensor_apds_probe_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run_result.returncode == 0, (
        "devices_color_sensor_apds_probe_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )
    print(run_result.stdout)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
