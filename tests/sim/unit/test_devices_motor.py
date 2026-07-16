"""Off-hardware acceptance proof for ticket DB-004 (device-bus-tickets.md).

Compiles ``devices_motor_harness.cpp`` together with ``TestSim::SimPlant``
(``tests/_infra/sim/sim_plant.cpp`` — ticket 108-002's real Devices::I2CBus
implementation) plus its own ``tests/sim/plant/{wheel,otos}_plant.cpp``
physics dependencies, ``devices/velocity_pid.cpp``, and
``devices/nezha_motor.cpp`` against the SAME ``source/devices/`` headers
every ARM build compiles, with ``-DHOST_BUILD`` so the HOST_BUILD fork is
what gets exercised — no MicroBitI2C, no CODAL, no wall clock, no real
sleeps. Mirrors ``test_plant.py``'s/``test_devices_color_sensor_apds_probe.
py``'s shape exactly: compile with the system C++ compiler, run the
resulting binary, assert it exits 0.

Migrated by sprint 108 ticket 009 off the deleted ``source/devices/
i2c_bus_host.cpp`` scripted-FIFO Devices::I2CBus fake — see
``devices_motor_harness.cpp``'s own header and ``scripted_i2c_hook.h`` for
the migration rationale (SimPlant + a FIFO-scripting hook, in place of the
deleted concrete fake's own queueWrite()/queueRead() surface).

Collected under ``tests/sim/unit/`` alongside the other harness wrappers —
already within ``pyproject.toml``'s ``testpaths = ["tests/sim"]``, no
configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_devices_motor.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_DEVICES_DIR = _SOURCE_DIR / "devices"
_INFRA_SIM_DIR = _REPO_ROOT / "tests" / "_infra" / "sim"
_PLANT_DIR = _REPO_ROOT / "tests" / "sim" / "plant"

_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "devices_motor_harness.cpp"
_SIM_PLANT_SRC = _INFRA_SIM_DIR / "sim_plant.cpp"
_WHEEL_PLANT_SRC = _PLANT_DIR / "wheel_plant.cpp"
_OTOS_PLANT_SRC = _PLANT_DIR / "otos_plant.cpp"
_BODY_KINEMATICS_SRC = _SOURCE_DIR / "kinematics" / "body_kinematics.cpp"
_VELOCITY_PID_SRC = _DEVICES_DIR / "velocity_pid.cpp"
_NEZHA_MOTOR_SRC = _DEVICES_DIR / "nezha_motor.cpp"

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


def _all_sources():
    return [
        _HARNESS_SRC,
        _SIM_PLANT_SRC,
        _WHEEL_PLANT_SRC,
        _OTOS_PLANT_SRC,
        _BODY_KINEMATICS_SRC,
        _VELOCITY_PID_SRC,
        _NEZHA_MOTOR_SRC,
    ]


def test_devices_motor_harness_compiles_and_passes(tmp_path):
    """Compile the Devices motor leaf + armor + PID sources, SimPlant, and
    the harness; assert every scenario passes."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "devices_motor_harness"

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
        "devices_motor_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "devices_motor_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
