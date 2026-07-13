"""Off-hardware acceptance proof for ticket 100-DBX (THE COMPLETE CUTOVER):
Subsystems::DeviceBusHardware / DeviceBusMotor / DeviceBusOdometer
(source/subsystems/device_bus_hardware.{h,cpp}).

Compiles ``device_bus_hardware_harness.cpp`` together with the REAL
``source/subsystems/device_bus_hardware.cpp`` (the bridge under test) and
every HOST_BUILD implementation ``Devices::DeviceBus`` needs (mirrors
``test_device_bus_lifecycle.py``'s own source list exactly --
``i2c_bus_host.cpp``/``clock_host.cpp`` scripted fakes, ``velocity_pid.cpp``,
``nezha_motor.cpp``, ``otos.cpp``, ``color_sensor.cpp``, ``line_sensor.cpp``,
``device_bus.cpp``), with ``-DHOST_BUILD`` so both the ``Devices::`` HOST_BUILD
fork and ``device_bus_hardware.h``'s own ``#ifndef HOST_BUILD`` constructor
split resolve to their host (no ``MicroBitI2C&``, no CODAL) form. No
``config/boot_config.cpp`` is needed -- the harness hand-builds its own
``msg::MotorConfig``/``Config::OtosBootConfig`` fixture values directly,
mirroring ``source/devices/bringup_main.cpp``'s own precedent for the same
reason (the isolation/host-build boundary, not a missing dependency).

Collected under ``tests/sim/unit/`` alongside the other harness wrappers --
already within ``pyproject.toml``'s ``testpaths = ["tests/sim"]``, no
configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_device_bus_hardware.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_DEVICES_DIR = _SOURCE_DIR / "devices"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "device_bus_hardware_harness.cpp"
_BRIDGE_SRC = _SOURCE_DIR / "subsystems" / "device_bus_hardware.cpp"
_I2C_HOST_FAKE_SRC = _DEVICES_DIR / "i2c_bus_host.cpp"
_CLOCK_HOST_FAKE_SRC = _DEVICES_DIR / "clock_host.cpp"
_VELOCITY_PID_SRC = _DEVICES_DIR / "velocity_pid.cpp"
_NEZHA_MOTOR_SRC = _DEVICES_DIR / "nezha_motor.cpp"
_OTOS_SRC = _DEVICES_DIR / "otos.cpp"
_COLOR_SENSOR_SRC = _DEVICES_DIR / "color_sensor.cpp"
_LINE_SENSOR_SRC = _DEVICES_DIR / "line_sensor.cpp"
_DEVICE_BUS_SRC = _DEVICES_DIR / "device_bus.cpp"

_SOURCES = [
    _HARNESS_SRC,
    _BRIDGE_SRC,
    _I2C_HOST_FAKE_SRC,
    _CLOCK_HOST_FAKE_SRC,
    _VELOCITY_PID_SRC,
    _NEZHA_MOTOR_SRC,
    _OTOS_SRC,
    _COLOR_SENSOR_SRC,
    _LINE_SENSOR_SRC,
    _DEVICE_BUS_SRC,
]

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


def test_device_bus_hardware_harness_compiles_and_passes(tmp_path):
    """Compile the real DeviceBusHardware bridge + its Devices deps + the harness; assert every scenario passes."""
    for src in _SOURCES:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "device_bus_hardware_harness"

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
            *[str(src) for src in _SOURCES],
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "device_bus_hardware_harness.cpp / device_bus_hardware.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "device_bus_hardware_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
