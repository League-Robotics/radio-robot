"""Off-hardware acceptance proof for ticket 105-001 (SUC-018), App::RobotLoop
(``source/app/robot_loop.{h,cpp}``) -- the boot loop + main cycle body
extracted from ``source/main.cpp``.

Compiles ``app_robot_loop_harness.cpp`` together with every HOST_BUILD
implementation it needs (``robot_loop.cpp`` itself, every ``app/`` module it
composes, every ``devices/`` leaf those modules touch, the HOST_BUILD
``Devices::Clock``/``Devices::Sleeper`` fakes, ``TestSim::SimPlant``
(``tests/_infra/sim/sim_plant.cpp`` -- ticket 108-002's real Devices::I2CBus
implementation, plus its own ``tests/sim/plant/{wheel,otos}_plant.cpp``
physics dependencies), and the wire codec ``App::Comms``/``App::Telemetry``
need to encode/decode) with ``-DHOST_BUILD``, against the SAME headers every
ARM build compiles -- ``robot_loop.h``/``robot_loop.cpp`` include no
``MicroBit.h`` anywhere in this graph (the ticket's own acceptance
criterion). Mirrors ``test_app_preamble.py``/``test_app_odometry.py``'s
exact shape: compile with the system C++ compiler, run the resulting
binary, assert it exits 0.

Migrated by sprint 108 ticket 009 off the deleted ``source/devices/
i2c_bus_host.cpp`` scripted-FIFO Devices::I2CBus fake — see
``app_robot_loop_harness.cpp``'s own header and ``scripted_i2c_hook.h`` for
the migration rationale.

Collected under ``tests/sim/unit/`` -- already within ``pyproject.toml``'s
``testpaths = ["tests/sim"]``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_app_robot_loop.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_TESTS_SIM_DIR = _REPO_ROOT / "tests" / "sim"
_INFRA_SIM_DIR = _REPO_ROOT / "tests" / "_infra" / "sim"
_PLANT_DIR = _REPO_ROOT / "tests" / "sim" / "plant"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "app_robot_loop_harness.cpp"

_ROBOT_LOOP_SRC = _SOURCE_DIR / "app" / "robot_loop.cpp"
_PREAMBLE_SRC = _SOURCE_DIR / "app" / "preamble.cpp"
_COMMS_SRC = _SOURCE_DIR / "app" / "comms.cpp"
_TELEMETRY_SRC = _SOURCE_DIR / "app" / "telemetry.cpp"
_DEADMAN_SRC = _SOURCE_DIR / "app" / "deadman.cpp"
_DRIVE_SRC = _SOURCE_DIR / "app" / "drive.cpp"
_ODOMETRY_SRC = _SOURCE_DIR / "app" / "odometry.cpp"

_NEZHA_MOTOR_SRC = _SOURCE_DIR / "devices" / "nezha_motor.cpp"
_VELOCITY_PID_SRC = _SOURCE_DIR / "devices" / "velocity_pid.cpp"
_OTOS_SRC = _SOURCE_DIR / "devices" / "otos.cpp"
_COLOR_SENSOR_SRC = _SOURCE_DIR / "devices" / "color_sensor.cpp"
_LINE_SENSOR_SRC = _SOURCE_DIR / "devices" / "line_sensor.cpp"
_CLOCK_HOST_FAKE_SRC = _SOURCE_DIR / "devices" / "clock_host.cpp"

_BODY_KINEMATICS_SRC = _SOURCE_DIR / "kinematics" / "body_kinematics.cpp"

_WIRE_SRC = _SOURCE_DIR / "messages" / "wire.cpp"
_WIRE_RUNTIME_SRC = _SOURCE_DIR / "messages" / "wire_runtime.cpp"

_SIM_PLANT_SRC = _INFRA_SIM_DIR / "sim_plant.cpp"
_WHEEL_PLANT_SRC = _PLANT_DIR / "wheel_plant.cpp"
_OTOS_PLANT_SRC = _PLANT_DIR / "otos_plant.cpp"

# Matches every other tests/sim/unit harness's own compiled standard.
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


def test_app_robot_loop_harness_compiles_and_passes(tmp_path):
    """Compile App::RobotLoop + every module/leaf it composes + the harness; assert every scenario passes."""
    sources = [
        _HARNESS_SRC,
        _ROBOT_LOOP_SRC,
        _PREAMBLE_SRC,
        _COMMS_SRC,
        _TELEMETRY_SRC,
        _DEADMAN_SRC,
        _DRIVE_SRC,
        _ODOMETRY_SRC,
        _NEZHA_MOTOR_SRC,
        _VELOCITY_PID_SRC,
        _OTOS_SRC,
        _COLOR_SENSOR_SRC,
        _LINE_SENSOR_SRC,
        _CLOCK_HOST_FAKE_SRC,
        _BODY_KINEMATICS_SRC,
        _WIRE_SRC,
        _WIRE_RUNTIME_SRC,
        _SIM_PLANT_SRC,
        _WHEEL_PLANT_SRC,
        _OTOS_PLANT_SRC,
    ]
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "app_robot_loop_harness"

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
            str(_TESTS_SIM_DIR),
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
        "app_robot_loop_harness.cpp / its dependencies failed to compile "
        "-- confirm no MicroBit.h dependency leaked into robot_loop.{h,cpp}:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run_result.returncode == 0, (
        "app_robot_loop_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
