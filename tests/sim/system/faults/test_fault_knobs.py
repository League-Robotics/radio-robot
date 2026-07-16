"""Off-hardware acceptance proof, migrated (ticket 108-004) from TestSim::
WheelPlant's three fault-injection knobs (``setDisconnected()``/
``freezePosition()``/``setDropoutRate()``, ``tests/sim/plant/wheel_plant.h``)
driven through the deleted ``TestSim::SimApi`` (105-005, SUC-022) onto the
same knobs now surfaced per-port on ``TestSim::SimPlant``
(``tests/_infra/sim/sim_plant.h``) via ``TestSim::SimHarness::plant()``, and
asserted against the FIRMWARE's own observable reaction in decoded
telemetry -- the retargeted ``sim-hardware-fault-injection.md`` issue's ask,
delivered against this sprint's own plant/harness rather than the deleted
SimMotor sim.

Compiles ``fault_knobs_harness.cpp`` together with ``sim_plant.cpp``
(``tests/_infra/sim/`` -- replacing the deleted ``sim_api.cpp``),
``wire_test_codec.cpp``, the plant sources, and the same full HOST_BUILD
Devices/App/messages/kinematics dependency graph ``test_sim_api.py`` already
compiles, with ``-DHOST_BUILD``, against the SAME headers every ARM build
compiles. Mirrors ``test_sim_api.py``'s exact shape: compile with the system
C++ compiler, run the resulting binary, assert it exits 0.

Collected under ``tests/sim/system/faults/`` -- already within
``pyproject.toml``'s ``testpaths = ["tests/sim"]``, no configuration change
needed. Run just these scenarios with:

    uv run python -m pytest tests/sim/system/ -k fault -v
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/system/faults/test_fault_knobs.py -> faults -> system -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "source"
_FAULTS_DIR = pathlib.Path(__file__).resolve().parent
_SUPPORT_DIR = _FAULTS_DIR.parent.parent / "support"
_PLANT_DIR = _FAULTS_DIR.parent.parent / "plant"
_INFRA_SIM_DIR = _REPO_ROOT / "tests" / "_infra" / "sim"

_HARNESS_SRC = _FAULTS_DIR / "fault_knobs_harness.cpp"
_SIM_PLANT_SRC = _INFRA_SIM_DIR / "sim_plant.cpp"
_WIRE_TEST_CODEC_SRC = _SUPPORT_DIR / "wire_test_codec.cpp"
_WHEEL_PLANT_SRC = _PLANT_DIR / "wheel_plant.cpp"
_OTOS_PLANT_SRC = _PLANT_DIR / "otos_plant.cpp"

_APP_SOURCES = [
    _SOURCE_DIR / "app" / "robot_loop.cpp",
    _SOURCE_DIR / "app" / "comms.cpp",
    _SOURCE_DIR / "app" / "telemetry.cpp",
    _SOURCE_DIR / "app" / "deadman.cpp",
    _SOURCE_DIR / "app" / "drive.cpp",
    _SOURCE_DIR / "app" / "odometry.cpp",
    _SOURCE_DIR / "app" / "preamble.cpp",
]
_DEVICE_SOURCES = [
    _INFRA_SIM_DIR / "sim_clock.cpp",
    _SOURCE_DIR / "devices" / "velocity_pid.cpp",
    _SOURCE_DIR / "devices" / "nezha_motor.cpp",
    _SOURCE_DIR / "devices" / "otos.cpp",
    _SOURCE_DIR / "devices" / "color_sensor.cpp",
    _SOURCE_DIR / "devices" / "line_sensor.cpp",
]
_MESSAGE_SOURCES = [
    _SOURCE_DIR / "messages" / "wire.cpp",
    _SOURCE_DIR / "messages" / "wire_runtime.cpp",
]
_KINEMATICS_SOURCES = [
    _SOURCE_DIR / "kinematics" / "body_kinematics.cpp",
]

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
    return (
        [_HARNESS_SRC, _SIM_PLANT_SRC, _WIRE_TEST_CODEC_SRC, _WHEEL_PLANT_SRC, _OTOS_PLANT_SRC]
        + _APP_SOURCES
        + _DEVICE_SOURCES
        + _MESSAGE_SOURCES
        + _KINEMATICS_SOURCES
    )


def test_fault_knobs_harness_compiles_and_passes(tmp_path):
    """Compile the fault-knob harness + its full dependency graph; assert
    every scenario passes."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "fault_knobs_harness"

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
            str(_SUPPORT_DIR),
            "-I",
            str(_PLANT_DIR),
            "-I",
            str(_INFRA_SIM_DIR),
            "-o",
            str(binary),
            *[str(src) for src in sources],
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "fault_knobs_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run_result.returncode == 0, (
        "fault_knobs_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )
    print(run_result.stdout)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
