"""src/tests/sim/system/test_scripted_twist_demo.py -- 105-006 (SUC-023), this
sprint's own Definition of Done: the headless scripted-twist demo.

Compiles ``scripted_twist_demo_harness.cpp`` together with ``sim_plant.cpp``
(``src/sim/`` -- ticket 108-004's migration off the deleted
``sim_api.cpp``), ``wire_test_codec.cpp``, the plant sources, and the same
full HOST_BUILD Devices/App/messages/kinematics dependency graph
``test_sim_api.py``/``test_fault_knobs.py`` already compile, with
``-DHOST_BUILD``, against the SAME headers every ARM build compiles.
Mirrors their exact shape: compile with the system C++ compiler, run the
resulting binary, assert it exits 0, print its stdout (the human-readable
cycle-by-cycle trace).

Collected under ``src/tests/sim/system/`` -- already within ``pyproject.toml``'s
``testpaths = ["src/tests/sim"]``, no configuration change needed. Run just this
scenario with:

    uv run python -m pytest src/tests/sim/system/test_scripted_twist_demo.py -v

Runnable standalone, with the harness's own printed trace visible (pytest
captures stdout by default; ``-s`` disables that capture) -- this IS the
sprint's own stakeholder-visible "run one command and see the sim loop
move" proof (SUC-023):

    uv run python src/tests/sim/system/test_scripted_twist_demo.py

(equivalent to ``pytest src/tests/sim/system/test_scripted_twist_demo.py -v -s``
-- see this file's own ``__main__`` block below). For the trace without any
pytest assertion machinery at all, compile and run the harness directly,
e.g.:

    c++ -std=c++20 -DHOST_BUILD -I source -I src/tests/sim/support -I src/tests/sim/plant \\
        -I src/sim \\
        -o /tmp/scripted_twist_demo \\
        src/tests/sim/system/scripted_twist_demo_harness.cpp \\
        src/sim/sim_plant.cpp src/tests/sim/support/wire_test_codec.cpp \\
        src/tests/sim/plant/wheel_plant.cpp src/tests/sim/plant/otos_plant.cpp \\
        src/firm/app/robot_loop.cpp src/firm/app/comms.cpp src/firm/app/telemetry.cpp \\
        src/firm/app/deadman.cpp src/firm/app/drive.cpp src/firm/app/odometry.cpp \\
        src/firm/app/preamble.cpp src/sim/sim_clock.cpp \\
        src/firm/devices/velocity_pid.cpp src/firm/devices/nezha_motor.cpp src/firm/devices/otos.cpp \\
        src/firm/devices/color_sensor.cpp src/firm/devices/line_sensor.cpp \\
        src/firm/messages/wire.cpp src/firm/messages/wire_runtime.cpp src/firm/kinematics/body_kinematics.cpp \\
    && /tmp/scripted_twist_demo
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/system/test_scripted_twist_demo.py -> system -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_SYSTEM_DIR = pathlib.Path(__file__).resolve().parent
_SUPPORT_DIR = _SYSTEM_DIR.parent / "support"
_PLANT_DIR = _SYSTEM_DIR.parent / "plant"
_INFRA_SIM_DIR = _REPO_ROOT / "src" / "sim"

_HARNESS_SRC = _SYSTEM_DIR / "scripted_twist_demo_harness.cpp"
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
    _SOURCE_DIR / "app" / "pilot.cpp",
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
# 109-003: robot_loop.h now includes app/pilot.h -> motion/executor.h ->
# motion/jerk_trajectory.h -> vendor/ruckig.
_RUCKIG_INCLUDE = _REPO_ROOT / "vendor" / "ruckig" / "include"
_RUCKIG_SRC_DIR = _REPO_ROOT / "vendor" / "ruckig" / "src"
_MOTION_SOURCES = [
    _SOURCE_DIR / "motion" / "jerk_trajectory.cpp",
    _SOURCE_DIR / "motion" / "executor.cpp",
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
        + _MOTION_SOURCES
        + sorted(_RUCKIG_SRC_DIR.glob("*.cpp"))
    )


def test_scripted_twist_demo_compiles_and_tells_the_story(tmp_path):
    """Compile scripted_twist_demo_harness.cpp + its full dependency graph;
    assert the demo's every phase passes, and print its own trace."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "scripted_twist_demo"

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
            "-I",
            str(_RUCKIG_INCLUDE),
            "-o",
            str(binary),
            *[str(src) for src in sources],
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "scripted_twist_demo_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    print(run_result.stdout)
    assert run_result.returncode == 0, (
        "scripted_twist_demo_harness reported a phase failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    # -s: don't capture stdout -- this is the "run one command and see the
    # sim loop move" invocation this file's own docstring documents.
    sys.exit(pytest.main([__file__, "-v", "-s"]))
