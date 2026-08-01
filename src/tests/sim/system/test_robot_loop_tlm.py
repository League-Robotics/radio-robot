"""Off-hardware, end-to-end acceptance proof for the TLM wire-command
surface driven through the REAL App::RobotLoop (ticket 125-006,
telemetry-emit-policy-rebuild-spec.md Part 8, sim criteria #8/#9/#10).

Compiles ``robot_loop_tlm_harness.cpp`` against TestSim::SimHarness
(``src/sim/sim_harness.h``) plus its full dependency graph, with
``-DHOST_BUILD``, against the SAME headers every ARM build compiles.
Mirrors ``test_sim_api.py``'s exact shape (same composition root, same
``_APP_SOURCES``-style source lists) -- see that file's own header for the
full rationale; see ``robot_loop_tlm_harness.cpp``'s own file header for
why these scenarios live here rather than in
``src/tests/sim/unit/app_robot_loop_harness.cpp`` (that harness predates
and is unrelated to this ticket's own breakage -- an independent
Planner/Configurator constructor-shape change, see the harness's header).

Collected under ``src/tests/sim/system/`` -- already within
``pyproject.toml``'s ``testpaths = ["src/tests/sim"]``, no configuration
change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/system/test_robot_loop_tlm.py -> system -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_SYSTEM_DIR = pathlib.Path(__file__).resolve().parent
_SUPPORT_DIR = _SYSTEM_DIR.parent / "support"
_PLANT_DIR = _SYSTEM_DIR.parent / "plant"
_INFRA_SIM_DIR = _REPO_ROOT / "src" / "sim"

_HARNESS_SRC = _SYSTEM_DIR / "robot_loop_tlm_harness.cpp"
_SIM_PLANT_SRC = _INFRA_SIM_DIR / "sim_plant.cpp"
_WIRE_TEST_CODEC_SRC = _SUPPORT_DIR / "wire_test_codec.cpp"
_BENCH_TEST_CONFIG_SRC = _SUPPORT_DIR / "bench_test_config.cpp"
_WHEEL_PLANT_SRC = _PLANT_DIR / "wheel_plant.cpp"
_OTOS_PLANT_SRC = _PLANT_DIR / "otos_plant.cpp"

# Mirrors test_sim_api.py's own _APP_SOURCES/_MOTION_SOURCES/_DEVICE_SOURCES/
# _CONFIG_SOURCES/_MESSAGE_SOURCES/_KINEMATICS_SOURCES exactly -- SAME
# composition root (TestSim::SimHarness), so the SAME full build-source list
# applies (see MEMORY note: "Planner .cpp needs FOUR build source lists" --
# this pytest file is a fifth copy, keep it in lockstep with test_sim_api.py
# if the Planner/RobotLoop dependency graph ever changes again).
_APP_SOURCES = [
    _SOURCE_DIR / "app" / "robot_loop.cpp",
    _SOURCE_DIR / "app" / "comms.cpp",
    # debug.cpp (129-003): App::debugf()'s only implementation --
    # TestSim::SimHarness's constructor always calls
    # App::setDebugSink(&comms_) now (HOST_BUILD is defined below,
    # so the real, non-stub setDebugSink()/debugf() are what this
    # graph links), mirroring src/sim/CMakeLists.txt's own
    # APP_SOURCES entry.
    _SOURCE_DIR / "app" / "debug.cpp",
    _SOURCE_DIR / "app" / "configurator.cpp",
    _SOURCE_DIR / "app" / "telemetry.cpp",
    _REPO_ROOT / "src" / "motion" / "planner" / "profile.cpp",
    _REPO_ROOT / "src" / "motion" / "planner" / "estimation.cpp",
    _REPO_ROOT / "src" / "motion" / "planner" / "wheel_pid.cpp",
    _REPO_ROOT / "src" / "motion" / "planner" / "shape.cpp",
    _REPO_ROOT / "src" / "motion" / "planner" / "wheel_trim.cpp",
    _REPO_ROOT / "src" / "motion" / "planner" / "planner.cpp",
    _SOURCE_DIR / "app" / "drive.cpp",
    _REPO_ROOT / "src" / "motion" / "odometry.cpp",
    _SOURCE_DIR / "app" / "preamble.cpp",
]
# 128-015: the deleted closed-loop wheel-velocity PID (formerly the sole
# entry here) is gone outright -- zero instantiations; App::Drive holds no
# controller of its own (open-loop duty from calibrated speed, drive.h's
# own header). See src/motion/DESIGN.md's "wheel control generations" note.
_MOTION_SOURCES = []
_DEVICE_SOURCES = [
    _INFRA_SIM_DIR / "sim_clock.cpp",
    _SOURCE_DIR / "devices" / "nezha_motor.cpp",
    _SOURCE_DIR / "devices" / "otos.cpp",
    _SOURCE_DIR / "devices" / "color_sensor.cpp",
    _SOURCE_DIR / "devices" / "line_sensor.cpp",
]
_CONFIG_SOURCES = [
    _SOURCE_DIR / "config" / "persisted_tuning.cpp",
]
_MESSAGE_SOURCES = [
    _SOURCE_DIR / "messages" / "wire.cpp",
    _SOURCE_DIR / "messages" / "wire_runtime.cpp",
]
_KINEMATICS_SOURCES = [
    _REPO_ROOT / "src" / "motion" / "body_kinematics.cpp",
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
        [_HARNESS_SRC, _SIM_PLANT_SRC, _WIRE_TEST_CODEC_SRC, _BENCH_TEST_CONFIG_SRC,
         _WHEEL_PLANT_SRC, _OTOS_PLANT_SRC]
        + _APP_SOURCES
        + _MOTION_SOURCES
        + _DEVICE_SOURCES
        + _CONFIG_SOURCES
        + _MESSAGE_SOURCES
        + _KINEMATICS_SOURCES
    )


def test_robot_loop_tlm_harness_compiles_and_passes(tmp_path):
    """Compile the TLM/RobotLoop end-to-end graph + the harness; assert every scenario passes."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "robot_loop_tlm_harness"

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
            str(_REPO_ROOT / "src"),
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
        "robot_loop_tlm_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run_result.returncode == 0, (
        "robot_loop_tlm_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )
    print(run_result.stdout)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
