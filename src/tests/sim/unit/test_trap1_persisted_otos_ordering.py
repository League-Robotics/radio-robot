"""src/tests/sim/unit/test_trap1_persisted_otos_ordering.py -- 132-015's own
regression proof for trap 1 (the-configuration-object.md): persisted OTOS
tuning must reach the chip-level calibration-scalar register, which only
happens when Core::RobotGraph::loadPersistedTuning() runs AFTER
Core::RobotLoop::boot() -- never before (every Hardware::RealOtos setter is a
no-op until begin() sets initialized_ = true, and begin() itself only runs
inside boot()'s own Preamble loop).

Compiles ``trap1_persisted_otos_ordering_harness.cpp`` together with the same
full HOST_BUILD dependency graph every other post-gut sim/unit harness
compiles (SimHarness composes the real Core::RobotLoop graph -- see
sim_harness.h's own header). Mirrors test_sim_harness_configure.py's own
source-list shape exactly (same composition root, same dependency graph).

    uv run python -m pytest src/tests/sim/unit/test_trap1_persisted_otos_ordering.py -v -s
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_trap1_persisted_otos_ordering.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_UNIT_DIR = pathlib.Path(__file__).resolve().parent
_SUPPORT_DIR = _UNIT_DIR.parent / "support"
_PLANT_DIR = _UNIT_DIR.parent / "plant"
_INFRA_SIM_DIR = _REPO_ROOT / "src" / "firm" / "platform" / "host"

_HARNESS_SRC = _UNIT_DIR / "trap1_persisted_otos_ordering_harness.cpp"
_SIM_PLANT_SRC = _INFRA_SIM_DIR / "sim_plant.cpp"
_WIRE_TEST_CODEC_SRC = _SUPPORT_DIR / "wire_test_codec.cpp"
_WHEEL_PLANT_SRC = _PLANT_DIR / "wheel_plant.cpp"
_OTOS_PLANT_SRC = _PLANT_DIR / "otos_plant.cpp"

# Same dependency graph as test_sim_harness_configure.py -- both harnesses
# compose the real Core::RobotGraph via sim_harness.h's TestSim::SimHarness.
_APP_SOURCES = [
    _SOURCE_DIR / "core" / "robot_loop.cpp",
    _SOURCE_DIR / "core" / "comms.cpp",
    _SOURCE_DIR / "core" / "debug.cpp",
    _SOURCE_DIR / "core" / "configurator.cpp",
    _SOURCE_DIR / "core" / "telemetry.cpp",
    # src/firm/motion/ (planner/navigator/odometry) is GONE -- the
    # exploratory-kernel rewrite (2026-08-15) folded the wheel control it
    # fed into Control::DifferentialDrive, one class + fiber.
    _SOURCE_DIR / "control" / "differential_drive.cpp",
    _SOURCE_DIR / "core" / "preamble.cpp",
    _SOURCE_DIR / "core" / "boot_wiring.cpp",
    _SOURCE_DIR / "core" / "boot_calibration.cpp",
]
_DEVICE_SOURCES = [
    _INFRA_SIM_DIR / "sim_clock.cpp",
    _SOURCE_DIR / "hardware" / "nezha" / "nezha_motor.cpp",
    _SOURCE_DIR / "hardware" / "generic" / "real_otos.cpp",
    _SOURCE_DIR / "hardware" / "planetx" / "color_sensor.cpp",
    _SOURCE_DIR / "hardware" / "planetx" / "line_sensor.cpp",
]
# This test's own reason for being: robot_loop.cpp/boot_wiring.cpp pull in
# config/persisted_tuning.h's TuningStore/TuningSnapshot/serializeSnapshot()
# seam -- the harness links persisted_tuning.cpp directly and drives it
# through a SeededTuningStore double (its own file header).
_CONFIG_SOURCES = [
    _SOURCE_DIR / "config" / "persisted_tuning.cpp",
    _SOURCE_DIR / "config" / "boot_config.cpp",
]
_MESSAGE_SOURCES = [
    _SOURCE_DIR / "messages" / "wire.cpp",
    _SOURCE_DIR / "messages" / "wire_runtime.cpp",
]
_KINEMATICS_SOURCES = [
    _REPO_ROOT / "src" / "firm" / "kinematics" / "differential.cpp",
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
        + _CONFIG_SOURCES
        + _MESSAGE_SOURCES
        + _KINEMATICS_SOURCES
    )


def test_trap1_persisted_otos_ordering_harness_compiles_and_passes(tmp_path):
    """Compile trap1_persisted_otos_ordering_harness.cpp + its full dependency
    graph; assert both ordering scenarios pass."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "trap1_persisted_otos_ordering_harness"

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
        "trap1_persisted_otos_ordering_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    print(run_result.stdout)
    assert run_result.returncode == 0, (
        "trap1_persisted_otos_ordering_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
