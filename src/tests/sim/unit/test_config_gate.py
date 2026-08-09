"""src/tests/sim/unit/test_config_gate.py -- ticket 114-001's own acceptance
proof: the configuration-completeness gate (App::RobotLoop::configured_/
markConfigured()/isConfigured()) makes "unconfigured" a real, refusable
state on TestSim::SimHarness, and the configured-then-accepted transition
actually produces real, measured wheel motion -- not merely an
ACK_STATUS_OK ack (the gap this ticket's own thrown-and-resolved exception
found: MotorArmor::configure() used to forward nothing to the wrapped
NezhaMotor, so a "configured" motor was still functionally dead -- see
sprint.md's Architecture Revision 1 / Decision 6).

Compiles ``config_gate_harness.cpp`` together with the same full HOST_BUILD
dependency graph ``test_sim_harness_configure.py``/``test_behavior_lock.py``
already compile (SimHarness composes the real App::RobotLoop graph), PLUS
``bench_test_config.cpp`` (the config-gate harness's own configured-then-
accepted scenario needs a real, nonzero configuration to push).

    uv run python -m pytest src/tests/sim/unit/test_config_gate.py -v -s
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_config_gate.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_UNIT_DIR = pathlib.Path(__file__).resolve().parent
_SUPPORT_DIR = _UNIT_DIR.parent / "support"
_PLANT_DIR = _UNIT_DIR.parent / "plant"
_INFRA_SIM_DIR = _REPO_ROOT / "src" / "firm" / "platform" / "host"

_HARNESS_SRC = _UNIT_DIR / "config_gate_harness.cpp"
_SIM_PLANT_SRC = _INFRA_SIM_DIR / "sim_plant.cpp"
_WIRE_TEST_CODEC_SRC = _SUPPORT_DIR / "wire_test_codec.cpp"
_BENCH_TEST_CONFIG_SRC = _SUPPORT_DIR / "bench_test_config.cpp"
_WHEEL_PLANT_SRC = _PLANT_DIR / "wheel_plant.cpp"
_OTOS_PLANT_SRC = _PLANT_DIR / "otos_plant.cpp"

# 115-006 (gut S1): heading_source.cpp/pilot.cpp/motion/executor.cpp/
# motion/jerk_trajectory.cpp/vendor/ruckig are all DELETED along with the
# rest of the motion stack -- sim_harness.h no longer includes app/pilot.h
# (or transitively motion/executor.h -> vendor/ruckig) at all, so none of
# those sources are compiled into this harness any more (mirrors
# test_sim_harness_configure.py's own identical note).
_APP_SOURCES = [
    _SOURCE_DIR / "app" / "robot_loop.cpp",
    _SOURCE_DIR / "app" / "comms.cpp",
    # debug.cpp (129-003): App::debugf()'s only implementation --
    # TestSim::SimHarness's constructor always calls
    # App::setDebugSink(&comms_) now (HOST_BUILD is defined below,
    # so the real, non-stub setDebugSink()/debugf() are what this
    # graph links), mirroring src/firm/platform/host/CMakeLists.txt's own
    # APP_SOURCES entry.
    _SOURCE_DIR / "app" / "debug.cpp",
    # configurator.cpp -- App::Configurator (command-ingestion-ring-buffered-
    # comms-subsystem-routing-two-stops.md §6): the CONFIG lifecycle moved
    # out of RobotLoop into its own module, which RobotLoop now holds a
    # reference to -- so this graph must link it.
    _SOURCE_DIR / "app" / "configurator.cpp",
    _SOURCE_DIR / "app" / "telemetry.cpp",
    # Planner integration (2026-07-26): the on-robot Motion::Planner now
    # drives the loop -- its library core joins every RobotLoop-linking
    # dependency graph.
    _REPO_ROOT / "src" / "motion" / "planner" / "profile.cpp",
    _REPO_ROOT / "src" / "motion" / "planner" / "estimation.cpp",
    _REPO_ROOT / "src" / "motion" / "planner" / "shape.cpp",
    _REPO_ROOT / "src" / "motion" / "planner" / "planner.cpp",
    _REPO_ROOT / "src" / "motion" / "navigator" / "arc_solver.cpp",  # 135-004
    _REPO_ROOT / "src" / "motion" / "navigator" / "navigator.cpp",  # 135-004
    _SOURCE_DIR / "app" / "drive.cpp",
    _REPO_ROOT / "src" / "motion" / "odometry.cpp",
    _SOURCE_DIR / "app" / "preamble.cpp",
    # 130-002 -- the shared composition root (App::composeRobot()/
    # RobotGraph) sim_harness.h now boots through, plus its
    # Config::boot_config-reading calibration helpers (the "four-source-
    # list trap" this ticket's own note calls out).
    _SOURCE_DIR / "app" / "boot_wiring.cpp",
    _SOURCE_DIR / "app" / "boot_calibration.cpp",
]
# 128-015: the deleted closed-loop wheel-velocity PID (formerly the sole
# entry here) is gone outright -- zero instantiations; App::Drive holds no
# controller of its own (open-loop duty from calibrated speed, drive.h's
# own header). See src/motion/DESIGN.md's "wheel control generations" note.
_MOTION_SOURCES = []
_DEVICE_SOURCES = [
    _INFRA_SIM_DIR / "sim_clock.cpp",
    _SOURCE_DIR / "hardware" / "nezha" / "nezha_motor.cpp",
    _SOURCE_DIR / "hardware" / "generic" / "real_otos.cpp",
    _SOURCE_DIR / "hardware" / "planetx" / "color_sensor.cpp",
    _SOURCE_DIR / "hardware" / "planetx" / "line_sensor.cpp",
]
# 114-004: robot_loop.cpp now #includes config/persisted_tuning.h and calls
# its pure serializeSnapshot()/Config::TuningStore seam directly.
_CONFIG_SOURCES = [
    _SOURCE_DIR / "config" / "persisted_tuning.cpp",
    # 130-002 -- both composition roots now bake the SAME robot-JSON
    # calibration by default (unify-sim-and-robot-composition-roots.md
    # work item 2).
    _SOURCE_DIR / "config" / "boot_config.cpp",
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


def test_config_gate_harness_compiles_and_passes(tmp_path):
    """Compile config_gate_harness.cpp + its full dependency graph; assert
    every scenario passes."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "config_gate_harness"

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
            str(_REPO_ROOT / "src" / "motion" / "planner"),  # 135-004: navigator.h's bare #include "planner.h"
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
        "config_gate_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    print(run_result.stdout)
    assert run_result.returncode == 0, (
        "config_gate_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
