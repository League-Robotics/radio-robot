"""src/tests/sim/unit/test_configure_entry_points.py -- ticket 132-007's own
acceptance test (the-configuration-object.md, sprint 132 "configuration
discipline"): Config::Robot's derived-value methods
(effectiveTrackWidth()/rotationOffsetPos()/rotationOffsetNeg()/
velocityFilterWeight(), config/robot.h) plus five of the six subsystem
configure() entry points (Drive::configure(),
App::configurePlanner()/configureMotor()/configureOtos()). RobotLoop::
configure() -- the sixth -- is covered by a scenario added to
test_configurator_loadbaked.py instead, since it needs the full composition
root; the other five do not.

Compiles ``configure_entry_points_harness.cpp`` together with drive.cpp,
boot_calibration.cpp, config/boot_config.cpp, and the standalone
Motion::Planner sources (planner.cpp/profile.cpp/estimation.cpp/shape.cpp),
runs the resulting binary, and asserts it exits 0 -- printing its own
human-readable per-scenario trace. No I2C bus, no sim plant: every
Devices::Motor/Devices::Otos this harness touches is a small in-file test
double implementing the pure interface directly.

    uv run python -m pytest src/tests/sim/unit/test_configure_entry_points.py -v -s
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_configure_entry_points.py -> unit -> sim -> tests -> src -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_UNIT_DIR = pathlib.Path(__file__).resolve().parent

_HARNESS_SRC = _UNIT_DIR / "configure_entry_points_harness.cpp"

_APP_SOURCES = [
    _SOURCE_DIR / "app" / "drive.cpp",
    _SOURCE_DIR / "app" / "boot_calibration.cpp",
]
_MOTION_SOURCES = [
    _REPO_ROOT / "src" / "motion" / "planner" / "profile.cpp",
    _REPO_ROOT / "src" / "motion" / "planner" / "estimation.cpp",
    _REPO_ROOT / "src" / "motion" / "planner" / "shape.cpp",
    _REPO_ROOT / "src" / "motion" / "planner" / "planner.cpp",
    _REPO_ROOT / "src" / "motion" / "navigator" / "arc_solver.cpp",  # 135-004
    _REPO_ROOT / "src" / "motion" / "navigator" / "navigator.cpp",  # 135-004
]
_CONFIG_SOURCES = [
    # Config::default*Config()/defaultPlannerLimits() -- bootPlannerLimits()
    # (boot_calibration.cpp, linked above but unused by this harness's own
    # scenarios) still calls Config::defaultPlannerLimits() at link time.
    _SOURCE_DIR / "config" / "boot_config.cpp",
]
_DEVICE_SOURCES = [
    # Devices::Otos's virtual destructor is declared out-of-line (otos.h)
    # and defined here -- RecordingOtos's own dtor chain needs the symbol
    # even though this harness never constructs a RealOtos.
    _SOURCE_DIR / "hardware" / "generic" / "real_otos.cpp",
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
    return [_HARNESS_SRC] + _APP_SOURCES + _MOTION_SOURCES + _CONFIG_SOURCES + _DEVICE_SOURCES


def test_configure_entry_points(tmp_path):
    """Compile configure_entry_points_harness.cpp + its dependency graph;
    assert every derived-value method and Drive/configurePlanner/
    configureMotor/configureOtos scenario passes."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "configure_entry_points_harness"

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
            "-o",
            str(binary),
            *[str(src) for src in sources],
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "configure_entry_points_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    print(run_result.stdout)
    assert run_result.returncode == 0, (
        "configure_entry_points_harness reported a failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
