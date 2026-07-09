"""Off-hardware acceptance proof for ticket 094-004 (SUC-001/SUC-002/SUC-004):
the REWRITTEN Subsystems::Drivetrain -- now holding a Hardware& and owning a
Motion::SegmentExecutor + an 8-slot segment ring instead of the pre-094
held-output faceplate.

Compiles ``drivetrain_harness.cpp`` together with the real sources it now
transitively depends on -- ``subsystems/drivetrain.cpp``, ``kinematics/
body_kinematics.cpp``, the lifted executor (``motion/{segment_executor,
jerk_trajectory,stop_condition}.cpp`` + vendored Ruckig), the SimHardware
plant (``subsystems/sim_hardware.cpp`` + ``hal/sim/*.cpp`` +
``hal/velocity_pid.cpp``), and the REAL NezhaHardware/NezhaMotor
(``subsystems/nezha_hardware.cpp`` + ``hal/nezha/nezha_motor.cpp``) against
the HOST_BUILD scripted I2CBus fake (``com/i2c_bus_host.cpp``) for the
sprint's mandatory staging-only verification scenario -- against the SAME
``source/subsystems/drivetrain.h`` every ARM build compiles. Mirrors
``test_segment_executor.py``'s/``test_nezha_flipflop.py``'s own shape:
compile with the system C++ compiler, run the resulting binary, assert it
exits 0.

Collected under ``tests/sim/unit/`` -- already within ``pyproject.toml``'s
``testpaths``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_drivetrain.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "drivetrain_harness.cpp"

_SOURCES = [
    _HARNESS_SRC,
    _SOURCE_DIR / "subsystems" / "drivetrain.cpp",
    _SOURCE_DIR / "kinematics" / "body_kinematics.cpp",
    _SOURCE_DIR / "motion" / "segment_executor.cpp",
    _SOURCE_DIR / "motion" / "jerk_trajectory.cpp",
    _SOURCE_DIR / "motion" / "stop_condition.cpp",
    _SOURCE_DIR / "subsystems" / "sim_hardware.cpp",
    _SOURCE_DIR / "hal" / "sim" / "physics_world.cpp",
    _SOURCE_DIR / "hal" / "sim" / "sim_motor.cpp",
    _SOURCE_DIR / "hal" / "sim" / "sim_odometer.cpp",
    _SOURCE_DIR / "hal" / "velocity_pid.cpp",
    _SOURCE_DIR / "subsystems" / "nezha_hardware.cpp",
    _SOURCE_DIR / "hal" / "nezha" / "nezha_motor.cpp",
    _SOURCE_DIR / "hal" / "otos" / "otos_odometer.cpp",
    _SOURCE_DIR / "com" / "i2c_bus_host.cpp",
]

_RUCKIG_INCLUDE = _REPO_ROOT / "libraries" / "ruckig" / "include"
_RUCKIG_SRC_DIR = _REPO_ROOT / "libraries" / "ruckig" / "src"

# gnu++20 (GNU extensions -- newlib exposes M_PI, which Ruckig's roots.hpp
# needs) plus -fno-exceptions/-fno-rtti, matching test_segment_executor.py's
# own precedent -- this harness transitively compiles Ruckig too.
_CXX_STANDARD = "gnu++20"
# -DHOST_BUILD=1 marks this as a host build (segment_executor.cpp's
# kOutputHops/kDeadTime compile split resolves to the sim value; i2c_bus.h/
# nezha_motor.cpp select their HOST_BUILD forks) -- matches
# test_segment_executor.py's/test_nezha_flipflop.py's own fix.
_CONSTRAINT_FLAGS = ["-fno-exceptions", "-fno-rtti", "-DHOST_BUILD=1"]


def _find_cxx_compiler() -> str:
    """Locate a usable system C++ compiler, preferring c++ then clang++/g++."""
    import shutil

    for candidate in ("c++", "clang++", "g++"):
        found = shutil.which(candidate)
        if found:
            return found
    pytest.skip("no system C++ compiler (c++/clang++/g++) found on PATH")
    raise AssertionError("unreachable")  # pragma: no cover


def test_drivetrain_harness_compiles_and_passes(tmp_path):
    """Compile the Drivetrain 094-004 harness and assert every scenario passes."""
    for src in _SOURCES:
        assert src.is_file(), f"source missing: {src}"
    assert _RUCKIG_INCLUDE.is_dir(), f"ruckig include missing: {_RUCKIG_INCLUDE}"
    ruckig_srcs = sorted(_RUCKIG_SRC_DIR.glob("*.cpp"))
    assert ruckig_srcs, f"no vendored ruckig sources under {_RUCKIG_SRC_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "drivetrain_harness"

    compile_result = subprocess.run(
        [
            cxx,
            f"-std={_CXX_STANDARD}",
            *_CONSTRAINT_FLAGS,
            "-Wall",
            "-Wextra",
            "-I",
            str(_SOURCE_DIR),
            "-I",
            str(_RUCKIG_INCLUDE),
            "-o",
            str(binary),
            *[str(s) for s in _SOURCES],
            *[str(s) for s in ruckig_srcs],
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "drivetrain_harness.cpp / drivetrain.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "drivetrain_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
