"""Off-hardware acceptance proof for ticket 094-001 (SUC-001):
Motion::SegmentExecutor (source/motion/segment_executor.{h,cpp}) -- the
lift of Subsystems::Planner's non-GOTO internals (two Motion::JerkTrajectory
channels, encoder-only stop-condition evaluation, Motion::MotionBaseline
capture, the divergence replan and its compile-split dead-time, the
presolved graceful decel-to-zero) plus the one genuinely new piece of
control logic this sprint needs: a 3-phase PRE_PIVOT -> TRANSLATE ->
TERMINAL_PIVOT sequencer that turns one Motion::Segment (source/motion/
segment.h) into a chain of single-channel Ruckig solves.

Read-only from planner.cpp/planner.h -- this ticket does not modify
Subsystems::Planner itself (that happens in ticket 094-002). No
Subsystems::Drivetrain or blackboard dependency -- built and tested here in
isolation, exactly like Motion::JerkTrajectory/Subsystems::Planner already
are (test_jerk_trajectory.py / test_planner.py).

Mirrors test_planner.py's own shape: compiles
``segment_executor_harness.cpp`` together with ``source/motion/
segment_executor.cpp`` and its two real dependencies, ``source/motion/
jerk_trajectory.cpp`` and ``source/motion/stop_condition.cpp`` (all
dependency-free -- no MicroBit.h, no I2CBus, no Motion::VelocityRamp: this
executor has no ramp_-driven goal kind to carry forward, so unlike
planner.cpp it never links velocity_ramp.cpp), against the SAME
``source/motion/segment_executor.h``/``segment.h`` any future firmware
build will compile. Compiles with the system C++ compiler, runs the
resulting binary, asserts it exits 0.

Collected under ``tests/sim/unit/`` -- already within ``pyproject.toml``'s
``testpaths``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_segment_executor.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "segment_executor_harness.cpp"
_SEGMENT_EXECUTOR_SRC = _SOURCE_DIR / "motion" / "segment_executor.cpp"
_STOP_CONDITION_SRC = _SOURCE_DIR / "motion" / "stop_condition.cpp"
_JERK_TRAJECTORY_SRC = _SOURCE_DIR / "motion" / "jerk_trajectory.cpp"
_RUCKIG_INCLUDE = _REPO_ROOT / "libraries" / "ruckig" / "include"
_RUCKIG_SRC_DIR = _REPO_ROOT / "libraries" / "ruckig" / "src"

# gnu++20 (GNU extensions -- newlib exposes M_PI, which Ruckig's roots.hpp
# needs) plus -fno-exceptions/-fno-rtti, matching test_planner.py's/
# test_jerk_trajectory.py's own precedent -- this executor transitively
# compiles Ruckig too, so it must build under the SAME constraints the
# firmware itself imposes.
_CXX_STANDARD = "gnu++20"
# -DHOST_BUILD=1 marks this as a host build, exactly as the sim CMake does
# (tests/_infra/sim/CMakeLists.txt) -- without it segment_executor.cpp would
# resolve its plant-specific kOutputHops dead-time to the REAL-BRICK value
# (the flip-flop's measured ~80 ms actuation lag), over-compensating a
# near-zero-lag host plant. A host harness is a host build (matches
# test_planner.py's own sprint-093 fix).
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


def test_segment_executor_harness_compiles_and_passes(tmp_path):
    """Compile the SegmentExecutor harness and assert every scenario passes."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _SEGMENT_EXECUTOR_SRC.is_file(), f"segment_executor.cpp missing: {_SEGMENT_EXECUTOR_SRC}"
    assert _STOP_CONDITION_SRC.is_file(), f"stop_condition.cpp missing: {_STOP_CONDITION_SRC}"
    assert _JERK_TRAJECTORY_SRC.is_file(), f"jerk_trajectory.cpp missing: {_JERK_TRAJECTORY_SRC}"
    assert _RUCKIG_INCLUDE.is_dir(), f"ruckig include missing: {_RUCKIG_INCLUDE}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"
    ruckig_srcs = sorted(_RUCKIG_SRC_DIR.glob("*.cpp"))
    assert ruckig_srcs, f"no vendored ruckig sources under {_RUCKIG_SRC_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "segment_executor_harness"

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
            str(_HARNESS_SRC),
            str(_SEGMENT_EXECUTOR_SRC),
            str(_STOP_CONDITION_SRC),
            str(_JERK_TRAJECTORY_SRC),
            *[str(s) for s in ruckig_srcs],
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "segment_executor_harness.cpp / segment_executor.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "segment_executor_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
