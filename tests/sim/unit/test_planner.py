"""Off-hardware acceptance proof for ticket 084-001 (SUC-001/SUC-002/SUC-003):
Subsystems::Planner (source/subsystems/planner.{h,cpp}) -- the goal-closure
engine ported (concept) from source_old/superstructure/Planner.{h,cpp} +
source_old/commands/MotionCommand.{h,cpp} onto the already-generated
msg::PlannerCommand/PlannerState/PlannerConfig/StopCondition types. This
ticket lands no wire verb -- Planner is built and tested here in isolation.

``planner_harness.cpp`` also carries ticket 084-005's own `state().mode`
coverage (Decision 6's `velocityShapedMode()` fold: an unbounded VELOCITY
goal reports `STREAMING`, a bounded one -- or TURN/RT, which always carry
their own built-in stop -- reports `TIMED`), since it is the natural
isolated-Planner-level home for that assertion; the wire-facing `mode=`
character itself is covered end to end in ``tests/sim/unit/
test_mode_machine.py``.

Compiles ``planner_harness.cpp`` together with ``source/subsystems/
planner.cpp`` and its two real dependencies, ``source/motion/
velocity_ramp.cpp`` and ``source/motion/stop_condition.cpp`` (all
dependency-free -- no MicroBit.h, no I2CBus), against the SAME
``source/subsystems/planner.h`` every ARM build compiles. Mirrors
``test_drivetrain.py``'s shape exactly: compile with the system C++
compiler, run the resulting binary, assert it exits 0.

Collected under ``tests/sim/unit/`` -- already within ``pyproject.toml``'s
``testpaths``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_planner.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "planner_harness.cpp"
_PLANNER_SRC = _SOURCE_DIR / "subsystems" / "planner.cpp"
_VELOCITY_RAMP_SRC = _SOURCE_DIR / "motion" / "velocity_ramp.cpp"
_STOP_CONDITION_SRC = _SOURCE_DIR / "motion" / "stop_condition.cpp"

# messages/common.h documents its own target as "CODAL C++11" -- build the
# host harness to the same standard so it exercises exactly the language
# subset the firmware itself uses.
_CXX_STANDARD = "c++11"


def _find_cxx_compiler() -> str:
    """Locate a usable system C++ compiler, preferring c++ then clang++/g++."""
    import shutil

    for candidate in ("c++", "clang++", "g++"):
        found = shutil.which(candidate)
        if found:
            return found
    pytest.skip("no system C++ compiler (c++/clang++/g++) found on PATH")
    raise AssertionError("unreachable")  # pragma: no cover


def test_planner_harness_compiles_and_passes(tmp_path):
    """Compile the Planner harness and assert every scenario passes."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _PLANNER_SRC.is_file(), f"planner.cpp missing: {_PLANNER_SRC}"
    assert _VELOCITY_RAMP_SRC.is_file(), f"velocity_ramp.cpp missing: {_VELOCITY_RAMP_SRC}"
    assert _STOP_CONDITION_SRC.is_file(), f"stop_condition.cpp missing: {_STOP_CONDITION_SRC}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "planner_harness"

    compile_result = subprocess.run(
        [
            cxx,
            f"-std={_CXX_STANDARD}",
            "-Wall",
            "-Wextra",
            "-I",
            str(_SOURCE_DIR),
            "-o",
            str(binary),
            str(_HARNESS_SRC),
            str(_PLANNER_SRC),
            str(_VELOCITY_RAMP_SRC),
            str(_STOP_CONDITION_SRC),
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "planner_harness.cpp / planner.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "planner_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
