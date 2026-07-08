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

Ticket 087-003 note: Planner's own ``tick()`` signature and output edge are
unchanged in shape by sprint 087's blackboard wiring (see
``planner_harness.cpp``'s own updated header comment) -- no compile-command
or scenario change was needed here.

Ticket 089-003 update: ``planner.h``/``planner.cpp`` now ``#include
"motion/jerk_trajectory.h"`` (DISTANCE's new linear channel -- see
``planner_harness.cpp``'s own updated header comment), which in turn
``#include``s the vendored Ruckig headers -- this compile command gained
``source/motion/jerk_trajectory.cpp``, the vendored ``libraries/ruckig/src/
*.cpp`` sources, and the ``libraries/ruckig/include`` path (mirroring
``test_jerk_trajectory.py``'s own compile command exactly), plus the
``gnu++20``/``-fno-exceptions``/``-fno-rtti`` flags Ruckig's own build
requires (``roots.hpp``'s ``M_PI`` usage under GNU extensions -- see
``test_jerk_trajectory.py``'s own comment).

Ticket 089-005 update: TURN/ROTATION migrate onto the SAME rotational
Motion::JerkTrajectory channel (Decision 9) -- the last goal-kind migration
this sprint, so ``Planner::tick()``'s dispatch collapses to the clean
``mode_ == GO_TO`` binary the architecture doc describes as the sprint's end
state, and ``applyStopAnticipation()`` is deleted in full. No compile-command
change here (still the same source list) -- see ``planner_harness.cpp``'s
own updated header comment for the new/rewritten TURN/ROTATION scenarios.

Ticket 089-006 update: the CONSOLIDATION pass -- no compile-command change
(same source list) -- see ``planner_harness.cpp``'s own updated header
comment for the two genuinely new scenarios this ticket adds (a VELOCITY/
bare-R cruise+decel spot-check, and the guard-1/stop-not-fired proof for
DISTANCE and TURN) on top of what tickets 003-005 already built. This
file's own compiled-and-passes assertion is also this ticket's SUC-004
verification point: ``test_jerk_trajectory.py``'s
``scenarioJerkSentinelMapsToInfinity`` (ticket 002) is the actual j_max/
yaw_jerk_max sentinel-mapping coverage AC4 asks to confirm -- re-run, not
re-implemented, by the full-suite pass this ticket records.
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
_JERK_TRAJECTORY_SRC = _SOURCE_DIR / "motion" / "jerk_trajectory.cpp"
_RUCKIG_INCLUDE = _REPO_ROOT / "libraries" / "ruckig" / "include"
_RUCKIG_SRC_DIR = _REPO_ROOT / "libraries" / "ruckig" / "src"

# 089-003: gnu++20 (GNU extensions -- newlib exposes M_PI, which Ruckig's
# roots.hpp needs) plus -fno-exceptions/-fno-rtti, matching
# test_jerk_trajectory.py's/test_ruckig_smoke.py's own precedent -- Planner
# now transitively compiles Ruckig, so it must build under the SAME
# constraints the firmware itself imposes.
_CXX_STANDARD = "gnu++20"
_CONSTRAINT_FLAGS = ["-fno-exceptions", "-fno-rtti"]


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
    assert _JERK_TRAJECTORY_SRC.is_file(), f"jerk_trajectory.cpp missing: {_JERK_TRAJECTORY_SRC}"
    assert _RUCKIG_INCLUDE.is_dir(), f"ruckig include missing: {_RUCKIG_INCLUDE}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"
    ruckig_srcs = sorted(_RUCKIG_SRC_DIR.glob("*.cpp"))
    assert ruckig_srcs, f"no vendored ruckig sources under {_RUCKIG_SRC_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "planner_harness"

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
            str(_PLANNER_SRC),
            str(_VELOCITY_RAMP_SRC),
            str(_STOP_CONDITION_SRC),
            str(_JERK_TRAJECTORY_SRC),
            *[str(s) for s in ruckig_srcs],
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
