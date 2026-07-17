"""src/tests/sim/system/test_boundary_velocity.py -- sprint 109 ticket
006's own acceptance: cross-boundary carry (the "no decel between
same-vmax commands" headline requirement), sign-reversal/pivot-mismatch
forcing a full decel to rest, pivot->pivot rotational carry, and the
divergence-replan triggers staying safe under a plant disturbance.

Compiles ``boundary_velocity_harness.cpp`` together with
``motion/executor.cpp``/``jerk_trajectory.cpp`` and the vendored Ruckig
sources -- a pure ``Motion::Executor`` logic test (no App::Pilot/
RobotLoop/wire), mirroring ``src/tests/sim/unit/motion_executor_harness.cpp``'s
own precedent exactly, just placed under ``sim/system/`` to match this
ticket's own "Verification command":

    uv run python -m pytest src/tests/sim/system/ -k "boundary or divergence or handoff"
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/system/test_boundary_velocity.py -> system -> sim -> tests -> src -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "boundary_velocity_harness.cpp"
_EXECUTOR_SRC = _SOURCE_DIR / "motion" / "executor.cpp"
_JERK_TRAJECTORY_SRC = _SOURCE_DIR / "motion" / "jerk_trajectory.cpp"
_RUCKIG_INCLUDE = _REPO_ROOT / "vendor" / "ruckig" / "include"
_RUCKIG_SRC_DIR = _REPO_ROOT / "vendor" / "ruckig" / "src"

# Match the firmware build exactly (test_jerk_trajectory.py/test_motion_executor.py's
# own precedent).
_CXX_STANDARD = "gnu++20"
_CONSTRAINT_FLAGS = ["-fno-exceptions", "-fno-rtti"]


def _find_cxx_compiler() -> str:
    import shutil

    for candidate in ("c++", "clang++", "g++"):
        found = shutil.which(candidate)
        if found:
            return found
    pytest.skip("no system C++ compiler (c++/clang++/g++) found on PATH")
    raise AssertionError("unreachable")  # pragma: no cover


def test_boundary_velocity_harness_compiles_and_passes(tmp_path):
    assert _HARNESS_SRC.is_file(), f"harness missing: {_HARNESS_SRC}"
    assert _EXECUTOR_SRC.is_file(), f"executor.cpp missing: {_EXECUTOR_SRC}"
    assert _JERK_TRAJECTORY_SRC.is_file(), f"jerk_trajectory.cpp missing: {_JERK_TRAJECTORY_SRC}"
    assert _RUCKIG_INCLUDE.is_dir(), f"ruckig include missing: {_RUCKIG_INCLUDE}"
    ruckig_srcs = sorted(_RUCKIG_SRC_DIR.glob("*.cpp"))
    assert ruckig_srcs, f"no vendored ruckig sources under {_RUCKIG_SRC_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "boundary_velocity_harness"

    compile_cmd = [
        cxx,
        f"-std={_CXX_STANDARD}",
        *_CONSTRAINT_FLAGS,
        "-O2",
        "-Wall",
        "-I", str(_SOURCE_DIR),
        "-I", str(_RUCKIG_INCLUDE),
        "-o", str(binary),
        str(_HARNESS_SRC),
        str(_EXECUTOR_SRC),
        str(_JERK_TRAJECTORY_SRC),
        *[str(s) for s in ruckig_srcs],
    ]
    compiled = subprocess.run(compile_cmd, capture_output=True, text=True)
    assert compiled.returncode == 0, (
        "boundary_velocity_harness.cpp failed to compile under -std=gnu++20 "
        f"-fno-exceptions -fno-rtti:\nstdout:\n{compiled.stdout}\nstderr:\n{compiled.stderr}"
    )

    run = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run.returncode == 0, (
        f"boundary_velocity_harness reported a scenario failure (exit {run.returncode}):\n"
        f"{run.stdout}\n{run.stderr}"
    )
    assert "OK: all boundary-velocity/divergence-replan scenarios passed" in run.stdout, run.stdout


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
