"""Off-hardware acceptance proof for ticket 100-005 (SUC-005/SUC-006/SUC-007).

Compiles ``drive_policy_harness.cpp`` together with ``source/drive/
{policy,tracker,motion_plan,drivetrain,master_profile,arc_math}.cpp`` and the
vendored Ruckig sources (``libraries/ruckig/src/*.cpp``) using the system
C++ compiler under the firmware's EXACT build constraints (``gnu++20
-fno-exceptions -fno-rtti``, mirroring ``test_drive_tracker.py``/``test_
drive_plan.py``), runs the resulting binary, and asserts it exits 0. Mirrors
those files' compile-and-run pattern: no CMake, no ARM toolchain, no
hardware.

The harness exercises ``Drive::evaluate()`` directly (synthetic RefState/
TrackerOutput/StepInput values, not a solved MotionPlan) for exact,
deterministic control over every branch: the replan sustain/rate-limit/
N-max state machine, the terminal SETTLING dwell + literal-zero snap (plus a
dedicated no-reversal regression sweep), the flying-handoff envelope, the
pose-fix absorb/bypass rules (including suppression during an active
terminal dwell), the T_plan+1.5s timeout's never-silent resolution, and
purity/determinism. The full closed-loop ``MotionPlan::step()`` composition
against a real solved plan and a first-order plant stub is
``drive_step_harness.cpp``/``test_drive_step.py`` instead.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_drive_policy.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_DRIVE_DIR = _SOURCE_DIR / "drive"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "drive_policy_harness.cpp"
_DRIVE_SOURCES = [
    _DRIVE_DIR / "policy.cpp",
    _DRIVE_DIR / "tracker.cpp",
    _DRIVE_DIR / "motion_plan.cpp",
    _DRIVE_DIR / "drivetrain.cpp",
    _DRIVE_DIR / "master_profile.cpp",
    _DRIVE_DIR / "arc_math.cpp",
]
_RUCKIG_INCLUDE = _REPO_ROOT / "libraries" / "ruckig" / "include"
_RUCKIG_SRC_DIR = _REPO_ROOT / "libraries" / "ruckig" / "src"

# Match the firmware build EXACTLY (test_jerk_trajectory.py's own precedent).
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


def test_drive_policy_harness_compiles_and_passes(tmp_path):
    assert _HARNESS_SRC.is_file(), f"harness missing: {_HARNESS_SRC}"
    for src in _DRIVE_SOURCES:
        assert src.is_file(), f"source missing: {src}"
    assert _RUCKIG_INCLUDE.is_dir(), f"ruckig include missing: {_RUCKIG_INCLUDE}"
    ruckig_srcs = sorted(_RUCKIG_SRC_DIR.glob("*.cpp"))
    assert ruckig_srcs, f"no vendored ruckig sources under {_RUCKIG_SRC_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "drive_policy_harness"

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
        *[str(s) for s in _DRIVE_SOURCES],
        *[str(s) for s in ruckig_srcs],
    ]
    compiled = subprocess.run(compile_cmd, capture_output=True, text=True)
    assert compiled.returncode == 0, (
        "drive_policy_harness.cpp failed to compile under -std=gnu++20 "
        f"-fno-exceptions -fno-rtti:\nstdout:\n{compiled.stdout}\nstderr:\n{compiled.stderr}"
    )

    run = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run.returncode == 0, (
        f"drive_policy_harness reported a scenario failure (exit {run.returncode}):\n"
        f"{run.stdout}\n{run.stderr}"
    )
    assert "OK: all Drive:: policy scenarios passed" in run.stdout, run.stdout


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
