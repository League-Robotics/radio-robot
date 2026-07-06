"""Off-hardware acceptance proof for ticket 084-001 (SUC-001/SUC-002/SUC-003):
Motion::VelocityRamp (source/motion/velocity_ramp.{h,cpp}) -- the body-level
(v, omega) motion profiler ported from source_old/control/
BodyVelocityController.{h,cpp} minus its kinematics/saturate/motor-output
tail (architecture-update.md (084) Decision 3).

Compiles ``velocity_ramp_harness.cpp`` together with ``source/motion/
velocity_ramp.cpp`` (its one, dependency-free, real dependency) against the
SAME ``source/motion/velocity_ramp.h`` every ARM build compiles. Mirrors
``test_drivetrain.py``'s shape exactly: compile with the system C++
compiler, run the resulting binary, assert it exits 0.

Collected under ``tests/sim/unit/`` -- already within ``pyproject.toml``'s
``testpaths``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_velocity_ramp.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "velocity_ramp_harness.cpp"
_VELOCITY_RAMP_SRC = _SOURCE_DIR / "motion" / "velocity_ramp.cpp"

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


def test_velocity_ramp_harness_compiles_and_passes(tmp_path):
    """Compile the VelocityRamp harness and assert every scenario passes."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _VELOCITY_RAMP_SRC.is_file(), f"velocity_ramp.cpp missing: {_VELOCITY_RAMP_SRC}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "velocity_ramp_harness"

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
            str(_VELOCITY_RAMP_SRC),
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "velocity_ramp_harness.cpp / velocity_ramp.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "velocity_ramp_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
