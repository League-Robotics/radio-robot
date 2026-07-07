"""Off-hardware acceptance proof for ticket 086-005 (SUC-005/SUC-006):
LeverArm::sensorToCentre()/centreToSensor() (source/hal/lever_arm.h) -- the
OTOS lever-arm (mounting-offset) compensation math ported from source_old/
hal/capability/OtosLeverArm.h, preserving the same-instant-heading contract
that a past regression (commit db11b7c) violated.

Compiles ``lever_arm_harness.cpp`` against the SAME ``source/hal/
lever_arm.h`` every ARM build compiles (header-only -- nothing else to
link). Mirrors ``test_stop_condition.py``'s shape exactly: compile with the
system C++ compiler, run the resulting binary, assert it exits 0.

Collected under ``tests/sim/unit/`` -- already within ``pyproject.toml``'s
``testpaths``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_lever_arm.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "lever_arm_harness.cpp"
_LEVER_ARM_HEADER = _SOURCE_DIR / "hal" / "lever_arm.h"

# messages/common.h documents its own target as "CODAL C++11" -- build the
# host harness to the same standard so it exercises exactly the language
# subset the firmware itself uses.
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


def test_lever_arm_harness_compiles_and_passes(tmp_path):
    """Compile the LeverArm harness and assert every scenario passes."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _LEVER_ARM_HEADER.is_file(), f"lever_arm.h missing: {_LEVER_ARM_HEADER}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "lever_arm_harness"

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
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "lever_arm_harness.cpp / lever_arm.h failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "lever_arm_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
