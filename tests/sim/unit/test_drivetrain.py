"""Off-hardware acceptance proof for ticket 079-003 (SUC-004/SUC-005/SUC-006).

Compiles ``drivetrain_harness.cpp`` together with ``source/subsystems/
drivetrain.cpp`` and its one real dependency, ``source/kinematics/
body_kinematics.cpp`` (both dependency-free -- no MicroBit.h, no I2CBus),
against the SAME ``source/subsystems/drivetrain.h`` every ARM build compiles.
Mirrors ``test_motor_policy.py``'s shape exactly (see that file's docstring
for the pattern this follows): compile with the system C++ compiler, run the
resulting binary, assert it exits 0.

Collected under ``tests/sim/unit/`` alongside ``test_motor_policy.py``,
``test_i2c_bus_clearance.py``, and ``test_placeholder.py`` -- already within
``pyproject.toml``'s ``testpaths = ["tests/sim"]``, no configuration change
needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_drivetrain.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "drivetrain_harness.cpp"
_DRIVETRAIN_SRC = _SOURCE_DIR / "subsystems" / "drivetrain.cpp"
_BODY_KINEMATICS_SRC = _SOURCE_DIR / "kinematics" / "body_kinematics.cpp"

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


def test_drivetrain_harness_compiles_and_passes(tmp_path):
    """Compile the Drivetrain reshape harness and assert every scenario passes."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _DRIVETRAIN_SRC.is_file(), f"drivetrain.cpp missing: {_DRIVETRAIN_SRC}"
    assert _BODY_KINEMATICS_SRC.is_file(), (
        f"body_kinematics.cpp missing: {_BODY_KINEMATICS_SRC}"
    )
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "drivetrain_harness"

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
            str(_DRIVETRAIN_SRC),
            str(_BODY_KINEMATICS_SRC),
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
