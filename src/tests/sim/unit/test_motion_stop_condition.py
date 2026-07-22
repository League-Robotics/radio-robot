"""Off-hardware acceptance proof for ticket 116-002 (SUC-050/SUC-051/
SUC-052/SUC-054), ``Motion::StopCondition``
(``src/firm/motion/stop_condition.{h,cpp}``).

Compiles ``motion_stop_condition_harness.cpp`` together with
``src/firm/motion/stop_condition.cpp`` ONLY -- no ``TestSim::SimClock``,
no ``App::``/``Devices::`` fakes of any kind, since the module takes every
reading (``now``/``pathLength``/``theta``) as a plain parameter rather than
reading from a held collaborator (see ``stop_condition.h``'s file header).
This is itself part of what the compile step proves: zero dependency on
``App::MoveQueue``, ``App::Drive``, or any ``msg::*`` wire type. Mirrors
``test_app_deadman.py``'s shape: compile with the system C++ compiler, run
the resulting binary, assert it exits 0.

Collected under ``src/tests/sim/unit/`` -- already within
``pyproject.toml``'s ``testpaths = ["src/tests/sim"]``, no configuration
change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_motion_stop_condition.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "motion_stop_condition_harness.cpp"
_STOP_CONDITION_SRC = _SOURCE_DIR / "motion" / "stop_condition.cpp"

# Matches every other src/tests/sim/unit harness's own compiled standard --
# the project's actual compiled standard is -std=gnu++20.
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


def test_motion_stop_condition_harness_compiles_and_passes(tmp_path):
    """Compile Motion::StopCondition + the harness and assert every scenario passes."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _STOP_CONDITION_SRC.is_file(), f"stop_condition.cpp missing: {_STOP_CONDITION_SRC}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "motion_stop_condition_harness"

    compile_result = subprocess.run(
        [
            cxx,
            f"-std={_CXX_STANDARD}",
            "-Wall",
            "-Wextra",
            "-DHOST_BUILD",
            "-I",
            str(_SOURCE_DIR),
            "-o",
            str(binary),
            str(_HARNESS_SRC),
            str(_STOP_CONDITION_SRC),
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "motion_stop_condition_harness.cpp / stop_condition.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run_result.returncode == 0, (
        "motion_stop_condition_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
