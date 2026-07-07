"""Off-hardware acceptance proof for ticket 084-001 (SUC-001/SUC-002/SUC-003):
Motion::evaluateStopCondition (source/motion/stop_condition.{h,cpp}) -- the
pure stop-condition predicate ported from source_old/control/
StopCondition.cpp, scoped to the five kinds architecture-update.md (084)
Decision 4 keeps this sprint (STOP_TIME/STOP_DISTANCE/STOP_HEADING/
STOP_POSITION/STOP_ROTATION); STOP_SENSOR/STOP_COLOR/STOP_LINE_ANY report a
distinct UNSUPPORTED result rather than silently never firing.

Compiles ``stop_condition_harness.cpp`` together with ``source/motion/
stop_condition.cpp`` against the SAME ``source/motion/stop_condition.h``
every ARM build compiles. Mirrors ``test_drivetrain.py``'s shape exactly:
compile with the system C++ compiler, run the resulting binary, assert it
exits 0.

Collected under ``tests/sim/unit/`` -- already within ``pyproject.toml``'s
``testpaths``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_stop_condition.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "stop_condition_harness.cpp"
_STOP_CONDITION_SRC = _SOURCE_DIR / "motion" / "stop_condition.cpp"

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


def test_stop_condition_harness_compiles_and_passes(tmp_path):
    """Compile the evaluateStopCondition harness and assert every scenario passes."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _STOP_CONDITION_SRC.is_file(), f"stop_condition.cpp missing: {_STOP_CONDITION_SRC}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "stop_condition_harness"

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
            str(_STOP_CONDITION_SRC),
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "stop_condition_harness.cpp / stop_condition.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "stop_condition_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
