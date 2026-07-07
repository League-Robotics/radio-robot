"""Off-hardware acceptance proof for ticket 082-002 (SUC-002):
Subsystems::PoseEstimator (source/subsystems/pose_estimator.{h,cpp}) --
encoder-only dead-reckoning plus EkfTiny fusion, exercised with synthetic
msg::MotorState/msg::PoseEstimate observations (no real HAL).

Compiles ``pose_estimator_harness.cpp`` together with ``source/
subsystems/pose_estimator.cpp`` and ``source/estimation/ekf_tiny.cpp``
using the system C++ compiler, runs the resulting binary, and asserts it
exits 0. Mirrors ``test_ekf_tiny.py`` / ``test_tlm_frame.py``'s
compile-and-run pattern: no hardware, no CODAL, no CMake, with
``libraries/tinyekf/`` on the include path (``ekf_tiny.h``'s
``#include <tinyekf.h>`` is header-only).

Backfilled by ticket 082-005 -- pose_estimator_harness.cpp existed since
ticket 082-002 but had no pytest wrapper (unlike test_tlm_frame.py /
test_dev_loop_pose_estimator.py for tickets 003/004), so it was not
CI-collected. This file closes that gap, following the exact established
shape.

Collected under ``tests/sim/unit/`` alongside the existing harness
wrappers -- already within ``pyproject.toml``'s ``testpaths = ["tests/sim",
"tests/unit"]``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_pose_estimator.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_TINYEKF_DIR = _REPO_ROOT / "libraries" / "tinyekf"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "pose_estimator_harness.cpp"
_POSE_ESTIMATOR_SRC = _SOURCE_DIR / "subsystems" / "pose_estimator.cpp"
_EKF_TINY_SRC = _SOURCE_DIR / "estimation" / "ekf_tiny.cpp"

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


def test_pose_estimator_harness_compiles_and_passes(tmp_path):
    """Compile the PoseEstimator harness and assert every scenario passes."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _POSE_ESTIMATOR_SRC.is_file(), f"pose_estimator.cpp missing: {_POSE_ESTIMATOR_SRC}"
    assert _EKF_TINY_SRC.is_file(), f"ekf_tiny.cpp missing: {_EKF_TINY_SRC}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"
    assert _TINYEKF_DIR.is_dir(), f"libraries/tinyekf missing: {_TINYEKF_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "pose_estimator_harness"

    compile_result = subprocess.run(
        [
            cxx,
            f"-std={_CXX_STANDARD}",
            "-Wall",
            "-Wextra",
            "-I",
            str(_SOURCE_DIR),
            "-I",
            str(_TINYEKF_DIR),
            "-o",
            str(binary),
            str(_HARNESS_SRC),
            str(_POSE_ESTIMATOR_SRC),
            str(_EKF_TINY_SRC),
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "pose_estimator_harness.cpp (or one of its real sources) failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "pose_estimator_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
