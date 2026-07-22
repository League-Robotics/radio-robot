"""Off-hardware acceptance proof for sprint 117 ticket 002 (SUC-057),
App::StateEstimator (``src/firm/app/state_estimator.{h,cpp}``).

Compiles ``app_state_estimator_harness.cpp`` together with
``src/firm/app/state_estimator.cpp`` alone -- unlike ``test_app_odometry.py``,
this module has NO ``Devices::`` leaf / SimPlant dependency to link (see
the harness's own file header: pure computation, no I2C bus, no owned
clock). Mirrors every other ``src/tests/sim/unit`` harness's shape: compile
with the system C++ compiler, run the resulting binary, assert it exits 0.

Collected under ``src/tests/sim/unit/`` -- already within
``pyproject.toml``'s ``testpaths``.
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_app_state_estimator.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "app_state_estimator_harness.cpp"
_STATE_ESTIMATOR_SRC = _SOURCE_DIR / "app" / "state_estimator.cpp"

# Matches every other src/tests/sim/unit harness's own compiled standard.
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


def test_app_state_estimator_harness_compiles_and_passes(tmp_path):
    """Compile App::StateEstimator + the harness (HOST_BUILD) and assert
    every scenario passes."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _STATE_ESTIMATOR_SRC.is_file(), f"state_estimator.cpp missing: {_STATE_ESTIMATOR_SRC}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "app_state_estimator_harness"

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
            str(_STATE_ESTIMATOR_SRC),
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "app_state_estimator_harness.cpp / state_estimator.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run_result.returncode == 0, (
        "app_state_estimator_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
