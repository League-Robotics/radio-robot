"""Off-hardware acceptance proof for ticket 103-004 (SUC-004), App::Deadman
(``source/app/deadman.{h,cpp}``).

Compiles ``app_deadman_harness.cpp`` together with ``source/app/deadman.cpp``
and the TestSim::SimClock host-test fake (``tests/_infra/sim/
sim_clock.cpp``) against ``source/devices/clock.h`` (a pure interface,
sprint 108 ticket 010) with ``-DHOST_BUILD`` -- no MicroBit.h, no CODAL, no
wall clock, no real sleeps. Mirrors ``test_devices_clock.py``'s shape
exactly: compile with the system C++ compiler, run the resulting binary,
assert it exits 0.

Collected under ``tests/sim/unit/`` -- already within ``pyproject.toml``'s
``testpaths = ["tests/sim"]``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_app_deadman.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_INFRA_SIM_DIR = _REPO_ROOT / "tests" / "_infra" / "sim"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "app_deadman_harness.cpp"
_DEADMAN_SRC = _SOURCE_DIR / "app" / "deadman.cpp"
_CLOCK_HOST_SRC = _INFRA_SIM_DIR / "sim_clock.cpp"

# Matches every other tests/sim/unit harness's own compiled standard --
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


def test_app_deadman_harness_compiles_and_passes(tmp_path):
    """Compile App::Deadman + the HOST_BUILD Clock fake + the harness and assert every scenario passes."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _DEADMAN_SRC.is_file(), f"deadman.cpp missing: {_DEADMAN_SRC}"
    assert _CLOCK_HOST_SRC.is_file(), f"HOST_BUILD Clock fake missing: {_CLOCK_HOST_SRC}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "app_deadman_harness"

    compile_result = subprocess.run(
        [
            cxx,
            f"-std={_CXX_STANDARD}",
            "-Wall",
            "-Wextra",
            "-DHOST_BUILD",
            "-I",
            str(_SOURCE_DIR),
            "-I",
            str(_INFRA_SIM_DIR),
            "-o",
            str(binary),
            str(_HARNESS_SRC),
            str(_DEADMAN_SRC),
            str(_CLOCK_HOST_SRC),
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "app_deadman_harness.cpp / deadman.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run_result.returncode == 0, (
        "app_deadman_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
