"""Off-hardware acceptance proof for ticket DB-003 (device-bus-tickets.md),
migrated by sprint 108 ticket 010 to the pure-interface split.

Compiles ``devices_clock_harness.cpp`` together with the TestSim::SimClock/
SimSleeper host-test fake implementation (``src/sim/
sim_clock.cpp``) against ``src/firm/devices/clock.h`` (now a pure interface,
every ARM build compiles the same header) with ``-DHOST_BUILD``, matching
every other src/tests/sim/unit harness's own compile shape — no MicroBit.h, no
CODAL, no wall clock, no real sleeps. Mirrors ``test_devices_i2c_bus.py``'s
shape exactly: compile with the system C++ compiler, run the resulting
binary, assert it exits 0.

Collected under ``src/tests/sim/unit/`` alongside the other harness wrappers —
already within ``pyproject.toml``'s ``testpaths = ["src/tests/sim"]``, no
configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_devices_clock.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_INFRA_SIM_DIR = _REPO_ROOT / "src" / "sim"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "devices_clock_harness.cpp"
_HOST_FAKE_SRC = _INFRA_SIM_DIR / "sim_clock.cpp"

# messages/common.h documents its own target as "CODAL C++11" — build the
# host harness to the same standard so it exercises exactly the language
# subset the firmware itself uses (matches every other src/tests/sim/unit
# harness's own _CXX_STANDARD).
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


def test_devices_clock_harness_compiles_and_passes(tmp_path):
    """Compile the Devices::Clock/Sleeper HOST_BUILD fake + harness and assert every scenario passes."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _HOST_FAKE_SRC.is_file(), f"HOST_BUILD fake missing: {_HOST_FAKE_SRC}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "devices_clock_harness"

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
            str(_HOST_FAKE_SRC),
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "devices_clock_harness.cpp / clock_host.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "devices_clock_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
