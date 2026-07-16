"""Off-hardware acceptance proof for ticket DB-002 (device-bus-tickets.md).

Compiles ``measurement_ring_harness.cpp`` against ``src/firm/devices/
measurement_ring.h`` and ``src/firm/devices/interpolation.h`` -- both
header-only, no companion ``.cpp`` for either -- with ``-DHOST_BUILD`` for
consistency with every other src/tests/sim/unit harness, though neither header
actually forks on HOST_BUILD (both are plain host-clean C++: no bus, no
CODAL). Mirrors ``test_devices_types.py``'s shape exactly (that ticket's
headers are likewise header-only): compile with the system C++ compiler,
run the resulting binary, assert it exits 0.

Collected under ``src/tests/sim/unit/`` alongside the other harness wrappers --
already within ``pyproject.toml``'s ``testpaths = ["src/tests/sim"]``, no
configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_measurement_ring.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "measurement_ring_harness.cpp"
_MEASUREMENT_RING_HDR = _SOURCE_DIR / "devices" / "measurement_ring.h"
_INTERPOLATION_HDR = _SOURCE_DIR / "devices" / "interpolation.h"

# messages/common.h documents its own target as "CODAL C++11" -- build the
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


def test_measurement_ring_harness_compiles_and_passes(tmp_path):
    """Compile the MeasurementRing<T>/interpolation harness and assert every scenario passes."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _MEASUREMENT_RING_HDR.is_file(), f"measurement_ring.h missing: {_MEASUREMENT_RING_HDR}"
    assert _INTERPOLATION_HDR.is_file(), f"interpolation.h missing: {_INTERPOLATION_HDR}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "measurement_ring_harness"

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
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "measurement_ring_harness.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "measurement_ring_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )
    assert "OK" in run_result.stdout


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
