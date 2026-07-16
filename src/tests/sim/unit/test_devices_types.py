"""Off-hardware acceptance proof for ticket DB-001 (device-bus-tickets.md).

Compiles ``devices_types_harness.cpp`` under ``-DHOST_BUILD`` against
``src/firm/devices/device_types.h``/``src/firm/devices/device_config.h`` (header
-only -- no companion ``.cpp`` exists for either yet) and runs the resulting
binary, asserting it exits 0. Mirrors ``test_nezha_flipflop.py``/
``test_motor_policy.py``'s shape exactly (see those files' docstrings for the
pattern this follows): compile with the system C++ compiler, run the
resulting binary, assert success.

Every check the harness performs is a compile-time ``static_assert`` (every
Devices reading/config type is ``std::is_trivially_copyable`` and
``std::is_standard_layout``), so a failure here can show up as either a
nonzero *compile* return code (the static_assert fired) or a nonzero *run*
return code (defense in depth -- see the harness's own file header).
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_devices_types.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "devices_types_harness.cpp"
_DEVICE_TYPES_HDR = _SOURCE_DIR / "devices" / "device_types.h"
_DEVICE_CONFIG_HDR = _SOURCE_DIR / "devices" / "device_config.h"

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


def test_devices_types_harness_compiles_and_passes(tmp_path):
    """Compile the Devices value/config-type harness and assert it passes."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _DEVICE_TYPES_HDR.is_file(), f"device_types.h missing: {_DEVICE_TYPES_HDR}"
    assert _DEVICE_CONFIG_HDR.is_file(), f"device_config.h missing: {_DEVICE_CONFIG_HDR}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "devices_types_harness"

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
        "devices_types_harness.cpp failed to compile (a static_assert may "
        "have fired -- a Devices reading/config type is not trivially_"
        f"copyable/standard_layout):\nstdout:\n{compile_result.stdout}\n"
        f"stderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "devices_types_harness reported failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )
    assert "PASS" in run_result.stdout


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
