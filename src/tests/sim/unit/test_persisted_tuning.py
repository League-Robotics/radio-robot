"""Off-hardware acceptance proof for ticket 114-004 (SUC-003),
Config::PersistedTuning's PURE logic (``src/firm/config/
persisted_tuning.{h,cpp}``).

Compiles ``persisted_tuning_harness.cpp`` together with the real
``src/firm/config/persisted_tuning.cpp`` against the same
``src/firm/messages/config.h`` every ARM build compiles, with
``-DHOST_BUILD`` -- under that define, persisted_tuning.cpp's own ARM-only
``Config::MicroBitTuningStore`` adapter (guarded ``#ifndef HOST_BUILD``)
compiles out entirely; this test exercises ONLY serializeSnapshot()/
deserializeSnapshot()/shouldWipe(), which have zero MicroBitStorage/
hardware dependency (persisted_tuning.h's own file header). Mirrors
``test_measurement_ring.py``'s exact shape: compile with the system C++
compiler, run the resulting binary, assert it exits 0.

The real MicroBitStorage flash round-trip is explicitly NOT exercised by
this test or any other agent-run test in this tree -- see ticket 004's own
Testing section and ticket 006's stakeholder bench checklist.

Collected under ``src/tests/sim/unit/`` -- already within
``pyproject.toml``'s ``testpaths = ["src/tests/sim"]``, no configuration
change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_persisted_tuning.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "persisted_tuning_harness.cpp"
_PERSISTED_TUNING_SRC = _SOURCE_DIR / "config" / "persisted_tuning.cpp"

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


def test_persisted_tuning_harness_compiles_and_passes(tmp_path):
    """Compile Config::PersistedTuning's pure logic + the harness; assert every scenario passes."""
    sources = [_HARNESS_SRC, _PERSISTED_TUNING_SRC]
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "persisted_tuning_harness"

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
            *[str(src) for src in sources],
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "persisted_tuning_harness.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "persisted_tuning_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )
    assert "OK" in run_result.stdout


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
