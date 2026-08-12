"""Off-hardware acceptance proof for sprint 136 ticket 005 (SUC-003),
``Types::versionTag()`` (``src/firm/types/version_tag.{h,cpp}``).

``versionTag()`` used to live in ``main.cpp`` and was ARM-only (it read the
CODAL-only boot path); it has never had a test. 136-005 relocated it to
``types/`` -- pure string logic, no CODAL dependency, and (the change that
actually makes it testable) taking the version string as an explicit
parameter rather than reading the generated/gitignored
``FIRMWARE_VERSION_STR`` macro directly.

Compiles ``types_version_tag_harness.cpp`` together with
``src/firm/types/version_tag.cpp`` with a narrow ``-I <repo>/src/firm``
include path -- no ``MicroBit.h`` anywhere in this graph. Mirrors
``test_firm_types_robot_state.py``'s exact compile-and-run shape.
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_types_version_tag.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "types_version_tag_harness.cpp"
_VERSION_TAG_CPP = _SOURCE_DIR / "types" / "version_tag.cpp"

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


def test_types_version_tag_harness_compiles_and_passes(tmp_path):
    """Compile version_tag.h/.cpp's own harness with a narrow -I src/firm
    path and run it -- normal major.date.build case plus the "?" fallback."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _VERSION_TAG_CPP.is_file(), f"version_tag.cpp missing: {_VERSION_TAG_CPP}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "types_version_tag_harness"

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
            str(_VERSION_TAG_CPP),
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "types_version_tag_harness.cpp failed to compile against a narrow "
        "-I <repo>/src/firm path:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run_result.returncode == 0, (
        "types_version_tag_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
