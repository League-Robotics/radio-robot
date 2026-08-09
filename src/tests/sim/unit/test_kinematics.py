"""Off-hardware acceptance proof for ``src/firm/kinematics/``.

Compiles ``kinematics_harness.cpp`` together with the three kinematics
translation units (``kinematics.cpp``, ``differential_kinematics.cpp``,
``mecanum_kinematics.cpp``) with ``-DHOST_BUILD``, against the SAME headers
every ARM build compiles. Mirrors ``test_devices_types.py`` /
``test_app_drive.py``'s exact shape: compile with the system C++ compiler,
run the resulting binary, assert it exits 0.

Why this exists: ``Kinematics::DifferentialKinematics`` carries the former
``BodyKinematics`` math, which every robot in this fleet drives through, and
``Kinematics::MecanumKinematics`` has no other coverage anywhere -- nothing
in the firmware constructs it yet.
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_kinematics.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_KINEMATICS_DIR = _SOURCE_DIR / "kinematics"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "kinematics_harness.cpp"

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


def test_kinematics_harness_compiles_and_passes(tmp_path):
    """Compile Kinematics::Model + both implementations + the harness; assert
    every scenario passes."""
    sources = [
        _HARNESS_SRC,
        _KINEMATICS_DIR / "kinematics.cpp",
        _KINEMATICS_DIR / "differential_kinematics.cpp",
        _KINEMATICS_DIR / "mecanum_kinematics.cpp",
    ]
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "kinematics_harness"

    compile_result = subprocess.run(
        [
            cxx,
            f"-std={_CXX_STANDARD}",
            "-Wall",
            "-Wextra",
            "-DHOST_BUILD",
            "-I",
            str(_SOURCE_DIR),
            *[str(s) for s in sources],
            "-o",
            str(binary),
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "kinematics_harness.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run_result.returncode == 0, (
        f"kinematics_harness reported failures:\n{run_result.stdout}\n"
        f"{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
