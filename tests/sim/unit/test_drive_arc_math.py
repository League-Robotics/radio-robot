"""Off-hardware acceptance proof for ticket 100-002 (SUC-002/SUC-008).

Compiles ``drive_arc_math_harness.cpp`` together with
``source/drive/arc_math.cpp`` using the system C++ compiler under the
firmware's EXACT build constraints (``gnu++20 -fno-exceptions -fno-rtti``,
mirroring ``test_jerk_trajectory.py``/``test_ruckig_smoke.py``), runs the
resulting binary, and asserts it exits 0. Mirrors those files'
compile-and-run pattern: no CMake, no ARM toolchain, no hardware, no Ruckig
dependency (arc_math is pure geometry, no vendored library involved).
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_drive_arc_math.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "drive_arc_math_harness.cpp"
_ARC_MATH_SRC = _SOURCE_DIR / "drive" / "arc_math.cpp"

# Match the firmware build EXACTLY (test_jerk_trajectory.py's own precedent).
_CXX_STANDARD = "gnu++20"
_CONSTRAINT_FLAGS = ["-fno-exceptions", "-fno-rtti"]


def _find_cxx_compiler() -> str:
    import shutil

    for candidate in ("c++", "clang++", "g++"):
        found = shutil.which(candidate)
        if found:
            return found
    pytest.skip("no system C++ compiler (c++/clang++/g++) found on PATH")
    raise AssertionError("unreachable")  # pragma: no cover


def test_drive_arc_math_harness_compiles_and_passes(tmp_path):
    assert _HARNESS_SRC.is_file(), f"harness missing: {_HARNESS_SRC}"
    assert _ARC_MATH_SRC.is_file(), f"arc_math.cpp missing: {_ARC_MATH_SRC}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "drive_arc_math_harness"

    compile_cmd = [
        cxx,
        f"-std={_CXX_STANDARD}",
        *_CONSTRAINT_FLAGS,
        "-O2",
        "-Wall",
        "-I", str(_SOURCE_DIR),
        "-o", str(binary),
        str(_HARNESS_SRC),
        str(_ARC_MATH_SRC),
    ]
    compiled = subprocess.run(compile_cmd, capture_output=True, text=True)
    assert compiled.returncode == 0, (
        "drive_arc_math_harness.cpp failed to compile under -std=gnu++20 "
        f"-fno-exceptions -fno-rtti:\nstdout:\n{compiled.stdout}\nstderr:\n{compiled.stderr}"
    )

    run = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run.returncode == 0, (
        f"drive_arc_math_harness reported a scenario failure (exit {run.returncode}):\n"
        f"{run.stdout}\n{run.stderr}"
    )
    assert "OK: all Drive:: arc_math scenarios passed" in run.stdout, run.stdout


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
