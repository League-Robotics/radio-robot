"""First-pass acceptance for the vendored Ruckig library (libraries/ruckig).

Compiles ``ruckig_smoke_harness.cpp`` together with the vendored Ruckig sources
(``libraries/ruckig/src/*.cpp``) under the firmware's EXACT build constraints
(``-std=c++20 -fno-exceptions -fno-rtti``, compile-time DoF so no heap), runs the
binary, and asserts it exits 0. Proves the C++20-upgrade + Ruckig-vendoring path
is viable for the firmware: Ruckig compiles without exceptions/RTTI and produces
a decelerate-to-rest, never-reverse trajectory (the terminal reverse-spin fix).

Mirrors test_velocity_pid.py's compile-and-run pattern. See
clasi/issues/planner-motion-planning-via-vendored-ruckig.md.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_ruckig_smoke.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "ruckig_smoke_harness.cpp"
_RUCKIG_INCLUDE = _REPO_ROOT / "libraries" / "ruckig" / "include"
_RUCKIG_SRC_DIR = _REPO_ROOT / "libraries" / "ruckig" / "src"

# Match the firmware build EXACTLY: gnu++20 (the CMakeLists override -- C++20
# with GNU extensions, so newlib exposes M_PI, which Ruckig's roots.hpp needs),
# no exceptions, no RTTI. Ruckig<DOFs> defaults throw_error=false so validation
# returns Result codes rather than throwing -- required under -fno-exceptions.
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


def test_vendored_ruckig_compiles_and_produces_a_rest_terminating_trajectory(tmp_path):
    assert _HARNESS_SRC.is_file(), f"harness missing: {_HARNESS_SRC}"
    assert _RUCKIG_INCLUDE.is_dir(), f"ruckig include missing: {_RUCKIG_INCLUDE}"
    ruckig_srcs = sorted(_RUCKIG_SRC_DIR.glob("*.cpp"))
    assert ruckig_srcs, f"no vendored ruckig sources under {_RUCKIG_SRC_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "ruckig_smoke"

    compile_cmd = [
        cxx,
        f"-std={_CXX_STANDARD}",
        *_CONSTRAINT_FLAGS,
        "-O2",
        "-Wall",
        "-I", str(_RUCKIG_INCLUDE),
        "-o", str(binary),
        str(_HARNESS_SRC),
        *[str(s) for s in ruckig_srcs],
    ]
    compiled = subprocess.run(compile_cmd, capture_output=True, text=True)
    assert compiled.returncode == 0, (
        "vendored Ruckig failed to compile under -std=c++20 -fno-exceptions "
        f"-fno-rtti:\nstdout:\n{compiled.stdout}\nstderr:\n{compiled.stderr}"
    )

    run = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run.returncode == 0, (
        f"ruckig smoke harness reported a failure (exit {run.returncode}):\n"
        f"{run.stdout}\n{run.stderr}"
    )
    # Sanity: the harness prints its trajectory summary line.
    assert "no-reverse trajectory" in run.stdout, run.stdout


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
