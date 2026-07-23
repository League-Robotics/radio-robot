"""Off-hardware acceptance proof for ``Motion::VelocityShaper``
(``src/firm/motion/velocity_shaper.{h,cpp}``), decel-into-the-goal
campaign (follow-on to
``clasi/issues/angle-stop-overshoot-61-73-percent-on-hardware.md``).

Compiles ``motion_velocity_shaper_harness.cpp`` together with
``src/firm/motion/velocity_shaper.cpp`` ONLY -- no ``TestSim::SimClock``,
no ``App::``/``Devices::`` fakes of any kind, mirroring
``test_motion_stop_condition.py``'s own shape exactly (same compiler
discovery, same ``-std=gnu++20``/``-DHOST_BUILD`` flags, same
compile-then-run-then-assert-exit-0 pattern).

Collected under ``src/tests/sim/unit/`` -- already within
``pyproject.toml``'s ``testpaths = ["src/tests/sim"]``, no configuration
change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_motion_velocity_shaper.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "motion_velocity_shaper_harness.cpp"
_VELOCITY_SHAPER_SRC = _SOURCE_DIR / "motion" / "velocity_shaper.cpp"

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


def test_motion_velocity_shaper_harness_compiles_and_passes(tmp_path):
    """Compile Motion::VelocityShaper + the harness and assert every scenario passes."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _VELOCITY_SHAPER_SRC.is_file(), f"velocity_shaper.cpp missing: {_VELOCITY_SHAPER_SRC}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "motion_velocity_shaper_harness"

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
            str(_VELOCITY_SHAPER_SRC),
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "motion_velocity_shaper_harness.cpp / velocity_shaper.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run_result.returncode == 0, (
        "motion_velocity_shaper_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
