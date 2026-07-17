"""Off-hardware acceptance proof for sprint 109 ticket 001 (restoring
``Motion::JerkTrajectory`` -- originally ticket 089-002, SUC-002/SUC-003/
SUC-004/SUC-005).

Compiles ``jerk_trajectory_harness.cpp`` together with
``src/firm/motion/jerk_trajectory.cpp`` and the vendored Ruckig sources
(``src/vendor/ruckig/src/*.cpp``) using the system C++ compiler under the
firmware's EXACT build constraints (``gnu++20 -fno-exceptions -fno-rtti``,
mirroring ``test_ruckig_smoke.py``'s original precedent), runs the
resulting binary, and asserts it exits 0. No CMake, no ARM toolchain, no
hardware.

A second, non-compiled test statically confirms
``Motion::JerkTrajectory`` never references a measured-observation
identifier (``leftObs``/``rightObs``) -- the class's current-state seeding
is exclusively internal (its own last sample) or an explicit caller-
provided seed.
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_jerk_trajectory.py -> unit -> sim -> tests -> src -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "jerk_trajectory_harness.cpp"
_JERK_TRAJECTORY_HEADER = _SOURCE_DIR / "motion" / "jerk_trajectory.h"
_JERK_TRAJECTORY_SRC = _SOURCE_DIR / "motion" / "jerk_trajectory.cpp"
_RUCKIG_INCLUDE = _REPO_ROOT / "src" / "vendor" / "ruckig" / "include"
_RUCKIG_SRC_DIR = _REPO_ROOT / "src" / "vendor" / "ruckig" / "src"

# Match the firmware build EXACTLY (test_ruckig_smoke.py's own precedent):
# gnu++20 (GNU extensions -- newlib exposes M_PI, which Ruckig's roots.hpp
# needs), no exceptions, no RTTI.
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


def test_jerk_trajectory_harness_compiles_and_passes(tmp_path):
    assert _HARNESS_SRC.is_file(), f"harness missing: {_HARNESS_SRC}"
    assert _JERK_TRAJECTORY_SRC.is_file(), f"jerk_trajectory.cpp missing: {_JERK_TRAJECTORY_SRC}"
    assert _RUCKIG_INCLUDE.is_dir(), f"ruckig include missing: {_RUCKIG_INCLUDE}"
    ruckig_srcs = sorted(_RUCKIG_SRC_DIR.glob("*.cpp"))
    assert ruckig_srcs, f"no vendored ruckig sources under {_RUCKIG_SRC_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "jerk_trajectory_harness"

    compile_cmd = [
        cxx,
        f"-std={_CXX_STANDARD}",
        *_CONSTRAINT_FLAGS,
        "-O2",
        "-Wall",
        "-I", str(_SOURCE_DIR),
        "-I", str(_RUCKIG_INCLUDE),
        "-o", str(binary),
        str(_HARNESS_SRC),
        str(_JERK_TRAJECTORY_SRC),
        *[str(s) for s in ruckig_srcs],
    ]
    compiled = subprocess.run(compile_cmd, capture_output=True, text=True)
    assert compiled.returncode == 0, (
        "jerk_trajectory_harness.cpp failed to compile under -std=gnu++20 "
        f"-fno-exceptions -fno-rtti:\nstdout:\n{compiled.stdout}\nstderr:\n{compiled.stderr}"
    )

    run = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run.returncode == 0, (
        f"jerk_trajectory_harness reported a scenario failure (exit {run.returncode}):\n"
        f"{run.stdout}\n{run.stderr}"
    )
    assert "OK: all Motion::JerkTrajectory scenarios passed" in run.stdout, run.stdout


def _strip_line_comments(text: str) -> str:
    """Drop everything from ``//`` to end-of-line on each line.

    Both source files under test use ``//`` line comments exclusively (no
    ``/* */`` block comments span multiple lines) -- this is enough to tell
    "used as code" apart from "named in a doc comment explaining the
    boundary this class deliberately does NOT cross" (jerk_trajectory.h's
    own class comment names leftObs/rightObs exactly to document that
    exclusion, which is the point, not a violation of it).
    """
    return "\n".join(line.split("//", 1)[0] for line in text.splitlines())


def test_never_reads_measured_observations():
    """Static pin: Motion::JerkTrajectory's CODE never names leftObs/rightObs.

    Its current-state seeding is exclusively internal (its own last
    sample, via reset()/seedCurrent()/the previous solve or sample() call)
    or an explicit caller-provided seed (retarget()/reanchor()'s own
    arguments) -- never a measured-observation argument
    (architecture-update.md (089) Decision 8). The class's own doc comment
    names leftObs/rightObs to document this exclusion explicitly, so this
    check scans CODE only (comments stripped), not raw file text.
    """
    header_code = _strip_line_comments(_JERK_TRAJECTORY_HEADER.read_text())
    source_code = _strip_line_comments(_JERK_TRAJECTORY_SRC.read_text())
    for forbidden in ("leftObs", "rightObs"):
        assert forbidden not in header_code, (
            f"{_JERK_TRAJECTORY_HEADER} uses {forbidden!r} in code -- "
            "Motion::JerkTrajectory must never read a measured observation"
        )
        assert forbidden not in source_code, (
            f"{_JERK_TRAJECTORY_SRC} uses {forbidden!r} in code -- "
            "Motion::JerkTrajectory must never read a measured observation"
        )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
