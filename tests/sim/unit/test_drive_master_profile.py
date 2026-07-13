"""Off-hardware acceptance proof for ticket 100-002 (SUC-002/SUC-008).

Compiles ``drive_master_profile_harness.cpp`` together with
``source/drive/master_profile.cpp`` and the vendored Ruckig sources
(``libraries/ruckig/src/*.cpp``) using the system C++ compiler under the
firmware's EXACT build constraints (``gnu++20 -fno-exceptions -fno-rtti``,
mirroring ``test_jerk_trajectory.py``/``test_ruckig_smoke.py``), runs the
resulting binary, and asserts it exits 0. Mirrors those files'
compile-and-run pattern: no CMake, no ARM toolchain, no hardware.

A second, non-compiled test statically confirms ``Drive::MasterProfile``'s
public API has no measured-observation-shaped parameter anywhere (the
seeding contract carried forward from jerk_trajectory.h/test_jerk_
trajectory.py's own static pin) -- its ONLY way to acquire a nonzero
starting state is seedCurrent()/the class's own remembered last sample.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_drive_master_profile.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "drive_master_profile_harness.cpp"
_MASTER_PROFILE_HEADER = _SOURCE_DIR / "drive" / "master_profile.h"
_MASTER_PROFILE_SRC = _SOURCE_DIR / "drive" / "master_profile.cpp"
_RUCKIG_INCLUDE = _REPO_ROOT / "libraries" / "ruckig" / "include"
_RUCKIG_SRC_DIR = _REPO_ROOT / "libraries" / "ruckig" / "src"

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


def test_drive_master_profile_harness_compiles_and_passes(tmp_path):
    assert _HARNESS_SRC.is_file(), f"harness missing: {_HARNESS_SRC}"
    assert _MASTER_PROFILE_SRC.is_file(), f"master_profile.cpp missing: {_MASTER_PROFILE_SRC}"
    assert _RUCKIG_INCLUDE.is_dir(), f"ruckig include missing: {_RUCKIG_INCLUDE}"
    ruckig_srcs = sorted(_RUCKIG_SRC_DIR.glob("*.cpp"))
    assert ruckig_srcs, f"no vendored ruckig sources under {_RUCKIG_SRC_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "drive_master_profile_harness"

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
        str(_MASTER_PROFILE_SRC),
        *[str(s) for s in ruckig_srcs],
    ]
    compiled = subprocess.run(compile_cmd, capture_output=True, text=True)
    assert compiled.returncode == 0, (
        "drive_master_profile_harness.cpp failed to compile under -std=gnu++20 "
        f"-fno-exceptions -fno-rtti:\nstdout:\n{compiled.stdout}\nstderr:\n{compiled.stderr}"
    )

    run = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run.returncode == 0, (
        f"drive_master_profile_harness reported a scenario failure (exit {run.returncode}):\n"
        f"{run.stdout}\n{run.stderr}"
    )
    assert "OK: all Drive::MasterProfile scenarios passed" in run.stdout, run.stdout


def _strip_line_comments(text: str) -> str:
    """Drop everything from ``//`` to end-of-line on each line -- mirrors
    test_jerk_trajectory.py's own helper for the identical situation (a doc
    comment naming the excluded shape to document the exclusion is the
    point, not a violation of it)."""
    return "\n".join(line.split("//", 1)[0] for line in text.splitlines())


def test_never_reads_measured_observations():
    """Static pin: Drive::MasterProfile's CODE never names leftObs/rightObs
    (or a generically named ``measured``/``observed`` parameter) -- its
    current-state seeding is exclusively internal (its own last sample, via
    reset()/seedCurrent()/the previous solve or sample() call), never a
    measured-observation argument (master_profile.h's own seeding-contract
    doc comment, carried forward verbatim from jerk_trajectory.h's Decision
    8). The class's own doc comments discuss this exclusion by name, so
    this check scans CODE only (comments stripped), not raw file text.
    """
    header_code = _strip_line_comments(_MASTER_PROFILE_HEADER.read_text())
    source_code = _strip_line_comments(_MASTER_PROFILE_SRC.read_text())
    for forbidden in ("leftObs", "rightObs", "measured"):
        assert forbidden not in header_code, (
            f"{_MASTER_PROFILE_HEADER} uses {forbidden!r} in code -- "
            "Drive::MasterProfile must never read a measured observation"
        )
        assert forbidden not in source_code, (
            f"{_MASTER_PROFILE_SRC} uses {forbidden!r} in code -- "
            "Drive::MasterProfile must never read a measured observation"
        )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
