"""Off-hardware acceptance proof for sprint 124 ticket 007 (SUC-004),
``Types::RobotState`` (``src/firm/types/robot_state.h``).

Two independent checks, both grep-enforceable/compile-enforceable per the
ticket's own Testing section:

1. ``test_robot_state_header_has_no_forbidden_includes`` -- the literal
   grep the ticket's acceptance criteria specify: no ``messages/`` or
   ``msg::`` token anywhere in the header.
2. ``test_firm_types_robot_state_harness_compiles_and_passes`` -- compiles
   ``firm_types_robot_state_harness.cpp`` (which includes ONLY
   ``firm/types/robot_state.h``) with the system C++ compiler against a
   narrow ``-I <repo>/src`` include path -- no ``src/firm/messages``,
   ``src/firm/app``, or ``src/firm/config`` on the path at all, so any
   accidental dependency on those trees fails to resolve at compile time,
   not just at grep time. Then runs the resulting binary and asserts every
   scenario passes (trivially-copyable static_assert + golden-copy field
   round-trip, no information loss vs. the former
   ``Motion::StateEstimator::Input``'s 16 flat fields).

Mirrors test_motion_stop_condition.py's exact compile-and-run shape.
"""

import pathlib
import re
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_firm_types_robot_state.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_ROBOT_STATE_HEADER = _REPO_ROOT / "src" / "firm" / "types" / "robot_state.h"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "firm_types_robot_state_harness.cpp"

# Matches every other src/tests/sim/unit harness's own compiled standard --
# the project's actual compiled standard is -std=gnu++20.
_CXX_STANDARD = "c++20"

_FORBIDDEN_PATTERN = re.compile(r"messages/|msg::")


def _find_cxx_compiler() -> str:
    """Locate a usable system C++ compiler, preferring c++ then clang++/g++."""
    import shutil

    for candidate in ("c++", "clang++", "g++"):
        found = shutil.which(candidate)
        if found:
            return found
    pytest.skip("no system C++ compiler (c++/clang++/g++) found on PATH")
    raise AssertionError("unreachable")  # pragma: no cover


def test_robot_state_header_has_no_forbidden_includes():
    """grep -n "messages/\\|msg::" src/firm/types/robot_state.h returns nothing."""
    assert _ROBOT_STATE_HEADER.is_file(), f"robot_state.h missing: {_ROBOT_STATE_HEADER}"

    text = _ROBOT_STATE_HEADER.read_text()
    offending = [
        f"line {i}: {line}"
        for i, line in enumerate(text.splitlines(), start=1)
        if _FORBIDDEN_PATTERN.search(line)
    ]
    assert not offending, (
        "robot_state.h must stay dependency-free (no messages/ or msg:: token), found:\n"
        + "\n".join(offending)
    )


def test_firm_types_robot_state_harness_compiles_and_passes(tmp_path):
    """Compile robot_state.h's own harness with a narrow -I src path and run it."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _ROBOT_STATE_HEADER.is_file(), f"robot_state.h missing: {_ROBOT_STATE_HEADER}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "firm_types_robot_state_harness"

    compile_result = subprocess.run(
        [
            cxx,
            f"-std={_CXX_STANDARD}",
            "-Wall",
            "-Wextra",
            "-I",
            str(_REPO_ROOT / "src"),
            "-o",
            str(binary),
            str(_HARNESS_SRC),
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "firm_types_robot_state_harness.cpp failed to compile against a narrow "
        "-I <repo>/src path (robot_state.h must not reach outside src/firm/types/ "
        "for anything beyond <cstdint>):\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run_result.returncode == 0, (
        "firm_types_robot_state_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
