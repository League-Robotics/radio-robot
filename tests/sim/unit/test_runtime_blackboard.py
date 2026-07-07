"""Off-hardware acceptance proof for ticket 087-002 (SUC-001/SUC-006).

Compiles ``runtime_blackboard_harness.cpp`` (default-constructing
``Rt::Blackboard`` from ``source/runtime/blackboard.h`` and round-tripping a
representative post/take on every command-plane queue/mailbox, including
``commandsIn``'s ``Subsystems::CommunicatorToCommandProcessorCommand``
payload from the newly extracted, CODAL-free ``source/subsystems/
wire_command.h``) with the system C++ compiler, runs the resulting binary, and
asserts it exits 0. Both ``blackboard.h`` and ``wire_command.h`` are
dependency-free beyond ``<cstdint>``/``messages/*.h``/``runtime/queue.h``/
``subsystems/hardware.h`` -- no MicroBit.h, no I2CBus, no CMake, no ARM
toolchain. Mirrors ``test_runtime_queue.py``'s shape exactly (see that
file's docstring for the pattern this follows).

Collected under ``tests/sim/unit/`` -- already within ``pyproject.toml``'s
``testpaths = ["tests/sim"]``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_runtime_blackboard.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "runtime_blackboard_harness.cpp"

# messages/common.h documents its own target as "CODAL C++11" -- build the
# host harness to the same standard so it exercises exactly the language
# subset the firmware itself uses.
_CXX_STANDARD = "c++11"


def _find_cxx_compiler() -> str:
    """Locate a usable system C++ compiler, preferring c++ then clang++/g++."""
    import shutil

    for candidate in ("c++", "clang++", "g++"):
        found = shutil.which(candidate)
        if found:
            return found
    pytest.skip("no system C++ compiler (c++/clang++/g++) found on PATH")
    raise AssertionError("unreachable")  # pragma: no cover


def test_runtime_blackboard_harness_compiles_and_passes(tmp_path):
    """Compile the Rt::Blackboard harness and assert every scenario passes."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "runtime_blackboard_harness"

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
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "runtime_blackboard_harness.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "runtime_blackboard_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
