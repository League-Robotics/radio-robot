"""Off-hardware acceptance proof for ticket 105-002 (SUC-019),
``TestSupport::FakeTransport`` (``src/tests/sim/support/fake_transport.h``).

Compiles ``fake_transport_harness.cpp`` with ``-DHOST_BUILD``.
``fake_transport.h`` only pulls in ``src/firm/app/comms.h`` for the abstract
``App::Transport`` base class (a pure interface -- no ``.cpp`` to link), so
this is the smallest possible compile unit: no ``comms.cpp``, no
``wire.cpp``/``wire_runtime.cpp``, no ``MicroBit.h`` anywhere in this graph.
Mirrors ``test_app_comms.py``'s exact shape: compile with the system C++
compiler, run the resulting binary, assert it exits 0.

Collected under ``src/tests/sim/unit/`` -- already within ``pyproject.toml``'s
``testpaths = ["src/tests/sim"]``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_fake_transport.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_TESTS_SIM_DIR = _REPO_ROOT / "src" / "tests" / "sim"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "fake_transport_harness.cpp"
_FAKE_TRANSPORT_HDR = _TESTS_SIM_DIR / "support" / "fake_transport.h"

# Matches every other src/tests/sim/unit harness's own compiled standard --
# the project's actual compiled standard is -std=gnu++20 (095-003's
# finding; see wire_runtime.h's own file header).
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


def test_fake_transport_harness_compiles_and_passes(tmp_path):
    """Compile FakeTransport + the harness (HOST_BUILD) and assert every scenario passes."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _FAKE_TRANSPORT_HDR.is_file(), f"fake_transport.h missing: {_FAKE_TRANSPORT_HDR}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "fake_transport_harness"

    compile_result = subprocess.run(
        [
            cxx,
            f"-std={_CXX_STANDARD}",
            "-Wall",
            "-Wextra",
            "-DHOST_BUILD",
            "-I",
            str(_SOURCE_DIR),
            "-I",
            str(_TESTS_SIM_DIR),
            "-o",
            str(binary),
            str(_HARNESS_SRC),
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "fake_transport_harness.cpp / fake_transport.h failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run_result.returncode == 0, (
        "fake_transport_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
