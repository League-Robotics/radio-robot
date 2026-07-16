"""Off-hardware acceptance proof for ticket 103-004 (SUC-004), App::Comms
(``src/firm/app/comms.{h,cpp}``).

Compiles ``app_comms_harness.cpp`` together with ``src/firm/app/comms.cpp``,
``src/firm/messages/wire.cpp``, and ``src/firm/messages/wire_runtime.cpp`` with
``-DHOST_BUILD`` so comms.cpp's ``SerialTransport``/``RadioTransport`` ARM
adapters (guarded ``#ifndef HOST_BUILD``) are compiled out entirely -- no
``MicroBit.h`` anywhere in this graph. Mirrors ``test_wire_codec.py``'s
exact shape: compile with the system C++ compiler, run the resulting
binary, assert it exits 0.

Collected under ``src/tests/sim/unit/`` -- already within ``pyproject.toml``'s
``testpaths = ["src/tests/sim"]``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_app_comms.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_TESTS_SIM_DIR = _REPO_ROOT / "src" / "tests" / "sim"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "app_comms_harness.cpp"
_COMMS_SRC = _SOURCE_DIR / "app" / "comms.cpp"
_WIRE_SRC = _SOURCE_DIR / "messages" / "wire.cpp"
_WIRE_RUNTIME_SRC = _SOURCE_DIR / "messages" / "wire_runtime.cpp"

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


def test_app_comms_harness_compiles_and_passes(tmp_path):
    """Compile App::Comms + the harness (HOST_BUILD) and assert every scenario passes."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _COMMS_SRC.is_file(), f"comms.cpp missing: {_COMMS_SRC}"
    assert _WIRE_SRC.is_file(), f"wire.cpp missing (run scripts/gen_messages.py?): {_WIRE_SRC}"
    assert _WIRE_RUNTIME_SRC.is_file(), f"wire_runtime.cpp missing: {_WIRE_RUNTIME_SRC}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "app_comms_harness"

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
            str(_COMMS_SRC),
            str(_WIRE_SRC),
            str(_WIRE_RUNTIME_SRC),
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "app_comms_harness.cpp / comms.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run_result.returncode == 0, (
        "app_comms_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
