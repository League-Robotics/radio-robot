"""Off-hardware acceptance proof for ticket 103-005 (SUC-005), App::Telemetry
(``source/app/telemetry.{h,cpp}``).

Compiles ``app_telemetry_harness.cpp`` together with ``source/app/telemetry.cpp``,
``source/app/comms.cpp``, ``source/messages/wire.cpp``, and
``source/messages/wire_runtime.cpp`` with ``-DHOST_BUILD`` so
``comms.cpp``'s ``SerialTransport``/``RadioTransport`` ARM adapters (guarded
``#ifndef HOST_BUILD``) are compiled out entirely -- no ``MicroBit.h``
anywhere in this graph. Mirrors ``test_app_comms.py``'s exact shape:
compile with the system C++ compiler, run the resulting binary, assert it
exits 0.

Collected under ``tests/sim/unit/`` -- already within ``pyproject.toml``'s
``testpaths = ["tests/sim"]``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_app_telemetry.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_TESTS_SIM_DIR = _REPO_ROOT / "tests" / "sim"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "app_telemetry_harness.cpp"
_TELEMETRY_SRC = _SOURCE_DIR / "app" / "telemetry.cpp"
_COMMS_SRC = _SOURCE_DIR / "app" / "comms.cpp"
_WIRE_SRC = _SOURCE_DIR / "messages" / "wire.cpp"
_WIRE_RUNTIME_SRC = _SOURCE_DIR / "messages" / "wire_runtime.cpp"

# Matches every other tests/sim/unit harness's own compiled standard --
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


def test_app_telemetry_harness_compiles_and_passes(tmp_path):
    """Compile App::Telemetry + the harness (HOST_BUILD) and assert every scenario passes."""
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _TELEMETRY_SRC.is_file(), f"telemetry.cpp missing: {_TELEMETRY_SRC}"
    assert _COMMS_SRC.is_file(), f"comms.cpp missing: {_COMMS_SRC}"
    assert _WIRE_SRC.is_file(), f"wire.cpp missing (run scripts/gen_messages.py?): {_WIRE_SRC}"
    assert _WIRE_RUNTIME_SRC.is_file(), f"wire_runtime.cpp missing: {_WIRE_RUNTIME_SRC}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "app_telemetry_harness"

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
            str(_TELEMETRY_SRC),
            str(_COMMS_SRC),
            str(_WIRE_SRC),
            str(_WIRE_RUNTIME_SRC),
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "app_telemetry_harness.cpp / telemetry.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run_result.returncode == 0, (
        "app_telemetry_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )
    # Surfaced for the ticket's own "report measured cadence" acceptance
    # criterion -- visible in `pytest -s` / CI logs without needing to
    # re-run the binary by hand.
    print(run_result.stdout)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
