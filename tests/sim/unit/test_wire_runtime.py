"""Off-hardware acceptance proof for ticket 095-004 (SUC-003).

Compiles ``wire_runtime_harness.cpp`` (exercising every WireRuntime
primitive in ``source/messages/wire_runtime.{h,cpp}``: varint, zigzag,
fixed32, length-delimited framing + its nesting-depth bound,
packed-repeated max_count clamping, unknown-field skip, and base64) with the
system C++ compiler, runs the resulting binary, and asserts it exits 0.
``wire_runtime.h`` is dependency-free beyond ``<cstddef>``/``<cstdint>`` --
no MicroBit.h, no CODAL, no ARM toolchain. Mirrors
``test_runtime_blackboard.py``'s shape exactly for the normal build.

A second test recompiles the SAME harness with
``-fsanitize=address,undefined`` and reruns every scenario (including the
malformed-input ones -- truncated varint, over-claiming length-delimited
field, invalid base64 padding, an exactly-sized packed-repeated output
array) under the sanitizers, proving the acceptance criteria's "never reads
past the buffer end" / "does not overflow a fixed-size output array"
requirements, not just that the functions return the right bool.

Collected under ``tests/sim/unit/`` -- already within ``pyproject.toml``'s
``testpaths = ["tests/sim"]``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_wire_runtime.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "wire_runtime_harness.cpp"
_WIRE_RUNTIME_SRC = _SOURCE_DIR / "messages" / "wire_runtime.cpp"

# wire_runtime.h documents its own target as the project's ACTUAL compiled
# standard (-std=gnu++20, per 095-003's finding) -- build the host harness
# to the same standard.
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


def _compile_and_run(tmp_path, binary_name: str, extra_flags: list[str]) -> None:
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _WIRE_RUNTIME_SRC.is_file(), f"wire_runtime.cpp missing: {_WIRE_RUNTIME_SRC}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / binary_name

    compile_result = subprocess.run(
        [
            cxx,
            f"-std={_CXX_STANDARD}",
            "-Wall",
            "-Wextra",
            *extra_flags,
            "-I",
            str(_SOURCE_DIR),
            "-o",
            str(binary),
            str(_HARNESS_SRC),
            str(_WIRE_RUNTIME_SRC),
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "wire_runtime_harness.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run_result.returncode == 0, (
        "wire_runtime_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


def test_wire_runtime_harness_compiles_and_passes(tmp_path):
    """Compile the WireRuntime harness (normal host flags) and assert every scenario passes."""
    _compile_and_run(tmp_path, "wire_runtime_harness", [])


def test_wire_runtime_harness_asan_ubsan_malformed_input(tmp_path):
    """Recompile under ASan/UBSan and rerun every scenario -- proves the malformed-input
    and packed-repeated-clamp acceptance criteria never read/write out of bounds, not just
    that they return the correct bool."""
    _compile_and_run(
        tmp_path,
        "wire_runtime_harness_asan",
        ["-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-g"],
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
