"""src/tests/sim/unit/test_wire_golden_vector_harness.py -- sprint 124
ticket 004 (SUC-002, REQUIRED, non-negotiable): compiles and runs
``wire_golden_vector_harness.cpp`` against THE shared cross-language
golden-vector fixture, ``src/tests/fixtures/wire_golden_vectors.txt``.

Named ``..._harness.py`` (not the bare ``test_wire_golden_vectors.py`` the
harness's own filename would naively suggest, mirroring
``wire_runtime_harness.cpp``/``test_wire_runtime.py``'s convention)
DELIBERATELY: ``src/tests/unit/test_wire_golden_vectors.py`` (the host-side
pytest reader of the SAME fixture) already claims that basename, and
pytest's default (non-package) import mode raises an import-file-mismatch
error if two collected test modules share a basename in different
directories with no ``__init__.py`` -- the same reason ``test_host_wire_
codec.py`` (sprint 123) is not named ``test_wire_codec.py`` (already taken
by ``src/tests/sim/unit/test_wire_codec.py``).

Mirrors ``test_wire_runtime.py``'s exact compile-and-run shape: compile the
harness (+ ``wire_runtime.cpp``) with the system C++ compiler, run it with
the shared fixture's path as argv[1], assert exit 0. A second test
recompiles under ASan/UBSan and reruns, proving the harness's fixture
parsing and WireRuntime composition never read/write out of bounds.
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_wire_golden_vector_harness.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "wire_golden_vector_harness.cpp"
_WIRE_RUNTIME_SRC = _SOURCE_DIR / "messages" / "wire_runtime.cpp"
_FIXTURE_PATH = _REPO_ROOT / "src" / "tests" / "fixtures" / "wire_golden_vectors.txt"

# wire_runtime.h documents the project's actual compiled standard as
# -std=gnu++20 (095-003's finding) -- build this harness to the same
# standard, matching every other sim/unit/*_harness.cpp wrapper.
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


def _compile(tmp_path, binary_name: str, extra_flags: list[str]) -> pathlib.Path:
    assert _HARNESS_SRC.is_file(), f"harness source missing: {_HARNESS_SRC}"
    assert _WIRE_RUNTIME_SRC.is_file(), f"wire_runtime.cpp missing: {_WIRE_RUNTIME_SRC}"
    assert _FIXTURE_PATH.is_file(), f"shared golden-vector fixture missing: {_FIXTURE_PATH}"

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
        "wire_golden_vector_harness.cpp failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )
    return binary


def _compile_and_run(tmp_path, binary_name: str, extra_flags: list[str]) -> None:
    binary = _compile(tmp_path, binary_name, extra_flags)
    run_result = subprocess.run([str(binary), str(_FIXTURE_PATH)], capture_output=True, text=True)
    assert run_result.returncode == 0, (
        "wire_golden_vector_harness reported a mismatch against the shared fixture "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


def test_golden_vector_harness_compiles_and_passes(tmp_path):
    """Compile the golden-vector harness (normal host flags) and assert every
    fixture row's C++-computed wire bytes match the shared fixture exactly."""
    _compile_and_run(tmp_path, "wire_golden_vector_harness", [])


def test_golden_vector_harness_asan_ubsan(tmp_path):
    """Recompile under ASan/UBSan and rerun against the same fixture -- proves
    the fixture-parsing and WireRuntime-composition code never reads/writes
    out of bounds, not just that it returns the right bytes."""
    _compile_and_run(
        tmp_path,
        "wire_golden_vector_harness_asan",
        ["-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-g"],
    )


def test_golden_vector_harness_reports_the_expected_vector_count(tmp_path):
    """A lightweight guard against the harness silently loading zero rows
    (e.g. a path/format mismatch that happens to still exit 0) -- cross-
    checks the harness's own stdout count line against the host-side
    fixture parser's count, so a divergence in HOW MANY vectors either
    side sees is caught here rather than assumed."""
    binary = _compile(tmp_path, "wire_golden_vector_harness_count", [])
    run_result = subprocess.run([str(binary), str(_FIXTURE_PATH)], capture_output=True, text=True)
    assert run_result.returncode == 0, run_result.stdout + run_result.stderr

    # src/tests/unit/test_wire_golden_vectors.py owns the canonical Python
    # parser (load_golden_vectors()) -- import it directly rather than
    # re-implementing a second parser here, which would itself become a
    # second hand-maintained copy of fixture-parsing logic.
    unit_dir = _REPO_ROOT / "src" / "tests" / "unit"
    if str(unit_dir) not in sys.path:
        sys.path.insert(0, str(unit_dir))
    from test_wire_golden_vectors import load_golden_vectors as host_load_golden_vectors

    host_count = len(host_load_golden_vectors())
    assert f"{host_count} vector(s) loaded" in run_result.stdout, (
        f"C++ harness loaded a different vector count than the host parser ({host_count}):\n{run_result.stdout}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
