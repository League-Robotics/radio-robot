"""Off-hardware acceptance proof for App::FakeOtos
(``src/firm/app/fake_otos.{h,cpp}``) -- the bench implementation of the
Devices::Otos interface introduced by the otos-fake-seam refactor.

Compiles ``app_fake_otos_harness.cpp`` together with the HOST_BUILD
implementations it needs (``src/firm/app/fake_otos.cpp``,
``src/firm/app/odometry.cpp``,
``src/motion/body_kinematics.cpp``) with ``-DHOST_BUILD``, against
the SAME headers every ARM build compiles. Mirrors ``test_app_odometry.py``'s
shape: compile with the system C++ compiler, run the binary, assert exit 0.

Collected under ``src/tests/sim/unit/`` -- already within ``pyproject.toml``'s
``testpaths``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_app_fake_otos.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "app_fake_otos_harness.cpp"
_FAKE_OTOS_SRC = _SOURCE_DIR / "app" / "fake_otos.cpp"
_ODOMETRY_SRC = _REPO_ROOT / "src" / "motion" / "odometry.cpp"
_BODY_KINEMATICS_SRC = _REPO_ROOT / "src" / "motion" / "body_kinematics.cpp"
# otos.cpp carries the abstract Devices::Otos base's out-of-line anchor
# (~Otos / vtable / typeinfo) that any concrete subclass -- FakeOtos -- links.
_OTOS_SRC = _SOURCE_DIR / "devices" / "otos.cpp"

# Matches every other src/tests/sim/unit harness's own compiled standard.
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


def test_app_fake_otos_harness_compiles_and_passes(tmp_path):
    """Compile App::FakeOtos + App::Odometry + BodyKinematics + the harness;
    assert every scenario passes."""
    sources = [
        _HARNESS_SRC,
        _FAKE_OTOS_SRC,
        _ODOMETRY_SRC,
        _BODY_KINEMATICS_SRC,
        _OTOS_SRC,
    ]
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "app_fake_otos_harness"

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
            str(_REPO_ROOT / "src"),
            "-o",
            str(binary),
            *[str(src) for src in sources],
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "app_fake_otos_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run_result.returncode == 0, (
        "app_fake_otos_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
