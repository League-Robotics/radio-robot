"""Ticket 109-007 (sim-honors-otos-calibration.md) acceptance proof.

Compiles ``sim_fidelity_harness.cpp`` together with ``sim_plant.cpp``
(``src/sim/``), ``otos.cpp`` (``src/firm/devices/`` -- the REAL firmware
leaf), and the two plant sources (``src/tests/sim/plant/``) -- a much
lighter dependency graph than ``test_sim_api.py``/``test_fault_knobs.py``
need (no App::RobotLoop, no messages/, no motion/Ruckig): Devices::Otos and
every TestSim:: plant class are devices/-isolation-invariant leaves with no
messages/app dependency, so this harness needs only the devices+plant
sources themselves. Mirrors every other ``tests/sim/system/`` harness's
compile-then-run shape.

Collected under ``src/tests/sim/system/faults/`` -- already within
``pyproject.toml``'s ``testpaths = ["src/tests/sim"]``. Run just this file
with:

    uv run python -m pytest src/tests/sim/system/faults/test_sim_fidelity.py -v
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/system/faults/test_sim_fidelity.py -> faults -> system -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[5]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_FAULTS_DIR = pathlib.Path(__file__).resolve().parent
_PLANT_DIR = _FAULTS_DIR.parent.parent / "plant"
_INFRA_SIM_DIR = _REPO_ROOT / "src" / "sim"

_HARNESS_SRC = _FAULTS_DIR / "sim_fidelity_harness.cpp"
_SIM_PLANT_SRC = _INFRA_SIM_DIR / "sim_plant.cpp"
_OTOS_SRC = _SOURCE_DIR / "devices" / "otos.cpp"
_WHEEL_PLANT_SRC = _PLANT_DIR / "wheel_plant.cpp"
_OTOS_PLANT_SRC = _PLANT_DIR / "otos_plant.cpp"
_BODY_KINEMATICS_SRC = _SOURCE_DIR / "kinematics" / "body_kinematics.cpp"

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


def _all_sources():
    return [
        _HARNESS_SRC,
        _SIM_PLANT_SRC,
        _OTOS_SRC,
        _WHEEL_PLANT_SRC,
        _OTOS_PLANT_SRC,
        _BODY_KINEMATICS_SRC,
    ]


def test_sim_fidelity_harness_compiles_and_passes(tmp_path):
    """Compile the sim-fidelity harness + its (small) dependency graph;
    assert every scenario passes."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "sim_fidelity_harness"

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
            str(_PLANT_DIR),
            "-I",
            str(_INFRA_SIM_DIR),
            "-o",
            str(binary),
            *[str(src) for src in sources],
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "sim_fidelity_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run_result.returncode == 0, (
        "sim_fidelity_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )
    print(run_result.stdout)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
