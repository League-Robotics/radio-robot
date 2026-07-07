"""Off-hardware acceptance proof for ticket 081-003 (Subsystems::SimHardware,
Hal::PhysicsWorld, Hal::SimMotor, Hal::SimOdometer).

Compiles ``sim_hardware_harness.cpp`` together with the REAL
``source/hal/sim/physics_world.cpp``, ``source/hal/sim/sim_motor.cpp``,
``source/hal/sim/sim_odometer.cpp``, ``source/hal/velocity_pid.cpp`` (ticket
081-001's shared PID, the SAME class Hal::NezhaMotor calls), and
``source/subsystems/sim_hardware.cpp``, against the SAME ``source/
subsystems/hardware.h`` seam ``test_hardware_seam.py`` exercises against
``Subsystems::NezhaHardware`` — with ``-DHOST_BUILD`` so the HOST_BUILD-gated
paths (PhysicsWorld's std::mt19937 members, SimOdometer's noise generator)
compile. Mirrors that file's shape exactly: compile with the system C++
compiler, run the resulting binary, assert it exits 0.

No CMake needed yet (078's Decision 9 precedent, reused here per ticket
081-003's own acceptance criteria) — ``source/hal/sim/*`` and
``source/subsystems/sim_hardware.cpp`` are excluded from the ARM firmware
build (CMakeLists.txt's blanket ``.*/hal/sim/.*`` regex, plus an explicit
``.*/subsystems/sim_hardware\\.cpp$`` exclusion) and are not yet compiled by
any CMake-based host build either (that is ticket 004's
``tests/_infra/sim/CMakeLists.txt``) — this harness is this ticket's own,
self-contained pre-004 acceptance mechanism.

Collected under ``tests/sim/unit/`` alongside the other harness wrappers —
already within ``pyproject.toml``'s ``testpaths = ["tests/sim", "tests/unit"]``,
no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/unit/test_sim_hardware.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "sim_hardware_harness.cpp"
_PHYSICS_WORLD_SRC = _SOURCE_DIR / "hal" / "sim" / "physics_world.cpp"
_SIM_MOTOR_SRC = _SOURCE_DIR / "hal" / "sim" / "sim_motor.cpp"
_SIM_ODOMETER_SRC = _SOURCE_DIR / "hal" / "sim" / "sim_odometer.cpp"
_VELOCITY_PID_SRC = _SOURCE_DIR / "hal" / "velocity_pid.cpp"
_SIM_HARDWARE_SRC = _SOURCE_DIR / "subsystems" / "sim_hardware.cpp"

_SOURCES = [
    _HARNESS_SRC,
    _PHYSICS_WORLD_SRC,
    _SIM_MOTOR_SRC,
    _SIM_ODOMETER_SRC,
    _VELOCITY_PID_SRC,
    _SIM_HARDWARE_SRC,
]

# messages/common.h documents its own target as "CODAL C++11" -- build the
# host harness to the same standard so it exercises exactly the language
# subset the firmware itself uses.
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


def test_sim_hardware_harness_compiles_and_passes(tmp_path):
    """Compile the SimHardware/PhysicsWorld/SimMotor harness and assert every scenario passes."""
    for src in _SOURCES:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "sim_hardware_harness"

    compile_result = subprocess.run(
        [
            cxx,
            f"-std={_CXX_STANDARD}",
            "-Wall",
            "-Wextra",
            "-DHOST_BUILD",
            "-I",
            str(_SOURCE_DIR),
            "-o",
            str(binary),
        ]
        + [str(src) for src in _SOURCES],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "sim_hardware_harness.cpp (or one of its real sources) failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run(
        [str(binary)], capture_output=True, text=True,
    )
    assert run_result.returncode == 0, (
        "sim_hardware_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
