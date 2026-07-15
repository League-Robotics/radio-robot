"""Off-hardware acceptance proof for ticket 105-003 (SUC-020): the
deterministic motor+OTOS plant (``tests/sim/plant/{wheel,otos}_plant.{h,cpp}``).

Compiles ``plant_harness.cpp`` together with the plant classes themselves
and the HOST_BUILD Devices/App/Kinematics sources they exercise
(``source/devices/i2c_bus_host.cpp``, ``source/devices/velocity_pid.cpp``,
``source/devices/nezha_motor.cpp``, ``source/devices/otos.cpp``,
``source/kinematics/body_kinematics.cpp``, ``source/app/odometry.cpp``) with
``-DHOST_BUILD``, against the SAME headers every ARM build compiles. Mirrors
``test_app_odometry.py``'s exact shape: compile with the system C++
compiler, run the resulting binary, assert it exits 0.

Collected under ``tests/sim/plant/`` -- already within ``pyproject.toml``'s
``testpaths = ["tests/sim"]``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# tests/sim/plant/test_plant.py -> plant -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "source"
_PLANT_DIR = pathlib.Path(__file__).resolve().parent

_HARNESS_SRC = _PLANT_DIR / "plant_harness.cpp"
_WHEEL_PLANT_SRC = _PLANT_DIR / "wheel_plant.cpp"
_OTOS_PLANT_SRC = _PLANT_DIR / "otos_plant.cpp"
_ODOMETRY_SRC = _SOURCE_DIR / "app" / "odometry.cpp"
_I2C_HOST_FAKE_SRC = _SOURCE_DIR / "devices" / "i2c_bus_host.cpp"
_VELOCITY_PID_SRC = _SOURCE_DIR / "devices" / "velocity_pid.cpp"
_NEZHA_MOTOR_SRC = _SOURCE_DIR / "devices" / "nezha_motor.cpp"
_OTOS_SRC = _SOURCE_DIR / "devices" / "otos.cpp"
_BODY_KINEMATICS_SRC = _SOURCE_DIR / "kinematics" / "body_kinematics.cpp"

# Matches every other tests/sim/unit harness's own compiled standard.
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


def test_plant_harness_compiles_and_passes(tmp_path):
    """Compile the plant classes + their Devices/App/Kinematics dependencies
    + the harness; assert every scenario passes."""
    sources = [
        _HARNESS_SRC,
        _WHEEL_PLANT_SRC,
        _OTOS_PLANT_SRC,
        _ODOMETRY_SRC,
        _I2C_HOST_FAKE_SRC,
        _VELOCITY_PID_SRC,
        _NEZHA_MOTOR_SRC,
        _OTOS_SRC,
        _BODY_KINEMATICS_SRC,
    ]
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"source/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "plant_harness"

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
            "-o",
            str(binary),
            *[str(src) for src in sources],
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "plant_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run_result.returncode == 0, (
        "plant_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


def test_plant_files_carry_no_heading_wrap_logic():
    """Acceptance-criterion self-check (SUC-020, architecture-update.md
    Decision 3): the plant carries no heading/angle-wrap state or logic of
    its own -- a grep for atan2/fmod-wrap/wrapAngle across tests/sim/plant/
    must find nothing."""
    import re

    pattern = re.compile(r"atan2|fmod.*M_PI|wrapAngle")
    offenders = []
    for path in sorted(_PLANT_DIR.glob("*.h")) + sorted(_PLANT_DIR.glob("*.cpp")):
        text = path.read_text()
        for lineno, line in enumerate(text.splitlines(), start=1):
            if pattern.search(line):
                offenders.append(f"{path.name}:{lineno}: {line.strip()}")

    assert not offenders, (
        "tests/sim/plant/ must carry no heading-wrap logic of its own "
        f"(Decision 3) -- found:\n" + "\n".join(offenders)
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
