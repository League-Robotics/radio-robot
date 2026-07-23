"""Off-hardware acceptance proof for ticket 116-005 (SUC-050/SUC-051/
SUC-052), ``App::MoveQueue`` (``src/firm/app/move_queue.{h,cpp}``).

Compiles ``app_move_queue_harness.cpp`` together with the HOST_BUILD
implementations it needs -- ``src/firm/app/move_queue.cpp``,
``src/firm/motion/stop_condition.cpp`` (116-002), ``src/firm/app/drive.cpp``
(116-004's ``setWheels()``), ``src/firm/app/odometry.cpp`` (116-003's
``pathLength()`` -- odometry.cpp also defines ``applyOtosSample()``, which
pulls in ``src/firm/devices/otos.cpp`` even though this harness never calls
it), ``src/firm/app/state_estimator.cpp`` (turn-prediction campaign:
``App::MoveQueue`` now holds a ``const StateEstimator&`` for its own
anticipation-lead stop-condition evaluation), ``src/sim/sim_plant.cpp`` +
its ``src/tests/sim/plant/{wheel,otos}_plant.cpp`` physics dependencies,
``src/firm/devices/velocity_pid.cpp``, ``src/firm/devices/nezha_motor.cpp``,
``src/firm/kinematics/body_kinematics.cpp``, and ``src/sim/sim_clock.cpp``
-- with ``-DHOST_BUILD``, against the SAME headers every ARM build compiles.
Mirrors ``test_app_drive.py``/``test_app_odometry.py``'s exact shape:
compile with the system C++ compiler, run the resulting binary, assert it
exits 0.

Collected under ``src/tests/sim/unit/`` -- already within ``pyproject.toml``'s
``testpaths = ["src/tests/sim"]``, no configuration change needed.
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/unit/test_app_move_queue.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_INFRA_SIM_DIR = _REPO_ROOT / "src" / "sim"
_PLANT_DIR = _REPO_ROOT / "src" / "tests" / "sim" / "plant"
_HARNESS_SRC = pathlib.Path(__file__).resolve().parent / "app_move_queue_harness.cpp"
_MOVE_QUEUE_SRC = _SOURCE_DIR / "app" / "move_queue.cpp"
_STOP_CONDITION_SRC = _SOURCE_DIR / "motion" / "stop_condition.cpp"
_VELOCITY_SHAPER_SRC = _SOURCE_DIR / "motion" / "velocity_shaper.cpp"
_DRIVE_SRC = _SOURCE_DIR / "app" / "drive.cpp"
_ODOMETRY_SRC = _SOURCE_DIR / "app" / "odometry.cpp"
_STATE_ESTIMATOR_SRC = _SOURCE_DIR / "app" / "state_estimator.cpp"
_SIM_PLANT_SRC = _INFRA_SIM_DIR / "sim_plant.cpp"
_SIM_CLOCK_SRC = _INFRA_SIM_DIR / "sim_clock.cpp"
_WHEEL_PLANT_SRC = _PLANT_DIR / "wheel_plant.cpp"
_OTOS_PLANT_SRC = _PLANT_DIR / "otos_plant.cpp"
_VELOCITY_PID_SRC = _SOURCE_DIR / "devices" / "velocity_pid.cpp"
_NEZHA_MOTOR_SRC = _SOURCE_DIR / "devices" / "nezha_motor.cpp"
_OTOS_SRC = _SOURCE_DIR / "devices" / "otos.cpp"
_BODY_KINEMATICS_SRC = _SOURCE_DIR / "kinematics" / "body_kinematics.cpp"

# Matches every other src/tests/sim/unit harness's own compiled standard --
# the project's actual compiled standard is -std=gnu++20.
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


def test_app_move_queue_harness_compiles_and_passes(tmp_path):
    """Compile App::MoveQueue + its collaborators + the harness; assert every
    scenario passes."""
    sources = [
        _HARNESS_SRC,
        _MOVE_QUEUE_SRC,
        _STOP_CONDITION_SRC,
        _VELOCITY_SHAPER_SRC,
        _DRIVE_SRC,
        _ODOMETRY_SRC,
        _STATE_ESTIMATOR_SRC,
        _SIM_PLANT_SRC,
        _SIM_CLOCK_SRC,
        _WHEEL_PLANT_SRC,
        _OTOS_PLANT_SRC,
        _VELOCITY_PID_SRC,
        _NEZHA_MOTOR_SRC,
        _OTOS_SRC,
        _BODY_KINEMATICS_SRC,
    ]
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "app_move_queue_harness"

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
            str(_INFRA_SIM_DIR),
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
        "app_move_queue_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run_result.returncode == 0, (
        "app_move_queue_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
