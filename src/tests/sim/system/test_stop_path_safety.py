"""Off-hardware acceptance proof for ticket 133-001 -- the derived-idle
safety-arbitration step ``App::RobotLoop::zeroUnownedMotion()``.

Compiles ``stop_path_safety_harness.cpp`` against TestSim::SimHarness
(``src/sim/sim_harness.h``) plus its full dependency graph, with
``-DHOST_BUILD``, against the SAME headers every ARM build compiles.
Mirrors ``test_robot_loop_tlm.py``'s exact shape (same composition root,
same ``_APP_SOURCES``-style source lists) -- see that file's own header for
the full rationale, and ``stop_path_safety_harness.cpp``'s own header for
the measured defect each scenario exists for.

Two independent proofs live here:

1. ``test_stop_path_safety_harness_compiles_and_passes`` -- the behavioral
   half: three end-to-end scenarios through the real ``App::RobotLoop``.
2. ``test_arbitration_step_writes_only_zero`` -- the STRUCTURAL half. The
   monotone contract ("this method may write only ``0.0f``") is what makes
   a loop-level write to a decider-owned field legitimate at all, and no
   runtime scenario can prove a negative about every possible future edit.
   This test reads the method's own body and fails if any ``cmdVelocity``
   assignment inside it is anything other than a literal zero. It is a
   guard against the contract being relaxed, not a substitute for the
   scenarios above.

Collected under ``src/tests/sim/system/`` -- already within
``pyproject.toml``'s ``testpaths = ["src/tests/sim"]``, no configuration
change needed.
"""

import pathlib
import re
import subprocess
import sys

import pytest

# src/tests/sim/system/test_stop_path_safety.py -> system -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_SYSTEM_DIR = pathlib.Path(__file__).resolve().parent
_SUPPORT_DIR = _SYSTEM_DIR.parent / "support"
_PLANT_DIR = _SYSTEM_DIR.parent / "plant"
_INFRA_SIM_DIR = _REPO_ROOT / "src" / "sim"

_HARNESS_SRC = _SYSTEM_DIR / "stop_path_safety_harness.cpp"
_ROBOT_LOOP_SRC = _SOURCE_DIR / "app" / "robot_loop.cpp"
_SIM_PLANT_SRC = _INFRA_SIM_DIR / "sim_plant.cpp"
_WIRE_TEST_CODEC_SRC = _SUPPORT_DIR / "wire_test_codec.cpp"
_BENCH_TEST_CONFIG_SRC = _SUPPORT_DIR / "bench_test_config.cpp"
_WHEEL_PLANT_SRC = _PLANT_DIR / "wheel_plant.cpp"
_OTOS_PLANT_SRC = _PLANT_DIR / "otos_plant.cpp"

# Mirrors test_robot_loop_tlm.py / test_sim_api.py exactly -- SAME
# composition root (TestSim::SimHarness), so the SAME full build-source list
# applies (see MEMORY note: "Planner .cpp needs FOUR build source lists" --
# this pytest file is one more copy, keep it in lockstep with those two if
# the Planner/RobotLoop dependency graph ever changes again).
_APP_SOURCES = [
    _ROBOT_LOOP_SRC,
    _SOURCE_DIR / "app" / "comms.cpp",
    _SOURCE_DIR / "app" / "debug.cpp",
    _SOURCE_DIR / "app" / "configurator.cpp",
    _SOURCE_DIR / "app" / "telemetry.cpp",
    _REPO_ROOT / "src" / "motion" / "planner" / "profile.cpp",
    _REPO_ROOT / "src" / "motion" / "planner" / "estimation.cpp",
    _REPO_ROOT / "src" / "motion" / "planner" / "shape.cpp",
    _REPO_ROOT / "src" / "motion" / "planner" / "planner.cpp",
    _REPO_ROOT / "src" / "motion" / "navigator" / "arc_solver.cpp",  # 135-004
    _REPO_ROOT / "src" / "motion" / "navigator" / "navigator.cpp",  # 135-004
    _SOURCE_DIR / "app" / "drive.cpp",
    _REPO_ROOT / "src" / "motion" / "odometry.cpp",
    _SOURCE_DIR / "app" / "preamble.cpp",
    _SOURCE_DIR / "app" / "boot_wiring.cpp",
    _SOURCE_DIR / "app" / "boot_calibration.cpp",
]
_DEVICE_SOURCES = [
    _INFRA_SIM_DIR / "sim_clock.cpp",
    _SOURCE_DIR / "devices" / "nezha_motor.cpp",
    _SOURCE_DIR / "devices" / "otos.cpp",
    _SOURCE_DIR / "devices" / "color_sensor.cpp",
    _SOURCE_DIR / "devices" / "line_sensor.cpp",
]
_CONFIG_SOURCES = [
    _SOURCE_DIR / "config" / "persisted_tuning.cpp",
    _SOURCE_DIR / "config" / "boot_config.cpp",
]
_MESSAGE_SOURCES = [
    _SOURCE_DIR / "messages" / "wire.cpp",
    _SOURCE_DIR / "messages" / "wire_runtime.cpp",
]
_KINEMATICS_SOURCES = [
    _REPO_ROOT / "src" / "motion" / "body_kinematics.cpp",
]

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
    return (
        [
            _HARNESS_SRC,
            _SIM_PLANT_SRC,
            _WIRE_TEST_CODEC_SRC,
            _BENCH_TEST_CONFIG_SRC,
            _WHEEL_PLANT_SRC,
            _OTOS_PLANT_SRC,
        ]
        + _APP_SOURCES
        + _DEVICE_SOURCES
        + _CONFIG_SOURCES
        + _MESSAGE_SOURCES
        + _KINEMATICS_SOURCES
    )


def test_stop_path_safety_harness_compiles_and_passes(tmp_path):
    """Compile the RobotLoop/SimHarness graph + the harness; assert every scenario passes."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"

    cxx = _find_cxx_compiler()
    binary = tmp_path / "stop_path_safety_harness"

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
            "-I",
            str(_REPO_ROOT / "src" / "motion" / "planner"),  # 135-004: navigator.h's bare #include "planner.h"
            "-I",
            str(_SUPPORT_DIR),
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
        "stop_path_safety_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run_result.returncode == 0, (
        "stop_path_safety_harness reported a scenario failure "
        f"(exit {run_result.returncode}):\n{run_result.stdout}\n{run_result.stderr}"
    )
    print(run_result.stdout)


def _method_body(source: str, signature: str) -> str:
    """Return the brace-matched body of the method whose definition starts with `signature`."""
    start = source.index(signature)
    open_brace = source.index("{", start)
    depth = 0
    for i in range(open_brace, len(source)):
        if source[i] == "{":
            depth += 1
        elif source[i] == "}":
            depth -= 1
            if depth == 0:
                return source[open_brace + 1 : i]
    raise AssertionError(f"unbalanced braces after {signature!r}")


def test_arbitration_step_writes_only_zero():
    """The monotone contract, structurally: the arbiter may write only 0.0f.

    A zero-only writer cannot originate motion, cannot fight a decider for
    control, and cannot produce a speed no decider asked for -- which is the
    entire justification for App::RobotLoop writing a decider-owned field at
    all (the ownership invariant at ``RobotLoop::publishWheels()``). The
    moment this method can emit a nonzero, App::RobotLoop is a third decider
    and that invariant is void, so the restriction is checked here rather
    than left to review.
    """
    source = _ROBOT_LOOP_SRC.read_text()
    body = _method_body(source, "void RobotLoop::zeroUnownedMotion()")

    assignments = re.findall(r"cmdVelocity\s*=\s*([^;]+);", body)
    assert len(assignments) == 2, (
        "expected exactly two cmdVelocity assignments (left and right) in "
        f"zeroUnownedMotion(); found {len(assignments)}: {assignments}"
    )
    for value in assignments:
        assert value.strip() == "0.0f", (
            "zeroUnownedMotion() assigned a non-zero value to cmdVelocity: "
            f"{value.strip()!r}. The safety arbiter's writes are restricted to "
            "zero -- see the ownership invariant at RobotLoop::publishWheels()."
        )

    # ... and it must not reach the wheels through any other field either:
    # a nonzero cmdAccel or a mode change would be the same violation wearing
    # a different name.
    assert "cmdAccel" not in body, (
        "zeroUnownedMotion() touched cmdAccel; the arbiter's whole surface is "
        "the two cmdVelocity zeroes"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
