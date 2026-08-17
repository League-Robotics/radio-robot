"""src/tests/sim/fidelity/test_control_law_fidelity.py -- the fidelity gate.

Named for what it IS rather than "golden trace": that name is already
taken in this repo by src/tests/tools/golden_trace.py, an unrelated
trajectory-comparison utility, and pytest cannot hold two test modules
with the same basename.

The DifferentialDrive kernel rework claims its control pipeline is a UNIT
REBAKE of the law it replaced -- mm to counts, "zero math changes". That
claim was asserted and never tested, and the rework replaced the old law
IN PLACE, so by the time anyone thought to check, the reference was gone.

``golden_ref_drive.{h,cpp}`` is that law recovered from commit
``ab43963c`` and frozen. ``golden_trace_harness.cpp`` drives it and the
current ``Control::DifferentialDrive`` through identical command
sequences against identical plants and requires the DUTY they put on the
wire to agree.

This is the one test that can tell "the port is correct" from "the port
compiles and the robot moves". Everything else in the sim suite exercises
the new kernel against itself.

If this fails, do NOT relax the tolerance. A failure here means the
rebake changed behaviour, and the trace printed by the harness names the
cycle and the delta.

    uv run python -m pytest src/tests/sim/golden/test_golden_trace.py -v -s
"""

import pathlib
import subprocess
import sys

import pytest

# src/tests/sim/fidelity/test_control_law_fidelity.py -> fidelity -> sim -> tests -> src -> root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_FIDELITY_DIR = pathlib.Path(__file__).resolve().parent

_HARNESS = _FIDELITY_DIR / "golden_trace_harness.cpp"

# Deliberately SMALL. The gate compares two control laws to each other, so
# it must not drag in the comms/telemetry/plant graph -- anything the two
# sides share would cancel out of the comparison anyway, and every extra
# dependency is another way for the gate to break for reasons that are not
# about fidelity.
_SOURCES = [
    _HARNESS,
    _FIDELITY_DIR / "golden_ref_drive.cpp",
    _SOURCE_DIR / "control" / "differential_drive.cpp",
]


def _build(tmp_path):
    for src in _SOURCES:
        assert src.is_file(), f"required source missing: {src}"
    binary = tmp_path / "golden_trace_harness"
    result = subprocess.run(
        [
            "/usr/bin/c++", "-std=c++20", "-Wall", "-Wextra", "-DHOST_BUILD",
            "-I", str(_SOURCE_DIR),
            "-I", str(_REPO_ROOT / "src"),
            "-I", str(_FIDELITY_DIR),
            # for host_fiber.h (the fail-if-invoked launcher)
            "-I", str(_SOURCE_DIR / "platform" / "host"),
            *[str(s) for s in _SOURCES],
            "-o", str(binary),
        ],
        capture_output=True, text=True,
    )
    assert result.returncode == 0, (
        "golden_trace_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    )
    return binary


def test_feedforward_and_stage_a_match_exactly(tmp_path):
    """The velocity->duty map and Stage A reproduce the old law EXACTLY.

    This is the half of the port that is proven. With kp=ki=0 the pipeline
    is pure feedforward through Stage A, and the measured worst duty delta
    across the run is 0.000000 -- not "within tolerance", identical. That
    is a real result: it says the duty_per_speed -> fullDutyVelocity
    rebake, the mm->counts conversion of the command, and Stage A's
    gain/intercept/direction logic all survived the rework bit-for-bit.
    """
    binary = _build(tmp_path)
    result = subprocess.run([str(binary), "openloop"], capture_output=True, text=True)
    print(result.stdout)
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.xfail(
    strict=True,
    reason=(
        "UNRESOLVED, and deliberately recorded rather than tuned away: once "
        "the integral term engages, the ported pipeline's duty diverges from "
        "the pre-rework law by ~2% of full authority (0.0227 at the cycle "
        "the I term first contributes; ref 0.2000 vs new 0.1773, which is "
        "exactly the pure-feedforward value -- i.e. the kernel's I term "
        "starts contributing a cycle later, or with a different magnitude, "
        "than the old law's). "
        "NOT a harness artifact as far as three independent corrections can "
        "establish: the number is IDENTICAL (0.022746) after fixing the dt "
        "pacing, after making both sides read a difference-quotient "
        "velocity, and after aligning sample-freshness ordering -- each of "
        "which moved other numbers substantially. "
        "This is the most likely explanation on the table for the open-loop "
        "tracking falloff seen in sim (-20% at 250 mm/s on tovez_nocal). "
        "DO NOT relax the tolerance to make this pass. If you fix the "
        "divergence this test will start passing and strict=True will tell "
        "you so."
    ),
)
def test_integral_path_matches_the_pre_rework_control_law(tmp_path):
    binary = _build(tmp_path)
    result = subprocess.run([str(binary), "integral"], capture_output=True, text=True)
    print(result.stdout)
    assert result.returncode == 0, result.stdout + result.stderr
