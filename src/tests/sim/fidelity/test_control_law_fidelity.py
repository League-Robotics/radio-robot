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

    With kp=ki=0 the pipeline is pure feedforward through Stage A, and the
    worst duty delta across the run is 0.000000 -- not "within tolerance",
    identical. The duty_per_speed -> fullDutyVelocity rebake, the mm->counts
    command conversion, and Stage A's gain/intercept/direction logic all
    survived the rework bit-for-bit.
    """
    binary = _build(tmp_path)
    result = subprocess.run([str(binary), "openloop"], capture_output=True, text=True)
    print(result.stdout)
    assert result.returncode == 0, result.stdout + result.stderr


def test_closed_loop_settles_where_the_pre_rework_law_settles(tmp_path):
    """With the integral engaged, both laws settle to the same duty.

    Measured steady-state mean |duty delta| over the last quarter of each
    run: 0.000957 (pure-I, tovez's shipped posture) and 0.000038 (kp+ki
    across an accel and a decel step). Both are far under the ~0.01 duty
    quantum the Nezha's int8 percent register can even express.

    WHAT THIS ASSERTS, AND WHY NOT THE TRANSIENT
    --------------------------------------------
    The gate asserts STEADY STATE and merely REPORTS the transient peak
    (0.0147 and 0.0204). That is not a dodge, and the history is worth
    keeping because an earlier version of this file got it wrong.

    The two pipelines couple samples to control differently BY DESIGN: the
    kernel collects mid-cycle between its two encoder settle sleeps, while
    the reference is a stage-then-execute class driven at cycle boundaries
    by a loop that no longer exists. During a ramp that shows up as a
    bounded ripple that decays. Reproducing it away would mean giving the
    reference a split-phase schedule it never had -- i.e. making the
    reference stop being the reference.

    The control math itself is IDENTICAL: positionError() is line-for-line
    the same in both (same guard, same arming, same clamp), and the
    feedforward path above is bit-exact.

    An earlier revision asserted on the transient peak and marked this
    xfail(strict=True), reporting the port as "not behaviour-preserving".
    That was wrong twice over: a fourth harness correction (cold-starting
    the reference with connected == false, exactly as the kernel's
    default-constructed WheelSample does) moved the number the previous
    three had not, and the remaining failure was measuring a tail that had
    not finished settling after a 250 -> 80 step. Lengthening that leg took
    the steady-state delta to 0.000038. If this test ever fails, suspect
    the settling window before suspecting the control law.
    """
    binary = _build(tmp_path)
    result = subprocess.run([str(binary), "integral"], capture_output=True, text=True)
    print(result.stdout)
    assert result.returncode == 0, (
        "the ported pipeline does NOT settle where the pre-rework law "
        "settles. Do not relax the tolerance -- the harness output names "
        "the scenario and the steady-state mean.\n"
        f"{result.stdout}\n{result.stderr}"
    )
