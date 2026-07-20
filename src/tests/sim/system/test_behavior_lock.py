"""src/tests/sim/system/test_behavior_lock.py -- sprint 111 ticket 001's own
numeric behavior-lock acceptance instrument (SUC-001): compiles and runs
``behavior_lock_harness.cpp`` (a D700 ``kArc`` straight + a 360deg
``kPivot``, each driven to completion through the REAL
``App::RobotLoop``/``App::Pilot``/``Motion::Executor`` graph via
``TestSim::SimHarness``, plus a same-boot 40-move reliability scenario) and
asserts each of the harness's own individually-named checks.

This is Step 0 of
``clasi/issues/motion-control-terminal-blips-reconciled-fix-plan.md`` --
"land a numeric jerk / single-lobe acceptance test first so every
subsequent deletion is guarded." Sprint 111 ticket 001 (which created this
file) was a PURE ADDITION: no production motion code
(``pilot.cpp``/``executor.cpp``) was touched by that ticket.

Sprint 112 ticket 001 (stakeholder decision, reviews Sec5.3 "differentiate
the emitted setpoints") is the one exception to "never touch production
code from this file": it deleted ``Motion::Executor::tick()``'s
``plan_lead`` peek-ahead sampling (F2's jerk-warp bug) AND switched
``behavior_lock_harness.cpp``'s ramp/terminal-bounds and single-lobe/
lobe-sign checks to grade the COMMANDED per-wheel setpoint
(``SimHarness::driveTargetVelLeft/Right()``) instead of the decoded/
measured wire trace -- grading the measured trace conflated the commanded
trajectory's own jerk-boundedness with the downstream velocity-PID/
actuation-lag tracking response to it, which no ``Motion::Executor``/
``App::Pilot`` control-law change can move. ``test_*_no_command_after_
terminal_zero`` (ticket-003's own check, below) intentionally stays on the
measured/decoded signal -- see that pair's own docstrings.

Sprint 112 ticket 002 (acceleration feedforward through the ``Drive``
mapping layer) re-pointed these SAME checks a second time, at a THIRD
signal: the PLANNED reference (``SimHarness::plannedRefLeft/Right()`` ->
``App::Pilot::refLeft/right()``, ``Motion::Executor``'s own jerk-limited
trajectory sampled before the heading-PD correction and before ``Drive``'s
own accel feedforward). Ticket 002's model feedforward
(``actuation_lag * a``, a deliberate, KEPT lag-compensation term) writes
into the COMMANDED signal ticket 001 had just cleaned -- Ruckig's own
acceleration is only piecewise-linear, so the feedforward's own time
derivative inherits the trajectory's jerk-segment step discontinuities,
regressing the COMMANDED signal's jerk-boundedness even though nothing
about the underlying SOLVED trajectory changed. Grading the PLANNED
reference instead (reviews Sec5.3's other clause: "record requested
endpoint, planned endpoint, measured endpoint... as separate telemetry
values") isolates "is the solved trajectory itself well-shaped" from both
downstream stages (the heading PD's reaction to noisy measured heading,
and the feedforward's own deliberate anticipation) -- see
``behavior_lock_harness.cpp``'s own header comment for the full
three-signal (PLANNED/COMMANDED/MEASURED) accounting.
``test_*_shelf_collapsed`` stays on the COMMANDED signal (it is about the
FINAL command reaching zero, FF included) and ``test_*_no_command_after_
terminal_zero`` stays on the MEASURED signal, both unchanged by this
ticket.

Ticket 003 (the ``App::Pilot::tick()`` stale-twist-on-idle fix) extends
this SAME harness with two new checks -- ``test_straight_shelf_collapsed``/
``test_pivot_shelf_collapsed``, below -- rather than adding its own
verification instrument, per this ticket's own "shared verification
instrument" convention. See those two tests' own docstrings for why they,
not ``test_*_no_command_after_terminal_zero`` (ticket 001's original
target for the xfail->pass flip), are what actually demonstrates ticket
003's fix.

Compiles ``behavior_lock_harness.cpp`` together with ``sim_plant.cpp``
(``src/sim/``), ``wire_test_codec.cpp``, the plant sources, and the same
full HOST_BUILD Devices/App/messages/kinematics/motion dependency graph
every sibling ``test_*.py`` in this directory already compiles (mirrors
``test_move_queue.py``'s exact shape).

Unlike the sibling harnesses (which run ALL their scenarios as one
pass/fail unit -- a single compile-and-run pytest function per file), this
harness never aborts on a scenario "failure": it always runs every check
and prints one machine-parseable
``RESULT: <name> :: PASS`` / ``RESULT: <name> :: FAIL :: <detail>`` line
per named behavior-lock assertion (behavior_lock_harness.cpp's own
``report()`` helper) -- because this ticket's whole point is to give each
check INDEPENDENT pass/xfail visibility (some genuinely pass today, some
don't; see each test function below for which and why). A SEPARATE,
harness-plumbing-only exit code (checkTrue()/fail(), the same idiom
move_queue_harness.cpp/heading_source_harness.cpp already use) reports
whether the harness itself is sane (command injected, no SOLVE_FAIL,
ACK_STATUS_DONE reached within budget) -- a nonzero exit there is a real
test-infrastructure bug, never an xfail candidate.

The harness is compiled and run exactly ONCE per pytest session (a
module-scoped fixture) -- compiling the full ~20-file HOST_BUILD graph
once and sharing the captured RESULT lines across all ~15 test functions
below is far cheaper than recompiling per named check, and the ticket's own
"independent visibility per named check" requirement (implementation plan
point 6) never required a separate binary per check, only a separate
report per check.

    uv run python -m pytest src/tests/sim/system/test_behavior_lock.py -v -s
"""

import pathlib
import re
import subprocess
import sys

import pytest

# src/tests/sim/system/test_behavior_lock.py -> system -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_SYSTEM_DIR = pathlib.Path(__file__).resolve().parent
_SUPPORT_DIR = _SYSTEM_DIR.parent / "support"
_PLANT_DIR = _SYSTEM_DIR.parent / "plant"
_INFRA_SIM_DIR = _REPO_ROOT / "src" / "sim"

_HARNESS_SRC = _SYSTEM_DIR / "behavior_lock_harness.cpp"
_SIM_PLANT_SRC = _INFRA_SIM_DIR / "sim_plant.cpp"
_WIRE_TEST_CODEC_SRC = _SUPPORT_DIR / "wire_test_codec.cpp"
_BENCH_TEST_CONFIG_SRC = _SUPPORT_DIR / "bench_test_config.cpp"
_WHEEL_PLANT_SRC = _PLANT_DIR / "wheel_plant.cpp"
_OTOS_PLANT_SRC = _PLANT_DIR / "otos_plant.cpp"

_APP_SOURCES = [
    _SOURCE_DIR / "app" / "robot_loop.cpp",
    _SOURCE_DIR / "app" / "comms.cpp",
    _SOURCE_DIR / "app" / "telemetry.cpp",
    _SOURCE_DIR / "app" / "deadman.cpp",
    _SOURCE_DIR / "app" / "drive.cpp",
    _SOURCE_DIR / "app" / "odometry.cpp",
    _SOURCE_DIR / "app" / "heading_source.cpp",
    _SOURCE_DIR / "app" / "preamble.cpp",
    _SOURCE_DIR / "app" / "pilot.cpp",
]
_DEVICE_SOURCES = [
    _INFRA_SIM_DIR / "sim_clock.cpp",
    _SOURCE_DIR / "devices" / "velocity_pid.cpp",
    _SOURCE_DIR / "devices" / "nezha_motor.cpp",
    _SOURCE_DIR / "devices" / "otos.cpp",
    _SOURCE_DIR / "devices" / "color_sensor.cpp",
    _SOURCE_DIR / "devices" / "line_sensor.cpp",
]
_MESSAGE_SOURCES = [
    _SOURCE_DIR / "messages" / "wire.cpp",
    _SOURCE_DIR / "messages" / "wire_runtime.cpp",
]
_KINEMATICS_SOURCES = [
    _SOURCE_DIR / "kinematics" / "body_kinematics.cpp",
]
_RUCKIG_INCLUDE = _REPO_ROOT / "vendor" / "ruckig" / "include"
_RUCKIG_SRC_DIR = _REPO_ROOT / "vendor" / "ruckig" / "src"
_MOTION_SOURCES = [
    _SOURCE_DIR / "motion" / "jerk_trajectory.cpp",
    _SOURCE_DIR / "motion" / "executor.cpp",
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
        [_HARNESS_SRC, _SIM_PLANT_SRC, _WIRE_TEST_CODEC_SRC, _BENCH_TEST_CONFIG_SRC,
         _WHEEL_PLANT_SRC, _OTOS_PLANT_SRC]
        + _APP_SOURCES
        + _DEVICE_SOURCES
        + _MESSAGE_SOURCES
        + _KINEMATICS_SOURCES
        + _MOTION_SOURCES
        + sorted(_RUCKIG_SRC_DIR.glob("*.cpp"))
    )


_RESULT_LINE_RE = re.compile(r"^RESULT: (\S+) :: (PASS|FAIL)(?: :: (.*))?$")


def _parse_results(stdout: str) -> dict:
    """Parses every machine-parseable ``RESULT: <name> :: PASS|FAIL[ ::
    <detail>]`` line the harness prints (behavior_lock_harness.cpp's own
    report() helper) into {name: (status, detail)}."""
    results = {}
    for line in stdout.splitlines():
        m = _RESULT_LINE_RE.match(line.strip())
        if m:
            name, status, detail = m.group(1), m.group(2), m.group(3) or ""
            results[name] = (status, detail)
    return results


@pytest.fixture(scope="module")
def harness_run(tmp_path_factory):
    """Compiles behavior_lock_harness.cpp + its full dependency graph ONCE
    for this whole module, runs it ONCE, and returns (run_result,
    parsed_results) for every test function below to share -- see this
    file's own header for why one compile-and-run is shared across ~15
    test functions instead of recompiling per named check."""
    sources = _all_sources()
    for src in sources:
        assert src.is_file(), f"required source missing: {src}"
    assert _SOURCE_DIR.is_dir(), f"src/firm/ tree missing: {_SOURCE_DIR}"

    cxx = _find_cxx_compiler()
    tmp_path = tmp_path_factory.mktemp("behavior_lock")
    binary = tmp_path / "behavior_lock_harness"

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
            str(_SUPPORT_DIR),
            "-I",
            str(_PLANT_DIR),
            "-I",
            str(_INFRA_SIM_DIR),
            "-I",
            str(_RUCKIG_INCLUDE),
            "-o",
            str(binary),
            *[str(src) for src in sources],
        ],
        capture_output=True,
        text=True,
    )
    assert compile_result.returncode == 0, (
        "behavior_lock_harness.cpp / its dependencies failed to compile:\n"
        f"stdout:\n{compile_result.stdout}\nstderr:\n{compile_result.stderr}"
    )

    run_result = subprocess.run([str(binary)], capture_output=True, text=True)
    print(run_result.stdout)
    return run_result, _parse_results(run_result.stdout)


def test_behavior_lock_harness_compiles_and_runs(harness_run):
    """Harness-plumbing sanity (acceptance criterion: 'uv run pytest
    collects and runs the new harness with no collection errors'). A
    nonzero exit here means the HARNESS itself is broken (bad inject, a
    SOLVE_FAIL, a command never reaching ACK_STATUS_DONE at all within
    budget) -- NOT a behavior-lock finding; those are reported via the
    RESULT: lines the other test functions in this module check
    individually below."""
    run_result, _ = harness_run
    assert run_result.returncode == 0, (
        "behavior_lock_harness reported a PLUMBING failure "
        f"(exit {run_result.returncode}) -- see captured stdout above (run with -s)"
    )


def _assert_result(harness_run_result, name: str):
    _, results = harness_run_result
    assert name in results, f"no RESULT line for {name!r} -- harness did not report this check"
    status, detail = results[name]
    assert status == "PASS", f"{name}: {detail}"


# --- D700 straight (kArc, deltaHeading=0) ---------------------------------


def test_straight_ramp_bounds(harness_run):
    """112-001: flipped from xfail to passing, grading the COMMANDED
    per-wheel setpoint at the time (two things landed together: the
    Motion::Executor::tick() plan_lead peek-ahead deletion, F2's jerk-warp
    bug, plus this harness re-pointed at SimHarness::driveTargetVelLeft/
    Right() instead of the decoded/measured wire trace).

    112-002 (acceleration feedforward through the Drive mapping layer)
    then wrote a DELIBERATE, KEPT model feedforward (actuation_lag * a)
    into that SAME commanded signal -- real lag-compensation overshoot,
    not a bug -- which reintroduced jerk-scale content into it (Ruckig's
    acceleration is only piecewise-linear, so the feedforward's own time
    derivative, actuation_lag * jerk, inherits the trajectory's jerk-
    segment step discontinuities). Measured directly with the feedforward
    genuinely engaged: sample 4 jerk 24800mm/s^3 vs bound 10800mm/s^3 on
    the commanded signal -- a real regression of 112-001's own flip.

    Stakeholder resolution (112-002, reviews Sec5.3's OTHER clause --
    "record requested endpoint, PLANNED endpoint, measured endpoint... as
    separate telemetry values"): keep the feedforward, and re-point this
    check a SECOND time at a third, distinct signal -- the PLANNED
    reference (SimHarness::plannedRefLeft/Right() -> App::Pilot::
    refLeft/right(): Motion::Executor's own jerk-limited trajectory,
    sampled before Pilot's heading-PD and before Drive's accel
    feedforward). Verified clean on this signal with the feedforward live.
    See behavior_lock_harness.cpp's own header comment for the full
    three-signal (PLANNED/COMMANDED/MEASURED) writeup."""
    _assert_result(harness_run, "straight_ramp_bounds")


def test_straight_terminal_bounds(harness_run):
    """112-001: flipped to passing as a side effect of the SAME two
    changes test_straight_ramp_bounds documents; 112-002 then regressed it
    on the COMMANDED signal the same way (feedforward-induced jerk at the
    terminal top-up transition: sample 41 jerk 13499.7mm/s^3 vs bound
    10800mm/s^3) and re-fixed it the same way -- grading the PLANNED
    reference instead. See test_straight_ramp_bounds's own docstring for
    the full history."""
    _assert_result(harness_run, "straight_terminal_bounds")


def test_straight_single_lobe_left(harness_run):
    """112-002: grades the PLANNED reference (SimHarness::plannedRefLeft())
    like test_straight_ramp_bounds above -- see that test's own docstring
    for the full history. The commanded signal's own feedforward-induced
    reshaping was severe enough here to also change the LOBE COUNT (2
    lobes instead of 1, not just an accel/jerk bound violation) when
    graded on SimHarness::driveTargetVelLeft() -- confirmed regressed with
    the feedforward genuinely engaged, and clean again once re-pointed at
    the PLANNED reference."""
    _assert_result(harness_run, "straight_single_lobe_left")


def test_straight_single_lobe_right(harness_run):
    """See test_straight_single_lobe_left's own docstring -- same grading,
    mirrored on the right wheel."""
    _assert_result(harness_run, "straight_single_lobe_right")


def test_straight_no_command_after_terminal_zero(harness_run):
    """The check ticket 001 originally slated to flip xfail->passing for
    ticket 003. Reconciled during ticket 003 (see its own completion
    notes): this was ALREADY a plain, non-xfailed pass BEFORE ticket 003's
    fix landed -- not because the "shelf" doesn't exist, but because this
    check is evaluated against the DECODED, MEASURED wheel-velocity trace,
    and the ideal sim's own terminal decel already drives that trace under
    the 15mm/s near-zero bar by completion (both traces settle to <5mm/s
    within one cycle of the DONE ack) -- holding an already-near-zero
    MEASURED value stale for the ~300ms deadman-lease window never crosses
    the bar again, so this particular check cannot see the fix either way.
    test_straight_shelf_collapsed (below) is ticket 003's own real,
    demonstrable regression fence -- it measures the COMMANDED target
    directly and the shelf-length metric it reports goes from 5 cycles
    (pre-fix) to 0 (post-fix). Kept as its own always-real assertion
    regardless, so a future regression that DOES show up in the measured
    trace is still caught immediately."""
    _assert_result(harness_run, "straight_no_command_after_terminal_zero")


def test_straight_shelf_collapsed(harness_run):
    """111-003's own real regression fence (see behavior_lock_harness.cpp's
    measureShelfCycles()/runShelfScenario() for the full mechanism): counts
    cycles from the D700 straight's own ACK_STATUS_DONE to the first cycle
    the COMMANDED PID target (Devices::Motor::velocityTarget(), the value
    App::Drive::tick() last wrote via setVelocity() -- NOT the measured/
    decoded telemetry velocity test_straight_no_command_after_terminal_zero
    checks above) reads EXACTLY 0.0f, and asserts it collapses to <=2
    cycles. Measured directly (recompiling this same harness against the
    pre-111-003 pilot.cpp): 5 cycles before the fix (the ~300ms deadman-
    lease shelf), 0 cycles after -- this is the assertion that actually
    demonstrates ticket 003's fix, since the measured-velocity check above
    cannot (see its own docstring)."""
    _assert_result(harness_run, "straight_shelf_collapsed")


# --- 360deg pivot (kPivot, distance=0) ------------------------------------


def test_pivot_ramp_bounds(harness_run):
    """112-001 confirmed Motion::Executor's own omegaFf contribution was
    already clean (the same same-instant sample() fix that cleared
    test_straight_ramp_bounds), but the pivot's COMMANDED per-wheel
    setpoint is not omegaFf alone: App::Pilot::tick() adds the heading PD
    correction term (heading_kp * (thetaRef - thetaMeasLead)) on top,
    reacting every cycle to the OTOS-measured heading -- not a smooth
    reference the way Executor's own sample() output is. Graded on the
    COMMANDED signal (SimHarness::driveTargetVelLeft/Right()), this check
    stayed xfail through 112-001 (an accel-bound violation, ~3722mm/s^2 vs
    bound 1728mm/s^2, pre-112-001) and would have stayed xfail under
    112-002's own accel feedforward too (which adds yet more jerk-scale
    content on top of the PD's own reaction).

    112-002 (stakeholder decision, reviews Sec5.3's "planned endpoint...
    as a separate telemetry value" clause) re-points this check at the
    PLANNED reference instead (SimHarness::plannedRefLeft/Right() ->
    App::Pilot::refLeft/right(): Motion::Executor's own omegaFf, sampled
    BEFORE the heading-PD correction and BEFORE the accel feedforward are
    added). The pivot's planned rotational channel carries neither the PD
    reaction nor the feedforward -- verified clean (PASS, not xfail) on
    this signal: the pivot's own SOLVED trajectory was never the problem,
    only what App::Pilot/App::Drive layer on top of it downstream, which
    this check no longer conflates in."""
    _assert_result(harness_run, "pivot_ramp_bounds")


def test_pivot_terminal_bounds(harness_run):
    _assert_result(harness_run, "pivot_terminal_bounds")


def test_pivot_single_lobe_left(harness_run):
    """112-001 (stakeholder decision, reviews Sec5.3) switched this check
    to grade the COMMANDED per-wheel setpoint -- on that signal the
    pivot's own sign-changing tail was real (5 lobes across the trace, not
    1), attributable to the terminal patch stack still live in the
    commanded trajectory (the same-sign overshoot carry, the min-speed
    floor) and the heading-PD-on-measured-heading dynamic
    test_pivot_ramp_bounds documents -- this check stayed xfail through
    112-001.

    112-002 re-points this check at the PLANNED reference instead (see
    test_pivot_ramp_bounds's own docstring for the full three-signal
    history) -- App::Pilot's downstream patch stack and PD reaction never
    touch the planned trajectory, so this is a clean single lobe there.
    Verified PASS with 112-002's own accel feedforward genuinely engaged."""
    _assert_result(harness_run, "pivot_single_lobe_left")


def test_pivot_single_lobe_right(harness_run):
    """See test_pivot_single_lobe_left's own docstring -- same grading and
    history, mirrored on the right wheel."""
    _assert_result(harness_run, "pivot_single_lobe_right")


def test_pivot_lobes_opposite_sign(harness_run):
    """Depends on pivot_single_lobe_left/right both being exactly 1 lobe
    each on whichever signal they grade -- see those two checks' own
    docstrings for the PLANNED-reference re-grade (112-002) that makes
    this true."""
    _assert_result(harness_run, "pivot_lobes_opposite_sign")


def test_pivot_no_command_after_terminal_zero(harness_run):
    """See test_straight_no_command_after_terminal_zero's own docstring --
    same honest currently-passing-either-way status here too, not xfailed;
    test_pivot_shelf_collapsed (below) is the check that actually
    demonstrates ticket 003's fix for this scenario."""
    _assert_result(harness_run, "pivot_no_command_after_terminal_zero")


def test_pivot_shelf_collapsed(harness_run):
    """See test_straight_shelf_collapsed's own docstring -- same mechanism,
    360deg pivot scenario. Measured directly: 5 cycles before the 111-003
    fix, 0 cycles after."""
    _assert_result(harness_run, "pivot_shelf_collapsed")


# --- Same-boot reliability (SUC-001 step 5, targets the driving issue's own
#     Sec1.8/F7 stale-executor-state finding) -------------------------------


def test_same_boot_all_moves_completed(harness_run):
    """40 consecutive alternating D700-straight/360deg-pivot Move commands
    on ONE booted SimHarness instance, no reboot between them (unlike
    turn_windage_sweep.py's own deliberate per-run isolation). Currently
    PASSES -- all 40 reach ACK_STATUS_DONE within budget; the Sec1.8/F7
    stale-executor-state bug does NOT reproduce in this ideal-sim
    same-boot scenario. See this ticket's own completion notes for the
    full writeup."""
    _assert_result(harness_run, "same_boot_all_moves_completed")


# --- Chained pivot->pivot (112-004: 109-009 boundary-velocity-carry
#     exception guardrail) ---------------------------------------------------


def test_chained_pivot_no_decel_at_boundary(harness_run):
    """112-004: two SAME-SIGN 180deg pivots injected back-to-back (the
    second enqueued via injectMove(..., replace=false) while the first is
    still running) must both complete, and the commanded per-wheel target
    must NOT drop near zero within 2 cycles of the first pivot's own
    completion -- proving Motion::Executor's own 109-009 boundary-velocity-
    carry exception (the carryingRotationalVelocity branch, preserved
    VERBATIM as a distinct code path by this ticket's own unified-
    completion rewrite) still hands the still-rotating channel straight to
    the successor instead of decelerating to rest at the boundary. Sprint
    111's own same-boot scenario (above) alternates straight/pivot with no
    chaining at all, so it never exercised this path; this is the first
    harness coverage of it through the full App::RobotLoop/App::Pilot/
    Motion::Executor graph (boundary_velocity_harness.cpp's own Scenario 4
    already covers the same mechanism at the raw Motion::Executor level,
    with no App::Pilot/App::RobotLoop in the loop)."""
    _assert_result(harness_run, "chained_pivot_no_decel_at_boundary")


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
