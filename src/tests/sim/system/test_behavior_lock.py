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

_ISSUE = "motion-control-terminal-blips-reconciled-fix-plan.md"


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
        [_HARNESS_SRC, _SIM_PLANT_SRC, _WIRE_TEST_CODEC_SRC, _WHEEL_PLANT_SRC, _OTOS_PLANT_SRC]
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
    """112-001: flipped from xfail to passing. Two things had to land
    together (stakeholder decision, reviews Sec5.3 "differentiate the
    emitted setpoints"):

    1. The production fix -- Motion::Executor::tick() no longer peeks
       plan_lead ahead of the current sample (F2's jerk-warp bug: peeking
       at `elapsed + lead` evaluated the reference at `2t` during the
       ramp-in, doubling commanded acceleration). Confirmed by direct A/B
       rebuild: BEFORE this fix even the COMMANDED (driveTargetVelLeft())
       trace itself spiked to ~4040mm/s^2 (bound 1350mm/s^2) right at
       activation; AFTER, the commanded trace is a clean a_max-bounded
       ramp with no spike at all.
    2. The harness fix -- this file's own behavior_lock_harness.cpp now
       differentiates the COMMANDED per-wheel setpoint
       (SimHarness::driveTargetVelLeft/Right()) for this check, not the
       DECODED/MEASURED wheel-velocity trace off the wire. Grading the
       measured trace conflated the commanded trajectory's own jerk-
       boundedness with the downstream velocity-PID/actuation-lag tracking
       response to it (a real, separate, ~130ms plant-lag phenomenon no
       Motion::Executor/App::Pilot control-law change can move) -- on the
       measured trace this check still failed post-fix (~1635mm/s^2 vs the
       same 1350mm/s^2 bound), even though the commanded reference it is
       supposed to grade was already clean. See behavior_lock_harness.cpp's
       own header comment for the full writeup."""
    _assert_result(harness_run, "straight_ramp_bounds")


def test_straight_terminal_bounds(harness_run):
    """112-001: also flips to passing, ahead of its originally-planned
    ticket (sprint.md assigns this flip to ticket 004, alongside the
    pivot lobe-shape checks below) -- an honest, verified-real early
    side effect of the SAME two changes test_straight_ramp_bounds
    documents (the plan_lead peek deletion plus behavior_lock_harness.cpp
    now grading the COMMANDED setpoint), not something this ticket set
    out to fix on purpose. Confirmed independently on both the measured
    and the commanded signal before this xfail marker was removed --
    kept honest per "set each marker to match reality," not left stale
    for ticket 004 to re-discover."""
    _assert_result(harness_run, "straight_terminal_bounds")


def test_straight_single_lobe_left(harness_run):
    _assert_result(harness_run, "straight_single_lobe_left")


def test_straight_single_lobe_right(harness_run):
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


@pytest.mark.xfail(
    strict=False,
    reason=(
        f"{_ISSUE}: 112-001's plan_lead/kPivotOvershootLeadSlope deletion is real and confirmed "
        "(Motion::Executor's own omegaFf contribution is provably clean -- same same-instant "
        "sample() fix, same mechanism that fully cleared test_straight_ramp_bounds) -- but unlike "
        "the straight case, the pivot's COMMANDED per-wheel setpoint is not Executor's omegaFf "
        "alone: it is deltaHeading != 0, so App::Pilot::tick() adds the heading PD correction term "
        "(heading_kp * (thetaRef - thetaMeasLead)) on top before Drive::setTwist(), and that PD "
        "term reacts every cycle to the OTOS-measured heading (App::HeadingSource), which is not a "
        "smooth reference the way Executor's own sample() output is. Direct A/B on the COMMANDED "
        "signal (post-harness-fix): the activation-region spike drops from an accel violation "
        "(~3722mm/s^2 vs bound 1728mm/s^2, pre-112-001) below the accel bound entirely, but a jerk "
        "violation remains one derivative up (~13824mm/s^3 vs bound 6912mm/s^3) -- smaller than the "
        "pre-fix commanded spike, real progress, but not clean. This is an App::Pilot-level (heading "
        "PD x measured-heading) finding, not anything left in Motion::Executor's own reference -- "
        "outside 112-001's Motion::Executor-only, pure-deletion scope. Not attributed to a specific "
        "future ticket; flagged for follow-up."
    ),
)
def test_pivot_ramp_bounds(harness_run):
    _assert_result(harness_run, "pivot_ramp_bounds")


def test_pivot_terminal_bounds(harness_run):
    _assert_result(harness_run, "pivot_terminal_bounds")


@pytest.mark.xfail(
    strict=False,
    reason=(
        f"{_ISSUE}: 112-001 (stakeholder decision, reviews Sec5.3) switched this check to grade "
        "the COMMANDED per-wheel setpoint (SimHarness::driveTargetVelLeft(), not the decoded/"
        "measured wire trace) -- on that commanded signal the pivot's own sign-changing tail is "
        "still real: 5 lobes across the trace, not 1 (was reported against the measured trace "
        "pre-112-001, same shape). Still attributable to the terminal patch stack still live in "
        "the commanded trajectory today (the same-sign overshoot carry, the min-speed floor -- "
        "ticket 004's own deletion scope) and/or the App::Pilot heading-PD-on-measured-heading "
        "dynamic test_pivot_ramp_bounds documents (its own reason, above) -- not this ticket's "
        "Motion::Executor-only, pure-deletion scope either way. Expected to flip when ticket 004 "
        "lands."
    ),
)
def test_pivot_single_lobe_left(harness_run):
    _assert_result(harness_run, "pivot_single_lobe_left")


@pytest.mark.xfail(
    strict=False,
    reason=(
        f"{_ISSUE}: same commanded-signal grading (112-001) and same finding as "
        "pivot_single_lobe_left, mirrored on the right wheel -- see that check's own reason."
    ),
)
def test_pivot_single_lobe_right(harness_run):
    _assert_result(harness_run, "pivot_single_lobe_right")


@pytest.mark.xfail(
    strict=False,
    reason=(
        f"{_ISSUE}: depends on pivot_single_lobe_left/right both being exactly 1 lobe each on the "
        "COMMANDED signal (112-001), which today's patch stack still does not produce -- see "
        "those two checks' own reasons."
    ),
)
def test_pivot_lobes_opposite_sign(harness_run):
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


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
