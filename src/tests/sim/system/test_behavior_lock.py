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
subsequent deletion is guarded." It is a PURE ADDITION: no production
motion code (``pilot.cpp``/``executor.cpp``) is touched by this ticket.

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
once and sharing the captured RESULT lines across all ~13 test functions
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
    file's own header for why one compile-and-run is shared across ~13
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


@pytest.mark.xfail(
    strict=False,
    reason=(
        f"{_ISSUE}: the current patch stack produces an activation-region accel spike on the "
        "straight (measured wheel accel ~4000mm/s^2 within the first cycle of Move activation, "
        "vs the configured a_max/a_decel-derived bound) -- fixed by sprint 2's deletion of the "
        "lead-sampling/padding machinery, not this ticket."
    ),
)
def test_straight_ramp_bounds(harness_run):
    _assert_result(harness_run, "straight_ramp_bounds")


@pytest.mark.xfail(
    strict=False,
    reason=(
        f"{_ISSUE}: the current patch stack's terminal top-up produces a jerk spike right at "
        "Move completion on the straight, exceeding the configured j_max-derived bound -- fixed "
        "by sprint 2's deletion of the terminal patch stack, not this ticket."
    ),
)
def test_straight_terminal_bounds(harness_run):
    _assert_result(harness_run, "straight_terminal_bounds")


def test_straight_single_lobe_left(harness_run):
    _assert_result(harness_run, "straight_single_lobe_left")


def test_straight_single_lobe_right(harness_run):
    _assert_result(harness_run, "straight_single_lobe_right")


def test_straight_no_command_after_terminal_zero(harness_run):
    """The ONE check ticket 003 (the App::Pilot::tick() stale-twist-on-idle
    fix) targets (SUC-002's own acceptance criterion). Currently PASSES for
    this D700-straight/deltaHeading=0 scenario in the ideal sim -- NOT
    xfailed (an xfail on an already-passing check would XPASS, technically
    tolerated under strict=False but misleading). See this ticket's own
    completion notes for why the documented post-completion "shelf" does
    not measurably reproduce here (deltaHeading=0 means the heading-lead
    channel most patch-stack padding operates on is inert for a pure
    straight). Kept as its own always-real assertion so a future
    regression (the shelf reappearing) is caught immediately, and so
    ticket 003 has a concrete, currently-green baseline to avoid breaking."""
    _assert_result(harness_run, "straight_no_command_after_terminal_zero")


# --- 360deg pivot (kPivot, distance=0) ------------------------------------


@pytest.mark.xfail(
    strict=False,
    reason=(
        f"{_ISSUE}: same activation-region accel-spike signature as straight_ramp_bounds, in "
        "the rotational domain -- fixed by sprint 2, not this ticket."
    ),
)
def test_pivot_ramp_bounds(harness_run):
    _assert_result(harness_run, "pivot_ramp_bounds")


def test_pivot_terminal_bounds(harness_run):
    _assert_result(harness_run, "pivot_terminal_bounds")


@pytest.mark.xfail(
    strict=False,
    reason=(
        f"{_ISSUE}: the pivot's own documented sign-changing terminal tail (+-15mm/s) splits "
        "the left wheel's trace into 3 lobes instead of 1 -- fixed by sprint 2's deletion of "
        "the pivot overshoot-lead/overshoot-carry machinery, not this ticket."
    ),
)
def test_pivot_single_lobe_left(harness_run):
    _assert_result(harness_run, "pivot_single_lobe_left")


@pytest.mark.xfail(
    strict=False,
    reason=(
        f"{_ISSUE}: same sign-changing terminal-tail signature as pivot_single_lobe_left, "
        "mirrored on the right wheel -- fixed by sprint 2, not this ticket."
    ),
)
def test_pivot_single_lobe_right(harness_run):
    _assert_result(harness_run, "pivot_single_lobe_right")


@pytest.mark.xfail(
    strict=False,
    reason=(
        f"{_ISSUE}: depends on pivot_single_lobe_left/right both being exactly 1 lobe each, "
        "which today's patch stack does not produce -- see those two checks' own reasons."
    ),
)
def test_pivot_lobes_opposite_sign(harness_run):
    _assert_result(harness_run, "pivot_lobes_opposite_sign")


def test_pivot_no_command_after_terminal_zero(harness_run):
    """See test_straight_no_command_after_terminal_zero's own docstring --
    same honest currently-passing status here too, not xfailed."""
    _assert_result(harness_run, "pivot_no_command_after_terminal_zero")


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
