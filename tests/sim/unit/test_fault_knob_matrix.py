"""tests/sim/unit/test_fault_knob_matrix.py -- ticket 100-010 (Tier-1
fault-knob matrix + lag-on validation, SUC-012).

Exercises the sim's existing fault knobs (`Sim.set_motor_lag`/`set_enc_slip`/
`set_enc_scale_error`/`set_stiction`/`set_trackwidth`/
`set_body_rotational_scrub`/`set_body_linear_scrub`, `tests/_infra/sim/
firmware.py`) against `source/drive/` through the now-live wire adapter
(tickets 100-007/100-008 -- every segment below is admitted and executed via
a real `segment` `CommandEnvelope`, never `sim.post_segment()`'s
admission-bypassing test backdoor). No `source/` change is expected or made
by this ticket -- see the ticket's own Implementation Plan ("pure
test-writing against the now-live adapter"); the Verify section of this
ticket's own instructions requires escalating (not silently patching) any
real defect a fault scenario surfaces.

Design reference: the issue's own "Testing: the four-tier ladder" section
(`clasi/sprints/100-.../issues/motion-stack-v2-....md`, "Tier 1 -- firmware
sim" paragraph) names this matrix's exact shape almost verbatim:
"enc_slip/scale -> tracker convergence vs true_pose; stiction -> terminal
walk-in, no premature DONE, no reversal; trackwidth error -> cross-gain
corrects radius; infeasible asks -> typed ERR, queue untouched." -- and the
"Control laws and numbers" section supplies the control-law table this file
cross-references in comments (k_theta=6.0, k_c=1.5e-5 rad/mm^2, k_s=2.0,
trimVMax=120mm/s carried at tier 0 / 60mm/s in this sim's own tighter
plant-capacity-scaled config -- `tests/_infra/sim/sim_api.cpp`'s
`defaultSimMotionConfig()` -- and the terminal walk-in band: "outside ->
clamp(k_s*e_along, 50mm/s stiction floor, 100mm/s), never negative;
overshot -> 0.0f").

-- MANDATORY motor_lag default (2026-07-11 false-green cross-reference) --

Every tracker/replan scenario in this file runs with `motor_lag` in the
120-140ms hardware-realistic band (`_MANDATORY_MOTOR_LAG` below, applied by
every call to `_apply_faults()` unless a caller explicitly overrides it --
no test in this file does). This is not a stylistic default: on
2026-07-11, a v2 validation pass that ran exclusively at the sim's
zero-lag default went green while the same behavior false-tripped on real
hardware (see `.clasi/knowledge/` and `tests/sim/unit/test_bare_loop_move_
and_tlm.py`'s own "2026-07-11" MOVE-200 sim-plant-gain-calibration note for
a related same-day fix) -- zero actuation lag is not a realistic plant
condition, and validating exclusively against it hides exactly the
lag-dependent failure modes (coast-past-target, replan-timing races,
terminal-walk-in overshoot) this matrix exists to catch. The sim's
zero-lag DEFAULT path is EXPLICITLY EXCLUDED from every tracker/replan
scenario here -- it is reserved for golden-TLM bit-exactness comparison
only (`tests/sim/unit/test_tlm_frame.py` and friends), a determinism
concern orthogonal to this file's own control-accuracy-under-fault concern,
and no test below ever omits `motor_lag` or sets it below 120ms.
`test_every_scenario_runs_with_hardware_realistic_motor_lag` (bottom of
this file) makes that mechanically, grep/review-verifiable: it parses this
file's OWN source and fails if any `set_motor_lag(` call ever appears with
an argument outside [120, 140].

-- Matrix structure --

5 fault knobs (motor_lag / enc_slip / stiction / trackwidth / scrub) x 2
segment kinds (a genuine curved arc, and a pivot) = 10 base-matrix cases
(`test_fault_matrix_admits_and_converges`), each proving: the segment is
ADMITTED over the wire, the drivetrain reaches a terminal (idle) state
within a generous wall-clock budget (the concrete "no hang" proxy every
case checks), and `true_pose` lands within a fault-tolerant-but-bounded
distance of the ideal end pose (no NaN, no unbounded divergence).

On top of that base coverage, four DEEPER checks -- one per ticket
100-010 acceptance-criteria bullet that names a *specific* mechanism, not
just "the matrix ran":
  - enc_slip / enc_scale_error -> checked against `sim.true_pose()`
    (Hal::PhysicsWorld's own ground truth), NEVER any fused/EKF estimate --
    this Sim ctypes ABI does not even expose a `fused_pose()` accessor
    (`tests/_infra/sim/firmware.py`'s full ABI surface has no such method),
    so this is not just a convention followed here but structurally the
    only pose accessor available for this purpose.
  - stiction -> a dedicated no-reversal regression (mirroring ticket
    100-005's own `scenarioTerminalWalkInBandsNeverNegative`, now through
    the full adapter+plant stack) plus a "not premature" DONE check.
  - trackwidth error -> cross-gain (k_c) bounds the resulting radius error
    (does not let it scale with the mismatch).
  - an infeasible ask (Drive::Verdict::EXIT_UNREACHABLE) under ACTIVE fault
    conditions still replies a typed ERR_RANGE with the queue untouched --
    proving fault knobs never turn a clean rejection into a hang or a
    silent wrong answer.
"""
from __future__ import annotations

import math
import pathlib
import re

import pytest

from _binary_envelope import ERR_RANGE, send_segment
from robot_radio.robot import legacy_translate

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Mandatory motor_lag default -- see module docstring's "2026-07-11
# false-green cross-reference" section. Every _apply_faults() call uses
# this unless explicitly overridden (no test in this file overrides it).
_MANDATORY_MOTOR_LAG = 130.0   # [ms] -- within the issue's 120-140ms band

_NOMINAL_TRACKWIDTH = 128.0   # [mm] -- Hal::PhysicsWorld::kDefaultTrackwidth,
                               # matches sim_api.cpp's defaultSimDrivetrainConfig()

ARC_LENGTH = 500.0                        # [mm] -- a genuine curved arc
ARC_DELTA_HEADING = math.radians(60.0)    # [rad]
PIVOT_DELTA_HEADING = math.radians(90.0)  # [rad]
STRAIGHT_LENGTH = 400.0                   # [mm] -- zero curvature: "a forward segment"


def _arc_goal():
    return legacy_translate.segment_for_seg(arc_length=ARC_LENGTH, delta_heading=ARC_DELTA_HEADING)


def _pivot_goal():
    return legacy_translate.segment_for_seg(delta_heading=PIVOT_DELTA_HEADING)


def _straight_goal():
    return legacy_translate.segment_for_seg(arc_length=STRAIGHT_LENGTH)


def _ideal_end_pose(arc_length: float, delta_heading: float,
                     start: tuple[float, float, float] = (0.0, 0.0, 0.0),
                     ) -> tuple[float, float, float]:
    """Ideal world-frame end pose after ONE constant-curvature primitive,
    starting at `start` -- transcribed independently from plain
    constant-curvature-arc geometry (never re-derived from source/drive/'s
    own implementation), mirroring test_drive_cutover_end_pose.py's own
    identically-named/shaped helper (each live-adapter test file
    transcribes this once, rather than importing another test module's
    private helper -- test_tour_closure.py's `_ideal_tour_poses()` is the
    same precedent)."""
    x0, y0, h0 = start
    if abs(delta_heading) < 1e-9:
        dx, dy = arc_length, 0.0
    elif abs(arc_length) < 1e-9:
        dx, dy = 0.0, 0.0
    else:
        radius = arc_length / delta_heading
        dx = radius * math.sin(delta_heading)
        dy = radius * (1.0 - math.cos(delta_heading))
    x = x0 + dx * math.cos(h0) - dy * math.sin(h0)
    y = y0 + dx * math.sin(h0) + dy * math.cos(h0)
    h = h0 + delta_heading
    return x, y, h


def _heading_error(measured: float, ideal: float) -> float:
    """Wrapped |measured - ideal|, radians, in [0, pi]."""
    return abs(math.atan2(math.sin(measured - ideal), math.cos(measured - ideal)))


def _apply_faults(sim, *, motor_lag: float | None = _MANDATORY_MOTOR_LAG,
                   enc_slip: float | None = None, enc_scale: float | None = None,
                   stiction: float | None = None, trackwidth: float | None = None,
                   rot_scrub: float | None = None, lin_scrub: float | None = None) -> None:
    """Apply zero or more fault knobs to `sim`, side=2 (both wheels) for
    every per-wheel knob -- symmetric injection, so a scenario's own
    divergence is attributable to the ONE knob under test rather than a
    confound of asymmetric left/right bias. `motor_lag` defaults to the
    MANDATORY 120-140ms band (see module docstring) and is applied on
    every call unless a caller explicitly passes `motor_lag=None`
    (no test in this file does)."""
    if motor_lag is not None:
        sim.set_motor_lag(2, motor_lag)
    if enc_slip is not None:
        sim.set_enc_slip(2, enc_slip)
    if enc_scale is not None:
        sim.set_enc_scale_error(2, enc_scale)
    if stiction is not None:
        sim.set_stiction(2, stiction)
    if trackwidth is not None:
        sim.set_trackwidth(trackwidth)
    if rot_scrub is not None:
        sim.set_body_rotational_scrub(rot_scrub)
    if lin_scrub is not None:
        sim.set_body_linear_scrub(lin_scrub)


def _run_and_settle(sim, seg, seconds: float, step: int = 24):
    """Send one primitive `segment` over the wire (real admission, not
    `sim.post_segment()`'s bypass), then tick until the drivetrain reports
    idle (`sim.active()` false) or `seconds` elapses. Returns
    `(idle_at, pose_at_idle)`: `idle_at` is None if the segment NEVER
    reports idle within the budget -- the concrete, wall-clock-bounded
    "did this fault cause a hang" proxy every case in this file checks;
    `pose_at_idle` is `sim.true_pose()` sampled at the EXACT tick idle
    first became true (None if idle was never reached), used by the
    stiction "not premature" check below."""
    reply = send_segment(sim, seg)
    assert reply.WhichOneof("body") == "ok", f"segment not admitted: {reply}"
    idle_at = None
    pose_at_idle = None
    for i in range(int(seconds * 1000 / step)):
        sim.tick_for(step)
        if idle_at is None and not sim.active():
            idle_at = (i + 1) * step / 1000.0
            pose_at_idle = sim.true_pose()
    return idle_at, pose_at_idle


def _run_and_check_no_reverse_creep(sim, seconds: float, step: int = 24, floor: float = 15.0):
    """Ticks `sim` for `seconds`, tracking each wheel's MEASURED velocity
    (`sim.vel()`) sign once it first becomes substantial (|v| > 20mm/s),
    asserting it never flips past a small settle-noise `floor` (mm/s) in
    the opposite direction afterward. Identical contract/shape to
    test_bare_loop_move_and_tlm.py's own `_run_and_check_no_reverse_creep`
    (that file's own precedent for "reversal, checked through the full
    live-adapter+plant stack" -- deliberately velocity-based, not a raw
    PWM/duty read: SimMotor's own low-level velocity PID legitimately
    commands small-magnitude NEGATIVE duty while actively braking a
    still-coasting wheel down toward a lower/zero target -- confirmed by
    direct experiment against this exact plant/adapter pairing, duty dips
    of a few percent with stiction active that never once show up as an
    actual measured reversal -- so a duty-level check would false-positive
    on ordinary terminal braking chatter the reversal-dwell armor already
    absorbs, not the wedge-hazard "wheel setpoint flips sign" this ticket's
    AC is actually about). Returns `max_abs_l, max_abs_r`."""
    ticks = int(seconds * 1000 / step)
    sign_l = sign_r = 0
    max_abs_l = max_abs_r = 0.0
    for _ in range(ticks):
        sim.tick_for(step)
        vel_l, vel_r = sim.vel()
        max_abs_l = max(max_abs_l, abs(vel_l))
        max_abs_r = max(max_abs_r, abs(vel_r))
        if sign_l == 0 and abs(vel_l) > 20.0:
            sign_l = 1 if vel_l > 0 else -1
        if sign_r == 0 and abs(vel_r) > 20.0:
            sign_r = 1 if vel_r > 0 else -1
        if sign_l == 1:
            assert vel_l > -floor, f"left wheel reversed: {vel_l} mm/s"
        if sign_r == 1:
            assert vel_r > -floor, f"right wheel reversed: {vel_r} mm/s"
    return max_abs_l, max_abs_r


# ---------------------------------------------------------------------------
# 1. Base matrix -- every fault knob x {arc, pivot}, through the live
#    adapter. AC: "Matrix covers motor_lag(120-140ms)/enc_slip/stiction/
#    trackwidth-error/scrub, each run against at least one arc AND one
#    pivot segment through the live adapter."
# ---------------------------------------------------------------------------

_MATRIX_SEGMENTS = {
    "arc": (_arc_goal, ARC_LENGTH, ARC_DELTA_HEADING, 10.0),
    "pivot": (_pivot_goal, 0.0, PIVOT_DELTA_HEADING, 6.0),
}

# Fault magnitudes below are the SAME order of magnitude already validated
# (by direct experiment against this exact plant/adapter pairing) to
# converge cleanly with motor_lag on -- see this file's own dedicated
# deeper tests below for the numbers each one produces. motor_lag's own
# row applies no ADDITIONAL knob (motor_lag is already the mandatory
# default on every row -- see _apply_faults()); it is still its own row so
# the matrix literally names all 5 knobs the AC lists, including the one
# that is a baseline-with-lag-only condition.
_MATRIX_FAULTS = {
    "motor_lag": {},
    "enc_slip": {"enc_slip": 0.12},
    "stiction": {"stiction": 20.0},
    "trackwidth": {"trackwidth": 158.0},
    "scrub": {"rot_scrub": 0.85, "lin_scrub": 0.9},
}


@pytest.mark.parametrize("segment_kind", sorted(_MATRIX_SEGMENTS))
@pytest.mark.parametrize("fault_name", sorted(_MATRIX_FAULTS))
def test_fault_matrix_admits_and_converges(sim, fault_name, segment_kind):
    goal_fn, arc_length, delta_heading, seconds = _MATRIX_SEGMENTS[segment_kind]
    _apply_faults(sim, **_MATRIX_FAULTS[fault_name])

    idle_at, _ = _run_and_settle(sim, goal_fn(), seconds=seconds)
    assert idle_at is not None, (
        f"{fault_name}/{segment_kind}: never reported idle within {seconds}s -- a hang, "
        "not a fault the drivetrain recovered from"
    )

    ideal = _ideal_end_pose(arc_length, delta_heading)
    x, y, h = sim.true_pose()
    assert math.isfinite(x) and math.isfinite(y) and math.isfinite(h), (
        f"{fault_name}/{segment_kind}: non-finite true_pose ({x}, {y}, {h})"
    )
    pos_err = math.hypot(x - ideal[0], y - ideal[1])
    h_err = _heading_error(h, ideal[2])
    # Generous, fault-tolerant-but-bounded envelope: test_drive_cutover_end_
    # pose.py's own zero-fault tolerances are 20-35mm/3-5deg; every single-
    # knob combination in this matrix (see the dedicated deeper tests below,
    # and this file's own exploratory measurements) lands well inside 70mm/
    # 8deg with motor_lag on -- this bound exists to catch genuine
    # divergence/runaway, not to be a tight accuracy gate (that is ticket
    # 011's bench-grid job).
    assert pos_err < 70.0, f"{fault_name}/{segment_kind}: pos_err {pos_err:.1f}mm too large"
    assert h_err < math.radians(8.0), (
        f"{fault_name}/{segment_kind}: h_err {math.degrees(h_err):.1f}deg too large"
    )


# ---------------------------------------------------------------------------
# 2. enc_slip / enc_scale_error -> checked against sim.true_pose() (NEVER
#    any fused/EKF estimate -- see module docstring; there is no
#    fused_pose() accessor on this Sim ABI at all).
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("segment_kind", sorted(_MATRIX_SEGMENTS))
def test_enc_slip_true_pose_convergence(sim, segment_kind):
    """15% encoder slip on both wheels (Hal::PhysicsWorld's REPORTED-
    accumulator-only error model -- physics_world.h's own file header:
    "the true/ground-truth accumulator and chassis pose are unaffected").
    Measured against this exact plant/adapter pairing: encoder-only dead
    reckoning would diverge tens of mm from true pose under this much slip
    (test_otos_fusion_live.py's own ~50mm-at-2s finding for a comparable
    injected slip), but the live SimOdometer (OTOS)/EKF fusion the sim
    always runs pulls the CONTROL LOOP's own state estimate back toward
    true pose -- this test's job is to confirm that correction survives
    all the way through a real tracked segment (not just a raw drive), by
    checking the ONE observable that cannot be fooled by a corrupted
    internal belief: sim.true_pose(), Hal::PhysicsWorld's own ground
    truth. It NEVER reads any fused/EKF pose estimate."""
    goal_fn, arc_length, delta_heading, seconds = _MATRIX_SEGMENTS[segment_kind]
    _apply_faults(sim, enc_slip=0.15)

    idle_at, _ = _run_and_settle(sim, goal_fn(), seconds=seconds)
    assert idle_at is not None, f"enc_slip/{segment_kind}: never settled -- a hang"

    ideal = _ideal_end_pose(arc_length, delta_heading)
    x, y, h = sim.true_pose()
    pos_err = math.hypot(x - ideal[0], y - ideal[1])
    h_err = _heading_error(h, ideal[2])
    # Measured (this exact pairing, motor_lag=130 + enc_slip=0.15): arc
    # pos_err ~31.6mm/h_err ~1.84deg, pivot pos_err ~0mm/h_err ~0.24deg --
    # 70mm/20mm and 6deg leave ample margin while still failing hard on a
    # genuine (unfused, unbounded) divergence.
    pos_bound = 70.0 if segment_kind == "arc" else 20.0
    assert pos_err < pos_bound, f"enc_slip/{segment_kind}: true_pose pos_err {pos_err:.1f}mm"
    assert h_err < math.radians(6.0), (
        f"enc_slip/{segment_kind}: true_pose h_err {math.degrees(h_err):.1f}deg"
    )


@pytest.mark.parametrize("segment_kind", sorted(_MATRIX_SEGMENTS))
def test_enc_scale_error_true_pose_convergence(sim, segment_kind):
    """+/-10% encoder SCALE error (Hal::PhysicsWorld's `encScaleErrL_/R_`
    -- a fixed multiplicative over/under-report, distinct from slip's
    fractional-of-motion-lost model, but the SAME "reported accumulator
    only, true chassis pose unaffected" contract) -- checked against
    sim.true_pose(), same rationale as test_enc_slip_true_pose_
    convergence above: this test NEVER reads any fused/EKF pose."""
    goal_fn, arc_length, delta_heading, seconds = _MATRIX_SEGMENTS[segment_kind]
    _apply_faults(sim, enc_scale=-0.1)

    idle_at, _ = _run_and_settle(sim, goal_fn(), seconds=seconds)
    assert idle_at is not None, f"enc_scale/{segment_kind}: never settled -- a hang"

    ideal = _ideal_end_pose(arc_length, delta_heading)
    x, y, h = sim.true_pose()
    pos_err = math.hypot(x - ideal[0], y - ideal[1])
    h_err = _heading_error(h, ideal[2])
    # Measured (this exact pairing, motor_lag=130 + enc_scale=-0.1): arc
    # pos_err ~29mm/h_err ~1.7deg, pivot near 0mm/<1deg.
    pos_bound = 70.0 if segment_kind == "arc" else 20.0
    assert pos_err < pos_bound, f"enc_scale/{segment_kind}: true_pose pos_err {pos_err:.1f}mm"
    assert h_err < math.radians(6.0), (
        f"enc_scale/{segment_kind}: true_pose h_err {math.degrees(h_err):.1f}deg"
    )


# ---------------------------------------------------------------------------
# 3. stiction -> terminal walk-in: no premature DONE_STOP, no reversal.
#    Mirrors ticket 100-005's own scenarioTerminalWalkInBandsNeverNegative
#    (drive_policy_harness.cpp's dedicated no-reversal regression), now
#    through the full adapter+plant stack, on a FORWARD (zero-curvature)
#    segment -- a genuine curved arc's own per-wheel differential (a
#    SLOWER inner wheel legitimately nears zero, and the tracker's own
#    heading trim is unclamped near a stop) is a different concern from
#    the along-track "never negative" contract this check is about; see
#    policy.h's own "no per-wheel sign reversal at NONZERO joint speed"
#    scoping (drivetrain.cpp) for why the guarantee under test here is
#    specifically a forward/zero-curvature one.
# ---------------------------------------------------------------------------

def test_stiction_terminal_walk_in_no_reversal_no_premature_done(sim):
    _apply_faults(sim, stiction=20.0)

    idle_at, pose_at_idle = _run_and_settle(sim, _straight_goal(), seconds=10.0)
    assert idle_at is not None, "stiction: never settled -- a hang"
    # "Not premature": DONE_STOP must not fire before the robot has
    # actually covered most of the commanded distance. Measured (this
    # pairing, stiction 15-45): idle_at lands ~2.47-2.54s for a 400mm
    # segment under 130ms lag -- an instant/degenerate "done" would be
    # far below this floor.
    assert idle_at > 1.0, f"stiction: idle reported suspiciously early ({idle_at}s) -- premature DONE_STOP?"
    ideal = _ideal_end_pose(STRAIGHT_LENGTH, 0.0)
    pos_err_at_idle = math.hypot(pose_at_idle[0] - ideal[0], pose_at_idle[1] - ideal[1])
    # Measured ~11-26mm across stiction 15-45mm/130ms lag; 70mm mirrors the
    # SAME margin test_drive_closed_loop.py's own lag-coast tolerance uses
    # (60mm) with extra headroom for stiction's own additional walk-in lag.
    assert pos_err_at_idle < 70.0, (
        f"stiction: DONE_STOP fired {pos_err_at_idle:.1f}mm from goal -- premature completion"
    )

    # Dedicated no-reversal regression (the AC's own "mirroring ticket
    # 005's own terminal-machine regression test" instruction): re-drive
    # the SAME fault-configured sim through a full settle window, this
    # time tracking measured wheel velocity for any reverse-creep.
    max_l, max_r = _run_and_check_no_reverse_creep(sim, seconds=4.0)
    # (max_l/max_r are near-zero here -- the segment already settled
    # above; the call's job is purely the per-tick no-reversal assertion.)
    assert max_l < 20.0 and max_r < 20.0, "stiction: wheel still moving well after DONE_STOP settled"


def test_stiction_terminal_walk_in_no_reversal_fresh_run(sim):
    """Companion to the settle-tail check above: a FRESH stiction-
    configured sim, tracking no-reversal from the very first tick through
    natural completion (not just the settle tail) -- covers the walk-in
    APPROACH itself (the "outside" floor/ceiling-clamped band), not only
    the final dwell."""
    _apply_faults(sim, stiction=20.0)
    reply = send_segment(sim, _straight_goal())
    assert reply.WhichOneof("body") == "ok"

    max_l, max_r = _run_and_check_no_reverse_creep(sim, seconds=8.0)
    assert max_l > 50.0 and max_r > 50.0, "stiction: segment never genuinely drove"


# ---------------------------------------------------------------------------
# 4. trackwidth error -> cross-gain (k_c) bounds the resulting radius
#    error. Control law (issue's own table): omega_cmd = omega_ref +
#    clamp(k_theta*e_theta + k_c*v_ref*e_cross, +/-trimOmegaMax), k_c =
#    1.5e-5 rad/mm^2 (sim_api.cpp's own defaultSimMotionConfig() --
#    track_k_cross). A trackwidth MISMATCH between the firmware's
#    configured value (admission/planning, bb.drivetrainConfig.trackwidth
#    -- unaffected by this knob) and the plant's own PHYSICAL trackwidth
#    (Hal::PhysicsWorld, this knob) creates a persistent radius/cross-track
#    error a pure open-loop (uncorrected) execution would let scale with
#    the mismatch; the tracker's cross-track trim term is what is supposed
#    to bound it instead.
# ---------------------------------------------------------------------------

def test_trackwidth_error_cross_gain_bounds_radius_error(sim):
    from firmware import Sim  # noqa: PLC0415 -- tests/sim/conftest.py's own `sim`
    # fixture "import after build_lib runs" precedent; by the time this test
    # body runs (it takes `sim` as a fixture param), build_lib has already
    # executed and _SIM_INFRA_DIR is already on sys.path.

    ideal = _ideal_end_pose(ARC_LENGTH, ARC_DELTA_HEADING)

    # Baseline: plant trackwidth == firmware's own configured value (no
    # mismatch) -- a SEPARATE Sim instance (the `sim` fixture only
    # provides one per test; a second, independently-torn-down instance,
    # imported lazily the same way tests/sim/conftest.py's own `sim`
    # fixture does -- "import after build_lib runs" -- is the direct way
    # to get a true zero-mismatch control run in the SAME test, rather
    # than a separately-seeded baseline number that could drift out of
    # sync with this file's own segment/fault shape over time).
    with Sim() as baseline_sim:
        _apply_faults(baseline_sim, trackwidth=_NOMINAL_TRACKWIDTH)
        idle_at, _ = _run_and_settle(baseline_sim, _arc_goal(), seconds=10.0)
        assert idle_at is not None, "trackwidth baseline: never settled -- a hang"
        bx, by, bh = baseline_sim.true_pose()
        baseline_pos_err = math.hypot(bx - ideal[0], by - ideal[1])

    # Mismatched: plant trackwidth ~23% wider than the firmware assumes.
    _apply_faults(sim, trackwidth=158.0)
    idle_at, _ = _run_and_settle(sim, _arc_goal(), seconds=10.0)
    assert idle_at is not None, "trackwidth mismatch: never settled -- a hang"
    x, y, h = sim.true_pose()
    pos_err = math.hypot(x - ideal[0], y - ideal[1])
    h_err = _heading_error(h, ideal[2])

    # Measured (this exact pairing, motor_lag=130): baseline pos_err
    # ~24.7mm; trackwidth in {98, 108, 148, 158}mm (vs nominal 128mm, up to
    # +/-23%) all land ~22-27mm -- essentially FLAT despite the mismatch,
    # which is exactly what a working cross-gain correction looks like (an
    # UNCORRECTED mismatch of this size would scale the radius error with
    # the trackwidth delta, not hold it near the zero-mismatch baseline).
    # 40mm of headroom above baseline is generous margin while still
    # failing hard if the correction were disabled and the error scaled.
    assert pos_err < baseline_pos_err + 40.0, (
        f"trackwidth error: pos_err {pos_err:.1f}mm grew well past the "
        f"{baseline_pos_err:.1f}mm zero-mismatch baseline -- cross-gain "
        "does not appear to be bounding the radius error"
    )
    assert pos_err < 70.0, f"trackwidth error: absolute pos_err {pos_err:.1f}mm too large"
    assert h_err < math.radians(8.0), f"trackwidth error: h_err {math.degrees(h_err):.1f}deg too large"


# ---------------------------------------------------------------------------
# 5. Infeasible ask, under ACTIVE fault conditions -> typed ERR_RANGE,
#    queue/bb.chainTail untouched, no hang, and the queue is genuinely
#    still usable afterward (not just superficially "untouched").
# ---------------------------------------------------------------------------

def test_infeasible_ask_under_fault_conditions_typed_err_queue_untouched(sim):
    """Drive::Verdict::EXIT_UNREACHABLE (drivetrain.cpp's own admit()): a
    50mm arc cannot reach a 400mm/s exit speed from rest under 800mm/s^2
    accel (v_max = sqrt(2*800*50) ~= 283mm/s < 400 -- the SAME numbers
    drive_admission_harness.cpp's own scenarioExitUnreachable() uses).
    admit() is a pure function of Limits + the FIRMWARE's configured
    trackwidth (bb.drivetrainConfig.trackwidth, untouched by
    sim.set_trackwidth()'s plant-only knob) -- fault knobs should not be
    able to change whether this is admitted, but this test's actual point
    is the negative-space guarantee: an infeasible ask must still fail
    CLEANLY (typed ERR, no hang, queue untouched) even with every other
    knob in this matrix simultaneously active, not just in the pristine
    zero-fault configuration test_binary_channel.py's own
    test_binary_segment_infeasible_admission_typed_err_queue_untouched
    already covers."""
    _apply_faults(sim, enc_slip=0.10, stiction=15.0, trackwidth=140.0)

    tail_before = sim.chain_tail()
    seg = legacy_translate.segment_for_seg(arc_length=50.0, exit_speed=400.0)
    # send_segment() routes synchronously (armor -> sim.command_on() ->
    # dearmor, tests/_infra/sim/firmware.py's own dt=0 synchronous-command
    # trick) -- this call RETURNING AT ALL, with a decoded reply, is itself
    # the "no hang" proof; admit()'s own rejection path (drivetrain.cpp)
    # never touches bb.segmentIn/bb.chainTail regardless of any trailing
    # tick, so there is no separate no-tick variant needed here.
    reply = send_segment(sim, seg)

    assert reply.WhichOneof("body") == "err", f"expected a typed ERR, got: {reply}"
    assert reply.err.code == ERR_RANGE
    # Verdict::EXIT_UNREACHABLE is enumerator index 1 (OK=0, EXIT_UNREACHABLE=1
    # -- source/drive/drivetrain.h).
    assert reply.err.field == 1

    assert sim.peek_segment_in(0) is None, "an admission rejection must leave the queue untouched"
    tail_after = sim.chain_tail()
    assert tail_after == tail_before, "an admission rejection must leave bb.chainTail untouched"

    # Silent-wrong-answer guard: the queue is genuinely still USABLE, not
    # just superficially untouched -- a subsequent FEASIBLE segment must
    # still be admitted and drive normally.
    reply2 = send_segment(sim, legacy_translate.segment_for_seg(arc_length=300.0))
    assert reply2.WhichOneof("body") == "ok", (
        f"a feasible segment after the rejection was not admitted: {reply2}"
    )


# ---------------------------------------------------------------------------
# 6. Mechanical, grep/review-verifiable regression guard: EVERY
#    set_motor_lag() call in this file uses a value inside the mandatory
#    120-140ms hardware-realistic band -- see module docstring's own
#    "MANDATORY motor_lag default" section for why. Parses this file's OWN
#    source rather than re-running every scenario, so it fails fast and
#    points directly at the offending line if a future edit ever
#    introduces a zero/low-lag tracker scenario here.
# ---------------------------------------------------------------------------

def test_every_scenario_runs_with_hardware_realistic_motor_lag():
    """Mechanically enforces the module docstring's own "MANDATORY
    motor_lag default" section by parsing this file's OWN source (rather
    than re-running every scenario): every fault-knob application in this
    file funnels through `_apply_faults()`, whose own `motor_lag` keyword
    defaults to `_MANDATORY_MOTOR_LAG` -- so the two invariants that
    together guarantee "every tracker/replan scenario here runs with
    120-140ms lag" are (1) that default itself is in-band, and (2) no test
    ever bypasses the shared helper (a second `set_motor_lag(` call site
    outside `_apply_faults()`'s own body) or overrides it out of band."""
    assert 120.0 <= _MANDATORY_MOTOR_LAG <= 140.0, (
        "_MANDATORY_MOTOR_LAG itself must be in the 120-140ms hardware-realistic band"
    )

    # Scan only the source ABOVE this function's own `def` line: every
    # real _apply_faults()/set_motor_lag() call site in this file lives in
    # an earlier test (this regression guard is deliberately the LAST test
    # defined), and excluding this function's own body sidesteps a
    # self-reference trap -- this docstring and the error strings below
    # legitimately quote `_apply_faults(...)`/`set_motor_lag(` example text
    # that would otherwise inflate their own counts.
    full_source = pathlib.Path(__file__).read_text()
    marker = "def test_every_scenario_runs_with_hardware_realistic_motor_lag"
    boundary = full_source.index(marker)
    source = full_source[:boundary]

    call_sites = re.findall(r"\.set_motor_lag\(", source)
    assert len(call_sites) == 1, (
        f"expected exactly ONE set_motor_lag() call site (inside _apply_faults()), "
        f"found {len(call_sites)} -- a scenario may be bypassing the shared helper "
        "and its mandatory 120-140ms default"
    )

    # Every _apply_faults() call site that overrides motor_lag= explicitly
    # (there are none in this file today -- every scenario relies on
    # _MANDATORY_MOTOR_LAG's own default) must still land in-band, never
    # None/zero-lag -- catches a future edit that tries to sneak a
    # zero-lag tracker scenario in via an explicit override instead of
    # bypassing the helper outright.
    overrides = re.findall(r"_apply_faults\([^)]*\bmotor_lag\s*=\s*([0-9.]+|None)", source)
    for raw in overrides:
        assert raw != "None", (
            "an _apply_faults(..., motor_lag=None) call disables the mandatory lag default"
        )
        value = float(raw)
        assert 120.0 <= value <= 140.0, (
            f"motor_lag override {value} outside the mandatory 120-140ms band -- "
            "2026-07-11 false-green: validating at zero lag went green in sim and "
            "false-tripped on real hardware. The sim's zero-lag default is reserved "
            "for golden-TLM bit-exactness tests only, never a tracker/replan scenario "
            "like the ones in this file."
        )


if __name__ == "__main__":
    import sys

    sys.exit(pytest.main([__file__, "-v"]))
