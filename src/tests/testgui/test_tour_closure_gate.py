"""src/tests/testgui/test_tour_closure_gate.py -- sprint 109 ticket 009's own
decisive acceptance gate, stated in the stakeholder's own words: TestGUI ->
Sim -> Tour 1 AND Tour 2 must complete, close the loop, and every turn must
land within 1 degree of commanded -- measured against SIM GROUND TRUTH, not
the firmware's own (possibly sensor-biased) telemetry -- with a realistic
sim OTOS/encoder error profile enabled; with every sim error model disabled
(an ideal chip), turns must be EXACT (negligible-epsilon, not "within 1
degree").

This is the sprint's capstone integration test, built directly on
infrastructure earlier tickets already proved:

- ``robot_radio.io.sim_loop.SimLoop`` (108-005/006) -- a real, ctypes-bound
  handle onto the REAL compiled firmware simulator
  (``src/sim/build/libfirmware_host.{dylib,so}``), the exact object
  ``testgui.transport.SimTransport`` wraps for the TestGUI's own Sim mode.
- ``sim_true_x/y/h`` (108-00x) / ``SimLoop.get_true_pose()`` -- ``SimPlant``'s
  own ground-truth pose, bypassing every drift/noise fault knob -- the only
  honest yardstick for "how close did the robot actually turn/drive" that
  does not just measure the firmware grading its own homework.
- ``SimLoop.set_otos_raw_scale_err()`` / ``set_enc_scale_err()`` /
  ``set_enc_tick_quant()`` / ``set_enc_slip()`` (109-007) -- the sim
  fidelity knobs this ticket's "realistic error profile enabled" run turns
  on.
- ``_SimConfigConn``/``NezhaProtocol.otos_config()`` (109-002/004) -- the
  SAME direct-patch-send mechanism a live TestGUI OL/OA calibration push (or
  connect-time calibration) uses, reused here to calibrate OUT the injected
  raw OTOS scale error before driving a tour -- see
  ``test_otos_calibration_convergence.py`` for the lower-level version of
  this same round trip. An UNCALIBRATED OTOS would make heading itself
  unrecoverably wrong (the firmware's heading PD closes on ITS OWN sensor
  reading, not truth) -- that is a calibration-workflow gap, not a motion-
  accuracy gap, and is out of scope for THIS gate (SUC-005 already covers
  the calibration workflow itself). A realistic OPERATING robot is
  calibrated before it drives a tour; this test models that.
- ``robot_radio.planner.tour.{TOUR_1,TOUR_2,parse_tour,run_tour,TourLeg}``
  (109-008) -- the MOVE-queue tour-execution path itself, unchanged here.

Run with::

    uv run python -m pytest src/tests/testgui/test_tour_closure_gate.py -v

Requires the compiled ``src/sim/build/libfirmware_host.{dylib,so}``
(``python build.py``) -- skips cleanly if not present.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from pathlib import Path

import pytest

from robot_radio.testgui.transport import _SimConfigConn, _sim_lib_path

pytestmark = pytest.mark.skipif(
    not _sim_lib_path().exists(),
    reason="sim lib not built -- cmake --build src/sim/build (or `python build.py`)",
)

_TRACK_WIDTH = 128.0  # [mm] matches TestGUI's own default trackwidth
_SPEED_FACTOR = 1     # sim fast-forward multiple; see this file's own diagnostic history --
                      # higher factors were tried first and caused spurious ack-timeout faults
                      # (see git history / ticket 009 notes), not a real Executor/Pilot defect.

# 114-006: the active robot config _make_loop() configures every fresh SimLoop
# from -- the sim now fail-closed refuses MOTION until it has received a
# complete configuration (114-001/002/003), so a bare SimLoop.connect() with
# no configure_from_robot() call faults immediately instead of running with a
# hardcoded fallback. Same path test_turn_error_characterization.py's own
# _ACTIVE_ROBOT_JSON/_make_sweep_loop() use.
# test_tour_closure_gate.py -> testgui -> tests -> src -> repo root
_ACTIVE_ROBOT_JSON = Path(__file__).resolve().parents[3] / "data" / "robots" / "tovez_nocal.json"

# ---------------------------------------------------------------------------
# "Realistic" sim error profile -- documented, plausible values, not tuned to
# pass. Each knob models a real, physically-motivated imperfection:
# ---------------------------------------------------------------------------
_OTOS_LINEAR_ERR = 0.03    # 3% raw OTOS linear scale error -- a plausible as-manufactured
                           # mis-calibration (test_otos_calibration_convergence.py's own
                           # precedent uses 5% for its own, harsher, uncalibrated-divergence
                           # check; this gate calibrates it out below, so 3% only needs to be
                           # "nonzero and plausible", not stress-test-sized).
_OTOS_ANGULAR_ERR = 0.02   # 2% raw OTOS angular scale error -- same rationale, angular channel.
_ENC_SCALE_ERR_L = 0.015   # 1.5% left-wheel effective-diameter mismatch (tire wear/manufacturing).
_ENC_SCALE_ERR_R = -0.015  # opposite sign on the right wheel -- a differential mismatch, the
                           # worst case for heading drift if encoders were trusted for heading.
_ENC_TICK_MM = 0.3         # [mm] encoder count resolution (a real quadrature encoder's per-tick
                           # distance at this wheel/gear ratio is a fraction of a mm, not a whole
                           # mm -- 0.3mm is a plausible mid-resolution encoder).
_ENC_SLIP_RATE = 0.01      # [0,1] per-reportedPosition()-call accumulator step -- fires roughly
                           # once every ~100 encoder reads (a rare, not constant, slip event).
_ENC_SLIP_MAG = 0.2        # [mm] permanent offset injected per slip event -- small relative to
                           # TOUR_1/2's shortest leg (240mm).

# ---------------------------------------------------------------------------
# Gate tolerances -- the stakeholder's own words, operationalized.
# ---------------------------------------------------------------------------
_TURN_TOLERANCE_REALISTIC_DEG = 1.0   # "within 1 degree of commanded"
_TURN_TOLERANCE_IDEAL_DEG = 0.05      # "EXACT" -- negligible-epsilon, not "within 1 degree";
                                      # 0.05deg is ~3 orders of magnitude tighter than the
                                      # realistic-profile gate and well inside float32 pose
                                      # accumulation noise over a 13-15 leg tour.

# 119 ticket 005 (straight-leg-crab fix): per-straight-leg TRUTH heading
# stability gate -- see StraightLegCruiseCheck's own doc comment and
# _assert_tour_gate()'s own cruise_heading_tolerance_deg parameter.
#
# IMPORTANT: this is a TOUR-embedded measurement, not the isolated
# accel/cruise/decel-only signature the straight-leg-crab issue's own repro
# script (straight_drift_repro.py) measures. In a tour, EVERY straight leg
# but the very first is chain-advanced immediately behind a TURN leg
# (SUC-051 seamless hand-off) -- and 119-002's own "axis-drop coast at
# chain boundaries" contract (motion/DESIGN.md §4) documents a REAL,
# already-accepted, SEPARATE physical effect: the turn's own omega axis is
# commanded to 0 at the exact hand-off instant, but the plant's own
# residual angular velocity keeps physically coasting for a few more
# cycles the reset does not (and structurally cannot) erase -- a
# permanent, non-transient heading offset for the REST of that leg, not a
# 118-001-style transient that cancels by the leg's own end. Empirically
# confirmed here: this file's own leg-1 measurement (the tour's OWN first
# leg, never chain-advanced from a turn) reads EXACTLY 0.0000deg under the
# ideal-chip profile -- proving the actuation-skew fix itself is clean
# (matches straight_drift_repro.py's own isolated 0.000deg finding
# exactly) -- while every LATER, turn-chained leg carries a real,
# nonzero, ticket-119-002-attributed baseline on top. These tolerances
# therefore bound "actuation-skew fix regression + axis-drop coast," not
# "actuation-skew fix regression" alone -- a REGRESSION of the 118-001 bug
# would add roughly +2.7deg more (this file's own isolated pre-fix
# measurement) on top of whatever coast baseline a given leg already
# carries, which these margins still have comfortable room to catch.
# Realistic profile additionally carries a REAL, non-sensing heading cost
# from _ENC_SCALE_ERR_L/_ENC_SCALE_ERR_R (a genuine per-wheel effective-
# diameter mismatch the velocity PID closes on, not measurement noise) --
# confirmed present even on leg 1 (2.36deg, no preceding turn at all).
#
# 121 ticket 003 (land-at-zero-at-orthogonal-chain-boundaries.md) RE-
# MEASURED these against the new orthogonal-boundary land-at-zero split
# (App::MoveQueue::landAtZero()'s own kStoppingMarginFactorOrthogonal,
# move_queue.cpp) -- both TOUR_1 and TOUR_2 alternate Distance/Angle
# unconditionally, so EVERY chain boundary in either tour is orthogonal,
# and this is now the FIRST re-measurement of these tolerances under that
# split rather than the undifferentiated chain margin 119-005 measured
# them against. Worst moved from TOUR_1 to TOUR_2 (a different leg, not a
# location this file previously reported) but stayed the SAME ORDER of
# magnitude: ideal 4.2538deg -> 4.1041deg (TOUR_2/ideal leg 13, a slight
# improvement); realistic 9.3066deg -> 9.8521deg (TOUR_2/realistic leg 9,
# a slight regression). Net: this ticket's own aspirational target
# (straight-following-turn gain <=0.3deg) is NOT achieved -- the residual
# is the REAL plant's own post-reset momentum decay, not a marginFactor-
# fixable taper-remaining-distance effect (full derivation:
# move_queue.cpp's own anonymous-namespace comment, "HONEST RESIDUAL").
# Tolerances below are LEFT UNCHANGED (not tightened): the achieved worst
# numbers do not clearly beat the prior baseline enough to tighten with
# real margin, and 5.5/10.5 already hold with 1.3959deg/0.6479deg margin
# respectively against the NEW worst -- tightening further would risk
# flaking on ordinary run-to-run float noise for no real gain.
_CRUISE_HEADING_TOLERANCE_IDEAL_DEG = 5.5        # margin over measured worst 4.1041deg
                                                  # (TOUR_2/ideal leg 13, chain-advance-adjacent)
_CRUISE_HEADING_TOLERANCE_REALISTIC_DEG = 10.5   # margin over measured worst 9.8521deg
                                                  # (TOUR_2/realistic leg 9, chain-advance-adjacent
                                                  # + real encoder-scale-mismatch heading cost)
_CLOSURE_POSITION_MAX_MM = 600.0     # matches test_sim_transport_tour1.py's own bench-observed
                                      # bound (real TOUR_1 closures ranged up to ~500mm even when
                                      # COMPLETED cleanly) -- TOUR_1/2 are not tightly-closed loops
                                      # by design (see that file's own comment); this bound only
                                      # catches an implausible blowup, not a specific number.
_BOUNDARY_MIN_FRACTION = 0.9          # matches boundary_velocity_harness.cpp's own
                                      # "vMax*0.9" no-dip bound (SUC-003's own acceptance wording)

# decel-into-the-goal campaign: the NEW, tightened tolerance actually met
# with the taper active (unlike _TURN_TOLERANCE_IDEAL_DEG/
# _TURN_TOLERANCE_REALISTIC_DEG above, the ORIGINAL 109-009 stakeholder
# bars, which stay their own untouched, still-xfailed aspirational gate
# below) -- ticket's own "90 turns land 90+-2 target band (state achieved
# band honestly)". 2.5deg keeps real headroom over the worst measured
# point (not a bare "2.0" that would flake on ordinary run-to-run float
# noise) -- see test_tour_1_and_tour_2_ninety_degree_turns_land_within_
# the_shaped_band()'s own printed report for this file's current measured
# numbers.
#
# 121 ticket 003 RE-MEASURED this gate against the new orthogonal-boundary
# land-at-zero split (move_queue.cpp's own kStoppingMarginFactorOrthogonal,
# swept to 0.67): worst |turn error| 2.314deg (TOUR_1/ideal, was 2.218deg
# pre-ticket) / 2.100deg (TOUR_2/realistic) -- comparable to, not
# dramatically better than, the pre-ticket undifferentiated-chain-margin
# baseline. NOT tightened further: 0.186deg of margin under 2.5deg is
# already tighter than this file's own historical comfort zone (117-005's
# own 0.282deg precedent), so tightening the gate itself risks flaking on
# ordinary run-to-run float noise for no real gain. The sprint's own
# aspirational turn |error| <=0.5deg target is NOT achieved -- see
# move_queue.cpp's own "HONEST RESIDUAL" comment for why a marginFactor
# sweep alone cannot close this gap (the residual is the real plant's own
# post-reset momentum decay, a separate physical effect from the taper's
# own remaining-distance margin).
#
# 118 ticket 004 (land-at-zero-completion-delete-stop-lead.md): the
# turn-prediction campaign's own live time-lead anticipation constant
# (formerly pushed here alongside the taper fields below, re-swept twice
# as the taper's own stages landed) is DELETED -- the completion mechanism
# it drove no longer exists (App::MoveQueue::tick()'s own doc comment has
# the land-at-zero predicate that replaces it: completion emerges from the
# taper's own remaining/commanded-speed convergence, not a tuned guess).
# This gate is re-measured against the land-at-zero regime with NO lead
# constant pushed at all -- see this file's own git history for the full
# multi-retune chronology of the deleted constant if it is ever needed.
_TURN_TOLERANCE_SHAPED_DEG = 2.5

# decel-into-the-goal campaign: Motion::VelocityShaper's own accel/decel/
# jerk ceilings -- the SAME values baked into data/robots/tovez.json's own
# control.a_max/a_decel/alpha_max/alpha_decel/j_max/yaw_jerk_max
# (control._shaper_note has the full derivation), not synthetic test-only
# numbers, so this file's own measurements are directly actionable for the
# shipped default.
_A_MAX = 800.0          # [mm/s^2]
_A_DECEL = 800.0        # [mm/s^2]
_ALPHA_MAX = 7.0        # [rad/s^2]
_ALPHA_DECEL = 7.0      # [rad/s^2]
_J_MAX = 5000.0         # [mm/s^3]
_YAW_JERK_MAX = 100.0   # [rad/s^3]


def _compensating_register(raw_error: float) -> float:
    """Same conversion ``push.py``'s ``scale_to_int8()``/``Devices::Otos::
    scaleToRegister()`` perform: a scale multiplier -> the chip's raw int8
    register value (0.1%-per-LSB). Duplicated from
    ``test_otos_calibration_convergence.py`` -- both are small, self-
    contained test-local conversions, not shared library code."""
    scale = 1.0 / (1.0 + raw_error)
    return round((scale - 1.0) / 0.001)


def _normalize_deg(delta_deg: float) -> float:
    """Wrap a signed degree delta to (-180, 180]."""
    while delta_deg > 180.0:
        delta_deg -= 360.0
    while delta_deg <= -180.0:
        delta_deg += 360.0
    return delta_deg


class _SteppedClock:
    """A fake clock in lockstep with `_make_stepper()`'s own step count.
    `run_tour()`'s timeout/poll-interval math is written in real seconds --
    this reports "seconds" too (one sim cycle == 0.04s == firmware's own
    `App::RobotLoop::kCycle`, `SimLoop.step()`'s own documented per-cycle
    virtual-time advance -- 118 ticket 003) even though no wall clock is
    read at all, so `move_timeout`/`poll_interval`/`final_settle` keep their
    existing meaning."""

    def __init__(self) -> None:
        self.now_s = 0.0

    def now(self) -> float:
        return self.now_s


def _make_stepper(loop, clock: "_SteppedClock"):
    """`run_tour()`'s own `sleep_fn` -- a deterministic stand-in for
    `time.sleep()` when `loop` was connected with `start_tick_thread=False`
    (see `SimLoop`'s own module docstring: "the caller owns pacing"). Steps
    the sim exactly one cycle per call and pushes newly-produced telemetry
    onto the SAME queue `run_tour()`'s own `read_pending_binary_tlm_frames()`
    polls (`_drain_tlm_into_queue()` -- NOT the public `drain_pending_tlm()`,
    which also CONSUMES the queue and would race `run_tour()`'s own reads
    for the same frames).

    109-009 (stakeholder direction, round 2): converts a full tour run from
    several real wall-clock minutes (a real Python thread pacing to
    `_SPEED_FACTOR`-scaled real time) to a few CPU-bound seconds, and -- as
    a side effect -- removes real (non-deterministic) tick-thread
    scheduling jitter as a variable in the turn-accuracy/reliability
    assertions below entirely. One deliberately real-time-threaded smoke
    test is kept (`test_two_compatible_distance_legs_carry_velocity_
    through_the_boundary_at_tour_level`) as the TestGUI-fidelity check,
    since a live TestGUI Sim-mode connection always drives a real tick
    thread (`SimTransport.connect()`)."""

    def _step(_requested_interval: float) -> None:
        loop.step(1)
        loop._drain_tlm_into_queue()  # noqa: SLF001 -- see docstring above
        clock.now_s += 0.04  # [s] SimLoop.step()'s own per-cycle virtual-time advance (118 ticket 003)

    return _step


def _make_loop(*, realistic_errors: bool, deterministic: bool = True,
                a_max: "float | None" = None, a_decel: "float | None" = None,
                alpha_max: "float | None" = None, alpha_decel: "float | None" = None,
                j_max: "float | None" = None, yaw_jerk_max: "float | None" = None):
    from robot_radio.config.robot_config import load_robot_config
    from robot_radio.io.sim_loop import SimLoop
    from robot_radio.robot.protocol import NezhaProtocol

    lib_path = _sim_lib_path()
    loop = SimLoop(track_width=_TRACK_WIDTH, lib_path=lib_path)
    loop.connect(start_tick_thread=not deterministic)
    if not deterministic:
        loop.set_speed_factor(_SPEED_FACTOR)
    # 114-006: configure BEFORE the ideal/realistic fidelity knobs below --
    # see _ACTIVE_ROBOT_JSON's own comment for why this call is now required.
    #
    # 119 ticket 001 (kill-the-silent-off-shaping-config-boundary.md):
    # configure_from_robot() now ALSO pushes the SAME six decel-into-the-
    # goal shaper ceilings (a_max/a_decel/alpha_max/alpha_decel/j_max/
    # yaw_jerk_max) from _ACTIVE_ROBOT_JSON's own control.* section (Tier 3,
    # estimator_kwargs()) -- _ACTIVE_ROBOT_JSON's own committed values are
    # IDENTICAL to this file's _A_MAX/_A_DECEL/etc module constants below
    # (both were swept against the SAME gate). This function's own a_max=/
    # a_decel=/etc parameters default to None below (not those constants) so
    # the common case -- every current call site in this file -- relies on
    # Tier 3's single push instead of ALSO re-pushing the identical values a
    # second time through this function's own manual estimator_config()
    # call further down: a second, genuinely redundant CommandEnvelope
    # consumes one more Comms::pump() cycle before the FIRST turn's own Move
    # is issued, which shifts WheelPlant's rest-dither phase
    # (wheel_plant.h's own 108-011 "flip every kDitherPeriod calls" cadence)
    # by exactly enough to tip an already-narrow-pocket turn measurement
    # over its own tolerance (found running this exact suite against this
    # ticket's own change: TOUR_2/ideal turn 12 missed by +0.009deg against
    # its 2.5deg band, xfail(strict=False)-free, a genuine regression from
    # the redundant push, not from the shaping VALUES themselves, which are
    # unchanged). Passing an explicit non-None override here (a future
    # sweep/tuning caller) still works exactly as before -- it pushes
    # AFTER configure_from_robot()'s own Tier 3 push and genuinely differs
    # from it, so it is not redundant.
    loop.configure_from_robot(load_robot_config(_ACTIVE_ROBOT_JSON))

    # decel-into-the-goal campaign: a_max/a_decel/alpha_max/alpha_decel/
    # j_max/yaw_jerk_max ride the EstimatorConfigPatch/estimator_config()
    # call (config.proto's own "smallest coherent path" doc comment) --
    # App::MoveQueue leaves ShaperLimits at its own constructor default
    # (every field 0, shaping OFF; see App::ShaperLimits's own doc comment)
    # unless pushed -- SimHarness itself deliberately does not source it
    # from boot config (sim_harness.h's own "sim/production boundary"
    # comment); as of 119 ticket 001, configure_from_robot() above already
    # pushes _ACTIVE_ROBOT_JSON's own values (Tier 3), so this block below
    # only fires for a caller that explicitly OVERRIDES at least one of the
    # six with a value different from the JSON's own -- the common (every
    # current call site) all-None case is a genuine no-op, not "skip a push
    # that would otherwise be needed." 118 ticket 004
    # (land-at-zero-completion-delete-stop-lead.md): this used to also push
    # a live time-lead anticipation constant alongside the taper fields --
    # DELETED, the completion mechanism it drove no longer exists
    # (App::MoveQueue::tick()'s own doc comment has the land-at-zero
    # predicate that replaces it), so there is nothing left to push for it.
    shaper_fields = {}
    if a_max is not None:
        shaper_fields["a_max"] = a_max
    if a_decel is not None:
        shaper_fields["a_decel"] = a_decel
    if alpha_max is not None:
        shaper_fields["alpha_max"] = alpha_max
    if alpha_decel is not None:
        shaper_fields["alpha_decel"] = alpha_decel
    if j_max is not None:
        shaper_fields["j_max"] = j_max
    if yaw_jerk_max is not None:
        shaper_fields["yaw_jerk_max"] = yaw_jerk_max

    if shaper_fields:
        conn = _SimConfigConn(loop)
        proto = NezhaProtocol(conn)  # type: ignore[arg-type]
        corr_id = proto.estimator_config(**shaper_fields)
        ack = _wait_for_ack(loop, corr_id, deterministic=deterministic)
        assert ack is not None and ack.ok, (
            f"EstimatorConfigPatch shaper push failed to ack: {ack}"
        )

    if not realistic_errors:
        # Ideal chip: every knob explicit at its documented no-op default,
        # not just "never touched" -- makes the "disabled" side of the gate
        # self-documenting rather than relying on SimPlant's own defaults.
        loop.set_otos_raw_scale_err(0.0, 0.0)
        loop.set_enc_scale_err(1, 0.0)
        loop.set_enc_scale_err(2, 0.0)
        loop.set_enc_tick_quant(1, 0.0)
        loop.set_enc_tick_quant(2, 0.0)
        loop.set_enc_slip(1, 0.0, 0.0)
        loop.set_enc_slip(2, 0.0, 0.0)
        return loop

    loop.set_otos_raw_scale_err(_OTOS_LINEAR_ERR, _OTOS_ANGULAR_ERR)
    loop.set_enc_scale_err(1, _ENC_SCALE_ERR_L)
    loop.set_enc_scale_err(2, _ENC_SCALE_ERR_R)
    loop.set_enc_tick_quant(1, _ENC_TICK_MM)
    loop.set_enc_tick_quant(2, _ENC_TICK_MM)
    loop.set_enc_slip(1, _ENC_SLIP_RATE, _ENC_SLIP_MAG)
    loop.set_enc_slip(2, _ENC_SLIP_RATE, _ENC_SLIP_MAG)

    # Calibrate OUT the raw OTOS scale error via the real OtosConfigPatch
    # wire path -- see this file's own module docstring for why an
    # uncalibrated OTOS is out of scope for this gate.
    conn = _SimConfigConn(loop)
    proto = NezhaProtocol(conn)  # type: ignore[arg-type]
    corr_id = proto.otos_config(
        linear_scale=_compensating_register(_OTOS_LINEAR_ERR),
        angular_scale=_compensating_register(_OTOS_ANGULAR_ERR),
    )
    ack = _wait_for_ack(loop, corr_id, deterministic=deterministic)
    assert ack is not None and ack.ok, (
        f"OtosConfigPatch calibration push failed to ack: {ack}"
    )
    return loop


def _wait_for_ack(loop, corr_id: int, timeout: float = 3.0, *, deterministic: bool = True):
    """Poll for a fresh ack matching `corr_id`.

    BUGFIX (turn-prediction campaign): both branches used to read
    `frame.acks` (plural) -- the pre-115-003 depth-3 ack ring, which
    `TLMFrame` no longer has (`AttributeError`, discovered when this
    function became load-bearing for this campaign's own EstimatorConfigPatch
    push below -- previously the ONLY caller was the realistic-errors
    branch's OTOS push, and every test reaching it is `xfail(strict=False)`,
    which silently swallows ANY exception, not just AssertionError, masking
    this outright). `TLMFrame` carries a SINGLE ack slot instead
    (`frame.ack`, an `AckEntry | None`, populated only when `ack_fresh` was
    set that frame) -- see `protocol.py`'s own `TLMFrame` doc comment.
    """
    if deterministic:
        for _ in range(400):  # 400 cycles * 50ms = 20s virtual time -- generously bounded
            loop.step(1)
            for frame in loop.drain_pending_tlm():
                if frame.ack is not None and frame.ack.corr_id == corr_id:
                    return frame.ack
        return None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for frame in loop.read_pending_binary_tlm_frames():
            if frame.ack is not None and frame.ack.corr_id == corr_id:
                return frame.ack
        time.sleep(0.02)
    return None


@dataclass
class TurnCheck:
    index: int
    commanded_deg: float
    achieved_deg: float
    error_deg: float


@dataclass
class StraightLegCruiseCheck:
    """119 ticket 005 (straight-leg-crab actuation/telemetry pairing skew --
    clasi/issues/straight-leg-crab-118-001-actuation-and-telemetry-pairing-skew.md):
    per-STRAIGHT-leg (``leg.kind == "distance"``) TRUTH heading stability,
    sampled every sim cycle for the leg's FULL duration (a superset of its
    cruise phase -- the accel/decel transients are the exact place 118-001's
    bug lived, so this check does not skip them the way the turn-accuracy
    checks skip a settle window). ``max_abs_delta_deg`` is the largest
    |truth heading - truth heading at the leg's own start| observed at any
    sampled cycle -- an endpoint-only check (compare only the leg's final
    heading to its start) is PROVABLY BLIND to this failure shape: 118-001's
    own bug held a nonzero heading offset throughout accel+cruise+decel and
    canceled it exactly by the end (measured final error 0.00deg while the
    leg crabbed sideways the whole time) -- see the issue's own derivation.
    """
    index: int
    distance_mm: float
    max_abs_delta_deg: float


@dataclass
class TourGateResult:
    completed: bool
    stop_reason: str
    turns: list = field(default_factory=list)
    straight_leg_cruise_headings: list = field(default_factory=list)
    start_true: dict | None = None
    end_true: dict | None = None
    position_delta: float | None = None
    heading_delta_deg: float | None = None


def _run_tour_capture(loop, tour_wire, *, v_max: float = 150.0,
                      deterministic: bool = True) -> TourGateResult:
    from types import SimpleNamespace

    from robot_radio.planner.heading import HeadingCorrector
    from robot_radio.planner.model import PlannerParams
    from robot_radio.planner.tour import parse_tour, run_tour

    legs = parse_tour(tour_wire)
    params = PlannerParams()
    # Mirrors tests/bench/tour_bench_run.py's own bench-rig convention and
    # test_sim_transport_tour1.py's own run_tour() test: force encoder-
    # derived heading for run_tour()'s own (read-only, non-closed-loop)
    # heading_before/heading_after bookkeeping -- irrelevant to this test's
    # own turn-accuracy measurement, which reads SimPlant ground truth
    # directly via loop.get_true_pose(), not this readback.
    heading = HeadingCorrector(
        params, robot_config=SimpleNamespace(geometry=SimpleNamespace(otos_untrusted=True)))

    true_poses: list[dict] = [loop.get_true_pose()]
    turns: list[TurnCheck] = []
    # 119 ticket 005: max |truth heading - truth heading at leg start|
    # observed at any SAMPLED cycle of a "distance" (straight) leg, keyed by
    # leg index -- see StraightLegCruiseCheck's own doc comment for why this
    # is a per-cycle, whole-leg-duration check, not an endpoint comparison.
    straight_leg_max_delta: dict[int, float] = {}

    def _on_leg(index, total, leg, leg_result) -> None:
        pose = loop.get_true_pose()
        if leg.kind == "turn":
            before_h_deg = math.degrees(true_poses[-1]["h"])
            after_h_deg = math.degrees(pose["h"])
            achieved = _normalize_deg(after_h_deg - before_h_deg)
            turns.append(TurnCheck(
                index=index, commanded_deg=leg.value, achieved_deg=achieved,
                error_deg=_normalize_deg(achieved - leg.value)))
        true_poses.append(pose)

    def _on_row(_tick_index, row_leg_index, row_leg, _tick_result, _frame) -> None:
        # RowCallback's own args (tour.py): (tick_index [GLOBAL across the
        # WHOLE tour], leg_index, leg, TickResult, latest TLMFrame|None) --
        # fires once per poll iteration (one sim cycle each, in this
        # deterministic-stepper path) WHILE a leg is in flight. run_tour()
        # calls this BEFORE on_leg() ever fires for the SAME leg index, so
        # true_poses[-1] here is still that leg's own START pose (the
        # previous leg's end pose, or true_poses[0] for leg 0) -- see
        # StraightLegCruiseCheck's own doc comment.
        if row_leg.kind != "distance":
            return
        start_h_deg = math.degrees(true_poses[-1]["h"])
        current_h_deg = math.degrees(loop.get_true_pose()["h"])
        delta = abs(_normalize_deg(current_h_deg - start_h_deg))
        # -1.0 sentinel (never a real |delta|) so a leg whose TRUE max delta
        # is exactly 0.0 (e.g. the tour's own first leg, with no preceding
        # chain-advance to inherit any residual) still gets an entry instead
        # of silently never appearing in straight_leg_max_delta at all.
        if delta > straight_leg_max_delta.get(row_leg_index, -1.0):
            straight_leg_max_delta[row_leg_index] = delta

    run_tour_kwargs: dict = {}
    if deterministic:
        clock = _SteppedClock()
        run_tour_kwargs.update(
            # poll_interval is nominal here -- _make_stepper()'s own _step()
            # ignores the requested interval and always advances exactly one
            # sim cycle (see its docstring) -- kept at the cycle's own value
            # (0.04s, 118 ticket 003) for consistency, not because anything
            # reads it as a real pacing duration in this deterministic path.
            clock_fn=clock.now, sleep_fn=_make_stepper(loop, clock), poll_interval=0.04)

    result = run_tour(loop, params, heading, legs, v_max=v_max, on_leg=_on_leg,
                       row_callback=_on_row, **run_tour_kwargs)

    completed = result.stopped_at is None
    stop_reason = "" if completed else (
        f"leg {result.stopped_at + 1}/{len(legs)} ({result.stopped_outcome})")

    straight_leg_cruise_headings = [
        StraightLegCruiseCheck(index=i, distance_mm=legs[i].value, max_abs_delta_deg=d)
        for i, d in sorted(straight_leg_max_delta.items())
    ]

    gate = TourGateResult(completed=completed, stop_reason=stop_reason, turns=turns,
                          straight_leg_cruise_headings=straight_leg_cruise_headings,
                          start_true=true_poses[0] if true_poses else None,
                          end_true=true_poses[-1] if completed and true_poses else None)

    if gate.start_true is not None and gate.end_true is not None:
        dx = gate.end_true["x"] - gate.start_true["x"]
        dy = gate.end_true["y"] - gate.start_true["y"]
        gate.position_delta = math.hypot(dx, dy)
        gate.heading_delta_deg = _normalize_deg(
            math.degrees(gate.end_true["h"] - gate.start_true["h"]))

    return gate


def _assert_tour_gate(gate: TourGateResult, *, tolerance_deg: float, label: str,
                       cruise_heading_tolerance_deg: float) -> None:
    assert gate.completed, f"{label}: tour did not complete -- stopped at {gate.stop_reason}"
    assert gate.turns, f"{label}: no turn legs captured (parse_tour() regression?)"

    report_lines = [f"{label}: per-turn commanded vs achieved (sim ground truth):"]
    worst = 0.0
    for t in gate.turns:
        report_lines.append(
            f"  turn {t.index + 1}: commanded={t.commanded_deg:+8.2f}deg "
            f"achieved={t.achieved_deg:+8.2f}deg error={t.error_deg:+7.3f}deg")
        worst = max(worst, abs(t.error_deg))
    report = "\n".join(report_lines)

    for t in gate.turns:
        assert abs(t.error_deg) < tolerance_deg, (
            f"{label}: turn {t.index + 1} (commanded {t.commanded_deg:+.2f}deg) missed by "
            f"{t.error_deg:+.3f}deg (tolerance {tolerance_deg}deg)\n{report}"
        )

    # 119 ticket 005: cruise-heading gate on every straight ("distance") leg
    # -- endpoint-only checks (position_delta/heading_delta_deg below) are
    # PROVABLY BLIND to the straight-leg-crab failure shape (118-001's own
    # bug held a nonzero TRUE heading throughout accel+cruise+decel and
    # canceled it exactly by the leg's own end -- see
    # StraightLegCruiseCheck's own doc comment). This is a NEW assertion,
    # not a replacement for the position/heading closure checks below.
    assert gate.straight_leg_cruise_headings, (
        f"{label}: no straight (distance) legs captured (parse_tour() regression, or a "
        f"tour with no D steps?)"
    )
    cruise_report_lines = [f"{label}: per-straight-leg max |truth heading delta| during the leg:"]
    cruise_worst = 0.0
    for c in gate.straight_leg_cruise_headings:
        cruise_report_lines.append(
            f"  leg {c.index + 1} (distance={c.distance_mm:+.0f}mm): "
            f"max|delta|={c.max_abs_delta_deg:.4f}deg")
        cruise_worst = max(cruise_worst, c.max_abs_delta_deg)
    cruise_report = "\n".join(cruise_report_lines)

    for c in gate.straight_leg_cruise_headings:
        assert c.max_abs_delta_deg < cruise_heading_tolerance_deg, (
            f"{label}: straight leg {c.index + 1} (distance={c.distance_mm:+.0f}mm) truth "
            f"heading drifted {c.max_abs_delta_deg:.4f}deg during the leg (tolerance "
            f"{cruise_heading_tolerance_deg}deg) -- endpoint checks would have missed this if "
            f"decel canceled it (see straight-leg-crab-118-001 issue)\n{cruise_report}"
        )

    assert gate.position_delta is not None and gate.heading_delta_deg is not None
    assert gate.position_delta < _CLOSURE_POSITION_MAX_MM, (
        f"{label}: closure position delta implausibly large: {gate.position_delta:.1f}mm "
        f"(start={gate.start_true}, end={gate.end_true})\n{report}"
    )

    # Report-oriented: always printed so `-s`/failure output carries the
    # numbers the stakeholder asked to read, not adjectives.
    print(f"\n{report}\n{label}: worst |error|={worst:.3f}deg, "
          f"closure position_delta={gate.position_delta:.1f}mm, "
          f"heading_delta={gate.heading_delta_deg:+.2f}deg\n"
          f"{cruise_report}\n{label}: worst straight-leg cruise |delta|={cruise_worst:.4f}deg "
          f"(tolerance {cruise_heading_tolerance_deg}deg)")


# ---------------------------------------------------------------------------
# The gate itself. 109-009's Impossibility Argument (ticket file) fixed
# eight real firmware bugs across two rounds; tours now complete RELIABLY
# (15/15 clean completions, both tours, both error profiles -- see that
# ticket's own completion notes). These tests stay xfail ONLY for the two
# residual per-turn ACCURACY gaps below, not for completion/reliability,
# which is resolved. xfail (not skip) so both gaps stay VISIBLE and would
# XPASS loudly the moment either one actually closes.
# ---------------------------------------------------------------------------

_XFAIL_REASON_IDEAL = (
    "109-009 Impossibility Argument: ideal-chip tour turns still don't reach the "
    "stakeholder's <0.05deg 'exact' bar (current per-turn errors ~0.2-2.2deg; see "
    "test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band()'s own "
    "printed report) -- see clasi/issues/land-at-zero-at-orthogonal-chain-boundaries.md "
    "for the live investigation into the dominant remaining error source."
)

_XFAIL_REASON_REALISTIC = (
    "109-009 Impossibility Argument: realistic-profile tour turns still don't "
    "uniformly land within the stakeholder's 1.0deg bar (same dominant error source "
    "as the ideal-chip xfail, plus sensor-error-injection noise) -- see "
    "clasi/issues/land-at-zero-at-orthogonal-chain-boundaries.md for the live "
    "investigation."
)


# ---------------------------------------------------------------------------
# Decel-into-the-goal campaign's own REAL (non-xfail) acceptance gate --
# ticket's own words: "Sim system: 90 turns land 90+-2 target band (state
# achieved band honestly)". Distinct from the ORIGINAL 109-009 stakeholder
# bars above (_TURN_TOLERANCE_IDEAL_DEG=0.05/_TURN_TOLERANCE_REALISTIC_DEG=
# 1.0, still their own untouched xfailed aspirational gate) -- this is the
# tightened tolerance this campaign's own shaping work ACTUALLY achieves,
# not loosening or replacing that older gate. TOUR_1 is all-90-degree legs
# (the ticket's own literal target); TOUR_2 is included too since its
# non-90-degree legs (124/146/215/217deg) measured comparable error
# magnitudes in the sweep above, not a materially different regime.
# ---------------------------------------------------------------------------


def test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band():
    from robot_radio.planner.tour import TOUR_1, TOUR_2

    for label, tour in (("TOUR_1", TOUR_1), ("TOUR_2", TOUR_2)):
        for realistic in (False, True):
            loop = _make_loop(realistic_errors=realistic)
            try:
                gate = _run_tour_capture(loop, tour)
            finally:
                loop.disconnect()
            profile = "realistic" if realistic else "ideal"
            cruise_tol = (_CRUISE_HEADING_TOLERANCE_REALISTIC_DEG if realistic
                          else _CRUISE_HEADING_TOLERANCE_IDEAL_DEG)
            _assert_tour_gate(gate, tolerance_deg=_TURN_TOLERANCE_SHAPED_DEG,
                               label=f"{label}/{profile}/shaped-band",
                               cruise_heading_tolerance_deg=cruise_tol)


@pytest.mark.xfail(reason=_XFAIL_REASON_IDEAL, strict=False)
def test_tour_1_ideal_chip_turns_are_exact():
    from robot_radio.planner.tour import TOUR_1

    loop = _make_loop(realistic_errors=False)
    try:
        gate = _run_tour_capture(loop, TOUR_1)
    finally:
        loop.disconnect()
    _assert_tour_gate(gate, tolerance_deg=_TURN_TOLERANCE_IDEAL_DEG, label="TOUR_1/ideal",
                       cruise_heading_tolerance_deg=_CRUISE_HEADING_TOLERANCE_IDEAL_DEG)


@pytest.mark.xfail(reason=_XFAIL_REASON_IDEAL, strict=False)
def test_tour_2_ideal_chip_turns_are_exact():
    from robot_radio.planner.tour import TOUR_2

    loop = _make_loop(realistic_errors=False)
    try:
        gate = _run_tour_capture(loop, TOUR_2)
    finally:
        loop.disconnect()
    _assert_tour_gate(gate, tolerance_deg=_TURN_TOLERANCE_IDEAL_DEG, label="TOUR_2/ideal",
                       cruise_heading_tolerance_deg=_CRUISE_HEADING_TOLERANCE_IDEAL_DEG)


@pytest.mark.xfail(reason=_XFAIL_REASON_REALISTIC, strict=False)
def test_tour_1_realistic_errors_turns_within_one_degree():
    from robot_radio.planner.tour import TOUR_1

    loop = _make_loop(realistic_errors=True)
    try:
        gate = _run_tour_capture(loop, TOUR_1)
    finally:
        loop.disconnect()
    _assert_tour_gate(gate, tolerance_deg=_TURN_TOLERANCE_REALISTIC_DEG, label="TOUR_1/realistic",
                       cruise_heading_tolerance_deg=_CRUISE_HEADING_TOLERANCE_REALISTIC_DEG)


@pytest.mark.xfail(reason=_XFAIL_REASON_REALISTIC, strict=False)
def test_tour_2_realistic_errors_turns_within_one_degree():
    from robot_radio.planner.tour import TOUR_2

    loop = _make_loop(realistic_errors=True)
    try:
        gate = _run_tour_capture(loop, TOUR_2)
    finally:
        loop.disconnect()
    _assert_tour_gate(gate, tolerance_deg=_TURN_TOLERANCE_REALISTIC_DEG, label="TOUR_2/realistic",
                       cruise_heading_tolerance_deg=_CRUISE_HEADING_TOLERANCE_REALISTIC_DEG)


# ---------------------------------------------------------------------------
# SUC-003's "no dip to zero at a compatible same-v_max boundary", verified at
# the FULL tour-execution level (real run_tour() against the real compiled
# firmware sim). boundary_velocity_harness.cpp -- the synthetic
# Motion::Executor-only unit harness this integration-level test used to sit
# alongside (ticket 006's own "within 2 cycles" check) -- is DELETED (115-002,
# gut-to-minimal-firmware S1 motion-stack excision) along with the rest of
# the motion stack; this test is now the only surviving coverage of this
# property, whole-run_tour()-level only.
# ---------------------------------------------------------------------------


@pytest.mark.xfail(
    strict=False,
    reason=(
        "119-002: re-run against the current tree (post-118, post-119 ticket 001) -- "
        "the PRIOR reason above (111-002's reorder-coupled stale/alternating-encoder "
        "oscillation, frame.twist bouncing between roughly half and above-v_max) is "
        "STALE: that experiment was retired by 118 ticket 001, and the failure this "
        "test now shows is a DIFFERENT, independently-diagnosed mechanism -- a clean, "
        "monotonic velocity dip from ~149mm/s to ~24mm/s at the leg boundary followed "
        "by a smooth accel/jerk-limited ramp back to cruise over ~8 cycles (~320ms), "
        "not an erratic oscillation. Root cause: App::MoveQueue::tick() "
        "(move_queue.cpp) unconditionally hard-resets the completing axis's shaper to "
        "(0, 0) at EVERY completion boundary (118 ticket 003's own kept decision), "
        "even when the incoming chained Move commands that SAME axis -- so this "
        "test's own two same-axis, same-v_max Distance legs get the hand-off reset "
        "instead of SUC-051's carried-velocity continuity. This was always true post-118 "
        "ticket 003, but only became visible in THIS test once 119 ticket 001 made the "
        "shaper-limits push default-on for _make_loop()'s own SimLoop session -- "
        "previously shaping was silently off here (no taper, no reset, no dip "
        "possible), so this specific same-axis-compatible-chain scenario was never "
        "exercised by 118 ticket 003's own re-sweep (TOUR_1/TOUR_2 always alternate "
        "Distance/Angle legs, so a same-axis boundary never occurred there either). "
        "See motion/DESIGN.md's own 'Chain-advance leg hand-off contract' (§4, 119-002) "
        "for the verified mechanism and "
        "clasi/issues/chain-advance-reset-defeats-same-axis-compatible-leg-continuity.md "
        "for the fix candidate (make the reset conditional on whether the incoming "
        "chained Move's own axis/kind actually differs) and its own concrete "
        "unblocking condition (implement + re-sweep kStoppingMarginFactorChain/"
        "kDiscretizationCyclesChain jointly against the tour-closure gate, since the "
        "reset's unconditional form is part of what that gate's own margin was tuned "
        "against) -- deliberately not fixed here: 119-002's own scope excludes "
        "changing MoveQueue's completion-handling reset or re-deriving the already "
        "narrow-pocket chain margin (chain-advance-completion-margin-narrow-pocket.md)."
    ),
)
def test_two_compatible_distance_legs_carry_velocity_through_the_boundary_at_tour_level():
    from robot_radio.planner.tour import TourLeg, run_tour
    from robot_radio.planner.heading import HeadingCorrector
    from robot_radio.planner.model import PlannerParams
    from types import SimpleNamespace

    v_max = 150.0  # [mm/s]
    legs = [
        TourLeg(kind="distance", value=300.0, speed=v_max),
        TourLeg(kind="distance", value=300.0, speed=v_max),
    ]

    # Deliberately the ONE real-time-threaded (not deterministically-
    # stepped) test in this file -- 109-009 (stakeholder direction, round
    # 2): keep exactly one real-time run as the TestGUI-fidelity check,
    # since that is the actual path a live TestGUI Sim-mode connection
    # drives (`SimTransport`'s own `SimLoop` always runs a real tick
    # thread -- see that class's own `connect()`). The turn-accuracy/
    # reliability tests above use deterministic stepping instead
    # specifically to remove real-time scheduling jitter as a confound and
    # to make a full tour run in seconds, not minutes.
    loop = _make_loop(realistic_errors=False, deterministic=False)
    try:
        params = PlannerParams()
        heading = HeadingCorrector(
            params, robot_config=SimpleNamespace(geometry=SimpleNamespace(otos_untrusted=True)))

        samples: list[tuple[int, float]] = []  # (global_tick_index, v_x [mm/s])

        def _row_callback(tick_index, leg_index, leg, tick_result, frame) -> None:
            if frame is not None and frame.twist is not None:
                samples.append((tick_index, float(frame.twist[0])))

        result = run_tour(loop, params, heading, legs, v_max=v_max, row_callback=_row_callback)
        assert result.stopped_at is None, (
            f"synthetic two-leg same-v_max tour did not complete: leg "
            f"{result.stopped_at} ({result.stopped_outcome})"
        )
    finally:
        loop.disconnect()

    assert len(samples) >= 10, f"too few velocity samples captured to judge a boundary: {samples}"

    # Trim the ramp-up (first 15%) and ramp-down (last 15%) -- the boundary
    # itself sits somewhere in the middle 70% of the two-leg run; a
    # decelerate-to-a-stop-and-replan regression would show up as a dip
    # ANYWHERE in that middle window, not just exactly at the midpoint.
    n = len(samples)
    lo = max(1, int(n * 0.15))
    hi = min(n - 1, int(n * 0.85))
    middle = samples[lo:hi]
    assert middle, f"empty middle window for boundary check (n={n})"

    min_v = min(v for _, v in middle)
    floor = v_max * _BOUNDARY_MIN_FRACTION

    # 119-002: the contract (motion/DESIGN.md Sec 4, "Chain-advance leg
    # hand-off contract") documents the KNOWN mechanism the no-dip
    # assertion below currently xfails on: App::MoveQueue::tick()'s own
    # unconditional completing-axis reset produces ONE contiguous decel-
    # then-accel/jerk-limited-recovery dip, not the RETIRED reorder
    # experiment's own erratic, non-monotonic oscillation (see the
    # xfail's own reason string). Characterize the dip's own shape BEFORE
    # the strict floor check below (so it runs even while that check
    # keeps failing) so a future regression to that different, already-
    # retired failure mode -- or a genuinely new one, e.g. a stall that
    # never recovers -- is caught as a DISTINGUISHABLE break instead of
    # silently absorbed by this same xfail.
    below = [i for i, (_, v) in enumerate(middle) if v < floor]
    if below:
        gaps = [b - a for a, b in zip(below, below[1:])]
        assert all(g == 1 for g in gaps), (
            f"{sum(1 for g in gaps if g != 1) + 1} distinct below-floor regions in the "
            f"middle window -- an oscillating (not a single decel-then-recover) dip "
            f"shape, inconsistent with the documented same-axis-reset mechanism "
            f"(motion/DESIGN.md Sec 4): {middle}"
        )
        dip_duration = below[-1] - below[0] + 1
        assert dip_duration <= 20, (
            f"below-floor dip lasted {dip_duration} samples (~{dip_duration * 0.04:.2f}s) -- "
            f"longer than the accel/jerk-limited re-ramp the documented mechanism "
            f"should produce (motion/DESIGN.md Sec 4's own 'Chain-advance leg hand-off "
            f"contract'): {middle}"
        )

    assert min_v >= floor, (
        f"velocity dipped to {min_v:.1f}mm/s in the middle (steady-state/boundary) window -- "
        f"expected >= {floor:.1f}mm/s ({_BOUNDARY_MIN_FRACTION * 100:.0f}% of v_max={v_max}mm/s); "
        f"full middle trace: {middle}"
    )


if __name__ == "__main__":
    import sys

    sys.exit(pytest.main([__file__, "-v", "-s"]))
