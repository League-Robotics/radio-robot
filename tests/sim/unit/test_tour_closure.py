"""tests/sim/unit/test_tour_closure.py -- sim-plant ground-truth closure
proof for the TestGUI's recorded "Tour 1" (stakeholder-directed OOP
debugging session, 2026-07-10; no CLASI ticket).

Drives the ACTUAL TOUR_1 wire sequence (``robot_radio.testgui.commands.
TOUR_1``) through the compiled sim (``tests/_infra/sim/build/
libfirmware_host``) via the SAME transport-level machinery the TestGUI's
``SimTransport`` uses -- ``robot_radio.io.sim_conn.SimConnection`` +
``robot_radio.robot.protocol.NezhaProtocol`` + ``robot_radio.testgui.
binary_bridge.translate_command()`` -- never a hand-rolled second command
path. Tovez no-cal geometry (trackwidth 128 mm) + every sim error knob at
its neutral/zero value, mirroring ``tests/testgui/test_tour1_geometry.py``'s
own "zero-error profile" convention.

Root cause under investigation (see the module-level report handed back to
the stakeholder): the firmware's own SEGMENT KINEMATICS (``Subsystems::
Drivetrain::config_.trackwidth``, consulted by ``BodyKinematics::inverse()``
and by ``Motion::SegmentExecutor::start()``'s ``arcScale_``) and the SIM
PLANT's ground-truth integration (``Hal::PhysicsWorld::trackwidth_``) used to
disagree whenever a caller pushed the plant's trackwidth via
``SimConnection.set_trackwidth()`` (a ctypes-only call, pure plant
ground-truth injection) without ALSO pushing an equivalent ``SET tw=``
config command over the wire (which is the only thing that updates the
firmware's OWN ``Drivetrain::config_.trackwidth`` via ``Rt::Configurator``,
confirmed live-verified against ``tests/_infra/sim/sim_api.cpp``'s
``drainConfig()``, which folds any posted ``bb.configIn`` entry into
``Drivetrain::configure()`` after every ``sim_tick()``/``sim_command_on()``
call). The TestGUI's own Connect flow ALREADY pushes ``SET tw=<geometry.
trackwidth>`` via ``__main__.py``'s ``_push_robot_calibration()`` ->
``calibration/push.py``'s ``calibration_commands()`` -- so the full GUI path
is not exposed to the mismatch. This test drives the sim at the SAME layer
``SimTransport`` does but WITHOUT going through that GUI-level calibration
push (matching this ticket's own guidance to reuse ``legacy_translate``/
``SimConnection``/``binary_bridge`` directly), so it exercises the sim's
OWN bare defaults -- exactly the scenario a caller who forgets (or never
knew to invoke) the GUI's calibration push hits. ``PhysicsWorld::
kDefaultTrackwidth`` (``source/hal/sim/physics_world.h``) previously seeded
BOTH the plant's bare default AND (via ``defaultSimDrivetrainConfig()`` in
``sim_api.cpp``) the firmware's own bare kinematics default at 150.0 mm --
an arbitrary legacy value ported from ``source_old``'s MockHAL, not the
project's real trackwidth (128 mm, ``data/robots/tovez_nocal.json``). Fixed
to 128.0 mm so the sim is self-consistent OUT OF THE BOX, belt-and-suspenders
with the GUI's own calibration push.

Second finding -- the RT over-rotation -- RESOLVED 2026-07-11 (this module's
two former ``xfail(strict=True)`` tests now pass and their markers are
removed). With trackwidth fully self-consistent, a single isolated ``RT
9000`` still over-rotated to ~110 deg. The blame initially fell on
``Motion::SegmentExecutor``'s replan/stop-decel cascade, but per-pass
instrumentation proved the executor's EMITTED omega integral was EXACTLY the
commanded angle (90.00/180.00 deg) at every stage -- the plan was never
wrong. Two real defects underneath:

1. **TLM cmd= mislabel** (``source/telemetry/tlm_frame.cpp``): the wire's
   cmd= field read ``bb.drivetrain.vel()`` -- the MEASURED array -- so
   telemetry showed command==measured and hid the tracking error entirely.
2. **Sim plant velocity-gain miscalibration** (``tests/_infra/sim/
   sim_api.cpp defaultMotorConfigSet()``): hand-typed kff=0.0038 vs the
   plant's exact 1/kNominalMaxSpeed=0.0025 (vel = duty * 400) overdrove
   every wheel ~1.25x its setpoint -- the entire ~+20 deg/pivot residual.
   Translate legs were immune because STOP_DISTANCE truncates on measured
   encoders; the pivot endgame ran in plan space and let the overdrive
   through. Full calibration set (all plant-specific quantities): kff exact,
   gentle kp/ki, velFiltAlpha=1.0 (honest measurement), v_body_max capped to
   the plant's own 400 mm/s ceiling less headroom, and the executor's sim
   ``kOutputHops`` re-measured at 1.5 passes for this gain set.
3. **Executor stop path made library-native** (2026-07-11, stakeholder-
   directed): the terminal ``solveToVelocity(0)`` re-arm at position-stop
   fire was replaced by RIDING each position solve's own to-rest tail (a
   Ruckig position profile ends with velocity and acceleration reaching 0
   simultaneously AT the target -- its tail IS the optimal graceful
   no-reverse stop; re-solving a velocity ramp mid-decel is what produced
   the terminal reversal dip). Position solves now carry Ruckig's own
   directional velocity band (min_velocity=0 for forward solves), so no
   replan can even ASK for reversal, and replan solve failures leave the
   in-flight plan untouched (a latent return-value-ignored bug restarted
   the plan's clock mid-flight when a solve failed).

   Final accuracy: pivots within ~0.5 deg (90->89.97), D legs within ~1mm,
   zero commanded or measured reversal anywhere; TOUR_1 closes to ~4mm /
   ~1.6 deg.
"""
from __future__ import annotations

import math
import pathlib
import sys

import pytest

# tests/sim/unit/test_tour_closure.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_HOST_DIR = _REPO_ROOT / "host"
if str(_HOST_DIR) not in sys.path:
    sys.path.insert(0, str(_HOST_DIR))

_SIM_BUILD = _REPO_ROOT / "tests" / "_infra" / "sim" / "build"
_LIB_PRESENT = any(
    (_SIM_BUILD / name).exists()
    for name in ("libfirmware_host.dylib", "libfirmware_host.so")
)

pytestmark = pytest.mark.skipif(
    not _LIB_PRESENT,
    reason="firmware sim lib not built (just build-sim, or: "
           "cd tests/_infra/sim && cmake --build build)",
)

if _LIB_PRESENT:
    from robot_radio.io.sim_conn import SimConnection
    from robot_radio.robot import legacy_verbs
    from robot_radio.robot.protocol import NezhaProtocol
    from robot_radio.testgui import binary_bridge, sim_prefs
    from robot_radio.testgui.commands import TOUR_1
    from robot_radio.testgui.traces import EncoderDeadReckoner


# ---------------------------------------------------------------------------
# Sim configuration -- tovez no-cal geometry + a genuinely neutral error
# profile (every knob this ABI can set to its documented no-op value).
# ---------------------------------------------------------------------------

_TRACKWIDTH = 128.0  # [mm] data/robots/tovez_nocal.json geometry.trackwidth
_STEP = 20  # [ms] matches transport.py's SimTransport _SIM_TICK_STEP_DURATION
_STREAM_PERIOD = 20  # [ms] telemetry.proto floor, matches SimTransport's own arm

# Grace ticks after issuing a step before an "idle" telemetry frame is
# trusted -- mirrors __main__.py's _TourRunner.SPINUP_S (0.2s @ 20ms/tick).
_SPINUP_TICKS = 6
# Per-step ceiling (sim time) before a step is considered hung.
_STEP_TIMEOUT_S = 15.0


def _apply_zero_error_profile(conn: "SimConnection") -> None:
    """Apply sim_prefs.DEFAULT_PROFILE (every noise/scrub knob neutral) with
    trackwidth pinned to the project's real value, via the SAME
    profile->setter mapping ``transport.py``'s ``_apply_profile_to_sim()``
    uses (``sim_prefs.PROFILE_TO_SIM_SETTER``) -- not a hand-typed second
    copy of the knob list, so a newly-added knob is picked up here too."""
    profile = dict(sim_prefs.DEFAULT_PROFILE)
    profile["trackwidth"] = _TRACKWIDTH
    for key, setter_name in sim_prefs.PROFILE_TO_SIM_SETTER.items():
        getattr(conn, setter_name)(profile[key])
    conn.set_enc_noise(2, profile["encoder_noise"])
    conn.set_enc_scale_error(0, profile["enc_scale_err_l"])
    conn.set_enc_scale_error(1, profile["enc_scale_err_r"])


# ---------------------------------------------------------------------------
# Ideal TOUR_1 trajectory -- derived from the SAME parser/segment-builder
# the sim itself dispatches through (legacy_verbs.tokenize_send_line +
# BINARY_DISPATCH), never a hand-rolled re-parse of the wire strings, so a
# future TOUR_1 edit or D/RT wire-format change can never silently drift
# this reference out of sync.
# ---------------------------------------------------------------------------


def _ideal_tour_poses(steps: list[str]) -> list[tuple[float, float, float]]:
    """Return the ideal rigid-body pose (x, y, h_rad) after each step,
    starting from (0, 0, 0).

    Each step's ``MotionSegment`` (arc_length/delta_heading, the v2
    primitive shape -- 100-007, THE CUTOVER) is built by the EXACT SAME
    code the sim dispatches (``legacy_verbs.BINARY_DISPATCH``, now
    returning a list of envelopes per step -- TOUR_1's own two verbs, D and
    RT, always decompose to exactly one primitive each, so ``[0]`` is safe
    here). D (``segment_for_distance()``) is now a PURE translate
    (delta_heading always 0); RT (``segment_for_rt()``) is now a PURE pivot
    (arc_length always 0) -- unlike the pre-cutover 3-phase
    Motion::SegmentExecutor contract this function used to transcribe
    (PRE_PIVOT/TRANSLATE/TERMINAL_PIVOT in one message), so the ideal
    integration collapses to "rotate by delta_heading, then translate
    arc_length along the resulting heading" -- an ideal, zero-slip,
    zero-coast rigid-body model of what the firmware's own kinematics are
    DEFINED to command, not an independently-invented one.
    """
    x = y = h = 0.0
    poses: list[tuple[float, float, float]] = []
    for line in steps:
        stripped, _corr_id = legacy_verbs.split_corr_id(line)
        verb, pos, kv = legacy_verbs.tokenize_send_line(stripped)
        envs = legacy_verbs.BINARY_DISPATCH[verb](pos, kv)
        seg = envs[0].segment
        h += seg.delta_heading
        x += seg.arc_length * math.cos(h)
        y += seg.arc_length * math.sin(h)
        poses.append((x, y, h))
    return poses


def _step_kind(line: str) -> str:
    """Return 'D' or 'RT' (TOUR_1's only two verbs) for a wire step."""
    return line.split()[0].upper()


# ---------------------------------------------------------------------------
# Sim tour runner -- drives one step at a time, polling the SAME `active`
# (bb.drivetrain.busy) binary-telemetry completion signal __main__.py's
# _TourRunner._wait_for_idle() uses, via the tick-thread's own continuous
# STREAM channel (conn.drain_binary_tlm()) rather than SNAP round trips --
# a closer match to what SimTransport's tick-thread actually does on every
# iteration.
# ---------------------------------------------------------------------------


def _run_tour_and_collect(conn: "SimConnection", proto: "NezhaProtocol",
                          steps: list[str]) -> tuple[
        list[tuple[float, float, float]], list[tuple[float, float]]]:
    """Run ``steps`` to completion; return (plant_poses_after_each_step,
    dead_reckoned_xy_after_each_step).

    ``plant_poses_after_each_step``: ground-truth (x, y, h_rad) sampled the
    moment each step's ``active`` flag drops (``conn.get_true_pose()``).

    ``dead_reckoned_xy_after_each_step``: the SAME step-boundary moments,
    but fed through ``EncoderDeadReckoner`` driven by every REPORTED
    encoder reading (``TLMFrame.enc``) observed along the way -- exactly
    what the GUI's canvas avatar would draw -- so the tour's dead-reckoned
    trace can be compared against the plant ground truth it should be
    faithfully tracking.
    """
    reckoner = EncoderDeadReckoner(_TRACKWIDTH)
    plant_poses: list[tuple[float, float, float]] = []
    dr_xy: list[tuple[float, float]] = []

    # Arm continuous binary telemetry once, mirroring SimTransport._tick_loop's
    # own connect-time "STREAM 50" arm (here at the protocol floor, 20ms).
    reply = binary_bridge.translate_command(proto, f"STREAM {_STREAM_PERIOD}")
    assert "OK" in reply or reply == "", f"STREAM arm failed: {reply!r}"
    conn.drain_binary_tlm()  # drop anything already queued

    for step in steps:
        reply = binary_bridge.translate_command(proto, step)
        assert reply.startswith("OK"), f"{step!r} rejected: {reply!r}"

        max_ticks = int(_STEP_TIMEOUT_S * 1000 / _STEP)
        last_active: bool | None = None
        settled = False
        for i in range(max_ticks):
            conn.tick(_STEP)
            frames = conn.drain_binary_tlm()
            for f in frames:
                tlm = f.tlm
                last_active = bool(tlm.active)
                if tlm.has_enc:
                    dr_xy_now = reckoner.update(float(tlm.enc_left), float(tlm.enc_right))
                    dr_xy_last = (dr_xy_now[0], dr_xy_now[1])
            if i >= _SPINUP_TICKS and last_active is False:
                settled = True
                break
        assert settled, (
            f"{step!r} did not settle (active=False) within "
            f"{_STEP_TIMEOUT_S:.0f}s sim time"
        )

        pose = conn.get_true_pose()
        plant_poses.append((pose["x"], pose["y"], pose["h"]))
        dr_xy.append(dr_xy_last)

    return plant_poses, dr_xy


@pytest.fixture
def tour_conn():
    """A fresh, zero-error, tovez-no-cal-geometry SimConnection + NezhaProtocol
    pair, torn down at the end of the test."""
    conn = SimConnection(tick_step=_STEP)
    result = conn.connect()
    assert "error" not in result, f"sim connect failed: {result}"
    _apply_zero_error_profile(conn)
    proto = NezhaProtocol(conn)
    try:
        yield conn, proto
    finally:
        conn.disconnect()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_firmware_kinematics_trackwidth_matches_plant_trackwidth(tour_conn):
    """The firmware's OWN kinematics trackwidth (``Subsystems::Drivetrain::
    config_.trackwidth``, read back via ``GET tw``) equals the value the
    plant's ground-truth physics is configured with (``_TRACKWIDTH``) --
    the precise, direct statement of the wheelbase-consistency bug this
    ticket fixes, with NO dependency on ``Motion::SegmentExecutor``'s own
    (separately tracked, independent) rotation-phase accuracy. FAILS
    before the fix (firmware defaults to ``PhysicsWorld::
    kDefaultTrackwidth`` = 150mm regardless of what the plant is set to);
    PASSES after it (both derive from the same, now-128mm, default).
    """
    from robot_radio.robot.pb2 import config_pb2

    conn, proto = tour_conn
    snapshot = proto.get_config_binary(config_pb2.CONFIG_DRIVETRAIN)
    assert snapshot is not None, "GET drivetrain config returned no snapshot"
    assert snapshot.drivetrain.trackwidth == pytest.approx(_TRACKWIDTH), (
        f"firmware kinematics trackwidth={snapshot.drivetrain.trackwidth}mm, "
        f"plant trackwidth={_TRACKWIDTH}mm -- MISMATCH: RT/D stop-target "
        "arcs are computed against the wrong wheelbase"
    )


def test_isolated_translate_leg_travels_commanded_distance(tour_conn):
    """A single, un-chained ``D`` command (TOUR_1's first leg, 345mm) travels
    the commanded distance, straight, from a cold start -- proves the
    wheelbase fix (firmware kinematics == plant == the project's real
    128mm trackwidth) for the TRANSLATE phase in complete isolation from
    everything else a chained tour could contaminate it with (a prior RT's
    residual motor state, accumulated heading error, ...).

    Deliberately NOT run as part of the full TOUR_1 sequence (see
    ``test_tour1_closes_the_loop``'s per-step breakdown): chaining a D leg
    immediately after an RT leg picks up a small (~15-20mm) excess-travel
    residual from the SAME rotation-phase stop-decel family this module's
    docstring documents (maybeReplanPivot/armPivotStopDecel leaving
    residual motor state at the hand-off) -- a real, separate, already
    out-of-scope characteristic, not a translate-accuracy regression. This
    test isolates the one claim it is meant to prove: TRANSLATE-phase
    encoder tracking, on a clean start, is accurate now that trackwidth is
    consistent.
    """
    conn, proto = tour_conn
    reply = binary_bridge.translate_command(proto, "D 200 200 345")
    assert reply.startswith("OK"), f"D 200 200 345 rejected: {reply!r}"
    for _ in range(int(_STEP_TIMEOUT_S * 1000 / _STEP)):
        conn.tick(_STEP)
    pose = conn.get_true_pose()
    traveled = math.hypot(pose["x"], pose["y"])
    assert abs(traveled - 345.0) <= 5.0, (  # [mm]
        f"isolated D 200 200 345 traveled {traveled:.2f}mm, expected ~345mm"
    )
    assert abs(math.degrees(pose["h"])) <= 1.0, (  # [deg]
        f"isolated D 200 200 345 changed heading by {math.degrees(pose['h']):.2f}deg, "
        "expected ~0 (D never rotates)"
    )


def test_isolated_rotation_leg_reveals_independent_residual(tour_conn):
    """A single, un-chained ``RT 9000`` command (TOUR_1's own RT step) from
    a cold start, with trackwidth fully self-consistent, rotates to ~90 deg.

    (Was xfail(strict) while this over-rotated to ~110 deg. Fixed
    2026-07-11: the residual was never the executor's plan -- its emitted
    omega integral was EXACTLY 90 deg -- but the sim plant overdriving
    every wheel ~1.25x its setpoint (stale hand-typed kff=0.0038 vs the
    plant's exact 1/kNominalMaxSpeed=0.0025; sim_api.cpp
    defaultMotorConfigSet()), compounded by segment_executor.cpp's sim
    kOutputHops modeling actuation lag the calibrated plant doesn't have.)
    """
    conn, proto = tour_conn
    reply = binary_bridge.translate_command(proto, "RT 9000")
    assert reply.startswith("OK"), f"RT 9000 rejected: {reply!r}"
    for _ in range(int(_STEP_TIMEOUT_S * 1000 / _STEP)):
        conn.tick(_STEP)
    pose = conn.get_true_pose()
    heading_deg = math.degrees(pose["h"])
    assert abs(heading_deg - 90.0) <= 5.0, (
        f"isolated RT 9000 rotated to {heading_deg:.2f}deg, expected ~90deg"
    )


def test_tour1_dead_reckoning_matches_plant_ground_truth(tour_conn):
    """The host-side EncoderDeadReckoner (what the GUI's canvas avatar
    draws) stays close to the plant's own ground-truth position throughout
    TOUR_1 -- the avatar is faithful to what the robot actually did, even
    where that deviates from the IDEAL target (the RT residual, see module
    docstring). Both integrate the SAME reported encoder stream with the
    SAME (now self-consistent) trackwidth, so they should never diverge by
    more than a small fraction of the tour's own leg lengths.
    """
    conn, proto = tour_conn
    plant_poses, dr_xy = _run_tour_and_collect(conn, proto, TOUR_1)

    failures = []
    for i, (step, (px, py, _ph), (dx, dy)) in enumerate(zip(TOUR_1, plant_poses, dr_xy)):
        dist = math.hypot(dx - px, dy - py)
        if dist > 15.0:  # [mm]
            failures.append(
                f"  step {i + 1} {step!r}: plant=({px:.1f},{py:.1f}) "
                f"dead-reckoned=({dx:.1f},{dy:.1f}) dist_err={dist:.2f}mm"
            )
    assert not failures, (
        "dead-reckoned avatar pose diverged from plant ground truth by "
        ">15mm:\n" + "\n".join(failures)
    )


def test_tour1_closes_the_loop(tour_conn):
    """TOUR_1 (as the TestGUI runs it) returns to within a tight tolerance
    of world origin at the ideal final heading (180 deg), and every step's
    plant pose (not just the endpoint) tracks the ideal trajectory --
    localizing where any drift accumulates, not just whether errors happen
    to cancel by the end.

    (Was xfail(strict) while each RT 9000 over-rotated ~+20 deg and the
    tour missed closure by ~199mm/+119deg. Fixed 2026-07-11 -- see
    test_isolated_rotation_leg_reveals_independent_residual's docstring
    for the root cause: sim plant velocity-gain miscalibration + phantom
    dead-time modeling, never the executor's plan.)
    """
    conn, proto = tour_conn
    ideal = _ideal_tour_poses(TOUR_1)
    actual, _dr = _run_tour_and_collect(conn, proto, TOUR_1)

    # Per-step trajectory match (localizes drift -- reported even though
    # only the final assertion below is load-bearing for xfail purposes).
    failures = []
    for i, (step, (ix, iy, ih), (ax, ay, ah)) in enumerate(zip(TOUR_1, ideal, actual)):
        dist = math.hypot(ax - ix, ay - iy)
        heading_err = math.degrees(
            math.atan2(math.sin(ah - ih), math.cos(ah - ih))
        )
        ok = dist <= 15.0 and abs(heading_err) <= 5.0
        marker = "" if ok else "  <-- FAIL"
        failures.append(
            f"  step {i + 1:2d} {step!r:>18}: ideal=({ix:7.1f},{iy:7.1f},"
            f"{math.degrees(ih):7.1f}deg) actual=({ax:7.1f},{ay:7.1f},"
            f"{math.degrees(ah):7.1f}deg) dist_err={dist:6.2f}mm "
            f"heading_err={heading_err:+7.2f}deg{marker}"
        )
    per_step_report = "\n".join(failures)

    ix, iy, ih = ideal[-1]
    ax, ay, ah = actual[-1]
    final_dist = math.hypot(ax - ix, ay - iy)
    final_heading_err = math.degrees(
        math.atan2(math.sin(ah - ih), math.cos(ah - ih))
    )

    # Tolerance rationale (2026-07-11, final calibration): with the sim
    # plant calibrated (exact feed-forward, honest velocity filter,
    # measured 1.5-pass dead time) and the executor's stops riding each
    # position solve's own to-rest tail, pivots land within ~0.5 deg and
    # D legs within ~1mm -- the measured whole-tour closure is ~4mm /
    # ~1.6 deg. 15mm/5deg leaves honest headroom; the pre-fix defect this
    # guards against was 199mm/+119deg.
    assert final_dist <= 15.0 and abs(final_heading_err) <= 5.0, (
        f"TOUR_1 did not close: final plant pose ({ax:.1f}, {ay:.1f}, "
        f"{math.degrees(ah):.1f}deg) vs ideal ({ix:.1f}, {iy:.1f}, "
        f"{math.degrees(ih):.1f}deg) -- dist_err={final_dist:.2f}mm "
        f"heading_err={final_heading_err:+.2f}deg (tolerance: 15mm / 5deg)\n"
        f"Per-step trajectory:\n{per_step_report}"
    )
