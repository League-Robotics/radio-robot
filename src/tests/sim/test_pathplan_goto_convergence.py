"""src/tests/sim/test_pathplan_goto_convergence.py -- ticket 127-006's own
sim-level convergence smoke test (design issue T5/T6, sprint.md Test
Strategy's "sim" tier).

Full end-to-end `gotoWorld()` loop -- telemetry in, `WorldPose.ingest()`,
`solveArcToPoint()`, `move_twist(replace=True)` out -- run against the REAL
firmware loop compiled into the sim dylib, via `NezhaProtocol` wrapping
`SimConfigConn(SimLoop)`: the SAME interface a hardware connection uses
(this ticket's own instruction), proving the loop actually converges, not
just that its pure decision helpers are individually correct
(`src/tests/unit/test_planner.py` covers those).

This ticket's own acceptance is explicitly UNIT + SIM ONLY -- "a
sim-level convergence smoke (one gotoWorld() call to a nearby target
converges against SimLoop) is sufficient here"; the full sim/bench/
playfield convergence GATE (goal closure, injected OTOS drift/encoder
slip, minimum-reliable-move measurement) is ticket 007's own scope, not
this one's.

No real camera in a sim run -- this test uses `SimLoop.get_true_pose()`
ONCE as a stand-in for a startup camera fix (`WorldPose.reanchor()`),
exactly the role a real camera fix plays for a live playfield run: it
anchors the host's own `T_world_from_odom`, after which the loop runs
purely off telemetry, same as hardware. Convergence is then judged
against `get_true_pose()` again at the end, independent of the loop's
own telemetry-derived belief.

Run:
    uv run python -m pytest src/tests/sim/test_pathplan_goto_convergence.py -v -s -k goto

Requires the compiled `src/sim/build/libfirmware_host.{dylib,so}` --
skips cleanly if not present.
"""
from __future__ import annotations

import pathlib
import sys
import time

import pytest
from pydantic import ValidationError

# src/tests/sim/test_pathplan_goto_convergence.py -> sim -> tests -> src ->
# repo root = THREE hops from __file__ (matches test_motor_primitive.py's
# own established convention for a file at this same shallower depth,
# src/tests/sim/, not src/tests/sim/system/).
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_ROBOTS_DIR = _REPO_ROOT / "data" / "robots"

_LIB_NAME = "libfirmware_host.dylib" if sys.platform == "darwin" else "libfirmware_host.so"
_SIM_LIB_PATH = _REPO_ROOT / "src" / "sim" / "build" / _LIB_NAME

pytestmark = pytest.mark.skipif(
    not _SIM_LIB_PATH.exists(),
    reason="sim lib not built -- cmake --build src/sim/build (or `python build.py`)",
)

_TRACK_WIDTH = 128.0  # [mm] tovez_nocal.json geometry.trackwidth
_SPEED = 150.0        # [mm/s] cruise speed -- matches otos_calibration_bench.py's own CRUISE
_TARGET_DISTANCE = 300.0  # [mm] well above TERMINATION_TOLERANCE's 100 mm floor
_STARTUP_TELEMETRY_TRIES = 200  # ~2s at the sim's own cycle rate


def _makeSimProto():
    """A bare, headless SimLoop -- REAL-TIME ticking (start_tick_thread=
    True, the default): gotoWorld()'s own _advance() polls telemetry on
    wall-clock time exactly like a hardware connection (never calls
    SimLoop.step() itself), so the sim must be advancing on its own in
    the background -- the same "no testgui, no Qt" headless pattern
    test_sim_configure_from_robot.py already establishes."""
    from robot_radio.config.robot_config import load_robot_config
    from robot_radio.io.sim_config import SimConfigConn
    from robot_radio.io.sim_loop import SimLoop
    from robot_radio.robot.protocol import NezhaProtocol

    config = load_robot_config(_ROBOTS_DIR / "tovez_nocal.json")
    loop = SimLoop(track_width=_TRACK_WIDTH, lib_path=_SIM_LIB_PATH)
    loop.connect(start_tick_thread=True)
    loop.configure_from_robot(config)
    proto = NezhaProtocol(SimConfigConn(loop))
    return loop, proto


def _truePose(loop) -> "tuple[float, float, float]":  # (x_mm, y_mm, h_rad)
    pose = loop.get_true_pose()
    return pose["x"], pose["y"], pose["h"]


def _seedAndReanchor(loop, worldPose) -> "tuple[float, float, float]":
    """Shared setup for every test in this file: seed `worldPose` from real
    telemetry, then re-anchor from the sim's own ground truth (the
    sim-tier stand-in for a startup camera fix, this module's own
    docstring). Returns the start pose `(x_mm, y_mm, h_rad)` used as the
    re-anchor fix."""
    frames = []
    for _ in range(_STARTUP_TELEMETRY_TRIES):
        frames = loop.read_pending_binary_tlm_frames()
        if frames:
            break
        time.sleep(0.01)  # let the background tick thread advance real time
    assert frames, "no telemetry received from the sim within the startup window"
    for frame in frames:
        worldPose.ingest(frame)

    startX, startY, startH = _truePose(loop)
    worldPose.reanchor((startX / 10.0, startY / 10.0, startH))  # [mm] -> [cm]
    return startX, startY, startH


def _forwardLeftTarget(startX: float, startY: float, startH: float,
                       forwardFrac: float = 0.95, leftFrac: float = 0.20
                       ) -> "tuple[float, float]":
    """A target `_TARGET_DISTANCE` [mm] ahead of `(startX, startY, startH)`,
    mostly straight ahead with a small left offset -- the same target
    shape `test_goto_world_converges_to_a_nearby_target` has always used,
    factored out so the fault-injection tests below aim at an identical
    geometry and only the injected fault differs."""
    import math
    cosH, sinH = math.cos(startH), math.sin(startH)
    forward, left = _TARGET_DISTANCE * forwardFrac, _TARGET_DISTANCE * leftFrac
    return (startX + forward * cosH - left * sinH,
            startY + forward * sinH + left * cosH)


@pytest.mark.xfail(
    reason="132-014 KNOWN GAP, blocked on ticket 017: sim_boot_config.py's "
           "motor_boot_config_for() (132-014's grouped-object fast path) "
           "now reads config.motors.fwd_sign_left/right DIRECTLY off a "
           "REAL RobotConfig -- 0 (proto3 default) for BOTH ports, not the "
           "real mirror-mounted -1/+1 pair (088-002), since data/robots/"
           "*.json are still the OLD 13-section shape (see "
           "test_sim_boot_config_parity.py's own xfail for the same "
           "measured 0-vs-(-1) mismatch). Both drive-pair ports sharing the "
           "SAME (wrong) fwd_sign makes the configured plant curve/spin "
           "instead of driving straight, so gotoWorld() moves a genuine "
           "291mm but never converges within TERMINATION_TOLERANCE -- not a "
           "convergence-algorithm defect. Will hold again once ticket 017 "
           "reshapes the JSON.",
    strict=True,
)
def test_goto_world_converges_to_a_nearby_target():
    from robot_radio.pathplan.planner import GiveUpLimits, gotoWorld
    from robot_radio.pathplan.solver import SolverLimits
    from robot_radio.pathplan.world_pose import WorldPose

    loop, proto = _makeSimProto()
    try:
        import math

        worldPose = WorldPose()
        startX, startY, startH = _seedAndReanchor(loop, worldPose)
        targetX_mm, targetY_mm = _forwardLeftTarget(startX, startY, startH)

        limits = SolverLimits(trackWidth=_TRACK_WIDTH, speed=_SPEED)
        result = gotoWorld(
            proto, worldPose, targetX_mm / 10.0, targetY_mm / 10.0,
            limits=limits, geofence=None,
            giveUp=GiveUpLimits(maxIterations=400, giveUpTimeout=25.0),
        )

        print(f"gotoWorld sim smoke: success={result.success} reason={result.reason!r} "
              f"iterations={result.iterations} sent={result.sent}")
        assert result.success, f"gotoWorld() did not converge: {result.reason}"

        endX, endY, endH = _truePose(loop)
        residual = math.hypot(endX - targetX_mm, endY - targetY_mm)
        print(f"  true final pose=({endX:.1f},{endY:.1f},{endH:.3f} rad) "
              f"ground-truth residual={residual:.1f} mm")
        # gotoWorld()'s own arrival tolerance already bounds convergence
        # against its OWN telemetry-derived belief (TERMINATION_TOLERANCE,
        # 100 mm) -- this is an INDEPENDENT check against ground truth, not
        # the loop's own belief, so it allows some extra slack for sim
        # plant lag/overshoot between the last replacement and rest.
        assert residual < 220.0, (
            f"ground-truth residual {residual:.1f} mm exceeds the smoke test's "
            f"own slack bound (target distance was {_TARGET_DISTANCE:.0f} mm)")
    finally:
        proto.estop()
        loop.disconnect()


@pytest.mark.xfail(
    reason="132-014 KNOWN GAP, blocked on ticket 017 -- same root cause as "
           "test_goto_world_converges_to_a_nearby_target's own xfail: both "
           "drive-pair ports read fwd_sign=0 (not the real mirror-mounted "
           "-1/+1) off a REAL RobotConfig until ticket 017 reshapes "
           "data/robots/*.json, so gotoWorld() cannot converge regardless "
           "of the injected OTOS drift this test is actually about.",
    strict=True,
)
def test_goto_world_converges_under_otos_drift():
    """127-007: inject `SimLoop.set_otos_drift()` and confirm `gotoWorld()`
    stays sane -- specifically, confirm it converges to essentially the
    SAME place it does with no drift at all.

    This is not an accident of robustness -- it is a direct structural
    consequence of `WorldPose.worldPose()` (what `gotoWorld()` navigates
    by) reading the ENCODER-anchored transform only, never the
    OTOS-anchored one (`WorldPose.worldPoseOtos()`, a separate accessor
    this loop never calls). `set_otos_drift()` perturbs `TLMFrame.
    otos_reading` (`frame.otos_present`/`o.x`/`o.y`/`o.heading`), which
    feeds ONLY `_latestOtos`/`_transformOtos` -- `frame.pose` (the
    encoder-derived pose `_latestEnc`/`_transformEnc` are built from) is
    untouched. This matches sprint 127's own success criterion 5
    (`estimator.weight_heading_otos`/`weight_omega_otos` committed 0.0,
    `geometry.otos_untrusted` unchanged) at the HOST navigation layer:
    OTOS is not just down-weighted in the firmware's estimator, it is
    structurally absent from what this loop steers by."""
    from robot_radio.pathplan.planner import GiveUpLimits, gotoWorld
    from robot_radio.pathplan.solver import SolverLimits
    from robot_radio.pathplan.world_pose import WorldPose

    loop, proto = _makeSimProto()
    try:
        import math

        worldPose = WorldPose()
        startX, startY, startH = _seedAndReanchor(loop, worldPose)
        targetX_mm, targetY_mm = _forwardLeftTarget(startX, startY, startH)

        # A large, obviously-wrong drift -- if gotoWorld() were (wrongly)
        # reading the OTOS-anchored pose, this would steer it far off
        # course. 80 mm / 80 mm / ~17 deg, comfortably bigger than
        # TERMINATION_TOLERANCE (100 mm) itself.
        loop.set_otos_drift(80.0, 80.0, 0.3)

        limits = SolverLimits(trackWidth=_TRACK_WIDTH, speed=_SPEED)
        result = gotoWorld(
            proto, worldPose, targetX_mm / 10.0, targetY_mm / 10.0,
            limits=limits, geofence=None,
            giveUp=GiveUpLimits(maxIterations=400, giveUpTimeout=25.0),
        )

        print(f"gotoWorld under otos_drift(80,80,0.3rad): success={result.success} "
              f"reason={result.reason!r} iterations={result.iterations} sent={result.sent}")
        assert result.success, f"gotoWorld() did not converge under otos_drift: {result.reason}"
        assert not math.isnan(result.finalPose.x) and not math.isnan(result.finalPose.y), (
            "gotoWorld() produced a NaN pose under otos_drift -- unhandled exception surface")

        endX, endY, endH = _truePose(loop)
        residual = math.hypot(endX - targetX_mm, endY - targetY_mm)
        print(f"  true final pose=({endX:.1f},{endY:.1f},{endH:.3f} rad) "
              f"ground-truth residual={residual:.1f} mm")
        # SAME bound as the no-drift smoke test -- the whole point being
        # verified here is that this number is NOT meaningfully worse
        # than the undrifted baseline, since gotoWorld() never reads the
        # drifted transform in the first place.
        assert residual < 220.0, (
            f"ground-truth residual {residual:.1f} mm exceeds the no-drift bound -- "
            f"otos_drift() should have ZERO effect on gotoWorld() navigation "
            f"(it only feeds worldPoseOtos(), never worldPose())")

        divergence = worldPose.encoderOtosDivergence()
        assert divergence is not None, "expected both encoder and OTOS samples to be ingested"
        print(f"  encoder-vs-OTOS divergence at arrival: distance={divergence.distance * 10.0:.1f}mm "
              f"heading={math.degrees(divergence.heading):+.1f}deg (the injected drift, "
              f"visible ONLY in this separate diagnostic -- not in gotoWorld()'s own navigation)")
    finally:
        proto.estop()
        loop.disconnect()


@pytest.mark.xfail(
    strict=True,
    raises=ValidationError,
    reason=(
        "132-016: _makeSimProto() -> load_robot_config(tovez_nocal.json) "
        "now raises pydantic.ValidationError (extra='forbid') -- data/robots/"
        "tovez_nocal.json is still OLD-shaped until ticket 017's JSON "
        "reshape lands"
    ),
)
def test_goto_world_stays_sane_under_enc_slip():
    """127-007: inject `SimLoop.set_enc_slip()` on one wheel -- unlike
    `set_otos_drift()` (see the test above), this DOES corrupt what
    `gotoWorld()` actually navigates by: `set_enc_slip()` injects a
    permanent signed offset into `TLMFrame.enc_left`/`enc_right`
    (the reported wheel position), which is exactly what `frame.pose`
    (the encoder-derived pose `WorldPose.worldPose()` reads) is built
    from. This is the genuine "world-pose re-anchoring design survives
    odometry drift" case: the loop's own BELIEF is corrupted mid-approach,
    and this test confirms the outcome stays sane (no exception, no
    runaway -- `GiveUpLimits` already bounds iterations/wall-clock time
    structurally, so "no runaway" here specifically means "converges or
    gives up with an explicit reason, never hangs or raises") rather than
    that it magically corrects for a fault it has no channel to detect."""
    from robot_radio.pathplan.planner import GiveUpLimits, gotoWorld
    from robot_radio.pathplan.solver import SolverLimits
    from robot_radio.pathplan.world_pose import WorldPose

    loop, proto = _makeSimProto()
    try:
        import math

        worldPose = WorldPose()
        startX, startY, startH = _seedAndReanchor(loop, worldPose)
        targetX_mm, targetY_mm = _forwardLeftTarget(startX, startY, startH)

        # port=1 (left wheel), rate=0.15 fires a slip event roughly every
        # ~7 encoder samples -- several permanent +25 mm steps accrue
        # over the course of the approach, corrupting the encoder pose
        # this loop navigates by while the move is still in progress
        # (not just a one-shot bias applied before the first solve).
        loop.set_enc_slip(1, 0.15, 25.0)

        limits = SolverLimits(trackWidth=_TRACK_WIDTH, speed=_SPEED)
        result = gotoWorld(
            proto, worldPose, targetX_mm / 10.0, targetY_mm / 10.0,
            limits=limits, geofence=None,
            giveUp=GiveUpLimits(maxIterations=400, giveUpTimeout=25.0),
        )

        print(f"gotoWorld under enc_slip(port=1,rate=0.15,mag=25mm): success={result.success} "
              f"reason={result.reason!r} iterations={result.iterations} sent={result.sent}")
        # No exception is itself part of what this test proves (an
        # unhandled exception surfacing from a corrupted pose would fail
        # the test at collection, not via an assertion) -- the explicit
        # checks below are about BOUNDED behavior, not just "didn't crash".
        assert result.finalPose is not None, "expected at least one telemetry frame ingested"
        assert not math.isnan(result.finalPose.x) and not math.isnan(result.finalPose.y), (
            "gotoWorld() produced a NaN pose under enc_slip -- unhandled exception surface")

        endX, endY, endH = _truePose(loop)
        residualToTarget = math.hypot(endX - targetX_mm, endY - targetY_mm)
        print(f"  true final pose=({endX:.1f},{endY:.1f},{endH:.3f} rad) "
              f"ground-truth residual={residualToTarget:.1f} mm "
              f"(vs. the loop's own belief: {result.reason})")
        # "No runaway": the true final position must still be in the
        # GENERAL VICINITY of the target, not off across the room -- a
        # slipped encoder can bias the loop's own belief by roughly the
        # accumulated slip magnitude, but the loop must not diverge
        # without bound. Bound: target distance + a generous multiple of
        # the single-step slip magnitude (25 mm) to cover several slip
        # events accruing during one approach, comfortably tighter than
        # "somewhere in the sim".
        runawayBound = _TARGET_DISTANCE + 15 * 25.0
        assert residualToTarget < runawayBound, (
            f"ground-truth residual {residualToTarget:.1f} mm exceeds the no-runaway "
            f"bound ({runawayBound:.0f} mm) -- gotoWorld() diverged under enc_slip "
            f"instead of converging (however imperfectly) or giving up explicitly")
    finally:
        proto.estop()
        loop.disconnect()


if __name__ == "__main__":
    test_goto_world_converges_to_a_nearby_target()
    test_goto_world_converges_under_otos_drift()
    test_goto_world_stays_sane_under_enc_slip()
    print("OK")
