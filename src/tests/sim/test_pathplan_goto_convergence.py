"""src/tests/sim/test_pathplan_goto_convergence.py -- ticket 127-006's own
sim-level convergence smoke test originally (design issue T5/T6); reshaped
135-007.

Full end-to-end `gotoWorld()`/`gotoRobot()`/`followPath()` -- telemetry
in, `WorldPose.ingest()`, `go_to()` out, wait for the ack ring's single
completion ack -- run against the REAL firmware loop compiled into the sim
dylib (`Motion::Navigator`, sprint 135 tickets 002-004), via
`NezhaProtocol` wrapping `SimConfigConn(SimLoop)`: the SAME interface a
hardware connection uses, proving the THIN SENDERS actually work
end-to-end against a real navigator, not just that their pure decision
helpers are individually correct (`src/tests/unit/test_planner.py` covers
those against hand-built fakes).

History (135-007): before this ticket, this file proved the HOST's OWN
arc-solving/replace-throttling loop converged, and specifically that it
was IMMUNE to injected OTOS drift/encoder slip because it navigated by its
own encoder-anchored `WorldPose`, never by OTOS. That whole premise is
gone: `gotoWorld()`/`gotoRobot()` no longer navigate by `WorldPose` at all
-- they send ONE `GO_TO` and wait for the firmware's own completion ack,
and `Motion::Navigator` navigates by the REAL OTOS pose internally, with
its own bounded staleness/disconnect handling (SUC-005). This file's own
former `test_goto_world_converges_under_otos_drift()`/
`test_goto_world_stays_sane_under_enc_slip()` are DELETED, not adapted --
injecting a sim-side `WorldPose` fault into a code path that no longer
reads `WorldPose` for navigation would prove nothing; OTOS staleness/
disconnect robustness is now a FIRMWARE property, ctest-covered by
`src/motion/navigator/tests/navigator_test.cpp` (tickets 002/003 of
sprint 135) instead. What replaces the deleted tests: smoke coverage that
the reshaped Python senders themselves work end-to-end against a real
`Motion::Navigator`, for both frames (`gotoWorld()`/`gotoRobot()`) and for
`followPath()`'s streamed-target mode.

Run:
    uv run python -m pytest src/tests/sim/test_pathplan_goto_convergence.py -v -s

Requires the compiled `src/firm/platform/host/build/libfirmware_host.{dylib,so}` --
skips cleanly if not present.
"""
from __future__ import annotations

import math
import pathlib
import sys
import time

import pytest

# src/tests/sim/test_pathplan_goto_convergence.py -> sim -> tests -> src ->
# repo root = THREE hops from __file__ (matches test_motor_primitive.py's
# own established convention for a file at this same shallower depth,
# src/tests/sim/, not src/tests/sim/system/).
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_ROBOTS_DIR = _REPO_ROOT / "data" / "robots"

_LIB_NAME = "libfirmware_host.dylib" if sys.platform == "darwin" else "libfirmware_host.so"
_SIM_LIB_PATH = _REPO_ROOT / "src" / "firm" / "platform" / "host" / "build" / _LIB_NAME

pytestmark = pytest.mark.skipif(
    not _SIM_LIB_PATH.exists(),
    reason="sim lib not built -- cmake --build src/firm/platform/host/build (or `python build.py`)",
)

_TRACK_WIDTH = 128.0  # [mm] tovez_nocal.json geometry.trackwidth
_SPEED = 150.0        # [mm/s] cruise speed -- matches otos_calibration_bench.py's own CRUISE
_TARGET_DISTANCE = 300.0  # [mm] well above a nearby-target smoke's own noise floor
_STARTUP_TELEMETRY_TRIES = 200  # ~2s at the sim's own cycle rate
_GROUND_TRUTH_SLACK_MM = 220.0  # generous slack over sim plant lag/overshoot between the last replace and rest


def _makeSimProto():
    """A bare, headless SimLoop -- REAL-TIME ticking (start_tick_thread=
    True, the default): gotoWorld()'s/gotoRobot()'s/followPath()'s own
    _advance() polls telemetry on wall-clock time exactly like a hardware
    connection (never calls SimLoop.step() itself), so the sim must be
    advancing on its own in the background -- the same "no testgui, no
    Qt" headless pattern test_sim_configure_from_robot.py already
    establishes."""
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


def _waitForFirstTelemetry(loop) -> "list":
    """Drain real telemetry frames until at least one arrives (~2s budget
    at the sim's own cycle rate) -- confirms the sim/firmware is actually
    ticking and configured before this file's tests issue their first
    `GO_TO`. Returns whatever frames were drained on the successful
    iteration; does NOT feed them into any `WorldPose` -- callers that
    want a `WorldPose` fix do that themselves (see `_seedAndReanchor()`
    below), and `test_goto_robot_...` deliberately does not, to prove
    `gotoRobot()` needs no pre-existing fix."""
    for _ in range(_STARTUP_TELEMETRY_TRIES):
        frames = loop.read_pending_binary_tlm_frames()
        if frames:
            return frames
        time.sleep(0.01)  # let the background tick thread advance real time
    raise AssertionError("no telemetry received from the sim within the startup window")


def _seedAndReanchor(loop, worldPose) -> "tuple[float, float, float]":
    """Shared setup: seed `worldPose` from real telemetry, then re-anchor
    from the sim's own ground truth (the sim-tier stand-in for a startup
    camera fix). Returns the start pose `(x_mm, y_mm, h_rad)` used as the
    re-anchor fix. `followPath()` genuinely needs this (its own
    `pursuitTarget()` call reads `worldPose.worldPose()` every cycle to
    pick the next target) -- `gotoWorld()`/`gotoRobot()` no longer need
    it for navigation, but a `WorldPose` this file's own `finalPose`/
    ground-truth-comparison assertions can trust is still convenient
    for those tests too, so both use it."""
    frames = _waitForFirstTelemetry(loop)
    for frame in frames:
        worldPose.ingest(frame)

    startX, startY, startH = _truePose(loop)
    worldPose.reanchor((startX / 10.0, startY / 10.0, startH))  # [mm] -> [cm]
    return startX, startY, startH


def _forwardLeftTarget(startX: float, startY: float, startH: float,
                       forwardFrac: float = 0.95, leftFrac: float = 0.20
                       ) -> "tuple[float, float]":
    """A target `_TARGET_DISTANCE` [mm] ahead of `(startX, startY, startH)`,
    mostly straight ahead with a small left offset."""
    cosH, sinH = math.cos(startH), math.sin(startH)
    forward, left = _TARGET_DISTANCE * forwardFrac, _TARGET_DISTANCE * leftFrac
    return (startX + forward * cosH - left * sinH,
            startY + forward * sinH + left * cosH)


def test_goto_world_converges_to_a_nearby_target():
    from robot_radio.pathplan.planner import GiveUpLimits, gotoWorld
    from robot_radio.pathplan.world_pose import WorldPose

    loop, proto = _makeSimProto()
    try:
        worldPose = WorldPose()
        startX, startY, startH = _seedAndReanchor(loop, worldPose)
        targetX_mm, targetY_mm = _forwardLeftTarget(startX, startY, startH)

        result = gotoWorld(
            proto, worldPose, targetX_mm / 10.0, targetY_mm / 10.0,
            geofence=None,
            giveUp=GiveUpLimits(maxIterations=400, giveUpTimeout=25.0),
        )

        print(f"gotoWorld sim smoke: success={result.success} reason={result.reason!r} "
              f"iterations={result.iterations} sent={result.sent}")
        assert result.success, f"gotoWorld() did not converge: {result.reason}"

        endX, endY, endH = _truePose(loop)
        residual = math.hypot(endX - targetX_mm, endY - targetY_mm)
        print(f"  true final pose=({endX:.1f},{endY:.1f},{endH:.3f} rad) "
              f"ground-truth residual={residual:.1f} mm")
        # gotoWorld() no longer judges its own arrival at all (135-007) --
        # Motion::Navigator's own completion ack is the sole success
        # signal (`result.success`, checked above). This is an INDEPENDENT
        # check against ground truth, not the firmware's own belief, with
        # slack for sim plant lag/overshoot between the last internal
        # replace and rest.
        assert residual < _GROUND_TRUTH_SLACK_MM, (
            f"ground-truth residual {residual:.1f} mm exceeds the smoke test's "
            f"own slack bound (target distance was {_TARGET_DISTANCE:.0f} mm)")
    finally:
        proto.estop()
        loop.disconnect()


def test_goto_robot_converges_without_any_preexisting_world_pose_fix():
    """gotoRobot() sim smoke, AND the property that motivated the
    rewrite: 135-007's ``gotoRobot()`` no longer transforms its own
    target -- the FIRMWARE resolves a ROBOT-frame ``GO_TO`` to world
    coordinates once, at acceptance, from its own live OTOS pose. Unlike
    the pre-135-007 version of this function (which raised ``RuntimeError``
    without a seeded ``WorldPose`` to compose the offset against), this
    call passes a totally fresh, NEVER-ingested ``WorldPose`` -- proving
    the property directly rather than merely asserting it never got
    involved."""
    from robot_radio.pathplan.planner import GiveUpLimits, gotoRobot
    from robot_radio.pathplan.world_pose import WorldPose

    loop, proto = _makeSimProto()
    try:
        _waitForFirstTelemetry(loop)  # confirm the sim/firmware is live -- discarded, never ingested

        worldPose = WorldPose()
        assert worldPose.worldPose() is None  # genuinely unseeded, by construction

        startX, startY, startH = _truePose(loop)  # ground truth ONLY -- never fed to gotoRobot()
        cosH, sinH = math.cos(startH), math.sin(startH)
        forward, left = _TARGET_DISTANCE * 0.95, _TARGET_DISTANCE * 0.20
        targetX_mm = startX + forward * cosH - left * sinH
        targetY_mm = startY + forward * sinH + left * cosH

        result = gotoRobot(
            proto, worldPose, forward / 10.0, left / 10.0,
            geofence=None,
            giveUp=GiveUpLimits(maxIterations=400, giveUpTimeout=25.0),
        )

        print(f"gotoRobot sim smoke: success={result.success} reason={result.reason!r} "
              f"iterations={result.iterations} sent={result.sent}")
        assert result.success, f"gotoRobot() did not converge: {result.reason}"

        endX, endY, endH = _truePose(loop)
        residual = math.hypot(endX - targetX_mm, endY - targetY_mm)
        print(f"  true final pose=({endX:.1f},{endY:.1f},{endH:.3f} rad) "
              f"ground-truth residual={residual:.1f} mm")
        assert residual < _GROUND_TRUTH_SLACK_MM, (
            f"ground-truth residual {residual:.1f} mm exceeds the smoke test's "
            f"own slack bound (target distance was {_TARGET_DISTANCE:.0f} mm)")
    finally:
        proto.estop()
        loop.disconnect()


def test_follow_path_streams_targets_and_reaches_the_terminal_waypoint():
    """followPath() sim smoke (135-007): streams pursuitTarget()'s picks
    as GO_TO commands along a short, gently-forward path and confirms the
    robot reaches the terminal waypoint -- the sim-tier proof that a
    reshaped followPath() still drives a real Motion::Navigator through a
    multi-waypoint run, not just that pursuitTarget()'s own geometry is
    correct in isolation (test_solver.py) or that the send/throttle
    bookkeeping is correct against hand-built fakes
    (test_planner.py)."""
    from robot_radio.nav.pose import Pose
    from robot_radio.pathplan.planner import GiveUpLimits, followPath
    from robot_radio.pathplan.world_pose import WorldPose

    loop, proto = _makeSimProto()
    try:
        worldPose = WorldPose()
        startX, startY, startH = _seedAndReanchor(loop, worldPose)
        cosH, sinH = math.cos(startH), math.sin(startH)

        def _forwardCm(distanceMm: float) -> "tuple[float, float]":
            return ((startX + distanceMm * cosH) / 10.0, (startY + distanceMm * sinH) / 10.0)

        waypoints = [Pose(x=x, y=y, heading=0.0)
                    for x, y in (_forwardCm(150.0), _forwardCm(300.0), _forwardCm(450.0))]

        result = followPath(
            proto, worldPose, waypoints, speed=_SPEED, geofence=None, tolerance=60.0,
            giveUp=GiveUpLimits(maxIterations=800, giveUpTimeout=25.0))

        print(f"followPath sim smoke: success={result.success} reason={result.reason!r} "
              f"waypointsReached={result.waypointsReached}/{len(waypoints)} "
              f"iterations={result.iterations} sent={result.sent}")
        assert result.success, f"followPath() did not reach the terminal waypoint: {result.reason}"
        assert result.waypointsReached == len(waypoints)

        endX, endY, endH = _truePose(loop)
        targetX_mm, targetY_mm = startX + 450.0 * cosH, startY + 450.0 * sinH
        residual = math.hypot(endX - targetX_mm, endY - targetY_mm)
        print(f"  true final pose=({endX:.1f},{endY:.1f},{endH:.3f} rad) "
              f"ground-truth residual={residual:.1f} mm")
        assert residual < _GROUND_TRUTH_SLACK_MM, (
            f"ground-truth residual {residual:.1f} mm exceeds the smoke test's own slack bound")
    finally:
        proto.estop()
        loop.disconnect()


if __name__ == "__main__":
    test_goto_world_converges_to_a_nearby_target()
    test_goto_robot_converges_without_any_preexisting_world_pose_fix()
    test_follow_path_streams_targets_and_reaches_the_terminal_waypoint()
    print("OK")
