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


def test_goto_world_converges_to_a_nearby_target():
    from robot_radio.pathplan.planner import GiveUpLimits, gotoWorld
    from robot_radio.pathplan.solver import SolverLimits
    from robot_radio.pathplan.world_pose import WorldPose

    loop, proto = _makeSimProto()
    try:
        worldPose = WorldPose()

        # Seed WorldPose from real telemetry, then re-anchor from the
        # sim's own ground truth -- the sim-tier stand-in for a startup
        # camera fix (module docstring above).
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

        # A nearby target: mostly straight ahead in the sim's own start
        # heading, slightly offset -- small enough for a fast sim smoke
        # test, well above the >=100 mm carrot-distance floor.
        import math
        cosH, sinH = math.cos(startH), math.sin(startH)
        forward, left = _TARGET_DISTANCE * 0.95, _TARGET_DISTANCE * 0.20
        targetX_mm = startX + forward * cosH - left * sinH
        targetY_mm = startY + forward * sinH + left * cosH

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


if __name__ == "__main__":
    test_goto_world_converges_to_a_nearby_target()
    print("OK")
