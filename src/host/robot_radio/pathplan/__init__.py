"""robot_radio.pathplan — host-side outer position loop (sprint 127).

New package for the design issue's T3-T8 work (`gotoWorld()`/`gotoRobot()`
and path following, built as a host-side outer loop over the firmware's
existing relative-move `Move` primitives -- no firmware or wire change).
`planner/` is taken by trajectory profiles from the pre-gut era; `path/` is
taken by pure curve geometry -- hence `pathplan/`.

Exports (grown incrementally, ticket by ticket):
  WorldPose         — host-side T_world_from_odom tracker, re-anchored from
                       camera fixes (127-004).
  Transform2        — the SE(2) transform WorldPose owns.
  PoseDivergence    — the encoder-vs-OTOS world-pose divergence WorldPose
                       exposes as a first-class output.
  solveArcToPoint   — the pure single-arc goto solver (127-005).
  SolverLimits      — solveArcToPoint's physical/safety limits (trackWidth,
                       speed, curvature slew limit, target-behind angle).
  ArcSolution       — solveArcToPoint's return type (v_x, omega, arcLength,
                       stop, bearing).
"""

from robot_radio.pathplan.solver import ArcSolution, SolverLimits, solveArcToPoint
from robot_radio.pathplan.world_pose import PoseDivergence, Transform2, WorldPose

__all__ = [
    "WorldPose",
    "Transform2",
    "PoseDivergence",
    "solveArcToPoint",
    "SolverLimits",
    "ArcSolution",
]
