"""robot_radio.pathplan — host-side thin `GO_TO` senders (sprint 135;
originally the sprint-127 host-side outer position loop).

New package for the design issue's T3-T8 work, originally a host-side
outer loop over the firmware's relative-move `Move` primitives; sprint 135
moved single-target arc-solving into the firmware itself (`Motion::
Navigator`) and shrank this package to thin `GO_TO` senders (135-007) --
see `planner.py`'s own module docstring for the full history. `planner/`
is taken by trajectory profiles from the pre-gut era; `path/` is taken by
pure curve geometry -- hence `pathplan/`.

Exports (grown incrementally, ticket by ticket):
  WorldPose         — host-side T_world_from_odom tracker, re-anchored from
                       camera fixes (127-004).
  Transform2        — the SE(2) transform WorldPose owns.
  PoseDivergence    — the encoder-vs-OTOS world-pose divergence WorldPose
                       exposes as a first-class output.
  pursuitTarget     — the lookahead-circle pure-pursuit target picker
                       followPath() streams as GO_TO commands (127-005/
                       135-007) -- the ONE piece of arc/path geometry that
                       stayed host-side; see solver.py's own docstring.
  gotoWorld         — thin GO_TO sender, world-frame target (127-006,
                       shrunk 135-007).
  gotoRobot         — thin GO_TO sender, robot-frame target (127-006,
                       shrunk 135-007) -- a peer of gotoWorld, not a
                       composition through it (the firmware itself now
                       resolves the robot-frame offset at acceptance).
  followPath        — streams pursuitTarget()'s picks as GO_TO commands
                       along a waypoint path (127-007/135-007).
  GotoResult        — gotoWorld()/gotoRobot()'s return type.
  FollowPathResult  — followPath()'s return type.
  GiveUpLimits      — the give-up policy (iteration/timeout caps).
  AckRetry          — the enqueue-ack verification/retry policy.
  MoveIdAllocator   — the strictly-monotonic Move.id/GoTo.id source a
                       caller issuing multiple sequential goto/Move calls
                       must share.
"""

from robot_radio.pathplan.planner import (
    AckRetry,
    FollowPathResult,
    GiveUpLimits,
    GotoResult,
    MoveIdAllocator,
    followPath,
    gotoRobot,
    gotoWorld,
)
from robot_radio.pathplan.solver import pursuitTarget
from robot_radio.pathplan.world_pose import PoseDivergence, Transform2, WorldPose

__all__ = [
    "WorldPose",
    "Transform2",
    "PoseDivergence",
    "pursuitTarget",
    "gotoWorld",
    "gotoRobot",
    "followPath",
    "GotoResult",
    "FollowPathResult",
    "GiveUpLimits",
    "AckRetry",
    "MoveIdAllocator",
]
