"""robot_radio.robot.sync_pose — Pure helpers for camera-to-firmware pose sync.

These helpers extract the core logic from ``cli.py:cmd_sync_pose`` and
``cli.py:_daemon_read_pose`` so it can be reused by the Robot Test GUI without
pulling in the CLI's argparse setup or ``sys.exit`` calls.

Resolution of OQ-1 (cmd_sync_pose importability)
-------------------------------------------------
``cli.py`` uses argparse (not Click) and ``cmd_sync_pose`` is a plain function,
so it is technically importable.  However, it calls ``sys.exit()`` on errors and
opens its own robot connection — neither is usable from the GUI.  This module
extracts the two pure operations:

1. ``daemon_read_pose`` — poll the aprilcam daemon for a tag's world pose.
2. ``pose_to_setpose_line`` — convert a world pose to a firmware ``SI`` wire string.

The GUI's sync-pose handler calls both in sequence, then passes the wire string
to ``transport.command()``.  All aprilcam imports are deferred so this module is
importable without the aprilcam package installed.

Wire command
------------
The GUI sends ``SI <x_mm> <y_mm> <h_cdeg>`` — the motion controller's internal
pose (``Odometry::setPose``).  This is preferred over ``OV`` because OV writes
the raw OTOS chip registers which are subsequently rotated by the OTOS mount
angle (``odomYawDeg``), causing a position error.  SI writes the pose the
firmware's motion controller drives against directly.

Unit conversion (mirrors cli.py:cmd_sync_pose)
-----------------------------------------------
- world_xy is in cm (daemon A1-centred frame) → SI wants mm: x_mm = round(x_cm * 10)
- heading: daemon reports tag orientation in world frame (0 = east, CCW+).
  That orientation IS the robot's forward heading — no offset.
  SI expects centi-degrees: h_cdeg = round(degrees(yaw_rad) * 100)
"""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    # aprilcam types — only for IDE / type-checker; never imported at runtime
    # unless aprilcam is installed.
    pass


def daemon_read_pose(
    dc: "object",
    cam: "object",
    tag_id: int = 100,
    timeout_s: float = 2.0,
) -> tuple[float, float, float] | None:
    """Read (x_cm, y_cm, yaw_rad) for *tag_id* from the aprilcam daemon.

    Parameters
    ----------
    dc:
        A connected ``DaemonControl`` instance (``aprilcam.client.control``).
    cam:
        Camera object returned by ``dc.list_cameras()[0]``.
    tag_id:
        AprilTag ID to read.  Default 100 (robot tag).
    timeout_s:
        Maximum seconds to poll for a calibrated reading.

    Returns
    -------
    tuple[float, float, float] or None
        ``(x_cm, y_cm, yaw_rad)`` in the world A1-centred frame (cm, rad),
        or ``None`` if the tag is not seen with calibrated world_xy within
        *timeout_s*.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        tf = dc.get_tags(cam)
        for t in tf.tags:
            if t.id == tag_id and t.world_xy is not None and t.yaw is not None:
                return float(t.world_xy[0]), float(t.world_xy[1]), float(t.yaw)
        time.sleep(0.03)
    return None


def pose_to_setpose_line(x_cm: float, y_cm: float, yaw_rad: float) -> str:
    """Convert a daemon world pose to a firmware ``SI`` wire string.

    Parameters
    ----------
    x_cm, y_cm:
        World position in centimetres (A1-centred frame, +x east, +y north).
    yaw_rad:
        Robot forward heading in radians (0 = east, CCW-positive).

    Returns
    -------
    str
        Ready-to-send wire string, e.g. ``"SI 1230 450 2700"``.

    Notes
    -----
    Conversion mirrors ``cli.py:cmd_sync_pose``:

    - x_mm = round(x_cm * 10)
    - y_mm = round(y_cm * 10)
    - h_cdeg = round(degrees(yaw_rad) * 100)
    """
    x_mm = round(x_cm * 10)
    y_mm = round(y_cm * 10)
    h_cdeg = round(math.degrees(yaw_rad) * 100)
    return f"SI {x_mm} {y_mm} {h_cdeg}"
