"""push_calibration — send calibration values to firmware.

Resolves the interface duality between the MCP path (NezhaProtocol) and the
CLI path (SerialConnection):

- When passed a ``NezhaProtocol``: if the proto already has a
  ``push_calibration`` method, delegates to it.  Otherwise extracts the
  underlying ``SerialConnection`` (``proto._conn``) and falls through to the
  direct-SET path.  This ensures forward-compatibility with the wiring that
  ticket 028-003 adds to NezhaProtocol.

- When passed a ``SerialConnection``: constructs and sends the v2 SET command
  sequence directly.

Both paths return a result dict with at minimum a ``"status"`` key.

Note: this module does NOT wire push_calibration into cli.py or robot_mcp.py.
That is ticket 028-003.
"""

from __future__ import annotations

import math
import sys
from typing import Any

from robot_radio.calibration.helpers import scale_to_int8


def push_calibration(conn_or_proto: Any, config: Any) -> dict[str, Any]:
    """Push calibration values to firmware.

    Parameters
    ----------
    conn_or_proto:
        Either a :class:`robot_radio.robot.protocol.NezhaProtocol` or a
        :class:`robot_radio.io.serial_conn.SerialConnection`.
    config:
        A :class:`robot_radio.config.robot_config.RobotConfig` (or any object
        with the same attribute structure).

    Returns
    -------
    dict
        ``{"status": "ok", ...}`` on success.  The dict may carry additional
        diagnostic keys (e.g. ``"commands"`` listing the verbs that were sent).
    """
    # Resolve duality: NezhaProtocol vs SerialConnection.
    from robot_radio.robot.protocol import NezhaProtocol
    from robot_radio.io.serial_conn import SerialConnection

    if isinstance(conn_or_proto, NezhaProtocol):
        proto = conn_or_proto
        # If NezhaProtocol has its own push_calibration method (added by
        # ticket 028-003), delegate to it.  Check both the instance and the
        # class so that either a monkey-patched instance attribute or a real
        # class method is found.
        try:
            _push_fn = object.__getattribute__(proto, "push_calibration")
        except AttributeError:
            _push_fn = getattr(type(proto), "push_calibration", None)
        if _push_fn is not None and callable(_push_fn):
            return _push_fn(config)
        # Otherwise extract the underlying connection and fall through.
        conn = proto._conn
    elif isinstance(conn_or_proto, SerialConnection):
        conn = conn_or_proto
    else:
        raise TypeError(
            f"push_calibration expects NezhaProtocol or SerialConnection, "
            f"got {type(conn_or_proto).__name__}"
        )

    return _push_via_conn(conn, config)


def calibration_commands(config: Any) -> list[tuple[str, int]]:
    """Build the v2 calibration wire-command sequence for *config*.

    Pure function — returns ``(command, read_timeout)`` pairs and sends nothing,
    so any transport (SerialConnection, NezhaProtocol, or the TestGUI's
    Transport) can push the same sequence.  Mirrors the logic in
    ``robot_radio.io.cli._push_calibration``; changes there should be
    ported here.

    The sequence:
      1. ``SET ml=<float>``  — mm_per_wheel_deg_left
      2. ``SET mr=<float>``  — mm_per_wheel_deg_right
      3. ``SET tw=<int>``    — trackwidth mm
      4. ``SET rotSlip=<float>`` — calibration.rotational_slip.  ALWAYS
         sent: an uncalibrated config (rotational_slip null/missing) pushes
         the documented "no correction" sentinel ``0`` (``effectiveSlip()``
         maps 0 -> 1.0), so a no-calibration robot NEUTRALIZES whatever
         value is baked into the firmware's compiled-in DefaultConfig
         instead of silently inheriting it.  This is what makes "select
         tovez nocal → turns are geometry-pure" true in the sim.
      5. ``OI``              — OTOS init (must precede OL/OA)
      6. ``OL <int8>``       — otos_linear_scale encoded
      7. ``OA <int8>``       — otos_angular_scale encoded
      8. ``SET odomOffX/Y/Yaw`` — only when nonzero
    """
    cmds: list[tuple[str, int]] = []

    # ── Wheel encoder calibration and trackwidth ──────────────────────────
    cal = getattr(config, "calibration", None)

    wd = getattr(getattr(config, "wheels", None), "wheel_diameter_mm", None)
    default_wheel_travel_calib = (math.pi * wd / 360.0) if wd is not None else None  # [mm/deg]

    wheel_travel_calib_left  = getattr(cal, "mm_per_wheel_deg_left",  None) if cal else None
    wheel_travel_calib_right = getattr(cal, "mm_per_wheel_deg_right", None) if cal else None
    wheel_travel_calib_left  = wheel_travel_calib_left  if wheel_travel_calib_left  is not None else default_wheel_travel_calib
    wheel_travel_calib_right = wheel_travel_calib_right if wheel_travel_calib_right is not None else default_wheel_travel_calib

    if wheel_travel_calib_left is not None:
        cmds.append((f"SET ml={wheel_travel_calib_left:.6f}", 200))
    if wheel_travel_calib_right is not None:
        cmds.append((f"SET mr={wheel_travel_calib_right:.6f}", 200))

    geom = getattr(config, "geometry", None)
    tw = getattr(geom, "trackwidth", None) if geom else None
    if tw is not None:
        cmds.append((f"SET tw={int(round(float(tw)))}", 200))

    # ── Rotational slip: always pushed, uncalibrated -> sentinel 0 ────────
    rot_slip = getattr(cal, "rotational_slip", None) if cal else None
    rot_slip = float(rot_slip) if rot_slip is not None else 0.0
    cmds.append((f"SET rotSlip={rot_slip:g}", 200))

    # ── OTOS init (must precede scalar writes) ────────────────────────────
    cmds.append(("OI", 500))

    # ── OTOS scalars ──────────────────────────────────────────────────────
    lin_scale = getattr(cal, "otos_linear_scale",  None) if cal else None
    ang_scale = getattr(cal, "otos_angular_scale", None) if cal else None
    lin_scale = float(lin_scale) if lin_scale is not None else 1.0
    ang_scale = float(ang_scale) if ang_scale is not None else 1.0

    cmds.append((f"OL {scale_to_int8(lin_scale)}", 200))
    cmds.append((f"OA {scale_to_int8(ang_scale)}", 200))

    # ── OTOS mounting offset (skip if all zero) ───────────────────────────
    off = getattr(geom, "odometry_offset_mm", None) if geom else None
    if off is not None:
        ox = float(off.x) if hasattr(off, "x") else 0.0
        oy = float(off.y) if hasattr(off, "y") else 0.0
        oyaw = math.degrees(float(off.yaw_rad)) if hasattr(off, "yaw_rad") else 0.0  # [deg]
        if ox != 0.0 or oy != 0.0 or oyaw != 0.0:
            cmds.append((f"SET odomOffX={ox:.3f}", 200))
            cmds.append((f"SET odomOffY={oy:.3f}", 200))
            cmds.append((f"SET odomYaw={oyaw:.3f}", 200))

    return cmds


def _push_via_conn(conn: Any, config: Any) -> dict[str, Any]:
    """Send ``calibration_commands(config)`` over *conn* (SerialConnection).

    Returns a dict with ``"status": "ok"`` and ``"commands"`` listing sent verbs.
    """
    sent: list[str] = []
    for cmd, read_timeout in calibration_commands(config):
        conn.send(cmd, read_timeout=read_timeout)
        sent.append(cmd)
    return {"status": "ok", "commands": sent}
