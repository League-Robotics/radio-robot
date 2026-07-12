"""Odometry tracker: converts firmware TLM frames to world-frame positions.

Primary interface (v2):
- ``tlm_to_dict(frame)`` — adapt an already-parsed ``TLMFrame`` (from
  ``NezhaProtocol``'s binary telemetry delivery, 097-003) into a dict.
- ``OdomTracker.update_from_tlm(frame)`` — feed a ``TLMFrame`` directly.

Legacy (kept for backward compatibility only, NOT on the v2 hot path):
- ``parse_so()`` — v1 SO stream format parser.  Marked ``@deprecated``.
  No internal v2 code calls this function.

Usage (v2)::

    from robot_radio.robot.protocol import TLMFrame
    from robot_radio.sensors.odom_tracker import OdomTracker

    tracker = OdomTracker(ref_world_pos=(0.0, 0.0), ref_world_yaw=0.0)
    # Feed a TLMFrame (obtained from NezhaProtocol's binary telemetry delivery):
    tracker.update_from_tlm(tlm_frame)
    print(tracker.x, tracker.y, tracker.heading)
"""

from __future__ import annotations

import math
import warnings


# ---------------------------------------------------------------------------
# v2 TLM frame helper
# ---------------------------------------------------------------------------

def tlm_to_dict(frame) -> dict | None:
    """Adapt an already-parsed ``TLMFrame`` into a dict with sensor fields,
    or ``None`` if ``frame`` is ``None``.

    097-003: previously delegated to the (now-retired) text-plane TLM line
    parser; ``NezhaProtocol``'s telemetry delivery is binary-native now
    (``TLMFrame`` objects, not text lines), so this function's job shrinks
    to the same dict adaptation it always did, just starting one step later
    -- from an already-built ``TLMFrame`` instead of parsing one out of a
    text line itself.

    Returns a dict with the available fields from the TLM frame, for
    example::

        {'pose': (x, y, heading), 'enc': (left, right), 't': 12345}

    Example::

        frame = proto.snap()
        result = tlm_to_dict(frame)
        # {'t': 500, 'pose': (350, -12, 1780), 'enc': (1024, 1019)}
    """
    if frame is None:
        return None

    out: dict = {}
    if frame.t is not None:
        out["t"] = frame.t
    if frame.pose is not None:
        out["pose"] = frame.pose
    if frame.enc is not None:
        out["enc"] = frame.enc
    if frame.vel is not None:
        out["vel"] = frame.vel
    if frame.mode is not None:
        out["mode"] = frame.mode
    return out if out else {}   # empty dict (not None) for a frame with no populated fields


# ---------------------------------------------------------------------------
# Legacy v1 SO parser — DEPRECATED, not used on the v2 hot path
# ---------------------------------------------------------------------------

def parse_so(line) -> tuple | None:
    """[DEPRECATED] Parse 'SO+1234-0567+090' → (x, y, heading) [mm, mm, deg] or None.

    This is the v1 SO stream format.  In v2 the SO stream does not exist.
    Kept for backward compatibility only — do not use in new code.
    Use ``tlm_to_dict()`` or ``OdomTracker.update_from_tlm()`` instead.
    """
    s = str(line).lstrip("<# ").strip()
    if not s.startswith("SO"):
        return None
    body = s[2:]
    try:
        parts, cur = [], ""
        for ch in body:
            if ch in "+-":
                if cur:
                    parts.append(int(cur))
                cur = ch
            else:
                cur += ch
        if cur:
            parts.append(int(cur))
        if len(parts) >= 3:
            return parts[0], parts[1], parts[2]
    except ValueError:
        return None
    return None


# ---------------------------------------------------------------------------
# OdomTracker — v2 primary interface
# ---------------------------------------------------------------------------

class OdomTracker:
    """Tracks robot pose from v2 TLMFrame.pose (x, y, heading) [mm, mm, cdeg].

    The primary update path is :meth:`update_from_tlm`, which accepts a
    ``TLMFrame`` from the v2 protocol layer.

    World-frame conversion is computed from the pose at anchor time.
    Units: all millimetres and centidegrees internally; the ``world_pos``
    property converts to cm for legacy callers.

    Constructor accepts either a ``RobotConfig`` instance (``config=``)
    or bare keyword args (``trackwidth``, ``wheel_travel_calib_left``,
    ``wheel_travel_calib_right``) for the kinematic parameters.  If neither
    is supplied the tracker operates in a simple pose-forwarding mode
    without wheel-based dead-reckoning.

    Parameters
    ----------
    ref_world_pos:
        Initial world position (x, y) [mm].  Defaults to (0, 0).
    ref_world_yaw:
        Initial world heading [rad] (CCW-positive, 0 = +x/east — matches
        the aprilcam world convention).  Defaults to 0.
    config:
        Optional ``RobotConfig``.  When supplied, ``trackwidth``,
        ``wheel_travel_calib_left``, and ``wheel_travel_calib_right`` are
        read from it.
    trackwidth:
        Track width [mm] (overrides config if supplied directly).
    wheel_travel_calib_left:
        Left-wheel mm per encoder degree [mm/deg] (calibration param).
    wheel_travel_calib_right:
        Right-wheel mm per encoder degree [mm/deg] (calibration param).
    """

    # Minimum movement before a new world position is appended to path.
    MIN_MOVE = 3.0  # [mm]

    def __init__(
        self,
        ref_world_pos: tuple[float, float] = (0.0, 0.0),
        ref_world_yaw: float = 0.0,
        *,
        config=None,
        trackwidth: float | None = None,
        wheel_travel_calib_left: float | None = None,
        wheel_travel_calib_right: float | None = None,
    ) -> None:
        self._ref_world: tuple[float, float] = ref_world_pos  # [mm]
        self._ref_yaw: float = ref_world_yaw  # [rad]

        # Kinematic parameters (optional; sourced from RobotConfig when supplied)
        if config is not None:
            tw = getattr(config, "trackwidth", None)
            self.trackwidth: float | None = float(tw) if tw is not None else trackwidth
            cal = getattr(config, "calibration", None)
            if cal is not None:
                self.wheel_travel_calib_left: float | None = getattr(cal, "mm_per_wheel_deg_left", None)
                self.wheel_travel_calib_right: float | None = getattr(cal, "mm_per_wheel_deg_right", None)
            else:
                self.wheel_travel_calib_left = wheel_travel_calib_left
                self.wheel_travel_calib_right = wheel_travel_calib_right
        else:
            self.trackwidth = trackwidth
            self.wheel_travel_calib_left = wheel_travel_calib_left
            self.wheel_travel_calib_right = wheel_travel_calib_right

        # State from TLM
        self._ref_pose: tuple[int, int, int] | None = None   # (x, y, cdeg) at anchor
        self._last_pose: tuple[int, int, int] | None = None  # (x, y, cdeg) latest

        # Path in world mm (list of (x, y))
        self.path: list[tuple[float, float]] = []

    # ------------------------------------------------------------------
    # Primary v2 update path
    # ------------------------------------------------------------------

    def update_from_tlm(self, frame) -> bool:
        """Update pose from a ``TLMFrame``.

        Parameters
        ----------
        frame:
            A ``TLMFrame`` object (from ``robot_radio.robot.protocol``).
            Must have a ``pose`` field ``(x, y, heading)`` [mm, mm, cdeg].

        Returns
        -------
        bool
            ``True`` if the frame contained a ``pose`` field and state was
            updated; ``False`` otherwise.
        """
        pose = getattr(frame, "pose", None)
        if pose is None:
            return False
        self._last_pose = pose  # (x, y, heading) [mm, mm, cdeg]

        # Auto-anchor on first pose received
        if not self.anchored:
            self.anchor_pose(pose)
            return True

        w = self._to_world(pose)
        if w is not None:
            if (not self.path or
                    math.hypot(w[0] - self.path[-1][0],
                               w[1] - self.path[-1][1]) > self.MIN_MOVE):
                self.path.append(w)
        return True

    def anchor_pose(self, pose: tuple[int, int, int]) -> None:
        """Set the reference pose (anchor point).

        Parameters
        ----------
        pose:
            ``(x, y, heading)`` [mm, mm, cdeg] from a ``TLMFrame``.
        """
        self._ref_pose = pose
        self._last_pose = pose
        w0 = self._to_world(pose)
        if w0 is not None and not self.path:
            self.path.append(w0)

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def anchored(self) -> bool:
        """True once the tracker has received its first pose."""
        return self._ref_pose is not None

    @property
    def x(self) -> int | None:
        """Raw x position from the latest TLM frame.  [mm]"""
        return self._last_pose[0] if self._last_pose is not None else None

    @property
    def y(self) -> int | None:
        """Raw y position from the latest TLM frame.  [mm]"""
        return self._last_pose[1] if self._last_pose is not None else None

    @property
    def heading(self) -> int | None:
        """Raw heading from the latest TLM frame.  [cdeg]"""
        return self._last_pose[2] if self._last_pose is not None else None

    @property
    def heading_degrees(self) -> float | None:
        """Heading in degrees (centidegrees / 100)."""
        if self._last_pose is None:
            return None
        return self._last_pose[2] / 100.0

    @property
    def heading_radians(self) -> float | None:
        """Heading in radians."""
        if self._last_pose is None:
            return None
        return math.radians(self._last_pose[2] / 100.0)

    @property
    def world_pos(self) -> tuple[float, float] | None:
        """Current world position in **cm** (legacy unit; use x/y for mm)."""
        if not self.anchored or self._last_pose is None:
            return None
        w = self._to_world(self._last_pose)
        if w is None:
            return None
        return (w[0] / 10.0, w[1] / 10.0)

    @property
    def world_yaw(self) -> float | None:
        """Current heading in world frame (CCW-positive radians, 0 = +x/east).

        Matches the aprilcam world convention directly (A1-centred, +x east,
        +y north, CCW-positive — verified empirically, see
        ``robot_radio.sensors.odometry``'s ``_apply()``). Firmware TLM
        heading is also CCW-positive (0 = firmware's own +X axis — see
        ``Odometry.cpp``'s ``pose.x += d*cos(theta); pose.y += d*sin(theta)``
        integration), so no sign flip is needed: the two frames differ only
        by the constant rotation offset established at anchor time.
        """
        if not self.anchored or self._last_pose is None:
            return None
        assert self._ref_pose is not None
        # Both TLM heading and world_yaw are CCW-positive — straight delta,
        # no sign flip (066-002 / CR-12: the prior `self._ref_yaw - delta_rad`
        # implemented a CW-positive world convention, which does not match
        # aprilcam's actual CCW-positive frame — confirmed by the new
        # convention test and by direct reading of Odometry.cpp / aprilcam's
        # own empirically-verified orientation convention).
        delta_rad = math.radians((self._last_pose[2] - self._ref_pose[2]) / 100.0)
        return self._ref_yaw + delta_rad

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _to_world(self, pose: tuple[int, int, int]) -> tuple[float, float] | None:
        """Convert a TLM pose to world-frame position.  [mm]

        Firmware TLM pose ``(x, y, heading)`` [mm, mm, cdeg] is already a
        proper Cartesian pose in firmware's own fixed (boot/reset-anchored)
        frame: heading 0 points along firmware's own +X axis and increasing
        heading rotates CCW — see ``Odometry.cpp``'s dead-reckoning
        integration, ``pose.x += d*cos(theta); pose.y += d*sin(theta)``. It
        is NOT a body-relative "x=right, y=forward" reading (066-002 /
        CR-12: that was this function's previous, untested — and wrong —
        assumption).

        aprilcam's world frame (A1-centred, +x east, +y north) uses the same
        CCW-positive, 0-along-+x convention (verified empirically — see
        ``robot_radio.sensors.odometry``'s ``_apply()``). The two frames
        therefore differ only by a fixed rotation + translation established
        at anchor time (no axis reinterpretation, no reflection): rotate the
        firmware-frame delta by the constant offset ``a`` between the two
        frames' headings, then translate by the anchor world position.
        """
        if self._ref_pose is None:
            return None
        dx = pose[0] - self._ref_pose[0]  # [mm]
        dy = pose[1] - self._ref_pose[1]  # [mm]
        # a = rotation from firmware-frame heading to world-frame heading,
        # both CCW-positive — constant once anchored.
        a = self._ref_yaw - math.radians(self._ref_pose[2] / 100.0)
        c, s = math.cos(a), math.sin(a)
        # Standard CCW rotation matrix R(a) applied to the firmware-frame
        # delta, then translated into the anchor's world position.
        wx = c * dx - s * dy  # [mm]
        wy = s * dx + c * dy  # [mm]
        return (
            self._ref_world[0] + wx,
            self._ref_world[1] + wy,
        )

    # ------------------------------------------------------------------
    # Legacy SO-based interface (v1 hot path — DEPRECATED, no v2 callers)
    # ------------------------------------------------------------------

    def anchor(self, conn, timeout_s: float = 1.0) -> bool:
        """[DEPRECATED] Block until a SO reading arrives and use it as the reference.

        This method uses the v1 SO stream and is preserved for backward
        compatibility only.  In v2, use ``update_from_tlm()`` instead.

        Returns True if anchored within timeout, False otherwise.
        """
        import time
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for line in conn.read_lines(duration=100):
                v = parse_so(line)
                if v is not None:
                    # Convert SO (mm) to TLM-like pose for anchoring.
                    # SO heading is in integer degrees; store as cdeg.
                    pose = (v[0], v[1], v[2] * 100)  # (mm, mm, cdeg)
                    self.anchor_pose(pose)
                    return True
        return False

    def drain(self, conn, duration: int = 40) -> None:  # [ms]
        """[DEPRECATED] Read all available SO lines (v1 legacy path)."""
        self.feed(conn.read_lines(duration=duration))

    def feed(self, lines) -> None:
        """[DEPRECATED] Process SO lines (v1 legacy path).

        For v2, feed TLMFrame objects via ``update_from_tlm()`` instead.
        """
        for line in lines:
            v = parse_so(line)
            if v is not None:
                # SO: (x, y, heading in deg); convert heading to cdeg
                pose = (v[0], v[1], v[2] * 100)  # (mm, mm, cdeg)
                self._last_pose = pose
                w = self._to_world(pose)
                if w is not None:
                    if (not self.path or
                            math.hypot(w[0] - self.path[-1][0],
                                       w[1] - self.path[-1][1]) > self.MIN_MOVE):
                        self.path.append(w)
