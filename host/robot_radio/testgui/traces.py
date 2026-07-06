"""robot_radio.testgui.traces — TraceModel: four-polyline world-cm pose accumulator.

Accumulates four world-cm polylines from incoming telemetry frames and camera
ground-truth poses.  Designed to be Qt-free so it is importable and testable
without PySide6 installed.

Public surface
--------------
TraceModel
    Holds four lists of (x_cm, y_cm) world points:
      - ``camera``  — ground-truth from aprilcam / SimTransport (green)
      - ``encoder`` — firmware encoder-only dead-reckoned pose (orange)
      - ``otos``    — raw OTOS sensor pose (cyan)
      - ``fused``   — firmware EKF/fused pose (magenta)

    ``anchor(x_cm, y_cm, yaw_rad)``
        Set the initial world pose that anchors all body-to-world transforms.
        Must be called before the first ``feed()`` invocation.  If not called,
        the first received TLMFrame automatically sets the anchor at the origin
        (0, 0, 0).

    ``feed(frame: TLMFrame)``
        Ingest one TLMFrame.  Appends one point to each trace whose data
        field is present in the frame.  Body-to-world transform uses the
        ``tw()`` pattern from ``tests/bench/ccw_square_50.py``:

            world_x = anchor_x + body_x * cos(h0) - body_y * sin(h0)
            world_y = anchor_y + body_x * sin(h0) + body_y * cos(h0)

        where ``h0`` is the heading at the anchor pose (radians).

        Delta/absolute interpretation per sensor (all three are absolute
        firmware world-frame poses, rotated into the display frame by
        ``_rw()`` — see 068-003 / architecture-update.md Decision 4):
          - ``frame.encpose`` — absolute encoder-only dead-reckoned pose
            (firmware ``Odometry::predict()``); difference from baseline
            (first reading after ``anchor()``/``clear()``) gives the
            firmware-frame displacement (cm).
          - ``frame.otos`` — absolute mm accumulation since OTOS was zeroed;
            difference from baseline gives firmware-frame displacement (cm).
          - ``frame.pose`` — absolute fused mm accumulation; difference from
            baseline gives firmware-frame displacement (cm).

    ``feed_truth(x_cm, y_cm, yaw_rad)``
        Append a point to the ``camera`` trace directly from a world-cm pose.

    ``clear()``
        Reset all four lists and the accumulated baselines.  After ``clear()``,
        the anchor remains set; the next ``feed()`` re-establishes baselines.

    ``enabled`` dict
        Per-trace on/off flag: ``model.enabled["camera"]``, ``model.enabled["encoder"]``,
        ``model.enabled["otos"]``, ``model.enabled["fused"]``.
        ``feed()`` always appends regardless of the enabled flag — the flag
        gates *rendering*, not accumulation.

Thread safety
-------------
TraceModel is NOT thread-safe.  The GUI must call ``feed()`` / ``clear()``
from the Qt main thread (after marshalling from the transport background thread).

OQ-2 resolution (playfield image path)
---------------------------------------
The default playfield image and calibration data live at:

    tests_old/old/playfield_tour/playfield.jpg
    tests_old/old/playfield_tour/playfield_calibration.json

(sprint 077's greenfield rebuild parked the pre-rebuild tree at ``tests_old/``,
not ``tests/old/`` — fixed in 083-003.)

Both files are located relative to the *installed package* source tree, not to
the current working directory.  ``canvas.py`` resolves them as:

    pathlib.Path(__file__).parent.parent.parent / "tests_old" / "old" / "playfield_tour" / ...

``__file__`` for this module is
``host/robot_radio/testgui/traces.py``; walking up three parents (``testgui/``
-> ``robot_radio/`` -> ``host/``) then up once more lands at the repo root.
This works when the package is installed editable (``pip install -e .`` or
``uv sync``).  If the assets are not found, ``canvas.py`` gracefully degrades
to a solid-colour background.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot_radio.robot.protocol import TLMFrame


class TraceModel:
    """Four-polyline world-cm pose accumulator.

    Accumulates world-cm points from telemetry frames using the body-to-world
    transform pattern from ``tests/bench/ccw_square_50.py``.

    Parameters
    ----------
    None.  Call ``anchor()`` before the first ``feed()`` call to set the
    initial world pose.  If not called, the anchor defaults to (0, 0, 0) on
    the first frame.

    Attributes
    ----------
    camera : list[tuple[float, float]]
        World-cm points from ground-truth (aprilcam / sim truth).
    encoder : list[tuple[float, float]]
        World-cm points from the firmware's encoder-only dead-reckoned pose.
    otos : list[tuple[float, float]]
        World-cm points from raw OTOS sensor.
    fused : list[tuple[float, float]]
        World-cm points from firmware EKF fused pose.
    enabled : dict[str, bool]
        Per-trace visibility flag.  Does not gate accumulation.
    """

    TRACE_NAMES = ("camera", "encoder", "otos", "fused")

    def __init__(self) -> None:
        # --- world polylines ---
        self.camera: list[tuple[float, float]] = []
        self.encoder: list[tuple[float, float]] = []
        self.otos: list[tuple[float, float]] = []
        self.fused: list[tuple[float, float]] = []

        # --- per-trace enabled flag ---
        self.enabled: dict[str, bool] = {
            "camera": True,
            "encoder": True,
            "otos": True,
            "fused": True,
        }

        # --- anchor pose: the world pose at the start of the trace ---
        self._anchor_x: float = 0.0   # cm
        self._anchor_y: float = 0.0   # cm
        self._anchor_h: float = 0.0   # radians
        self._anchor_set: bool = False

        # Cached cos/sin of anchor heading for tw() transform.
        self._ch: float = 1.0
        self._sh: float = 0.0

        # --- baselines: the first absolute reading after anchor()/clear() ---
        # encpose baseline: (x, y, heading) in (mm, mm, cdeg)
        self._encpose_baseline: tuple[int, int, int] | None = None
        # otos baseline: (x, y, heading) in (mm, mm, cdeg)
        self._otos_baseline: tuple[int, int, int] | None = None
        # pose/fused baseline: (x, y, heading) in (mm, mm, cdeg)
        self._pose_baseline: tuple[int, int, int] | None = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def anchor(self, x_cm: float, y_cm: float, yaw_rad: float) -> None:
        """Set the initial world pose for the body-to-world transform.

        Parameters
        ----------
        x_cm, y_cm:
            World position of the robot at the start of the trace (cm).
        yaw_rad:
            Robot forward heading at the start (radians, 0=east, CCW+).
        """
        self._anchor_x = x_cm
        self._anchor_y = y_cm
        self._anchor_h = yaw_rad
        self._ch = math.cos(yaw_rad)
        self._sh = math.sin(yaw_rad)
        self._anchor_set = True
        # Reset baselines so next feed() re-establishes them.
        self._reset_baselines()

    def feed(self, frame: "TLMFrame") -> None:
        """Ingest one TLMFrame and append to the appropriate trace lists.

        Sets anchor to (0, 0, 0) automatically on the first call if ``anchor()``
        was not called.

        Parameters
        ----------
        frame:
            Parsed telemetry frame from ``parse_tlm()``.  Missing sensors
            (``None`` fields) are silently skipped.
        """
        if not self._anchor_set:
            self.anchor(0.0, 0.0, 0.0)

        # --- encoder-only dead-reckoned pose (firmware-computed) ---
        if frame.encpose is not None:
            self._feed_encpose(frame.encpose)

        # --- OTOS odometry ---
        if frame.otos is not None:
            self._feed_otos(frame.otos)

        # --- fused / EKF pose ---
        if frame.pose is not None:
            self._feed_fused(frame.pose)

    def feed_truth(self, x_cm: float, y_cm: float, yaw_rad: float) -> None:
        """Append a camera ground-truth pose to the ``camera`` trace.

        Parameters
        ----------
        x_cm, y_cm:
            World position in centimetres (A1-centred frame).
        yaw_rad:
            Robot heading in radians.
        """
        self.camera.append((x_cm, y_cm))

    def clear(self) -> None:
        """Reset all four polylines and accumulated baselines.

        The anchor is preserved so the next ``feed()`` continues to use the
        same transform origin.  Baselines are cleared so the next frame
        after ``clear()`` re-establishes them.
        """
        self.camera.clear()
        self.encoder.clear()
        self.otos.clear()
        self.fused.clear()
        self._reset_baselines()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _reset_baselines(self) -> None:
        """Clear all sensor baselines."""
        self._encpose_baseline = None
        self._otos_baseline = None
        self._pose_baseline = None

    def _tw(self, bx_cm: float, by_cm: float) -> tuple[float, float]:
        """Body-to-world transform.

        Converts body-frame displacement (cm) from the anchor to world-cm.

        Parameters
        ----------
        bx_cm, by_cm:
            Body-frame displacement from the anchor origin (cm).
            +bx = forward, +by = left (right-hand coordinate system).

        Returns
        -------
        tuple[float, float]
            World-cm position (x_cm, y_cm).
        """
        wx = self._anchor_x + bx_cm * self._ch - by_cm * self._sh
        wy = self._anchor_y + bx_cm * self._sh + by_cm * self._ch
        return (wx, wy)

    def _rw(self, dx_cm: float, dy_cm: float, rot_rad: float) -> tuple[float, float]:
        """Rotate a firmware world-frame delta by ``rot_rad`` and add the anchor.

        Used by ``_feed_encpose``/``_feed_otos``/``_feed_fused`` (CR-10),
        which must NOT reuse ``_tw`` (the anchor-relative-only transform).
        ``encpose``/``otos``/``pose`` deltas are already expressed in the
        firmware's own (persistent, session-long) world frame — correct to
        render directly only when that frame happens to be aligned with the
        GUI's anchor frame, i.e. when the firmware pose was freshly zeroed at
        anchor time. Anchoring mid-session (a non-zero firmware heading at
        baseline time) breaks that assumption: the firmware frame is offset
        from the anchor frame by a fixed rotation, ``rot_rad = anchor_yaw -
        firmware_heading_at_baseline`` (computed by the caller from the
        baseline's stored ``hdg_cdeg``), which must be applied to every
        subsequent delta to keep the trace aligned with the camera trace.

        Parameters
        ----------
        dx_cm, dy_cm:
            Firmware world-frame displacement (cm) since the sensor's
            baseline reading.
        rot_rad:
            Rotation to apply: the angle between the anchor heading and the
            firmware's own heading at baseline time.

        Returns
        -------
        tuple[float, float]
            World-cm position (x_cm, y_cm), relative to the anchor.
        """
        c = math.cos(rot_rad)
        s = math.sin(rot_rad)
        wx = self._anchor_x + dx_cm * c - dy_cm * s
        wy = self._anchor_y + dx_cm * s + dy_cm * c
        return (wx, wy)

    def _feed_encpose(self, encpose: tuple[int, int, int]) -> None:
        """Compute encoder-only pose displacement and append to the encoder trace.

        Structurally identical to ``_feed_otos``/``_feed_fused`` (068-003):
        ``frame.encpose`` is the firmware's own encoder-only dead-reckoned
        pose (``Odometry::predict()``), already an absolute world-frame pose
        — the host no longer re-integrates raw wheel counts host-side.
        Rotates the firmware world-frame delta by
        ``(anchor_yaw - firmware_heading_at_baseline)`` (CR-10) — see
        ``_feed_otos``/``_rw``.

        Parameters
        ----------
        encpose:
            (x, y, heading) in (mm, mm, cdeg) absolute encoder-only pose from
            TLMFrame.encpose.
        """
        if self._encpose_baseline is None:
            self._encpose_baseline = encpose
            self.encoder.append(self._tw(0.0, 0.0))
            return

        dx_cm = (encpose[0] - self._encpose_baseline[0]) / 10.0
        dy_cm = (encpose[1] - self._encpose_baseline[1]) / 10.0
        rot = self._anchor_h - math.radians(self._encpose_baseline[2] / 100.0)
        self.encoder.append(self._rw(dx_cm, dy_cm, rot))

    def _feed_otos(self, otos: tuple[int, int, int]) -> None:
        """Compute OTOS displacement and append to the otos trace.

        Rotates the firmware world-frame delta by
        ``(anchor_yaw - firmware_heading_at_baseline)`` (CR-10), not the
        anchor heading alone — see ``_rw``.  This is correct in both cases:
        when the firmware pose was freshly zeroed at anchor time,
        ``baseline[2]`` is 0 and this reduces exactly to the old
        anchor-only rotation.

        Parameters
        ----------
        otos:
            (x, y, heading) in (mm, mm, cdeg) absolute OTOS pose from TLMFrame.otos.
        """
        if self._otos_baseline is None:
            self._otos_baseline = otos
            self.otos.append(self._tw(0.0, 0.0))
            return

        dx_cm = (otos[0] - self._otos_baseline[0]) / 10.0
        dy_cm = (otos[1] - self._otos_baseline[1]) / 10.0
        rot = self._anchor_h - math.radians(self._otos_baseline[2] / 100.0)
        self.otos.append(self._rw(dx_cm, dy_cm, rot))

    def _feed_fused(self, pose: tuple[int, int, int]) -> None:
        """Compute fused pose displacement and append to the fused trace.

        Rotates the firmware world-frame delta by
        ``(anchor_yaw - firmware_heading_at_baseline)`` (CR-10) — see
        ``_feed_otos``/``_rw``.

        Parameters
        ----------
        pose:
            (x, y, heading) in (mm, mm, cdeg) absolute fused pose from TLMFrame.pose.
        """
        if self._pose_baseline is None:
            self._pose_baseline = pose
            self.fused.append(self._tw(0.0, 0.0))
            return

        dx_cm = (pose[0] - self._pose_baseline[0]) / 10.0
        dy_cm = (pose[1] - self._pose_baseline[1]) / 10.0
        rot = self._anchor_h - math.radians(self._pose_baseline[2] / 100.0)
        self.fused.append(self._rw(dx_cm, dy_cm, rot))
