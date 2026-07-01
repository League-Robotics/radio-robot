"""robot_radio.testgui.traces — TraceModel: four-polyline world-cm pose accumulator.

Accumulates four world-cm polylines from incoming telemetry frames and camera
ground-truth poses.  Designed to be Qt-free so it is importable and testable
without PySide6 installed.

Public surface
--------------
TraceModel
    Holds four lists of (x_cm, y_cm) world points:
      - ``camera``  — ground-truth from aprilcam / SimTransport (green)
      - ``encoder`` — wheel-odometry integrated host-side (orange)
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

        Delta/absolute interpretation per sensor:
          - ``frame.enc`` — absolute cumulative mm values; difference from the
            baseline (first enc reading after ``anchor()``/``clear()``) gives
            the displacement, then converted to cm.
          - ``frame.otos`` — absolute mm accumulation since OTOS was zeroed;
            difference from baseline gives body-frame displacement (cm).
          - ``frame.pose`` — absolute fused mm accumulation; difference from
            baseline gives body-frame displacement (cm).

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

    tests/old/playfield_tour/playfield.jpg
    tests/old/playfield_tour/playfield_calibration.json

Both files are located relative to the *installed package* source tree, not to
the current working directory.  ``canvas.py`` resolves them as:

    pathlib.Path(__file__).parents[4] / "tests" / "old" / "playfield_tour" / ...

``__file__`` for this module is
``host/robot_radio/testgui/traces.py``, so ``parents[4]`` is the repo root.
This works when the package is installed editable (``pip install -e .`` or
``uv sync``).  If the assets are not found, ``canvas.py`` gracefully degrades
to a solid-colour background.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot_radio.robot.protocol import TLMFrame

# Wheel track width in mm (matches the ccw_square_50.py constant).
_TRACK_MM = 128.0


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
        World-cm points from wheel encoder odometry.
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
        # enc baseline: (left_mm, right_mm)
        self._enc_baseline: tuple[int, int] | None = None
        # otos baseline: (x_mm, y_mm, hdg_cdeg)
        self._otos_baseline: tuple[int, int, int] | None = None
        # pose/fused baseline: (x_mm, y_mm, hdg_cdeg)
        self._pose_baseline: tuple[int, int, int] | None = None

        # Accumulated encoder heading and xy displacement in body-frame (mm).
        self._enc_h: float = 0.0   # accumulated encoder heading (radians)
        self._enc_bx: float = 0.0  # accumulated body-frame x displacement (mm)
        self._enc_by: float = 0.0  # accumulated body-frame y displacement (mm)

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

        # --- encoder odometry ---
        if frame.enc is not None:
            self._feed_encoder(frame.enc)

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
        """Clear all sensor baselines and accumulated encoder state."""
        self._enc_baseline = None
        self._otos_baseline = None
        self._pose_baseline = None
        self._enc_h = 0.0
        self._enc_bx = 0.0
        self._enc_by = 0.0

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

    def _feed_encoder(self, enc: tuple[int, int]) -> None:
        """Integrate encoder deltas and append a world point to the encoder trace.

        Mirrors the host-side integration in ``ccw_square_50.py``.

        Parameters
        ----------
        enc:
            (left_mm, right_mm) cumulative encoder values from TLMFrame.enc.
        """
        if self._enc_baseline is None:
            # First reading — establish baseline; no displacement yet.
            self._enc_baseline = enc
            self._enc_h = 0.0
            self._enc_bx = 0.0
            self._enc_by = 0.0
            # Emit the anchor point.
            self.encoder.append(self._tw(0.0, 0.0))
            return

        dL = enc[0] - self._enc_baseline[0]
        dR = enc[1] - self._enc_baseline[1]

        # Guard against encoder reset jumps (e.g. after firmware ZERO enc).
        if abs(dL) > 5000 or abs(dR) > 5000:
            self._enc_baseline = enc
            # Don't reset body displacement — keep accumulating from here.
            return

        dC = (dL + dR) / 2.0
        dT = (dR - dL) / _TRACK_MM

        self._enc_h += dT
        self._enc_bx += dC * math.cos(self._enc_h)
        self._enc_by += dC * math.sin(self._enc_h)

        # Update baseline for incremental delta.
        self._enc_baseline = enc

        # Convert mm → cm for tw().
        self.encoder.append(self._tw(self._enc_bx / 10.0, self._enc_by / 10.0))

    def _feed_otos(self, otos: tuple[int, int, int]) -> None:
        """Compute OTOS displacement and append to the otos trace.

        Parameters
        ----------
        otos:
            (x_mm, y_mm, heading_cdeg) absolute OTOS pose from TLMFrame.otos.
        """
        if self._otos_baseline is None:
            self._otos_baseline = otos
            self.otos.append(self._tw(0.0, 0.0))
            return

        dx_cm = (otos[0] - self._otos_baseline[0]) / 10.0
        dy_cm = (otos[1] - self._otos_baseline[1]) / 10.0
        self.otos.append(self._tw(dx_cm, dy_cm))

    def _feed_fused(self, pose: tuple[int, int, int]) -> None:
        """Compute fused pose displacement and append to the fused trace.

        Parameters
        ----------
        pose:
            (x_mm, y_mm, heading_cdeg) absolute fused pose from TLMFrame.pose.
        """
        if self._pose_baseline is None:
            self._pose_baseline = pose
            self.fused.append(self._tw(0.0, 0.0))
            return

        dx_cm = (pose[0] - self._pose_baseline[0]) / 10.0
        dy_cm = (pose[1] - self._pose_baseline[1]) / 10.0
        self.fused.append(self._tw(dx_cm, dy_cm))
