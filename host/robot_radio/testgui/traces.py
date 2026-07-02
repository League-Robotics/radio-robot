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

    ``notify_reset_pending()``
        Force the next ``feed()``'s encoder handling to rebaseline without
        integrating, preserving the accumulated heading/position.  Call this
        the instant a reset-inducing command (``D``, ``ZERO enc``, ``ZERO``)
        is sent — before the robot's reply/telemetry arrives.  Wired from
        ``Transport.on_reset_pending`` in ``__main__.py``.  See CR-09 in
        ``testgui-trace-correctness-slow-tlm-and-anchor-rotation.md``.

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

# Encoder-reset detection (CR-09).  The firmware zeros the wheel encoders at
# the start of every distance/drive command (see ccw_square_50.py: "D resets
# encoders -> rebaseline").  This used to be inferred from telemetry
# magnitude (both counts reading near zero while the previous baseline was
# substantially non-zero) — but over the relay, TLM arrives at ~1-2 Hz while
# the robot moves 100-200 mm between frames, so the first post-reset frame
# often lands well past any small epsilon and the reset is missed (see
# testgui-trace-correctness-slow-tlm-and-anchor-rotation.md).  Resets are now
# signalled explicitly, at command-send time, via
# ``TraceModel.notify_reset_pending()`` (wired from
# ``Transport.on_reset_pending`` — see transport.py's command classifier),
# instead of guessed from data.


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

        # Geometric trackwidth (mm) and turn-scrub factor combine into the
        # effective trackwidth used to convert wheel-delta → heading.
        self._geom_track_mm: float = _TRACK_MM
        self._scrub_factor: float = 0.0
        self._track_mm: float = _TRACK_MM  # effective = geom * (1 + scrub)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def _recompute_track(self) -> None:
        self._track_mm = self._geom_track_mm * (1.0 + self._scrub_factor)

    def set_trackwidth_mm(self, trackwidth_mm: float) -> None:
        """Set the geometric trackwidth used for encoder heading integration.

        Sourced from the active robot config (e.g. tovez=128, togov=126).
        Falls back to the module default when not called.
        """
        self._geom_track_mm = float(trackwidth_mm)
        self._recompute_track()

    def set_turn_scrub_factor(self, factor: float) -> None:
        """Calibrate the encoder heading integration for turn scrub.

        Differential drives scrub during turns, so the wheel encoders
        over-report travel: a commanded/actual turn of θ registers as
        ``θ·(1+factor)`` of wheel-delta.  Raw integration with the geometric
        trackwidth therefore over-rotates the encoder track by ``(1+factor)``.
        Compensate by widening the effective trackwidth to
        ``geometric·(1+factor)`` so ``heading = (dR-dL)/effective`` recovers the
        true angle.

        ``factor`` is a per-robot calibration constant (0 = perfect, no scrub).
        In the simulator it equals the injected ``slip_turn_extra`` (0.26); on
        hardware it comes from turn-odometry calibration and is typically small.
        """
        self._scrub_factor = float(factor)
        self._recompute_track()

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

    def notify_reset_pending(self) -> None:
        """Signal that a reset-inducing command was just sent to the robot.

        Called by the GUI's ``Transport.on_reset_pending`` hook the instant
        it sends a command that zeroes the firmware's wheel encoders (``D``,
        ``ZERO enc``, ``ZERO``) — BEFORE the robot's reply/telemetry lands.
        Replaces the old magnitude-based reset heuristic
        (``_ENC_RESET_EPS_MM``/``_ENC_RESET_BASE_MM``), which missed resets
        when the first post-reset TLM frame arrived well past any small
        epsilon (slow relay TLM: ~1-2 Hz while the robot travels 100-200 mm
        between frames) — see
        ``testgui-trace-correctness-slow-tlm-and-anchor-rotation.md`` (CR-09).

        Forces ``_enc_baseline = None`` so the next ``_feed_encoder()`` call
        establishes a fresh baseline from whatever value that frame carries,
        no matter how large — no magnitude check.  Deliberately does NOT
        touch ``_enc_h``/``_enc_bx``/``_enc_by``: the accumulated heading and
        body-frame displacement must survive the reset (a reset only zeroes
        the firmware's *encoder counters*, not the robot's actual pose), so
        the encoder trace keeps following the robot's turns instead of
        collapsing its heading back toward the anchor orientation.
        """
        self._enc_baseline = None

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

    def _rw(self, dx_cm: float, dy_cm: float, rot_rad: float) -> tuple[float, float]:
        """Rotate a firmware world-frame delta by ``rot_rad`` and add the anchor.

        Used by ``_feed_otos``/``_feed_fused`` (CR-10), which must NOT reuse
        ``_tw`` (the body-frame-only transform used by the encoder trace).
        ``otos``/``pose`` deltas are already expressed in the firmware's own
        (persistent, session-long) world frame — correct to render directly
        only when that frame happens to be aligned with the GUI's anchor
        frame, i.e. when the firmware pose was freshly zeroed at anchor time.
        Anchoring mid-session (a non-zero firmware heading at baseline time)
        breaks that assumption: the firmware frame is offset from the anchor
        frame by a fixed rotation, ``rot_rad = anchor_yaw -
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

    def _feed_encoder(self, enc: tuple[int, int]) -> None:
        """Integrate encoder deltas and append a world point to the encoder trace.

        Mirrors the host-side integration in ``ccw_square_50.py``.

        Parameters
        ----------
        enc:
            (left_mm, right_mm) cumulative encoder values from TLMFrame.enc.
        """
        if self._enc_baseline is None:
            # First reading after clear()/anchor(), OR the first reading
            # after an explicit notify_reset_pending() signal (CR-09) —
            # establish a fresh baseline from whatever value this frame
            # carries, no matter how large; no magnitude check.
            #
            # Deliberately do NOT zero _enc_h/_enc_bx/_enc_by here: after
            # clear()/anchor() they are already zero (_reset_baselines()
            # zeroed them, so this is a no-op in that case); after
            # notify_reset_pending() they hold the heading/body-frame
            # displacement accumulated so far, which must survive a
            # command-boundary rebaseline — a reset only zeroes the
            # firmware's *encoder counters*, not the robot's actual pose.
            # Emit a point at the current (possibly non-anchor) pose so the
            # trace stays continuous across the rebaseline.
            self._enc_baseline = enc
            self.encoder.append(self._tw(self._enc_bx / 10.0, self._enc_by / 10.0))
            return

        dL = enc[0] - self._enc_baseline[0]
        dR = enc[1] - self._enc_baseline[1]

        # Guard against large encoder jumps (e.g. a full ZERO enc pose command).
        if abs(dL) > 5000 or abs(dR) > 5000:
            self._enc_baseline = enc
            # Don't reset body displacement — keep accumulating from here.
            return

        dC = (dL + dR) / 2.0
        dT = (dR - dL) / self._track_mm

        # Midpoint-arc integration (CR-15 item 5): use the heading at the
        # midpoint of this step, not the post-increment heading, matching the
        # convention used by PhysicsWorld::update, SimOdometer::tick, and
        # Odometry::predict. Post-increment integration systematically
        # over/under-rotates the accumulated (bx, by) on every turning step.
        hMid = self._enc_h + dT * 0.5
        self._enc_bx += dC * math.cos(hMid)
        self._enc_by += dC * math.sin(hMid)
        self._enc_h += dT

        # Update baseline for incremental delta.
        self._enc_baseline = enc

        # Convert mm → cm for tw().
        self.encoder.append(self._tw(self._enc_bx / 10.0, self._enc_by / 10.0))

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
            (x_mm, y_mm, heading_cdeg) absolute OTOS pose from TLMFrame.otos.
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
            (x_mm, y_mm, heading_cdeg) absolute fused pose from TLMFrame.pose.
        """
        if self._pose_baseline is None:
            self._pose_baseline = pose
            self.fused.append(self._tw(0.0, 0.0))
            return

        dx_cm = (pose[0] - self._pose_baseline[0]) / 10.0
        dy_cm = (pose[1] - self._pose_baseline[1]) / 10.0
        rot = self._anchor_h - math.radians(self._pose_baseline[2] / 100.0)
        self.fused.append(self._rw(dx_cm, dy_cm, rot))
