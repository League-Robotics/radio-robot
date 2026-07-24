"""robot_radio.testgui.traces — TraceModel: four-polyline world-cm pose accumulator.

Accumulates four world-cm polylines from incoming telemetry frames and camera
ground-truth poses.  Designed to be Qt-free so it is importable and testable
without PySide6 installed.

Public surface
--------------
EncoderDeadReckoner
    Host-side differential-drive dead reckoning from cumulative per-wheel
    encoder distance (``TLMFrame.enc``) -- (097) the fallback ``TraceModel.
    feed()`` uses to keep the ``encoder`` trace (and the canvas avatar, which
    prefers it) moving on the binary plane, which carries no ``encpose``
    field at all (096-001's trim). See its own class docstring.

TraceModel
    Holds four lists of (x_cm, y_cm) world points:
      - ``camera``  — ground-truth from aprilcam / SimTransport (green)
      - ``encoder`` — encoder-only dead-reckoned pose (orange): firmware
        ``encpose`` when present, else the ``EncoderDeadReckoner`` fallback
        (097)
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
``src/host/robot_radio/testgui/traces.py``; walking up three parents (``testgui/``
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

#: Fallback trackwidth (mm) used until a real robot config is wired in via
#: TraceModel(trackwidth=...)/set_trackwidth() -- matches the firmware's own
#: configured trackwidthMm for the project's main dev bot (tovez), the same
#: default sim_prefs.py/test_tour1_geometry.py already use.
_DEFAULT_TRACKWIDTH = 128.0  # [mm]

# TestGUI Sim command-surface fix (2026-07): idle-jitter trace-growth guard.
# Sim mode's tick thread keeps streaming telemetry forever once connected
# (not just during a tour/drive) -- and ticket 108-011's rest-encoder dither
# (WheelPlant's opt-in +-1 wire-LSB alternation while at rest, added so a
# stopped wheel doesn't false-positive Devices::MotorArmor's wedge-latch
# detector -- see tests/sim/plant/wheel_plant.h's own "Rest-dither tuning"
# comment) means a genuinely-idle sim's encoder-derived pose micro-jitters by
# a fraction of a millimetre every frame, forever. Every _feed_* helper below
# used to append EVERY frame's point unconditionally, so an idle connection
# grew its trace lists without bound -- visually indistinguishable from "the
# tour is still running" (the reported "point count climbs, resets, keeps
# counting" symptom) even though run_tour() had already returned. Below this
# threshold a new point is idle jitter, not real motion, and is dropped
# instead of appended -- see _append_if_moved(). 0.05cm (0.5mm) is 5x the
# dither's own +-0.1mm wheel-position amplitude (comfortably filters the
# noise) and two orders of magnitude below a single tick's real-motion
# displacement at any commanded speed the GUI's own S/T/D rows allow (>=1
# mm/s * a 20ms-50ms tick is already >=0.02-0.05mm... in practice every real
# drive command commands tens of mm/s, so real motion clears this bound by
# 100x+ per tick -- see tests/testgui/test_traces.py's own forward-drive
# assertions, unaffected by this change).
_TRACE_IDLE_EPSILON_CM = 0.05  # [cm]


class EncoderDeadReckoner:
    """Host-side differential-drive dead reckoning from cumulative per-wheel
    encoder distance (``TLMFrame.enc``), standing in for the firmware's own
    ``encpose`` field until sprint 098 wires ``Subsystems::PoseEstimator::
    tick()`` (see ``TraceModel.feed()``'s own docstring: binary telemetry
    has no wire representation for ``encpose`` at all -- 096-001's trim --
    so there is no firmware value to fall back to in the meantime).

    Standard differential-drive integration -- ``update()`` is called once
    per ``TLMFrame`` with that frame's cumulative ``enc_left``/``enc_right``
    (mm) and incrementally integrates a running body pose using each step's
    small-arc delta:

        d          = (dL + dR) / 2          -- forward travel this step
        dtheta     = (dR - dL) / trackwidth -- heading change this step
        theta_mid  = theta + dtheta / 2     -- midpoint heading (more
                                                accurate than a pure Euler
                                                step across a curved path)
        x += d * cos(theta_mid); y += d * sin(theta_mid); theta += dtheta

    A one-shot "diff the latest reading against a single baseline" scheme
    (the way ``TraceModel._feed_otos``/``_feed_fused`` treat an ALREADY-
    ABSOLUTE firmware pose) would be wrong here: ``enc_left``/``enc_right``
    only tell you total wheel travel, not the PATH shape in between (e.g. a
    tour's D-then-RT-then-D sequence changes heading multiple times) -- the
    integration must happen per-frame, incrementally, exactly like the
    firmware's own ``Odometry::predict()``.

    ``update()`` returns an ``(x, y, heading)`` triple in the SAME
    ``(mm, mm, cdeg)`` shape ``TLMFrame.encpose`` would have carried, so a
    caller can feed it straight into ``TraceModel._feed_encpose()`` --
    reusing that existing baseline/anchor-rotation machinery rather than
    inventing a second one.
    """

    def __init__(self, trackwidth: float = _DEFAULT_TRACKWIDTH) -> None:  # [mm]
        self._trackwidth = trackwidth
        self._prev_enc: tuple[float, float] | None = None
        self._x: float = 0.0      # [mm]
        self._y: float = 0.0      # [mm]
        self._theta: float = 0.0  # [rad]

    def set_trackwidth(self, trackwidth: float) -> None:  # [mm]
        """Update the trackwidth used by future ``update()`` calls.

        Does not retroactively rescale the already-accumulated pose --
        matches the firmware's own "trackwidth is a config value read at
        motion time" posture; a robot-selection change mid-session is rare
        and the accumulated drift from it is negligible relative to
        dead-reckoning's own error budget.
        """
        self._trackwidth = trackwidth

    def reset(self) -> None:
        """Zero the accumulated pose and drop the cached previous reading.

        Called by ``TraceModel._reset_baselines()`` (``anchor()``/
        ``clear()``) so a "Set Robot @ 0,0" reset also re-zeros the
        dead-reckoned pose, not just the display anchor.
        """
        self._prev_enc = None
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0

    def update(self, enc_left: float, enc_right: float) -> tuple[int, int, int]:
        """Integrate one frame's cumulative encoder reading (mm each).

        The FIRST call after construction/``reset()`` only caches the
        reading as the integration's zero point (no prior reading to diff
        against) and returns ``(0, 0, 0)`` -- mirrors ``TraceModel``'s own
        "first reading establishes the baseline" convention for otos/fused.
        """
        if self._prev_enc is None:
            self._prev_enc = (enc_left, enc_right)
            return (0, 0, 0)

        d_left = enc_left - self._prev_enc[0]
        d_right = enc_right - self._prev_enc[1]
        self._prev_enc = (enc_left, enc_right)

        d = (d_left + d_right) / 2.0
        dtheta = (d_right - d_left) / self._trackwidth if self._trackwidth else 0.0
        theta_mid = self._theta + dtheta / 2.0
        self._x += d * math.cos(theta_mid)
        self._y += d * math.sin(theta_mid)
        self._theta += dtheta

        return (
            int(round(self._x)),
            int(round(self._y)),
            int(round(math.degrees(self._theta) * 100.0)),
        )


class TraceModel:
    """Four-polyline world-cm pose accumulator.

    Accumulates world-cm points from telemetry frames using the body-to-world
    transform pattern from ``tests/bench/ccw_square_50.py``.

    Parameters
    ----------
    trackwidth : float, optional
        Trackwidth (mm) for the host-side encoder dead-reckoning fallback
        (``EncoderDeadReckoner``, used when a frame carries ``enc`` but no
        ``encpose`` -- see ``feed()``'s own docstring). Defaults to the
        project's usual trackwidth (128 mm); update live via
        ``set_trackwidth()`` (e.g. on a robot-config change).
    None otherwise.  Call ``anchor()`` before the first ``feed()`` call to
    set the initial world pose.  If not called, the anchor defaults to
    (0, 0, 0) on the first frame.

    Attributes
    ----------
    camera : list[tuple[float, float]]
        World-cm points from ground-truth (aprilcam / sim truth).
    encoder : list[tuple[float, float]]
        World-cm points from the encoder-only dead-reckoned pose --
        firmware ``encpose`` when present, else the host-side
        ``EncoderDeadReckoner`` fallback computed from ``enc`` (097).
    otos : list[tuple[float, float]]
        World-cm points from raw OTOS sensor.
    fused : list[tuple[float, float]]
        World-cm points from firmware EKF fused pose.
    enabled : dict[str, bool]
        Per-trace visibility flag.  Does not gate accumulation.
    encoder_yaw : float | None
        Current encoder-trace heading (radians, display/anchor frame) --
        the ``CanvasController`` avatar's heading source while the fused
        pose (098) is unavailable.  ``None`` until the first ``feed()`` of
        a frame carrying ``enc``/``encpose``.
    """

    TRACE_NAMES = ("camera", "encoder", "otos", "fused")

    def __init__(self, trackwidth: float = _DEFAULT_TRACKWIDTH) -> None:  # [mm]
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

        # --- host-side encoder dead reckoning (097) -- see feed()'s docstring ---
        self._dead_reckoner = EncoderDeadReckoner(trackwidth)
        self.encoder_yaw: float | None = None  # [rad] display-frame heading
        # Last dead-reckoned (or firmware, when present) encpose in the raw
        # firmware shape (mm, mm, cdeg) -- consumers that used to read the
        # wire's encpose= (the telemetry breakout panel) read this instead,
        # since binary TLM carries no encpose field (096-001's trim).
        self.last_encpose: tuple[int, int, int] | None = None

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

    def set_trackwidth(self, trackwidth: float) -> None:  # [mm]
        """Update the trackwidth used by the host-side encoder dead
        reckoning fallback (see ``feed()``'s own docstring).  Call this
        when the active robot config changes (e.g. ``__main__.py``'s
        robot-selection handler)."""
        self._dead_reckoner.set_trackwidth(trackwidth)

    def feed(self, frame: "TLMFrame") -> None:
        """Ingest one TLMFrame and append to the appropriate trace lists.

        Sets anchor to (0, 0, 0) automatically on the first call if ``anchor()``
        was not called.

        Parameters
        ----------
        frame:
            Parsed telemetry frame (``TLMFrame``, from ``NezhaProtocol``'s
            telemetry delivery).  Missing sensors (``None`` fields) are
            silently skipped.

        Encoder trace fallback (097)
        -----------------------------
        Binary telemetry has NO wire representation for ``encpose`` at all
        (096-001's trim; ``TLMFrame.from_pb2()`` never sets it) -- so on the
        binary plane ``frame.encpose`` is always ``None``. Rather than let
        the ``encoder`` trace (and the canvas avatar, which now prefers it
        -- see ``canvas.py``'s ``_update_marker()``) sit permanently empty
        until sprint 098 wires a real fused pose, this dead-reckons an
        equivalent pose HOST-SIDE from ``frame.enc`` (cumulative per-wheel
        distance, always present) via ``EncoderDeadReckoner``, in the SAME
        ``(mm, mm, cdeg)`` absolute-pose shape ``encpose`` would have used
        -- so it flows through the EXISTING ``_feed_encpose()`` baseline/
        anchor-rotation machinery unchanged. A real firmware ``encpose``
        (if a future build ever adds it back) always takes priority.

        Motion-state gate (``frame.active`` -- OOP sim-motor-state fix,
        refined 121-001)
        -----------------------------------------------------------------
        ``frame.active`` (``bb.drivetrain.busy`` -- TRUE while a motion is
        in progress, FALSE once it completes) is the authoritative "is the
        robot actually moving" signal -- stronger than the
        ``_TRACE_IDLE_EPSILON_CM`` dead-band above, which only filters
        SMALL jitter and would otherwise let a stopped motion's point
        COUNT climb forever off idle rest-dither alone. But the gate
        applies ONLY to the trace-point APPEND, never to the
        ``EncoderDeadReckoner`` integrator: a completed motion's taper
        end, its final control cycle, and the plant's mechanical coast are
        all REAL wheel travel that keeps arriving in ``frame.enc`` for a
        few more frames after ``frame.active`` drops to ``False``
        (121-001,
        ``encpose-active-gate-freezes-dead-reckoner-before-motion-ends.md``).
        Returning early here used to starve the integrator of that tail --
        observed on the bench as ``encpose`` running ~10 deg short per
        360 deg turn -- because a frame this method never reached also
        never reached ``_dead_reckoner.update()``. Below, the integrator
        (``_dead_reckoner.update()`` / ``last_encpose``) runs
        UNCONDITIONALLY on every frame carrying ``enc``; only the
        trace-list append (``_feed_encpose(..., append=...)``, and the
        ``_feed_otos``/``_feed_fused`` calls below) is gated on
        ``active is not False`` -- so the idle-trace-growth problem the
        gate was added to solve stays solved (only the polylines' growth
        freezes; the reckoner is O(1) state, not a growing list).
        ``active is None`` (older/pre-fault frames that never set the
        field) still appends, same as before -- "unknown" must not be
        treated the same as a confirmed idle.
        """
        if not self._anchor_set:
            self.anchor(0.0, 0.0, 0.0)

        append = frame.active is not False  # None (unknown) still appends

        # --- encoder-only dead-reckoned pose ---
        # The integrator runs unconditionally (121-001) -- see the
        # "Motion-state gate" note above; only the trace-point append
        # inside _feed_encpose() is gated by `append`.
        encpose = frame.encpose
        if encpose is None and frame.enc is not None:
            encpose = self._dead_reckoner.update(*frame.enc)
        if encpose is not None:
            self.last_encpose = encpose
            self._feed_encpose(encpose, append=append)

        if append:
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
        self._append_if_moved(self.camera, (x_cm, y_cm))

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
        self.last_encpose = None
        self._reset_baselines()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _reset_baselines(self) -> None:
        """Clear all sensor baselines."""
        self._encpose_baseline = None
        self._otos_baseline = None
        self._pose_baseline = None
        # 097: also re-zero the host-side dead-reckoning integrator so a
        # "Set Robot @ 0,0" reset (anchor() + clear()) restarts the encoder
        # trace from a clean pose, not just a clean display baseline.
        self._dead_reckoner.reset()
        self.encoder_yaw = None

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

    @staticmethod
    def _append_if_moved(trace: list, point: tuple[float, float]) -> None:
        """Append ``point`` to ``trace`` unless it is within
        ``_TRACE_IDLE_EPSILON_CM`` of the last point already in it -- see
        this module's own header comment for why (idle rest-jitter must not
        grow a trace forever). The very first point in an empty trace
        always appends (there is nothing to compare against yet)."""
        if trace:
            last_x, last_y = trace[-1]
            dx = point[0] - last_x
            dy = point[1] - last_y
            if dx * dx + dy * dy < _TRACE_IDLE_EPSILON_CM * _TRACE_IDLE_EPSILON_CM:
                return
        trace.append(point)

    def _feed_encpose(self, encpose: tuple[int, int, int], append: bool = True) -> None:
        """Compute encoder-only pose displacement and append to the encoder trace.

        Structurally identical to ``_feed_otos``/``_feed_fused`` (068-003):
        ``encpose`` is an ALREADY-ABSOLUTE world-frame pose — the firmware's
        own ``Odometry::predict()`` output when present, or (097, binary
        plane) the host-side ``EncoderDeadReckoner`` fallback ``feed()``
        synthesizes in the identical shape (see ``feed()``'s own
        docstring). Either way this method does not re-integrate raw wheel
        counts itself. Rotates the world-frame delta by
        ``(anchor_yaw - heading_at_baseline)`` (CR-10) — see
        ``_feed_otos``/``_rw``.

        Also updates ``self.encoder_yaw`` (097) — the current encoder-trace
        heading in the display/anchor frame, radians — the same quantity
        ``_update_marker``'s avatar-heading argument used to read
        exclusively from ``frame.pose`` (fused). Computed as ``rot +
        heading`` so it is display-frame-aligned the same way the returned
        world-cm position is.

        Parameters
        ----------
        encpose:
            (x, y, heading) in (mm, mm, cdeg) absolute encoder-only pose,
            firmware ``encpose`` or the host dead-reckoning fallback.
        append:
            Whether to append a point to the ``encoder`` trace on this
            call (121-001) — ``feed()`` passes ``False`` for a frame whose
            ``active`` gate says "don't grow the polyline" (a motion tail
            or a genuinely idle connection). The baseline and
            ``encoder_yaw`` bookkeeping below always run regardless of
            this flag, so a non-appending call still leaves both correct
            for the next call that DOES append. Defaults to ``True`` so
            any other caller (e.g. a test driving this method directly)
            keeps the pre-121-001 unconditional-append behavior.
        """
        if self._encpose_baseline is None:
            self._encpose_baseline = encpose
            if append:
                self.encoder.append(self._tw(0.0, 0.0))
            self.encoder_yaw = self._anchor_h
            return

        dx_cm = (encpose[0] - self._encpose_baseline[0]) / 10.0
        dy_cm = (encpose[1] - self._encpose_baseline[1]) / 10.0
        rot = self._anchor_h - math.radians(self._encpose_baseline[2] / 100.0)
        if append:
            self._append_if_moved(self.encoder, self._rw(dx_cm, dy_cm, rot))
        # encoder_yaw always tracks the latest heading, even on a frame
        # whose position was too small to append (idle jitter) or whose
        # append was gated off entirely (a motion tail, 121-001) -- it's a
        # scalar (the canvas avatar's heading source), not a growing list,
        # so there's no "grows forever" concern gating it would address.
        self.encoder_yaw = rot + math.radians(encpose[2] / 100.0)

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
        self._append_if_moved(self.otos, self._rw(dx_cm, dy_cm, rot))

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
        self._append_if_moved(self.fused, self._rw(dx_cm, dy_cm, rot))
