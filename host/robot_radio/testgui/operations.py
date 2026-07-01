"""robot_radio.testgui.operations — Operations panel for the Robot Test GUI.

Provides :class:`OperationsPanel`, a ``QGroupBox`` containing seven one-click
action buttons:

============================================================  =============================
Button                                                         Action
============================================================  =============================
Sync Pose from Camera                                          Read tag-100 from aprilcam;
                                                               send ``SI x_mm y_mm h_cdeg``
Zero Encoders                                                  Send ``ZERO enc``
STOP                                                           Send ``STOP``
Clear Traces                                                   Call ``clear_traces_cb`` hook
Refresh Playfield                                              Read cam-3 frame + calib from
                                                               daemon; deskew via daemon H;
                                                               call ``refresh_playfield_cb``
STREAM on/off toggle                                          Send ``STREAM 50`` / ``STREAM 0``
Set Robot @ 0,0                                                Call ``set_origin_cb`` hook
                                                               (display-only, no wire cmd)
============================================================  =============================

Design rules
------------
- All PySide6 imports are **deferred** inside ``build_panel()`` so this module
  is importable without PySide6 present.
- All aprilcam / daemon imports are **lazy** (inside handler bodies).
- Buttons that require a transport connection (everything except Clear Traces,
  STREAM, and Set Robot @ 0,0) start disabled; call ``set_connected(True)``
  after ``transport.connect()`` to enable them.
- STREAM button starts disabled like the others; toggling it on sends
  ``STREAM 50``, toggling it off sends ``STREAM 0``.
- "Set Robot @ 0,0" is always enabled (display-only, no transport needed).

Hooks for ticket 008 (canvas/TraceModel)
-----------------------------------------
Three callables accepted at construction time (or settable as attributes):

``clear_traces_cb``
    Called when the user clicks "Clear Traces".  Should clear all four
    polylines in the ``TraceModel``.  No-ops safely if ``None``.

``refresh_playfield_cb(pixmap, origin_x, origin_y)``
    Called with a deskewed ``QPixmap`` AND the daemon's A1 origin (cm) when
    the user clicks "Refresh Playfield".  The panel reads the playfield frame
    and calibration from the aprilcam daemon (camera index ``_PLAYFIELD_CAMERA_INDEX``),
    deskews via the daemon's live homography H, and passes both the rectified
    ``QPixmap`` and the A1 origin to this hook.  The canvas wires
    ``CanvasController.set_background(pixmap, origin_x=ox, origin_y=oy)`` here
    so the world→pixel transform updates atomically with the background.
    No-ops safely if ``None``.

``set_origin_cb``
    Called when the user clicks "Set Robot @ 0,0".  Should re-anchor the
    ``TraceModel`` so the current pose maps to (0, 0), clear traces, and
    move the avatar to world (0,0) with heading reset to 0°.
    Display-only: no motion command is sent.  No-ops safely if ``None``.

Sim fallback
------------
In sim mode there is no daemon.  The canvas uses the static
``playfield_calibration.json`` homography and assumes
origin = (field_w/2, field_h/2) so world (0,0) = image centre, which is
correct for the simulator's true-start pose.

Pure helper (Qt-free, importable in headless tests)
----------------------------------------------------
``build_setpose_command(x_cm, y_cm, yaw_rad) -> str``
    Wraps :func:`robot_radio.robot.sync_pose.pose_to_setpose_line`.
    Returns the ``SI`` wire string without any IO.

``is_sim_transport(transport) -> bool``
    Returns True when the transport is a ``SimTransport`` instance.
    Used to decide whether to disable "Sync Pose" (no camera in Sim mode).
"""

from __future__ import annotations

import logging
import math
from typing import Callable

_log = logging.getLogger(__name__)

# Aprilcam camera index for playfield refresh (ticket design: "cam 3").
_PLAYFIELD_CAMERA_INDEX = 3

# Stream interval for "STREAM on".
_STREAM_ON_MS = 50


# ---------------------------------------------------------------------------
# Qt-free pure helpers (testable without QApplication)
# ---------------------------------------------------------------------------


def build_setpose_command(x_cm: float, y_cm: float, yaw_rad: float) -> str:
    """Return the ``SI`` firmware wire string for a world pose.

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
    """
    from robot_radio.robot.sync_pose import pose_to_setpose_line
    return pose_to_setpose_line(x_cm, y_cm, yaw_rad)


def is_sim_transport(transport: object) -> bool:
    """Return True when *transport* is a ``SimTransport`` instance.

    Importing ``SimTransport`` here would create a circular dependency through
    ``transport.py``'s deferred Qt imports; instead we use a duck-type check on
    the class name so this helper is callable without PySide6.
    """
    return type(transport).__name__ == "SimTransport"


# ---------------------------------------------------------------------------
# Operations panel factory
# ---------------------------------------------------------------------------


def build_panel(
    log_cb: Callable[[str], None],
    transport_ref: dict,
    *,
    clear_traces_cb: Callable[[], None] | None = None,
    refresh_playfield_cb: "Callable[[object, float, float], None] | None" = None,
    set_origin_cb: Callable[[], None] | None = None,
) -> "tuple[object, object]":
    """Build and return the operations panel ``QGroupBox``.

    Parameters
    ----------
    log_cb:
        Callable accepting a log message string.  Called from the Qt main
        thread whenever an operation completes or fails.
    transport_ref:
        Mutable dict holding the active transport under key ``"transport"``.
        The panel reads this on each button click to get the current transport.
    clear_traces_cb:
        Optional hook called when "Clear Traces" is clicked.  Ticket 008
        wires this to ``TraceModel.clear()``.
    refresh_playfield_cb:
        Optional hook called with ``(pixmap, origin_x, origin_y)`` when
        "Refresh Playfield" is clicked.  ``pixmap`` is the deskewed ``QPixmap``
        from the daemon; ``origin_x`` and ``origin_y`` are the daemon's A1
        origin in cm.  Ticket 008 wires this to ``CanvasController.set_background``
        so the transform updates atomically with the background.
    set_origin_cb:
        Optional hook called when "Set Robot @ 0,0" is clicked.  Should
        re-anchor the TraceModel, clear traces, and move the avatar to world
        (0,0) with heading reset to 0°.
        Display-only: no motion command is sent.  No-ops safely if ``None``.

    Returns
    -------
    tuple[QGroupBox, OpsController]
        ``panel`` — the ``QGroupBox`` widget to embed in the window layout.
        ``controller`` — the :class:`OpsController` for ``set_connected()``
        calls and for wiring hooks after construction.
    """
    from PySide6.QtWidgets import (  # type: ignore[import-untyped]
        QGroupBox,
        QHBoxLayout,
        QPushButton,
        QVBoxLayout,
        QWidget,
    )
    from PySide6.QtCore import Qt  # type: ignore[import-untyped]

    panel = QGroupBox("Operations")
    panel.setObjectName("ops_panel")
    layout = QVBoxLayout(panel)
    layout.setSpacing(4)
    layout.setContentsMargins(4, 4, 4, 4)

    # Row 1: Sync Pose | Zero Encoders | STOP
    row1 = QWidget()
    row1_layout = QHBoxLayout(row1)
    row1_layout.setContentsMargins(0, 0, 0, 0)
    row1_layout.setSpacing(4)

    sync_btn = QPushButton("Sync Pose")
    sync_btn.setObjectName("ops_btn_sync_pose")
    sync_btn.setToolTip(
        "Read tag-100 from aprilcam daemon; send SI x_mm y_mm h_cdeg to firmware.\n"
        "Disabled in Sim mode (no camera)."
    )
    sync_btn.setEnabled(False)

    zero_btn = QPushButton("Zero Encoders")
    zero_btn.setObjectName("ops_btn_zero_encoders")
    zero_btn.setToolTip("Send ZERO enc to reset wheel encoder counters to zero.")
    zero_btn.setEnabled(False)

    stop_btn = QPushButton("STOP")
    stop_btn.setObjectName("ops_btn_stop")
    stop_btn.setToolTip("Send STOP (hard motor stop).")
    stop_btn.setEnabled(False)

    row1_layout.addWidget(sync_btn)
    row1_layout.addWidget(zero_btn)
    row1_layout.addWidget(stop_btn)
    layout.addWidget(row1)

    # Row 2: Clear Traces | Refresh Playfield | STREAM toggle
    row2 = QWidget()
    row2_layout = QHBoxLayout(row2)
    row2_layout.setContentsMargins(0, 0, 0, 0)
    row2_layout.setSpacing(4)

    clear_btn = QPushButton("Clear Traces")
    clear_btn.setObjectName("ops_btn_clear_traces")
    clear_btn.setToolTip("Clear all pose trace polylines from the canvas.")
    clear_btn.setEnabled(True)  # Works without transport

    refresh_btn = QPushButton("Refresh Playfield")
    refresh_btn.setObjectName("ops_btn_refresh_playfield")
    refresh_btn.setToolTip(
        "Capture a new playfield image from camera 3, deskew it via homography,\n"
        "and update the canvas background."
    )
    refresh_btn.setEnabled(False)

    stream_btn = QPushButton("STREAM: off")
    stream_btn.setObjectName("ops_btn_stream")
    stream_btn.setCheckable(True)
    stream_btn.setChecked(False)
    stream_btn.setToolTip(
        "Toggle telemetry streaming.\n"
        "ON → STREAM 50 (50 ms interval).\n"
        "OFF → STREAM 0 (stop streaming)."
    )
    stream_btn.setEnabled(False)

    row2_layout.addWidget(clear_btn)
    row2_layout.addWidget(refresh_btn)
    row2_layout.addWidget(stream_btn)
    layout.addWidget(row2)

    # Row 3: Set Robot @ 0,0 (display-only, always enabled)
    row3 = QWidget()
    row3_layout = QHBoxLayout(row3)
    row3_layout.setContentsMargins(0, 0, 0, 0)
    row3_layout.setSpacing(4)

    origin_btn = QPushButton("Set Robot @ 0,0")
    origin_btn.setObjectName("ops_btn_set_origin")
    origin_btn.setToolTip(
        "Re-anchor the avatar to the playfield centre (world 0,0).\n"
        "Physically place the robot at the playfield centre first.\n"
        "Display-only — sends NO motion command to the robot."
    )
    origin_btn.setEnabled(True)  # Works without transport (display-only)

    row3_layout.addWidget(origin_btn)
    row3_layout.addStretch()
    layout.addWidget(row3)

    # Buttons that need a transport connection.
    _transport_buttons = [sync_btn, zero_btn, stop_btn, refresh_btn, stream_btn]

    # ------------------------------------------------------------------ controller
    controller = OpsController(
        transport_ref=transport_ref,
        log_cb=log_cb,
        sync_btn=sync_btn,
        zero_btn=zero_btn,
        stop_btn=stop_btn,
        clear_btn=clear_btn,
        refresh_btn=refresh_btn,
        stream_btn=stream_btn,
        origin_btn=origin_btn,
        transport_buttons=_transport_buttons,
        clear_traces_cb=clear_traces_cb,
        refresh_playfield_cb=refresh_playfield_cb,
        set_origin_cb=set_origin_cb,
    )

    # Wire buttons to controller handlers.
    sync_btn.clicked.connect(controller.on_sync_pose)
    zero_btn.clicked.connect(controller.on_zero_encoders)
    stop_btn.clicked.connect(controller.on_stop)
    clear_btn.clicked.connect(controller.on_clear_traces)
    refresh_btn.clicked.connect(controller.on_refresh_playfield)
    stream_btn.toggled.connect(controller.on_stream_toggled)
    origin_btn.clicked.connect(controller.on_set_origin)

    return panel, controller


# ---------------------------------------------------------------------------
# OpsController — holds all handler logic
# ---------------------------------------------------------------------------


class OpsController:
    """Holds handler logic for the operations panel buttons.

    Separated from the widget-building code so handlers can be tested headlessly
    (by injecting a fake transport, fake QPixmap, etc.) without needing a full
    QApplication.

    Attributes
    ----------
    clear_traces_cb:
        Hook for "Clear Traces".  Ticket 008 wires this to ``TraceModel.clear()``.
    refresh_playfield_cb:
        Hook for "Refresh Playfield".  Called with ``(pixmap, origin_x, origin_y)``
        where ``pixmap`` is a deskewed ``QPixmap`` and ``origin_x``/``origin_y``
        are the daemon's A1 origin in cm.  Ticket 008 wires this to
        ``CanvasController.set_background`` so the transform and background
        update atomically.
    set_origin_cb:
        Hook for "Set Robot @ 0,0".  Re-anchors the TraceModel, moves the
        avatar to world (0,0), and resets heading to 0°.  Display-only: no
        motion command is sent.
    """

    def __init__(
        self,
        *,
        transport_ref: dict,
        log_cb: Callable[[str], None],
        sync_btn: "object",
        zero_btn: "object",
        stop_btn: "object",
        clear_btn: "object",
        refresh_btn: "object",
        stream_btn: "object",
        origin_btn: "object",
        transport_buttons: list,
        clear_traces_cb: Callable[[], None] | None = None,
        refresh_playfield_cb: "Callable[[object, float, float], None] | None" = None,
        set_origin_cb: Callable[[], None] | None = None,
    ) -> None:
        self._transport_ref = transport_ref
        self._log_cb = log_cb
        self._sync_btn = sync_btn
        self._zero_btn = zero_btn
        self._stop_btn = stop_btn
        self._clear_btn = clear_btn
        self._refresh_btn = refresh_btn
        self._stream_btn = stream_btn
        self._origin_btn = origin_btn
        self._transport_buttons = transport_buttons
        self.clear_traces_cb = clear_traces_cb
        self.refresh_playfield_cb = refresh_playfield_cb
        self.set_origin_cb = set_origin_cb
        self._stream_on = False  # tracks stream toggle state

    # ------------------------------------------------------------------
    # Public API — called by __main__.py after connect()/disconnect()
    # ------------------------------------------------------------------

    def set_connected(self, connected: bool, transport: "object | None" = None) -> None:
        """Enable or disable transport-dependent buttons.

        In Sim mode, the "Sync Pose" button is disabled (no camera) with a
        tooltip explaining why.  All other transport-dependent buttons are
        enabled.

        Parameters
        ----------
        connected:
            ``True`` after a successful ``transport.connect()``;
            ``False`` after ``transport.disconnect()``.
        transport:
            The active transport (used to detect Sim mode).
        """
        for btn in self._transport_buttons:
            btn.setEnabled(connected)  # type: ignore[attr-defined]

        if connected and transport is not None and is_sim_transport(transport):
            self._sync_btn.setEnabled(False)  # type: ignore[attr-defined]
            self._sync_btn.setToolTip(  # type: ignore[attr-defined]
                "Sync Pose is not available in Sim mode.\n"
                "(The simulator delivers ground-truth pose via on_truth callback;\n"
                "use it to observe the pose, not to seed it from a camera.)"
            )

        if not connected:
            # Reset stream toggle so next connect starts with streaming off.
            self._stream_btn.setChecked(False)  # type: ignore[attr-defined]
            self._stream_btn.setText("STREAM: off")  # type: ignore[attr-defined]
            self._stream_on = False

    # ------------------------------------------------------------------
    # Button handlers
    # ------------------------------------------------------------------

    def on_sync_pose(self) -> None:
        """Read tag-100 from the aprilcam daemon; send ``SI x_mm y_mm h_cdeg``."""
        transport = self._transport_ref.get("transport")
        if transport is None:
            self._log("[WARN] Sync Pose: not connected")
            return

        self._log("[INFO] Sync Pose: reading pose from aprilcam daemon...")
        try:
            pose = self._read_daemon_pose()
        except Exception as exc:
            self._log(f"[WARN] Sync Pose: daemon read failed: {exc}")
            return

        if pose is None:
            self._log(
                "[WARN] Sync Pose: tag 100 not seen within timeout "
                "(is the robot on the field and visible?)"
            )
            return

        x_cm, y_cm, yaw_rad = pose
        line = build_setpose_command(x_cm, y_cm, yaw_rad)
        self._log(
            f"[INFO] Sync Pose: daemon=({x_cm:.1f}cm, {y_cm:.1f}cm, "
            f"{math.degrees(yaw_rad):.1f}°) → {line}"
        )
        try:
            reply = transport.command(line, read_ms=500)
            if reply:
                self._log(f"[INFO] Sync Pose reply: {reply.strip()}")
            else:
                self._log("[INFO] Sync Pose: sent (no reply)")
        except Exception as exc:
            self._log(f"[ERROR] Sync Pose: command failed: {exc}")

    def on_zero_encoders(self) -> None:
        """Send ``ZERO enc`` to reset wheel encoder counters."""
        transport = self._transport_ref.get("transport")
        if transport is None:
            self._log("[WARN] Zero Encoders: not connected")
            return
        try:
            reply = transport.command("ZERO enc", read_ms=300)
            if reply:
                self._log(f"[INFO] Zero Encoders: {reply.strip()}")
            else:
                self._log("[INFO] Zero Encoders: sent")
        except Exception as exc:
            self._log(f"[ERROR] Zero Encoders: {exc}")

    def on_stop(self) -> None:
        """Send ``STOP`` (hard motor stop)."""
        transport = self._transport_ref.get("transport")
        if transport is None:
            self._log("[WARN] STOP: not connected")
            return
        try:
            transport.send("STOP")
            self._log("[INFO] STOP sent")
        except Exception as exc:
            self._log(f"[ERROR] STOP: {exc}")

    def on_clear_traces(self) -> None:
        """Clear all trace polylines (no transport command)."""
        if self.clear_traces_cb is not None:
            try:
                self.clear_traces_cb()
            except Exception as exc:
                self._log(f"[ERROR] Clear Traces: callback raised: {exc}")
                return
        self._log("[INFO] Clear Traces: done")

    def on_set_origin(self) -> None:
        """Re-anchor avatar to playfield centre (display-only, no wire command).

        Calls ``set_origin_cb`` which should:
        1. Re-anchor the ``TraceModel`` so the current pose maps to (0, 0).
        2. Clear existing trace polylines.
        3. Move the canvas avatar to world (0, 0).

        This is a GUI-only operation — no motion command is sent to the robot.
        """
        if self.set_origin_cb is not None:
            try:
                self.set_origin_cb()
            except Exception as exc:
                self._log(f"[ERROR] Set Robot @ 0,0: callback raised: {exc}")
                return
        self._log("[INFO] Set Robot @ 0,0: avatar anchored to centre")

    def on_refresh_playfield(self) -> None:
        """Capture a playfield image + calibration from aprilcam and call refresh_playfield_cb.

        Calls ``refresh_playfield_cb(pixmap, origin_x, origin_y)`` where:
        - ``pixmap`` is the deskewed ``QPixmap`` (warped using the daemon's H).
        - ``origin_x``, ``origin_y`` are the A1 origin (cm, corner-origin frame)
          from the daemon's TagFrame; the canvas uses these so world (0,0)
          maps to tag 1's real pixel position.
        """
        transport = self._transport_ref.get("transport")
        if transport is None:
            self._log("[WARN] Refresh Playfield: not connected")
            return

        self._log(f"[INFO] Refresh Playfield: capturing from camera {_PLAYFIELD_CAMERA_INDEX}...")
        try:
            result = self._capture_playfield_frame_and_calib()
        except Exception as exc:
            self._log(f"[WARN] Refresh Playfield: capture failed: {exc}")
            return

        if result is None:
            self._log("[WARN] Refresh Playfield: no image from daemon (is it running?)")
            return

        pixmap, origin_x, origin_y = result

        if self.refresh_playfield_cb is not None:
            try:
                self.refresh_playfield_cb(pixmap, origin_x, origin_y)
            except Exception as exc:
                self._log(f"[ERROR] Refresh Playfield: callback raised: {exc}")
                return
        self._log(
            f"[INFO] Refresh Playfield: done "
            f"(origin=({origin_x:.1f},{origin_y:.1f}) cm)"
        )

    def on_stream_toggled(self, checked: bool) -> None:
        """Toggle telemetry streaming; send ``STREAM 50`` or ``STREAM 0``."""
        transport = self._transport_ref.get("transport")
        if transport is None:
            # Button should be disabled when no transport — but handle defensively.
            self._stream_btn.setChecked(not checked)  # type: ignore[attr-defined]
            self._log("[WARN] STREAM toggle: not connected")
            return

        if checked:
            cmd = f"STREAM {_STREAM_ON_MS}"
            label = "STREAM: on"
        else:
            cmd = "STREAM 0"
            label = "STREAM: off"

        try:
            reply = transport.command(cmd, read_ms=300)
            self._stream_on = checked
            self._stream_btn.setText(label)  # type: ignore[attr-defined]
            if reply:
                self._log(f"[INFO] {cmd} → {reply.strip()}")
            else:
                self._log(f"[INFO] {cmd} sent")
        except Exception as exc:
            # Revert toggle state on failure.
            self._stream_btn.setChecked(not checked)  # type: ignore[attr-defined]
            self._log(f"[ERROR] STREAM toggle: {exc}")

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _log(self, text: str) -> None:
        """Deliver a log message via the registered callback."""
        try:
            self._log_cb(text)
        except Exception:
            pass

    def _read_daemon_pose(self) -> tuple[float, float, float] | None:
        """Read tag-100 world pose from the aprilcam daemon.

        Returns ``(x_cm, y_cm, yaw_rad)`` or ``None`` on failure.
        Raises exceptions on connection errors so the caller can log them.
        """
        try:
            from aprilcam.config import Config  # type: ignore[import]
            from aprilcam.client.control import DaemonControl  # type: ignore[import]
        except ImportError as exc:
            raise RuntimeError(
                f"aprilcam package not installed: {exc}"
            ) from exc

        from robot_radio.robot.sync_pose import daemon_read_pose

        dc = DaemonControl.connect_default(Config.load())
        try:
            cams = dc.list_cameras()
            if not cams:
                raise RuntimeError(
                    "aprilcam daemon reports no cameras — is a camera open?"
                )
            cam = cams[0]
            pose = daemon_read_pose(dc, cam, tag_id=100, timeout_s=3.0)
        finally:
            try:
                dc.close()
            except Exception:
                pass
        return pose

    def _capture_playfield_frame_and_calib(
        self,
    ) -> "tuple[object, float, float] | None":
        """Capture a frame from the aprilcam daemon and deskew via its live homography.

        Returns ``(QPixmap, origin_x, origin_y)`` on success, or ``None`` if the
        daemon is not available.  Raises on connection errors so the caller can log.

        The deskew uses the daemon's TagFrame homography H (not the static JSON),
        so the rectified image always matches the daemon's current calibration.
        origin_x / origin_y are the daemon's A1 offset (corner-origin cm) — the
        canvas uses these to place world (0,0) at tag 1's real pixel position.
        """
        try:
            from aprilcam.config import Config  # type: ignore[import]
            from aprilcam.client.control import DaemonControl  # type: ignore[import]
        except ImportError as exc:
            raise RuntimeError(f"aprilcam package not installed: {exc}") from exc

        dc = DaemonControl.connect_default(Config.load())
        try:
            cams = dc.list_cameras()
            if not cams:
                raise RuntimeError("aprilcam daemon reports no cameras")

            # Select the playfield camera by name-based heuristic: prefer cameras
            # whose name contains the index digit, or fall back to cams[0].
            cam_name: str = cams[0]
            for c in cams:
                if str(_PLAYFIELD_CAMERA_INDEX) in str(c):
                    cam_name = c
                    break
            _log.debug("Refresh Playfield: using camera %r", cam_name)

            # Capture BGR frame + calibration in a single daemon session.
            raw_bgr = dc.capture_frame(cam_name)
            tag_frame = dc.get_tags(cam_name)
        finally:
            try:
                dc.close()
            except Exception:
                pass

        if raw_bgr is None:
            return None

        return _deskew_bgr_with_tag_frame(raw_bgr, tag_frame)


def _bgr_ndarray_to_pixmap(bgr: "object") -> "object | None":
    """Convert a BGR numpy ndarray to a ``QPixmap``.

    Returns ``None`` on failure or if PySide6 is unavailable.
    """
    try:
        import numpy as np
        from PySide6.QtGui import QImage, QPixmap  # type: ignore[import-untyped]
        bgr_arr = np.ascontiguousarray(bgr)
        h, w, ch = bgr_arr.shape
        rgb = bgr_arr[:, :, ::-1].copy()
        qi = QImage(rgb.data, w, h, w * ch, QImage.Format.Format_RGB888)
        pm = QPixmap.fromImage(qi)
        if pm.isNull():
            return None
        return pm
    except Exception:
        _log.debug("_bgr_ndarray_to_pixmap failed", exc_info=True)
        return None


def _deskew_bgr_with_tag_frame(
    raw_bgr: "object",
    tag_frame: "object",
    ppc: float | None = None,
) -> "tuple[object, float, float] | None":
    """Deskew *raw_bgr* using the daemon TagFrame's homography and return calibration.

    Uses the live daemon homography (not the static JSON) so the rectified image
    always matches the daemon's current calibration.

    Parameters
    ----------
    raw_bgr:
        Raw BGR ndarray from ``DaemonControl.capture_frame()``.
    tag_frame:
        ``TagFrame`` from ``DaemonControl.get_tags()`` — carries ``.homography``
        (3×3), ``.origin_x``, ``.origin_y``, ``.field_width_cm``,
        ``.field_height_cm``.
    ppc:
        Pixels per cm.  If ``None``, uses ``canvas._PIXELS_PER_CM`` (8.0).

    Returns
    -------
    ``(QPixmap, origin_x, origin_y)`` on success, or ``None`` on failure.
    origin_x and origin_y are the daemon's A1 offset in cm (corner-origin frame).
    """
    try:
        import numpy as np
        import cv2
        from robot_radio.testgui.canvas import _PIXELS_PER_CM

        if ppc is None:
            ppc = _PIXELS_PER_CM

        homography_raw = getattr(tag_frame, "homography", None)
        if homography_raw is None:
            raise RuntimeError("TagFrame has no homography (camera not calibrated?)")

        H = np.array(homography_raw, dtype=float)
        if H.shape != (3, 3):
            raise RuntimeError(f"Homography shape {H.shape!r}; expected (3,3)")

        fw = float(getattr(tag_frame, "field_width_cm", 0.0))
        fh = float(getattr(tag_frame, "field_height_cm", 0.0))
        origin_x = float(getattr(tag_frame, "origin_x", 0.0))
        origin_y = float(getattr(tag_frame, "origin_y", 0.0))

        if fw <= 0 or fh <= 0:
            raise RuntimeError(
                f"TagFrame field dims ({fw},{fh}) invalid; is the camera calibrated?"
            )

        # warp = diag(ppc, ppc, 1) @ H  (matches PlayfieldCalibration.warp_matrix)
        scale = np.array([[ppc, 0, 0], [0, ppc, 0], [0, 0, 1]], dtype=float)
        warp = scale @ H
        out_w = max(1, int(round(fw * ppc)))
        out_h = max(1, int(round(fh * ppc)))

        deskewed = cv2.warpPerspective(raw_bgr, warp, (out_w, out_h))
        _log.debug(
            "Deskewed via daemon H: %dx%d px, origin=(%.1f,%.1f) cm",
            out_w, out_h, origin_x, origin_y,
        )

        pixmap = _bgr_ndarray_to_pixmap(deskewed)
        if pixmap is None:
            raise RuntimeError("Failed to convert deskewed BGR to QPixmap")
        return pixmap, origin_x, origin_y

    except Exception as exc:
        _log.debug("_deskew_bgr_with_tag_frame failed (%s)", exc)
        return None
