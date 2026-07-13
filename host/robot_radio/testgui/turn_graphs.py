"""robot_radio.testgui.turn_graphs — live time-series graph tabs for turns/drives.

Four matplotlib graph tabs that record while the robot is MOVING (idle frames
skipped) and plot every available estimate of each quantity over time, mirroring
the way the playfield shows OTOS vs camera:

  - Wheel speed     — commanded vs actual per-wheel velocity        [mm/s]
  - Wheel position  — per-wheel cumulative encoder distance         [mm]
  - Heading         — OTOS / fused / encoder / camera Δheading      [deg]
  - Distance        — OTOS / fused / encoder / camera displacement  [cm]

The recorder (`TurnTraceRecorder`) is Qt-free and unit-testable; the
`TurnGraphTabs` QTabWidget owns one recorder and four canvases. Heading is
UNWRAPPED per source so a ±360 turn draws a single continuous ramp rather than
a saw-tooth at the ±180 wrap.
"""
from __future__ import annotations

import math
from collections import defaultdict
from typing import Any

from .traces import EncoderDeadReckoner

# --- series keys, grouped per graph -----------------------------------------
WHEEL_SPEED = [("cmd_l", "cmd L"), ("cmd_r", "cmd R"), ("vel_l", "actual L"),
               ("vel_r", "actual R")]
WHEEL_POS = [("enc_l", "enc L"), ("enc_r", "enc R")]
HEADING = [("head_otos", "OTOS"), ("head_fused", "fused"),
           ("head_enc", "encoder"), ("head_cam", "camera")]
DISTANCE = [("dist_otos", "OTOS"), ("dist_fused", "fused"),
            ("dist_enc", "encoder"), ("dist_cam", "camera")]

# Motion gate: record a TLM frame only when the robot is actually moving.
_MOVING_SPEED = 5.0  # [mm/s] any wheel faster than this counts as motion
_IDLE_STOP = 1.0     # [s] freeze recording after this long with no motion


class _Unwrapper:
    """Accumulate a continuous angle from wrapped [-180,180] degree samples."""

    def __init__(self) -> None:
        self._prev: float | None = None
        self._cont = 0.0

    def push(self, deg: float) -> float:
        if self._prev is None:
            self._prev = deg
            self._cont = deg
            return deg
        d = deg - self._prev
        d = (d + 180.0) % 360.0 - 180.0
        self._cont += d
        self._prev = deg
        return self._cont

    def reset(self) -> None:
        self._prev = None
        self._cont = 0.0


def tlm_fields(frame: Any) -> dict:
    """Pull the quantities we plot out of a TLMFrame, defensively.

    Field names/units are confirmed against protocol.py's TLMFrame:
      enc  = (left, right) cumulative wheel distance [mm]
      vel  = (left, right) wheel velocity            [mm/s]
      cmd  = (left, right) COMMANDED wheel velocity   [mm/s] (may be absent)
      pose = (x, y, h)  fused    [mm, mm, cdeg]
      otos = (x, y, h)  raw OTOS [mm, mm, cdeg]
      active = bool motion-in-progress flag (may be absent)
    """
    def pair(v):
        return (float(v[0]), float(v[1])) if v is not None and len(v) >= 2 else None

    def triple(v):
        return (float(v[0]), float(v[1]), float(v[2])) if v is not None and len(v) >= 3 else None

    return {
        "enc": pair(getattr(frame, "enc", None)),
        "vel": pair(getattr(frame, "vel", None)),
        "cmd": pair(getattr(frame, "cmd_vel", None)),  # TLMFrame field is cmd_vel
        "pose": triple(getattr(frame, "pose", None)),
        "otos": triple(getattr(frame, "otos", None)),
        "active": getattr(frame, "active", None),
    }


class TurnTraceRecorder:
    """Qt-free accumulator of per-series (t, value) points.

    ``add_tlm``/``add_camera`` append points; idle TLM frames (no wheel motion)
    are skipped. ``clear`` restarts. Distances are displacement magnitude from
    the first recorded pose per source; headings are unwrapped Δ from the first
    recorded heading per source.
    """

    def __init__(self, trackwidth: float = 128.0) -> None:  # [mm]
        self._trackwidth = trackwidth
        self.clear()

    def clear(self) -> None:
        self.series: dict[str, list[tuple[float, float]]] = defaultdict(list)
        self._t0: float | None = None
        self._enc_dr = EncoderDeadReckoner(self._trackwidth)
        self._unwrap = {k: _Unwrapper() for k, _ in HEADING}
        self._h0: dict[str, float] = {}
        self._p0: dict[str, tuple[float, float]] = {}
        self._last_motion: float | None = None  # [s] wall time of last moving frame
        self._stopped = False                    # recording frozen after >1s idle

    def set_trackwidth(self, trackwidth: float) -> None:  # [mm]
        self._trackwidth = trackwidth
        self._enc_dr.set_trackwidth(trackwidth)

    def _t(self, now: float) -> float:
        if self._t0 is None:
            self._t0 = now
        return now - self._t0

    def _heading(self, key: str, now: float, deg: float) -> None:
        cont = self._unwrap[key].push(deg)
        if key not in self._h0:
            self._h0[key] = cont
        self.series[key].append((self._t(now), cont - self._h0[key]))

    def _distance(self, key: str, now: float, x_cm: float, y_cm: float) -> None:
        if key not in self._p0:
            self._p0[key] = (x_cm, y_cm)
        x0, y0 = self._p0[key]
        self.series[key].append((self._t(now), math.hypot(x_cm - x0, y_cm - y0)))

    def add_tlm(self, now: float, frame: Any) -> bool:
        """Record one telemetry frame. Returns True if recorded (moving).

        Recording auto-freezes after >1 s of no wheel motion (``_stopped``);
        the next moving frame auto-clears and starts a FRESH trace, so each
        turn is captured cleanly on its own axis.
        """
        f = tlm_fields(frame)
        vel = f["vel"] or (0.0, 0.0)
        moving = (f["active"] is True) or (max(abs(vel[0]), abs(vel[1])) > _MOVING_SPEED)
        if not moving:
            # Freeze the trace once the wheels have been idle for > 1 s.
            if (not self._stopped and self._last_motion is not None
                    and now - self._last_motion > _IDLE_STOP):
                self._stopped = True
            return False
        if self._stopped:
            # New motion after a freeze -> start a fresh trace.
            self.clear()
        self._last_motion = now
        t = self._t(now)
        if f["cmd"]:
            self.series["cmd_l"].append((t, f["cmd"][0]))
            self.series["cmd_r"].append((t, f["cmd"][1]))
        self.series["vel_l"].append((t, vel[0]))
        self.series["vel_r"].append((t, vel[1]))
        if f["enc"]:
            self.series["enc_l"].append((t, f["enc"][0]))
            self.series["enc_r"].append((t, f["enc"][1]))
            ex, ey, eh = self._enc_dr.update(f["enc"][0], f["enc"][1])  # mm,mm,cdeg
            self._heading("head_enc", now, eh / 100.0)
            self._distance("dist_enc", now, ex / 10.0, ey / 10.0)  # mm->cm
        if f["otos"]:
            self._heading("head_otos", now, f["otos"][2] / 100.0)
            self._distance("dist_otos", now, f["otos"][0] / 10.0, f["otos"][1] / 10.0)
        if f["pose"]:
            self._heading("head_fused", now, f["pose"][2] / 100.0)
            self._distance("dist_fused", now, f["pose"][0] / 10.0, f["pose"][1] / 10.0)
        return True

    def add_camera(self, now: float, x_cm: float, y_cm: float, heading_deg: float) -> None:
        """Record one camera ground-truth sample (world cm + heading deg)."""
        # Only record between the first moving frame and the idle-freeze, so
        # idle camera frames don't stretch the axis or bleed past a turn.
        if self._t0 is None or self._stopped:
            return
        self._heading("head_cam", now, heading_deg)
        self._distance("dist_cam", now, x_cm, y_cm)


# --- Qt widgets -------------------------------------------------------------
# Imported lazily-friendly at module top so the recorder half stays Qt-free
# for tests that import only TurnTraceRecorder.
from PySide6.QtCore import QTimer  # noqa: E402
from PySide6.QtWidgets import (  # noqa: E402
    QHBoxLayout, QLabel, QPushButton, QTabWidget, QVBoxLayout, QWidget,
)
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

# Stable per-series colors (matplotlib default cycle is fine but we pin the
# multi-source graphs so OTOS/fused/encoder/camera keep the playfield's mapping:
# camera=green, encoder=orange, otos=cyan, fused=magenta).
_SRC_COLOR = {
    "head_cam": "#2ca02c", "dist_cam": "#2ca02c",
    "head_enc": "#ff7f0e", "dist_enc": "#ff7f0e",
    "head_otos": "#17becf", "dist_otos": "#17becf",
    "head_fused": "#d62728", "dist_fused": "#d62728",
    "cmd_l": "#1f77b4", "cmd_r": "#9467bd",
    "vel_l": "#2ca02c", "vel_r": "#ff7f0e",
    "enc_l": "#1f77b4", "enc_r": "#d62728",
}


class _GraphCanvas(FigureCanvasQTAgg):
    """One matplotlib figure plotting a fixed set of recorder series vs time."""

    def __init__(self, title: str, ylabel: str, series: list[tuple[str, str]]) -> None:
        self._fig = Figure(figsize=(5, 3.2), tight_layout=True)
        super().__init__(self._fig)
        self._ax = self._fig.add_subplot(111)
        self._title = title
        self._ylabel = ylabel
        self._series = series

    def redraw(self, recorder: TurnTraceRecorder) -> None:
        ax = self._ax
        ax.clear()
        any_pts = False
        for key, label in self._series:
            pts = recorder.series.get(key)
            if not pts:
                continue
            any_pts = True
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            ax.plot(xs, ys, label=label, color=_SRC_COLOR.get(key), linewidth=1.4)
        ax.set_title(self._title)
        ax.set_xlabel("t [s]")
        ax.set_ylabel(self._ylabel)
        ax.grid(True, alpha=0.3)
        if any_pts:
            ax.legend(loc="best", fontsize=8, ncol=2)
        self.draw_idle()


class TurnGraphPanel(QWidget):
    """Clear-button header + a tab bar: [Playfield] + the four live graphs.

    Owns one ``TurnTraceRecorder``. Feed it with ``add_tlm(now, frame)`` and
    ``add_camera(now, x_cm, y_cm, heading_deg)`` from the telemetry/camera
    hooks (Qt main thread); only the currently-visible graph repaints
    (throttled), so recording stays cheap. Idle frames are skipped by the
    recorder; ``Clear`` restarts the traces.
    """

    def __init__(self, recorder: TurnTraceRecorder | None = None,
                 playfield_widget: QWidget | None = None,
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.recorder = recorder if recorder is not None else TurnTraceRecorder()

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(2)

        header = QHBoxLayout()
        self._clear_btn = QPushButton("Clear traces")
        self._clear_btn.setToolTip("Clear all recorded traces and restart recording")
        self._clear_btn.clicked.connect(self.clear)
        self._status = QLabel("")
        header.addWidget(self._clear_btn)
        header.addWidget(self._status)
        header.addStretch(1)
        outer.addLayout(header)

        self._tabs = QTabWidget()
        outer.addWidget(self._tabs, 1)
        if playfield_widget is not None:
            self._tabs.addTab(playfield_widget, "Playfield")

        self._speed = _GraphCanvas("Wheel speed — commanded vs actual", "mm/s", WHEEL_SPEED)
        self._pos = _GraphCanvas("Wheel position (encoder)", "mm", WHEEL_POS)
        self._head = _GraphCanvas("Heading Δ — OTOS / fused / encoder / camera", "deg", HEADING)
        self._dist = _GraphCanvas("Distance (displacement) — all sources", "cm", DISTANCE)
        for w, name in ((self._speed, "Wheel speed"), (self._pos, "Wheel position"),
                        (self._head, "Heading"), (self._dist, "Distance")):
            self._tabs.addTab(w, name)

        self._dirty = False
        self._tabs.currentChanged.connect(lambda _i: self._redraw_current())
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._maybe_redraw)
        self._timer.start(150)

    # feed hooks (call from the GUI thread) --------------------------------
    def add_tlm(self, now: float, frame: Any) -> None:
        if self.recorder.add_tlm(now, frame):
            self._dirty = True

    def add_camera(self, now: float, x_cm: float, y_cm: float, heading_deg: float) -> None:
        self.recorder.add_camera(now, x_cm, y_cm, heading_deg)
        self._dirty = True

    def clear(self) -> None:
        self.recorder.clear()
        self._redraw_current()

    # redraw plumbing ------------------------------------------------------
    def _maybe_redraw(self) -> None:
        if self._dirty:
            self._dirty = False
            self._redraw_current()

    def _redraw_current(self) -> None:
        w = self._tabs.currentWidget()
        if isinstance(w, _GraphCanvas):
            w.redraw(self.recorder)
        n = len(self.recorder.series.get("head_cam", [])) or len(self.recorder.series.get("vel_l", []))
        self._status.setText(f"{n} pts" if n else "")
