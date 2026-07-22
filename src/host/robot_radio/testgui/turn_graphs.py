"""robot_radio.testgui.turn_graphs — live time-series graph tabs for turns/drives.

Matplotlib graph tabs that record while the robot is MOVING and FREEZE the
instant it stops (2026-07-22 GUI stakeholder directive: "stop the charts
... do not run the charts when the robot is stopped") — see
``TurnTraceRecorder.add_tlm()``'s own docstring for the exact freeze
condition and how it coexists with the gap-break fix (``_append_series()``,
below) that makes skipping idle frames here SAFE: an earlier version of
this file also skipped idle frames, with no gap-break mechanism backing it
up, and THAT combination was a separate, confirmed defect of its own — a
straight interpolating line drawn across the un-recorded idle span. Plots
every available estimate of each quantity over time, mirroring the way the
playfield shows OTOS vs camera:

  - Wheel speed     — commanded vs actual per-wheel velocity        [mm/s]
  - Twist           — commanded vs actual body v_x [mm/s] and ω [deg/s]
  - Wheel position  — per-wheel cumulative encoder distance         [mm]
  - Heading         — OTOS / fused / encoder / camera Δheading      [deg]
  - Distance        — OTOS / fused / encoder / camera displacement  [cm]

The recorder (`TurnTraceRecorder`) is Qt-free and unit-testable; the
`TurnGraphTabs` QTabWidget owns one recorder and four canvases. Heading is
UNWRAPPED per source so a ±360 turn draws a single continuous ramp rather than
a saw-tooth at the ±180 wrap.

`StripChartCanvas` (110-002) is a `_GraphCanvas` variant that plots only the
trailing N seconds (default 10s) of each series at redraw time — a pure
windowing FILTER over the SAME `TurnTraceRecorder.series` these full-history
canvases read, never a second recorder or a second telemetry-consumption
path. `telemetry_panel.py`'s rolling strip-chart tabs are built on it,
sharing whichever `TurnTraceRecorder` the caller passes in (normally the
SAME one this module's own `TurnGraphPanel` owns).
"""
from __future__ import annotations

import math
from collections import defaultdict
from typing import Any, Callable

from .traces import EncoderDeadReckoner

# --- series keys, grouped per graph -----------------------------------------
WHEEL_SPEED = [("cmd_l", "cmd L"), ("cmd_r", "cmd R"), ("vel_l", "actual L"),
               ("vel_r", "actual R")]
# Body twist -- commanded vs actual v_x [mm/s] and omega [deg/s]. Commanded
# is the forward kinematics of the commanded wheel speeds (cmd_vel, sim Path
# B); actual is the fused body velocity already on the wire (frame.twist).
# omega is plotted in deg/s so it shares the axis with v_x [mm/s] at a
# comparable magnitude (a 2 rad/s pivot == 115 deg/s ~ a 150 mm/s straight).
TWIST = [("cmd_vx", "cmd v_x"), ("act_vx", "act v_x"),
         ("cmd_omega", "cmd ω"), ("act_omega", "act ω")]
WHEEL_POS = [("enc_l", "enc L"), ("enc_r", "enc R")]
HEADING = [("head_otos", "OTOS"), ("head_fused", "fused"),
           ("head_enc", "encoder"), ("head_cam", "camera")]
DISTANCE = [("dist_otos", "OTOS"), ("dist_fused", "fused"),
            ("dist_enc", "encoder"), ("dist_cam", "camera")]

# Motion gate (2026-07-22 GUI stakeholder directive): recording -- and by
# extension the strip charts' visible axis, which only advances when a new
# point lands -- freezes the instant the wheels are genuinely idle. See
# ``TurnTraceRecorder.add_tlm()``'s own docstring for the exact condition
# and why skipping idle frames here does not reopen the older gap defect.
_MOVING_SPEED = 5.0  # [mm/s] any wheel faster than this counts as motion

# Gap-break display honesty (2026-07-22 GUI stakeholder fix, defect 2b): a
# real, un-recorded gap between two consecutive points in the SAME series
# must break the plotted line, never draw a straight interpolating segment
# across it -- the stakeholder's own rule: "if the motor stopped, record
# the time at which it stopped as the time at which you break that line."
# _NOMINAL_TLM_PERIOD is the conservative (slow-end) telemetry cadence
# (~15-19 Hz observed -- see this module's own header); GAP_BREAK_FACTOR is
# how many nominal periods must elapse, with no point recorded in a given
# series, before that gap is treated as real (a stall/disconnect/idle span)
# rather than ordinary sample-to-sample jitter -- see
# ``TurnTraceRecorder._append_series()``.
_NOMINAL_TLM_PERIOD = 1.0 / 15.0   # [s]
GAP_BREAK_FACTOR = 5


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
        "twist": pair(getattr(frame, "twist", None)),  # (v_x [mm/s], omega [mrad/s])
        "pose": triple(getattr(frame, "pose", None)),
        "otos": triple(getattr(frame, "otos", None)),
        "active": getattr(frame, "active", None),
    }


class TurnTraceRecorder:
    """Qt-free accumulator of per-series (t, value) points.

    ONE shared freeze predicate governs EVERY append entry point (2026-07-22
    stakeholder follow-up: the first cut of the freeze only gated
    ``add_tlm()``'s own wheel-telemetry series; ``add_camera()`` kept
    appending unconditionally, so ``latest_t()`` — read by ``StripChartCanvas``
    for its trailing-window anchor — kept climbing off camera updates alone,
    and the telemetry-panel's rolling strip charts kept visibly scrolling
    even while every wheel series was correctly frozen and the TOP,
    full-history graphs looked stopped. ``self._robot_moving`` is the single
    owner of "is the robot moving right now" -- set by ``add_tlm()`` from
    its own frame's ``active``/velocity signal on EVERY call (moving or
    not), and consulted by BOTH ``add_tlm()`` and ``add_camera()`` before
    appending anything. No other state -- and no future caller of either
    method, or any future plotting widget reading ``series``/``latest_t()``
    -- needs its own copy of this logic to stay correct.

    ``add_tlm``/``add_camera`` both append points only while
    ``self._robot_moving`` is True — frozen (no append at all, from ANY
    source) the instant the robot is idle; see ``add_tlm()``'s own
    docstring for the exact moving condition. ``clear`` restarts. Distances
    are displacement magnitude from the first recorded pose per source;
    headings are unwrapped Δ from the first recorded heading per source.
    Every append goes through ``_append_series()``, which inserts a NaN
    break beforehand when the gap since that SAME series' last point is
    unusually large — see that method's own docstring (defect 2b, display
    honesty across real gaps, including the idle spans the freeze now
    skips).
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
        # Single shared freeze predicate -- see class docstring. False until
        # the first moving TLM frame arrives, so a camera sample that
        # somehow precedes any telemetry is not recorded either.
        self._robot_moving: bool = False

    def set_trackwidth(self, trackwidth: float) -> None:  # [mm]
        self._trackwidth = trackwidth
        self._enc_dr.set_trackwidth(trackwidth)

    def _append_series(self, key: str, t: float, value: float) -> None:
        """Append ``(t, value)`` to ``series[key]``, breaking the line first
        if the gap since this series' own last point is a real stall.

        If more than ``GAP_BREAK_FACTOR * _NOMINAL_TLM_PERIOD`` has elapsed
        since the last point recorded in THIS series (each series is gated
        independently — e.g. ``cmd_l`` only appends on frames that carry a
        commanded velocity, so it naturally has a different cadence than
        ``vel_l``), a NaN sentinel is inserted first, timestamped at that
        PRIOR point's own time (nudged forward by a negligible epsilon to
        keep timestamps strictly increasing) — matching the stakeholder's
        own rule verbatim: "if the motor stopped, record the time at which
        it stopped as the time at which you break that line," not the time
        the next sample happens to arrive. matplotlib does not draw a line
        segment across a NaN y-value, so this alone is enough to break the
        plotted line at both this canvas's full-history view and the
        trailing-window ``StripChartCanvas`` view (both simply read
        ``series[key]`` — no redraw-side change needed).
        """
        pts = self.series[key]
        if pts:
            last_t = pts[-1][0]
            if t - last_t > GAP_BREAK_FACTOR * _NOMINAL_TLM_PERIOD:
                pts.append((last_t + 1e-6, math.nan))
        pts.append((t, value))

    def _t(self, now: float) -> float:
        if self._t0 is None:
            self._t0 = now
        return now - self._t0

    def _heading(self, key: str, now: float, deg: float) -> None:
        cont = self._unwrap[key].push(deg)
        if key not in self._h0:
            self._h0[key] = cont
        self._append_series(key, self._t(now), cont - self._h0[key])

    def _distance(self, key: str, now: float, x_cm: float, y_cm: float) -> None:
        if key not in self._p0:
            self._p0[key] = (x_cm, y_cm)
        x0, y0 = self._p0[key]
        self._append_series(key, self._t(now), math.hypot(x_cm - x0, y_cm - y0))

    def add_tlm(self, now: float, frame: Any) -> bool:
        """Record one telemetry frame. Returns True iff it was recorded.

        Frozen-while-stopped (2026-07-22 GUI stakeholder directive,
        stated twice, verbatim the second time: "You have to stop the
        charts. Do not run the charts when the robot is stopped."):
        while the wheels are genuinely idle -- no active move AND every
        wheel's measured velocity under ``_MOVING_SPEED`` -- this method
        appends NOTHING to any series and returns False. No new point
        means no axis advance and no redraw-worthy change (``TurnGraphPanel.
        add_tlm()`` only flags dirty on a True return): the chart is
        literally frozen, full stop, the instant real motion ends. The
        very next moving frame resumes appending immediately -- no
        debounce, no latched "stopped" state to clear, this check is
        purely a function of the CURRENT frame, stateless from one call to
        the next.
        Interaction with the gap-break fix (defect 2b, ``_append_series()``):
        an idle span skipped here is EXACTLY the kind of real, un-recorded
        gap that mechanism exists for. When motion resumes, the elapsed
        time since each series' last (pre-idle) point is large, so a NaN
        sentinel is woven in first -- the resumed line breaks instead of
        interpolating across the frozen span. That is what makes it safe
        to skip idle frames here at all: an EARLIER version of this method
        also skipped idle frames (with no gap-break to back it up) and
        that combination was itself a confirmed defect -- a straight
        interpolated line drawn across the un-recorded gap. Gap-break
        closes that hole, so the freeze-while-stopped behavior the
        stakeholder wants can be implemented the direct, literal way
        (skip appending) without reopening it.
        Never auto-clears on any transition (110-001's own fix, preserved
        unchanged): only the explicit ``clear()`` (the "Clear traces"
        button) discards data.

        Updates ``self._robot_moving`` -- the single shared freeze
        predicate ``add_camera()`` also consults (see class docstring) --
        on EVERY call, moving or not, so it never lags behind the most
        recent telemetry frame.
        """
        f = tlm_fields(frame)
        vel = f["vel"] or (0.0, 0.0)
        moving = (f["active"] is True) or (max(abs(vel[0]), abs(vel[1])) > _MOVING_SPEED)
        self._robot_moving = moving
        if not moving:
            return False
        t = self._t(now)
        if f["cmd"]:
            self._append_series("cmd_l", t, f["cmd"][0])
            self._append_series("cmd_r", t, f["cmd"][1])
        self._append_series("vel_l", t, vel[0])
        self._append_series("vel_r", t, vel[1])
        # Body twist (commanded vs actual). Commanded = forward kinematics of
        # the commanded wheel speeds: v_x = (vR+vL)/2 [mm/s], omega = (vR-vL)/b
        # [rad/s] (BodyKinematics::forward's own convention), shown in deg/s.
        # Actual = fused body velocity on the wire (twist = v_x [mm/s], omega
        # [mrad/s]).
        tw = self._trackwidth
        if f["cmd"] and tw > 0.0:
            self._append_series("cmd_vx", t, (f["cmd"][1] + f["cmd"][0]) / 2.0)
            self._append_series("cmd_omega", t, math.degrees((f["cmd"][1] - f["cmd"][0]) / tw))
        if f["twist"]:
            self._append_series("act_vx", t, f["twist"][0])
            self._append_series("act_omega", t, math.degrees(f["twist"][1] / 1000.0))
        if f["enc"]:
            self._append_series("enc_l", t, f["enc"][0])
            self._append_series("enc_r", t, f["enc"][1])
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
        """Record one camera ground-truth sample (world cm + heading deg).

        Gated on the SAME ``self._robot_moving`` predicate ``add_tlm()``
        maintains (2026-07-22 stakeholder follow-up -- see class
        docstring): camera ground-truth arrives on its own, independent
        polling cadence, so if this kept appending while the wheels were
        idle, ``head_cam``/``dist_cam`` would keep growing on their own --
        and because ``StripChartCanvas`` anchors its trailing window on
        ``recorder.latest_t()`` (the max across EVERY series, not just the
        currently-displayed one), that alone was enough to keep the
        telemetry-panel's rolling strip charts visibly scrolling even
        while every wheel series was correctly frozen. Also requires
        ``t0`` to already be anchored -- a camera sample before the first
        TLM frame has no elapsed-time reference yet.
        """
        if self._t0 is None or not self._robot_moving:
            return
        self._heading("head_cam", now, heading_deg)
        self._distance("dist_cam", now, x_cm, y_cm)

    def latest_t(self) -> "float | None":
        """Most recent elapsed-time ``t`` value recorded across every
        series -- the reference point a TRAILING-WINDOW view (the
        telemetry-pane strip charts, 110-002) anchors its cutoff to, since
        different series can have different lengths/last-updated points
        (e.g. ``enc_l`` may lag ``cmd_l`` by a frame or two). Returns
        ``None`` if nothing has been recorded yet."""
        latest: "float | None" = None
        for pts in self.series.values():
            if pts:
                t = pts[-1][0]
                if latest is None or t > latest:
                    latest = t
        return latest


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
    # Twist: commanded (blue/purple) vs actual (green/orange), v_x then omega.
    "cmd_vx": "#1f77b4", "act_vx": "#2ca02c",
    "cmd_omega": "#9467bd", "act_omega": "#ff7f0e",
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


class StripChartCanvas(_GraphCanvas):
    """A ``_GraphCanvas`` variant that plots only the trailing ``window``
    seconds of each series at redraw time (110-002, telemetry-pane strip
    charts).

    This is a pure WINDOWING FILTER over the SAME ``TurnTraceRecorder.
    series`` the full-history top graphs already read -- not a second
    recorder, not a second telemetry-consumption path (per the origin
    issue's own instruction: "reuse, don't duplicate"). Once more than
    ``window`` seconds have accumulated, the oldest points scroll off this
    canvas's own left edge; ``recorder.series`` itself (and the unwindowed
    top-graph view of it) is completely unaffected -- the same list object
    is read, never mutated, by both views.
    """

    def __init__(self, title: str, ylabel: str, series: list[tuple[str, str]],
                 window: float = 10.0) -> None:  # [s]
        super().__init__(title, ylabel, series)
        self._window = window

    def redraw(self, recorder: TurnTraceRecorder) -> None:
        ax = self._ax
        ax.clear()
        any_pts = False
        now = recorder.latest_t()
        cutoff = None if now is None else now - self._window
        for key, label in self._series:
            pts = recorder.series.get(key)
            if not pts:
                continue
            if cutoff is not None:
                pts = [p for p in pts if p[0] >= cutoff]
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
        if now is not None:
            # Always show a fixed `window`-second-wide axis (even before
            # `window` seconds have actually elapsed) so the strip chart's
            # scale doesn't visibly jump as data first accumulates.
            ax.set_xlim(max(0.0, now - self._window), max(now, self._window))
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
                 parent: QWidget | None = None,
                 on_clear_extra: "Callable[[], None] | None" = None) -> None:
        super().__init__(parent)
        self.recorder = recorder if recorder is not None else TurnTraceRecorder()
        # OOP sim-motor-state fix (unify the two "Clear Traces" buttons):
        # optional hook invoked at the end of clear() so this panel's own
        # header "Clear traces" button ALSO clears the playfield TraceModel
        # (owned by __main__.py, outside this widget) -- see clear()'s own
        # docstring and __main__.py's _clear_traces()/wiring for the other
        # direction (the ops-panel button clearing this panel).
        self._on_clear_extra = on_clear_extra

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
        self._twist = _GraphCanvas("Body twist — commanded vs actual", "mm/s · deg/s", TWIST)
        self._pos = _GraphCanvas("Wheel position (encoder)", "mm", WHEEL_POS)
        self._head = _GraphCanvas("Heading Δ — OTOS / fused / encoder / camera", "deg", HEADING)
        self._dist = _GraphCanvas("Distance (displacement) — all sources", "cm", DISTANCE)
        for w, name in ((self._speed, "Wheel speed"), (self._twist, "Twist"),
                        (self._pos, "Wheel position"),
                        (self._head, "Heading"), (self._dist, "Distance")):
            self._tabs.addTab(w, name)

        self._dirty = False
        self._tabs.currentChanged.connect(lambda _i: self._redraw_current())
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._maybe_redraw)
        self._timer.start(150)

    def set_on_clear_extra(self, cb: "Callable[[], None] | None") -> None:
        """Set (or clear, with ``cb=None``) the ``on_clear_extra`` hook
        after construction -- lets a caller wire this panel to another
        trace store defined LATER in its own setup sequence (e.g.
        ``__main__.py``'s ``_clear_playfield_traces()``, which is defined
        after this panel is constructed)."""
        self._on_clear_extra = cb

    # feed hooks (call from the GUI thread) --------------------------------
    def add_tlm(self, now: float, frame: Any) -> None:
        # recorder.add_tlm() returns False while frozen (robot genuinely
        # idle -- see its own docstring): skip the dirty flag too, so a
        # frozen chart doesn't even attempt a redraw while nothing in its
        # data changed -- "no axis advance, nothing moves" all the way
        # through, not just at the data layer.
        if self.recorder.add_tlm(now, frame):
            self._dirty = True

    def add_camera(self, now: float, x_cm: float, y_cm: float, heading_deg: float) -> None:
        self.recorder.add_camera(now, x_cm, y_cm, heading_deg)
        self._dirty = True

    def clear(self) -> None:
        """Clear this panel's own four recorded traces and redraw.

        OOP sim-motor-state fix: also invokes ``on_clear_extra`` (if given
        at construction) so this header's "Clear traces" button clears the
        playfield ``TraceModel`` too -- unifying it with the ops-panel
        "Clear Traces" button, which calls this method in the other
        direction (see ``__main__.py``'s ``_clear_traces()``)."""
        self.recorder.clear()
        self._redraw_current()
        if self._on_clear_extra is not None:
            self._on_clear_extra()

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
