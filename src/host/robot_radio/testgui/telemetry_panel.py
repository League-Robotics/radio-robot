"""robot_radio.testgui.telemetry_panel — parsed-telemetry breakout panel.

A compact read-out that sits between the playfield canvas and the console log.
It consumes ``TLMFrame`` objects (already parsed by the transport) and shows
one labelled row per component:

    time   — robot clock (ticks up as frames arrive)      [ms]
    seq    — D10 sequence counter (ticks up)
    enc    — encoder counts (left, right)                 [mm]
    vel    — per-wheel measured speed  + velocity arrow   [mm/s]
    pose   — fused EKF pose (x, y, heading)               [mm, mm, deg]
    encpose— encoder-only dead-reckoned pose              [mm, mm, deg]
    otos   — raw OTOS pose                                [mm, mm, deg]
    twist  — fused body-frame velocity + velocity arrow   [mm/s, deg/s]
    heading src — App::HeadingSource's currently-active sensor (109-005,
                  SUC-004; decoded 110-002). This is a standing stakeholder
                  requirement ("I want to know if you're using the
                  [encoders]... that's a big deal") — the row is styled
                  loudly (amber background + "(fallback)" text) whenever
                  the source is ENCODER, not just a plain text value like
                  every other row above.

Velocity vectors (``vel`` and ``twist``) are additionally drawn as an arrow
whose direction is the body-frame direction of motion (forward = up, left =
left) and whose length is proportional to speed.  Poses are *not* drawn as
arrows — they are already shown on the canvas as trace polylines.

Design split (mirrors ``traces.py`` vs ``canvas.py``):
  * The formatting/geometry helpers at module scope are **Qt-free** and unit
    tested directly.
  * :func:`build_telemetry_panel` performs the lazy PySide6 import and returns
    ``(widget, controller)`` where ``controller.update_frame(frame)`` refreshes
    the read-out on the Qt main thread.
"""
from __future__ import annotations

import math
import re
from typing import Any

# Leading transport markers on a formatted log line: a run of direction /
# relay-prefix characters (``>`` ``<`` ``#``) and whitespace before the wire
# payload.  Used to peek at the payload's response tag.
_MARKER_RE = re.compile(r"^[<>#\s]+")


def is_telemetry_log_line(text: str) -> bool:
    """True if a formatted console log line carries a ``TLM`` telemetry frame.

    Console lines look like ``[HH:MM:SS] < TLM t=... enc=...`` (with possible
    relay double-markers ``< <``).  We strip the timestamp and any leading
    direction/relay markers, then test for the ``TLM`` response tag so the
    telemetry stream can be kept out of the console (it is broken out into the
    telemetry panel instead).
    """
    body = text.split("] ", 1)[1] if "] " in text else text
    body = _MARKER_RE.sub("", body)
    return body[:3].upper() == "TLM"

# Full-scale reference speed for arrow length: an arrow at this magnitude
# fills the indicator.  Faster than any normal bench move so the arrow rarely
# saturates.
_ARROW_FULL_SCALE = 400.0  # [mm/s]


# ---------------------------------------------------------------------------
# Qt-free helpers — velocity geometry and value formatting
# ---------------------------------------------------------------------------

def twist_velocity(twist: "tuple[int, ...] | None") -> "tuple[float, float, float] | None":
    """Return body-frame ``(v_x, v_y, omega)`` from a TLM ``twist`` tuple.

    ``v_x``/``v_y`` are in mm/s (forward, left); ``omega`` is converted from the
    wire's milli-rad/s to deg/s for display.  Handles both builds:
      * differential ``(v, omega_mrad)``   → ``v_x = v``, ``v_y = 0``.
      * mecanum ``(vx, vy, omega_mrad)``   → ``v_x = vx``, ``v_y = vy``.
    Returns ``None`` for a missing/short tuple.
    """
    if twist is None:
        return None
    if len(twist) == 2:
        v, omega_mrad = twist  # [mm/s] [mrad/s]
        v_x, v_y = float(v), 0.0
    elif len(twist) >= 3:
        v_x, v_y, omega_mrad = float(twist[0]), float(twist[1]), twist[2]  # [mm/s] [mm/s] [mrad/s]
    else:
        return None
    omega = math.degrees(omega_mrad / 1000.0)  # [deg/s]
    return v_x, v_y, omega


def wheel_velocity(vel: "tuple[int, ...] | None") -> "tuple[float, float] | None":
    """Return an approximate body ``(v_x, v_y)`` from a per-wheel ``vel`` tuple.

    Only the forward component is recoverable from wheel speeds alone without
    the drivetrain geometry, so ``v_y`` is 0 and ``v_x`` is the mean wheel
    speed — the translational speed the wheels are commanding.  Handles the
    2-tuple (differential) and 4-tuple (mecanum) forms.  Returns ``None`` for a
    missing/empty tuple.
    """
    if not vel:
        return None
    v_x = sum(float(w) for w in vel) / len(vel)  # [mm/s] mean wheel speed
    return v_x, 0.0


def body_to_screen(v_x: float, v_y: float) -> "tuple[float, float]":
    """Map body-frame velocity ``(v_x, v_y)`` to a screen delta ``(dx, dy)``.

    Body convention (REP-103): +x forward, +y left.  Screen convention: +x
    right, +y down.  We render forward as up and left as left, so:
      ``dx = -v_y`` (left → negative screen x), ``dy = -v_x`` (forward → up).
    The returned vector keeps the input magnitude; the widget scales it.
    """
    return -v_y, -v_x


def arrow_fraction(speed: float, full_scale: float = _ARROW_FULL_SCALE) -> float:
    """Clamp ``speed / full_scale`` to ``[0, 1]`` for arrow-length scaling."""
    if full_scale <= 0:
        return 0.0
    return max(0.0, min(1.0, speed / full_scale))


def _fmt(value: "int | float", digits: int = 0) -> str:
    """Format a number with fixed decimals, or ``—`` for ``None``."""
    if value is None:
        return "—"
    if digits == 0:
        return f"{value:+d}" if isinstance(value, int) else f"{value:+.0f}"
    return f"{value:+.{digits}f}"


def fmt_time(t: "int | None") -> str:
    """Robot clock ``t`` (ms) as seconds, e.g. ``12.345 s``; ``—`` if absent."""
    if t is None:
        return "—"
    return f"{t / 1000.0:.3f} s"


def fmt_seq(seq: "int | None") -> str:
    """Sequence counter as a plain integer string; ``—`` if absent."""
    return "—" if seq is None else str(seq)


def fmt_enc(enc: "tuple[int, int] | None") -> str:
    """Encoder counts ``(L, R)`` in mm; ``—`` if absent."""
    if enc is None:
        return "—"
    left, right = enc
    return f"L {left:+d}   R {right:+d}   mm"


def fmt_pose(pose: "tuple[int, int, int] | None") -> str:
    """Pose ``(x, y, heading)`` — mm, mm, cdeg → shown mm, mm, deg."""
    if pose is None:
        return "—"
    x, y, h_cdeg = pose
    return f"x {x:+d}   y {y:+d}   θ {h_cdeg / 100.0:+.1f}°"


def fmt_vel(vel: "tuple[int, ...] | None") -> str:
    """Per-wheel measured speed; 2-tuple (diff) or 4-tuple (mecanum)."""
    if not vel:
        return "—"
    if len(vel) == 2:
        return f"L {vel[0]:+d}   R {vel[1]:+d}   mm/s"
    return "  ".join(f"{w:+d}" for w in vel) + "   mm/s"


# HeadingSourceStatus raw values (telemetry.proto) -- mirrored here rather
# than importing telemetry_pb2 into this Qt-free formatting module, matching
# TLMFrame.from_pb2()'s own comment that these are the raw enum ints.
# HEADING_SOURCE_STATUS_OTOS = 0, HEADING_SOURCE_STATUS_ENCODER = 1.
_HEADING_SOURCE_ENCODER = 1


def is_heading_source_fallback(heading_source: "int | None") -> bool:
    """True when ``heading_source`` reports the ENCODER fallback (OTOS is
    NOT currently trusted for heading) -- the stakeholder-mandated
    visibility signal (SUC-004). ``None`` (undecoded / pre-109-005
    firmware) is NOT a fallback state -- there is nothing to flag."""
    return heading_source == _HEADING_SOURCE_ENCODER


def fmt_heading_source(heading_source: "int | None") -> str:
    """Render ``heading_source`` as a short, unambiguous label.

    ``None`` — undecoded (older firmware/frame path) — renders ``—``, the
    same placeholder every other absent field in this panel uses.
    ``HEADING_SOURCE_STATUS_OTOS`` (0) renders ``OTOS``.
    ``HEADING_SOURCE_STATUS_ENCODER`` (1) renders ``ENCODER (fallback)`` —
    deliberately verbose, not just ``ENCODER``, so the fallback state reads
    as an alert on its own, independent of the row's background styling.
    """
    if heading_source is None:
        return "—"
    if is_heading_source_fallback(heading_source):
        return "ENCODER (fallback)"
    return "OTOS"


def fmt_twist(twist: "tuple[int, ...] | None") -> str:
    """Body-frame twist; differential ``(v, ω)`` or mecanum ``(vx, vy, ω)``."""
    parsed = twist_velocity(twist)
    if parsed is None:
        return "—"
    v_x, v_y, omega = parsed
    if twist is not None and len(twist) == 2:
        return f"v {v_x:+.0f} mm/s   ω {omega:+.1f}°/s"
    return f"vx {v_x:+.0f}   vy {v_y:+.0f} mm/s   ω {omega:+.1f}°/s"


# ---------------------------------------------------------------------------
# Qt widget builder (lazy PySide6 import)
# ---------------------------------------------------------------------------

def build_telemetry_panel(recorder: "Any" = None) -> "tuple[Any, Any]":
    """Build the telemetry breakout panel.

    Returns ``(widget, controller)``.  ``controller.update_frame(frame)`` must
    be called on the Qt main thread with each ``TLMFrame``; it refreshes every
    value label and repaints the two velocity arrows.

    ``recorder`` (110-002) is the SAME ``turn_graphs.TurnTraceRecorder`` the
    top graph tabs (``TurnGraphPanel``) already own -- passed in by the
    caller (``__main__.py`` hands it ``graph_panel.recorder``) so the
    rolling 10-second strip charts occupying this panel's previously-unused
    right-hand space read the identical telemetry history the top graphs
    do. No second recorder is created and no telemetry frame is processed
    twice: this panel never calls ``add_tlm``/``add_camera`` on the
    recorder itself, it only reads ``recorder.series`` at redraw time. If
    omitted (e.g. a standalone test of this panel alone), a private, empty
    ``TurnTraceRecorder`` is created so the strip-chart tabs still render
    (with no data) rather than failing to build.
    """
    from PySide6.QtCore import Qt, QTimer  # type: ignore[import-untyped]
    from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPolygonF  # type: ignore[import-untyped]
    from PySide6.QtCore import QPointF
    from PySide6.QtWidgets import (  # type: ignore[import-untyped]
        QGridLayout,
        QLabel,
        QSizePolicy,
        QTabWidget,
        QWidget,
    )

    from .turn_graphs import (  # lazy, same reason as the PySide6 imports above
        DISTANCE,
        HEADING,
        WHEEL_POS,
        WHEEL_SPEED,
        StripChartCanvas,
        TurnTraceRecorder,
    )

    if recorder is None:
        recorder = TurnTraceRecorder()

    class _ArrowIndicator(QWidget):
        """Square widget that paints a velocity arrow from its centre.

        ``set_vector(v_x, v_y)`` takes a body-frame velocity (mm/s); the arrow
        points in the direction of motion (forward = up, left = left) with a
        length proportional to speed against :data:`_ARROW_FULL_SCALE`.
        """

        _SIDE = 54  # [px] widget side length

        def __init__(self) -> None:
            super().__init__()
            self._dx = 0.0
            self._dy = 0.0
            self._frac = 0.0
            self.setFixedSize(self._SIDE, self._SIDE)
            self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)

        def set_vector(self, v_x: float, v_y: float) -> None:
            speed = math.hypot(v_x, v_y)
            dx, dy = body_to_screen(v_x, v_y)
            norm = math.hypot(dx, dy)
            if norm > 1e-6:
                self._dx, self._dy = dx / norm, dy / norm
            else:
                self._dx = self._dy = 0.0
            self._frac = arrow_fraction(speed)
            self.update()

        def paintEvent(self, event: "object") -> None:  # type: ignore[override]
            painter = QPainter(self)
            painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
            w = self.width()
            h = self.height()
            cx, cy = w / 2.0, h / 2.0
            radius = min(w, h) / 2.0 - 4.0

            # Reference crosshair / bounding circle (faint).
            painter.setPen(QPen(QColor(120, 120, 120, 90), 1.0))
            painter.drawEllipse(QPointF(cx, cy), radius, radius)

            if self._frac <= 0.0:
                # No motion — draw a small dot at centre.
                painter.setBrush(QColor(150, 150, 150))
                painter.setPen(Qt.PenStyle.NoPen)
                painter.drawEllipse(QPointF(cx, cy), 2.5, 2.5)
                painter.end()
                return

            length = radius * self._frac
            tip_x = cx + self._dx * length
            tip_y = cy + self._dy * length

            pen = QPen(QColor(80, 200, 120), 2.5)  # green motion arrow
            pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            painter.setPen(pen)
            painter.drawLine(QPointF(cx, cy), QPointF(tip_x, tip_y))

            # Arrowhead.
            angle = math.atan2(self._dy, self._dx)
            head = 7.0
            spread = math.radians(26)
            left = QPointF(
                tip_x - head * math.cos(angle - spread),
                tip_y - head * math.sin(angle - spread),
            )
            right = QPointF(
                tip_x - head * math.cos(angle + spread),
                tip_y - head * math.sin(angle + spread),
            )
            painter.setBrush(QColor(80, 200, 120))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawPolygon(QPolygonF([QPointF(tip_x, tip_y), left, right]))
            painter.end()

    # ---- panel layout -----------------------------------------------------
    panel = QWidget()
    panel.setObjectName("telemetry_panel")
    grid = QGridLayout(panel)
    grid.setContentsMargins(8, 6, 8, 6)
    grid.setHorizontalSpacing(10)
    grid.setVerticalSpacing(3)

    mono = QFont("Menlo")
    mono.setStyleHint(QFont.StyleHint.Monospace)
    mono.setPointSize(11)

    title = QLabel("TELEMETRY")
    title.setObjectName("telemetry_title")
    tfont = QFont()
    tfont.setBold(True)
    title.setFont(tfont)
    grid.addWidget(title, 0, 0, 1, 3)

    # (label text, value objectName, wants-arrow, arrow objectName)
    rows = [
        ("time", "tlm_val_time", False, None),
        ("seq", "tlm_val_seq", False, None),
        ("enc", "tlm_val_enc", False, None),
        ("vel", "tlm_val_vel", True, "tlm_arrow_vel"),
        ("pose", "tlm_val_pose", False, None),
        ("encpose", "tlm_val_encpose", False, None),
        ("otos", "tlm_val_otos", False, None),
        ("twist", "tlm_val_twist", True, "tlm_arrow_twist"),
        ("heading src", "tlm_val_heading_source", False, None),
    ]

    value_labels: dict[str, Any] = {}
    arrows: dict[str, Any] = {}
    for i, (name, val_obj, wants_arrow, arrow_obj) in enumerate(rows, start=1):
        name_label = QLabel(name)
        name_label.setStyleSheet("color: #888;")
        grid.addWidget(name_label, i, 0, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

        val_label = QLabel("—")
        val_label.setObjectName(val_obj)
        val_label.setFont(mono)
        val_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        grid.addWidget(val_label, i, 1, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        value_labels[val_obj] = val_label

        if wants_arrow:
            arrow = _ArrowIndicator()
            arrow.setObjectName(arrow_obj)
            grid.addWidget(arrow, i, 2, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
            arrows[arrow_obj] = arrow

    # Rolling 10-second strip charts (110-002) occupy the right-hand column
    # that used to be pure horizontal-slack padding -- a tabbed widget
    # (playfield-mode tab styling: a plain QTabWidget, same as
    # TurnGraphPanel's own tab bar) spanning every row of this grid.
    strip_tabs = QTabWidget()
    strip_tabs.setObjectName("telemetry_strip_charts")
    _strip_speed = StripChartCanvas("Wheel speed", "mm/s", WHEEL_SPEED)
    _strip_pos = StripChartCanvas("Wheel position", "mm", WHEEL_POS)
    _strip_head = StripChartCanvas("Heading Δ", "deg", HEADING)
    _strip_dist = StripChartCanvas("Distance", "cm", DISTANCE)
    for w, obj_name, name in ((_strip_speed, "strip_chart_wheel_speed", "Wheel speed"),
                              (_strip_pos, "strip_chart_wheel_position", "Wheel position"),
                              (_strip_head, "strip_chart_heading", "Heading"),
                              (_strip_dist, "strip_chart_distance", "Distance")):
        w.setObjectName(obj_name)
        strip_tabs.addTab(w, name)

    def _redraw_current_strip_chart() -> None:
        w = strip_tabs.currentWidget()
        if isinstance(w, StripChartCanvas):
            w.redraw(recorder)

    strip_tabs.currentChanged.connect(lambda _i: _redraw_current_strip_chart())
    strip_timer = QTimer(panel)
    strip_timer.timeout.connect(_redraw_current_strip_chart)
    strip_timer.start(200)  # [ms] throttled redraw of the visible tab only

    grid.addWidget(strip_tabs, 0, 3, len(rows) + 1, 1)

    # Column 0-2 hold the label/value/arrow columns above; column 3 now
    # holds the strip-chart tabs and gets ALL the stretch (it previously
    # absorbed horizontal slack with nothing in it).
    grid.setColumnStretch(0, 0)
    grid.setColumnStretch(1, 0)
    grid.setColumnStretch(2, 0)
    grid.setColumnStretch(3, 1)

    class _TelemetryPanelController:
        """Refreshes the panel's labels and arrows from a ``TLMFrame``."""

        def __init__(self) -> None:
            self._values = value_labels
            self._arrows = arrows

        def update_frame(self, frame: "Any") -> None:
            """Update every read-out from *frame* (Qt main thread only)."""
            self._values["tlm_val_time"].setText(fmt_time(getattr(frame, "t", None)))
            self._values["tlm_val_seq"].setText(fmt_seq(getattr(frame, "seq", None)))
            self._values["tlm_val_enc"].setText(fmt_enc(getattr(frame, "enc", None)))
            self._values["tlm_val_vel"].setText(fmt_vel(getattr(frame, "vel", None)))
            self._values["tlm_val_pose"].setText(fmt_pose(getattr(frame, "pose", None)))
            self._values["tlm_val_encpose"].setText(fmt_pose(getattr(frame, "encpose", None)))
            self._values["tlm_val_otos"].setText(fmt_pose(getattr(frame, "otos", None)))
            self._values["tlm_val_twist"].setText(fmt_twist(getattr(frame, "twist", None)))

            heading_source = getattr(frame, "heading_source", None)
            heading_lbl = self._values["tlm_val_heading_source"]
            heading_lbl.setText(fmt_heading_source(heading_source))
            if is_heading_source_fallback(heading_source):
                # Loud, impossible-to-miss styling for the stakeholder's
                # "big deal" non-gyro state (SUC-004) -- amber background,
                # bold text, not just a plain label value like every other
                # row in this panel.
                heading_lbl.setStyleSheet(
                    "background-color: #ffb300; color: #1a1a1a; "
                    "font-weight: bold; padding: 1px 4px; border-radius: 3px;")
            else:
                heading_lbl.setStyleSheet("")

            wheel = wheel_velocity(getattr(frame, "vel", None))
            self._arrows["tlm_arrow_vel"].set_vector(*(wheel or (0.0, 0.0)))

            tw = twist_velocity(getattr(frame, "twist", None))
            self._arrows["tlm_arrow_twist"].set_vector(*(tw[:2] if tw else (0.0, 0.0)))

    return panel, _TelemetryPanelController()
