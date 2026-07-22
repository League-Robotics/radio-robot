"""robot_radio.testgui.commands — Schema-driven motion command definitions.

This module defines the data-driven command schema (``COMMANDS``) and the
pure wire-string builder (``build_wire_string``) used by the Robot Test GUI's
command-entry rows.

No PySide6 imports here — this module is usable in headless tests without a
display server or a Qt application instance.

Command schema
--------------
``COMMANDS`` is a list of ``CommandSpec`` dicts.  Each entry describes one
firmware motion command.  The ``build_command_rows`` helper in ``__main__.py``
reads the list and constructs one ``QHBoxLayout`` per entry.

Wire formats
------------
=================== ===========================================
Command             Wire string
=================== ===========================================
S  left right       ``S <left> <right>``
T  left right ms    ``T <left> <right> <ms>``
D  left right mm    ``D <left> <right> <mm>``
R  speed radius     ``R <speed> <radius>``
TURN hdg [eps]      ``TURN <hdg_cdeg>`` or ``TURN <h> eps=<e>``
RT deg              ``RT <rel_cdeg>``
G  x y speed        ``G <x> <y> <speed>``
=================== ===========================================

TURN heading and eps are supplied in degrees (human-friendly) but sent in
centidegrees (``deg * 100``) on the wire.  The heading field accepts any
angle (e.g. 270, -450); values outside [-180, 180] are wrapped onto the
equivalent absolute heading in (-180, 180] before conversion, so
``TURN 270°`` is sent as ``TURN -9000``.  The eps field is *optional*: when
its value equals the field default (0) it is omitted from the wire string,
producing a bare ``TURN <heading_cdeg>``.

RT is a RELATIVE in-place turn (positive = CCW/left) computed on the robot
from the encoder arc.  Its ``deg`` field is entered in degrees but sent in
centidegrees, producing ``RT <rel_cdeg>``.
"""

from __future__ import annotations

import re
from typing import Any, TypedDict

# 115-009 (gut S1's own test-sweep/green-bar ticket): `robot_radio.planner`
# is a dormant, NOT-deleted package (sprint 115 Design Rationale Decision
# 6 -- host planner/tour/path/nav code stays in the tree, expected to go
# dormant/broken this sprint, a separate future follow-up, not a defect).
# `planner.tour` itself raises AttributeError at import time now (it
# references `telemetry_pb2.ACK_STATUS_DONE`, part of the depth-3 ack ring
# 115-003's frame-v2 rewrite deleted). This module is NOT itself a tour/turn
# module -- COMMANDS/build_wire_string/goto_distance/goto_reached below are
# plain, unrelated command-schema/geometry helpers -- so a broken
# `planner.tour` import must not take the whole module down; only `TOURS`
# (below) degrades to empty.
try:
    from robot_radio.planner.tour import TOUR_1, TOUR_2
except (ImportError, AttributeError):
    TOUR_1 = TOUR_2 = []


# ---------------------------------------------------------------------------
# Parameter spec
# ---------------------------------------------------------------------------

class ParamSpec(TypedDict, total=False):
    """Description of a single parameter field in a command row.

    Required keys
    -------------
    name : str
        Short identifier used as the field label (e.g. ``"left"``).
    type : type
        Python type: ``int`` or ``float``.
    min : int | float
        Minimum value accepted by the firmware.
    max : int | float
        Maximum value accepted by the firmware.
    default : int | float
        Pre-filled field value.

    Optional keys
    -------------
    optional : bool
        When ``True``, the parameter may be omitted from the wire string
        (default ``False``).  Only the last parameter(s) may be optional.
    unit : str
        Human-visible unit string appended to the label (e.g. ``"mm/s"``).
    """
    name: str
    type: type
    min: int | float
    max: int | float
    default: int | float
    optional: bool
    unit: str


class CommandSpec(TypedDict, total=False):
    """Full specification for one command row.

    Required keys
    -------------
    label : str
        Firmware command verb (also used as the row button label).
    params : list[ParamSpec]
        Ordered list of parameter field specs.

    Optional keys
    -------------
    cdeg_fields : list[str]
        Names of ``int`` parameters whose UI unit is degrees but whose wire
        unit is centidegrees (multiply by 100 before formatting).
    optional_zero_fields : list[str]
        Names of parameters that, when zero, are omitted from the wire string.
        The ``eps`` field of TURN uses this so ``eps=0`` → omit.
    wrap_deg_fields : list[str]
        Names of degree-valued parameters normalized onto (-180, 180] before
        the centidegree conversion.  Values already within [-180, 180] pass
        through unchanged.  TURN ``heading`` uses this so any entered angle
        maps to the equivalent absolute heading.
    """
    label: str
    params: list[ParamSpec]
    cdeg_fields: list[str]
    optional_zero_fields: list[str]
    wrap_deg_fields: list[str]


# ---------------------------------------------------------------------------
# Command schema table
# ---------------------------------------------------------------------------
#
# Wire-shape range audit (sprint 085 ticket 001).  Every ``min``/``max`` below
# was checked by hand against ``docs/protocol-v2.md`` §10 (Motion Commands),
# implemented in sprint 084 — the citation is repeated per row so a future
# editor can re-verify a single row without re-reading the whole section.
# ``min``/``max`` here are in the UI's units (degrees for ``cdeg_fields``
# members; the firmware's own cdeg ceiling is noted alongside). Rows NOT
# listed as degree fields are already in the firmware's native unit (mm,
# mm/s, ms) and the UI range equals the wire range exactly.
#
#   S    -- "### S — Streaming (Watchdog) Drive": left/right -1000..1000 mm/s
#   T    -- "### T — Timed Drive": left/right -1000..1000 mm/s;
#           ms 1..30000 ms
#   D    -- "### D — Distance Drive": left/right -1000..1000 mm/s;
#           mm 1..10000 mm
#   R    -- "### R — Arc Drive (constant curvature, open-loop)":
#           speed -1000..1000 mm/s; radius -10000..10000 mm
#   TURN -- "### TURN — Absolute-Heading Turn-in-Place (closed-loop, fused
#           heading)": heading -18000..+18000 cdeg (±180°) — the UI's
#           heading field instead wraps any entered angle onto (-180, 180]
#           before conversion (see ``wrap_deg_fields`` below), so it need not
#           be range-limited to match; eps 10..1800 cdeg (0.1°..18°), i.e.
#           0.1..18 deg, default 300 cdeg (3°). The UI's eps min stays 0
#           (below the firmware's own 10 cdeg floor) as the sentinel for
#           "omit eps entirely, let the firmware apply its own 300 cdeg
#           default" — see ``optional_zero_fields`` below — but the UI max
#           must be 18 (deg) = 1800 cdeg, the firmware ceiling.
#   RT   -- "### RT — Relative Turn-in-Place (closed-loop, encoder arc)":
#           relAngle -180000..+180000 cdeg (±1800°) = deg -1800..1800.
#   G    -- "### G — Go-To (relative XY)": x/y -10000..10000 mm;
#           speed 1..1000 mm/s

COMMANDS: list[CommandSpec] = [
    {
        "label": "S",
        "params": [
            {"name": "left",  "type": int, "min": -1000, "max": 1000, "default": 200, "unit": "mm/s"},
            {"name": "right", "type": int, "min": -1000, "max": 1000, "default": 200, "unit": "mm/s"},
        ],
    },
    {
        "label": "T",
        "params": [
            {"name": "left",  "type": int, "min": -1000, "max": 1000, "default": 200, "unit": "mm/s"},
            {"name": "right", "type": int, "min": -1000, "max": 1000, "default": 200, "unit": "mm/s"},
            {"name": "ms",    "type": int, "min": 1,     "max": 30000, "default": 1000, "unit": "ms"},
        ],
    },
    {
        "label": "D",
        "params": [
            {"name": "left",  "type": int, "min": -1000, "max": 1000, "default": 200, "unit": "mm/s"},
            {"name": "right", "type": int, "min": -1000, "max": 1000, "default": 200, "unit": "mm/s"},
            {"name": "mm",    "type": int, "min": 1,     "max": 10000, "default": 500, "unit": "mm"},
        ],
    },
    {
        "label": "R",
        "params": [
            {"name": "speed",  "type": int, "min": -1000,  "max": 1000,  "default": 200, "unit": "mm/s"},
            {"name": "radius", "type": int, "min": -10000, "max": 10000, "default": 500, "unit": "mm"},
        ],
    },
    {
        "label": "TURN",
        "params": [
            {"name": "heading", "type": int, "min": -3600, "max": 3600, "default": 90, "unit": "deg"},
            {"name": "eps",     "type": int, "min": 0,     "max": 18,   "default": 0,  "unit": "deg", "optional": True},
        ],
        # heading/eps are entered in degrees (human-friendly) but sent in centidegrees.
        # heading accepts any angle; it is wrapped onto (-180, 180] on the wire.
        "cdeg_fields": ["heading", "eps"],
        "optional_zero_fields": ["eps"],
        "wrap_deg_fields": ["heading"],
    },
    {
        "label": "RT",
        "params": [
            {"name": "deg", "type": int, "min": -1800, "max": 1800, "default": 90, "unit": "deg"},
        ],
        # deg is entered in degrees (human-friendly) but sent in centidegrees.
        "cdeg_fields": ["deg"],
    },
    {
        "label": "G",
        "params": [
            {"name": "x",     "type": int, "min": -10000, "max": 10000, "default": 0,   "unit": "mm"},
            {"name": "y",     "type": int, "min": -10000, "max": 10000, "default": 0,   "unit": "mm"},
            {"name": "speed", "type": int, "min": 1,      "max": 1000,  "default": 200, "unit": "mm/s"},
        ],
    },
]


# ---------------------------------------------------------------------------
# Pure wire-string builder (no Qt dependencies)
# ---------------------------------------------------------------------------

def build_wire_string(spec: CommandSpec, values: dict[str, Any]) -> str:
    """Build the firmware wire string from a command spec and field values.

    Parameters
    ----------
    spec:
        A ``CommandSpec`` entry from ``COMMANDS``.
    values:
        Dict mapping parameter ``name`` to the current numeric value from
        the corresponding UI field (or from a test).  Values are expected
        to be ``int`` or ``float`` matching the field's ``type``.

    Returns
    -------
    str
        The ready-to-send wire string, e.g. ``"S 200 -150"`` or
        ``"TURN 9000 eps=300"``.

    Notes
    -----
    * Fields listed in ``spec["wrap_deg_fields"]`` that fall outside
      [-180, 180] are wrapped onto the equivalent angle in (-180, 180]
      (e.g. 270 → -90); in-range values pass through unchanged.
    * Fields listed in ``spec["cdeg_fields"]`` are multiplied by 100 before
      formatting (degree → centidegree conversion).
    * Fields listed in ``spec["optional_zero_fields"]`` are omitted from the
      output when their value is 0.
    * The label is always the first token.
    """
    label = spec["label"]
    cdeg_fields: set[str] = set(spec.get("cdeg_fields", []) or [])
    optional_zero_fields: set[str] = set(spec.get("optional_zero_fields", []) or [])
    wrap_deg_fields: set[str] = set(spec.get("wrap_deg_fields", []) or [])

    tokens: list[str] = [label]

    for param in spec["params"]:
        name = param["name"]
        raw = values.get(name, param.get("default", 0))
        deg = float(raw)

        # Wrap out-of-range angles onto (-180, 180]; leave in-range as typed.
        if name in wrap_deg_fields and not -180.0 <= deg <= 180.0:
            deg = ((deg + 180.0) % 360.0) - 180.0
            if deg == -180.0:
                deg = 180.0

        # Apply centidegree conversion if this field uses degrees on the UI.
        if name in cdeg_fields:
            wire_val = int(round(deg * 100))
        else:
            wire_val = int(round(deg))

        # Skip optional fields that are at their "omit" value (0).
        if name in optional_zero_fields and wire_val == 0:
            continue

        # TURN eps is formatted as eps=<val>; all other params are positional.
        if label == "TURN" and name == "eps":
            tokens.append(f"eps={wire_val}")
        else:
            tokens.append(str(wire_val))

    return " ".join(tokens)


# ---------------------------------------------------------------------------
# Pre-programmed tours
# ---------------------------------------------------------------------------
#
# 107-002: TOUR_1/TOUR_2's own raw wire-string geometry MOVED to
# planner/tour.py (architecture-update.md Decision 3, corrected during that
# document's own self-review to keep the dependency direction
# [Presentation] -> [Domain], not the reverse) -- this module now only reads
# the geometry back for GUI labeling (TOURS below). planner.tour.run_tour()
# (ticket 002) is what actually drives a tour now -- not a per-step wire
# string sent through this module's own build_wire_string()/binary_bridge
# translation, which targeted the now-deleted segment/replace envelope arms
# (see planner/tour.py's own module docstring for the full history).
# TOUR_1/TOUR_2 themselves are imported at module top (from planner.tour).

#: Named tours available to the GUI (label → ordered wire strings).
TOURS: dict[str, list[str]] = {
    "Tour 1": TOUR_1,
    "Tour 2": TOUR_2
}


# ---------------------------------------------------------------------------
# Camera-based GOTO (synthetic host-side command) — pure geometry helpers
# ---------------------------------------------------------------------------
#
# GOTO is not a firmware verb.  The GUI drives the robot to a world point by
# repeatedly (a) reading the camera ground-truth pose, (b) snapping the robot's
# internal pose to it (``SI``), and (c) re-issuing a firmware ``G`` go-to toward
# the fixed target — a camera-in-the-loop pure-pursuit corrected for odometry
# drift.  The loop stops when the robot is within ``eps`` of the target.


def goto_distance(
    target_x: float,  # [mm]
    target_y: float,  # [mm]
    cur_x: float,  # [mm]
    cur_y: float,  # [mm]
) -> float:
    """Return the Euclidean distance (mm) from the current point to the target."""
    dx = target_x - cur_x
    dy = target_y - cur_y
    return (dx * dx + dy * dy) ** 0.5


def goto_reached(
    target_x: float,  # [mm]
    target_y: float,  # [mm]
    cur_x: float,  # [mm]
    cur_y: float,  # [mm]
    eps: float,  # [mm]
) -> bool:
    """Return ``True`` when the current point is within ``eps`` of the target."""
    return goto_distance(target_x, target_y, cur_x, cur_y) <= eps


# ---------------------------------------------------------------------------
# Telemetry / SNAP mode parsing (Qt-free, for completion detection)
# ---------------------------------------------------------------------------

_MODE_RE = re.compile(r"\bmode=([A-Za-z])")


def parse_tlm_mode(reply: str) -> str | None:
    """Extract the single-character ``mode`` field from a TLM/SNAP reply.

    A ``SNAP`` command returns a telemetry frame such as
    ``"TLM t=1234 mode=I seq=5 ..."``.  The ``mode`` character reports the
    robot's motion state: ``I`` = idle, and ``S`` / ``T`` / ``D`` / ``G``
    (and other non-``I`` values) mean a motion command is still executing.

    Parameters
    ----------
    reply:
        The raw reply string from ``transport.command("SNAP")`` — may span
        multiple lines; the first ``mode=`` token found is used.

    Returns
    -------
    str | None
        The uppercase mode character, or ``None`` if no ``mode=`` field is
        present (e.g. an empty reply on timeout).
    """
    if not reply:
        return None
    m = _MODE_RE.search(reply)
    if m is None:
        return None
    return m.group(1).upper()
