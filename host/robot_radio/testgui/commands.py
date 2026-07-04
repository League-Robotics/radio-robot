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
            {"name": "eps",     "type": int, "min": 0,     "max": 180,  "default": 0,  "unit": "deg", "optional": True},
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
            {"name": "deg", "type": int, "min": -3600, "max": 3600, "default": 90, "unit": "deg"},
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
# A "tour" is an ordered list of firmware wire strings the GUI sends one at a
# time, waiting for each bounded move to physically complete (SNAP ``mode``
# returns to ``I`` = idle) before dispatching the next.  The tour is prefixed
# by a "Set Robot @ 0,0" origin reset performed by the GUI itself, so the list
# below contains only the motion steps.
#

TOUR_1: list[str] = [
    "D 200 200 345",
    "RT 9000",
    "D 200 200 240",
    "RT 9000",
    "D 200 200 700",
    "RT 9000",
    "D 200 200 480",
    "RT 9000",
    "D 200 200 700",
    "RT 9000",
    "D 200 200 240",
    "RT 9000",
    "D 200 200 345"
]


TOUR_2: list[str] = [
    "D 200 200 345",
    "RT 9000",
    "D 200 200 240",
    "RT 12400",
    "D 200 200 850",
    "RT -21700",
    "D 200 200 700",
    "RT 14600",
    "D 200 200 850",
    "RT 21500",
    "D 200 200 700",
    "RT -9000",
    "D 200 200 240",
    "RT -9000",
    "D 200 200 345",
]



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
