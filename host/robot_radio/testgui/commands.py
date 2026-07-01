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
centidegrees (``deg * 100``) on the wire.  The eps field is *optional*: when
its value equals the field default (0) it is omitted from the wire string,
producing a bare ``TURN <heading_cdeg>``.

RT is a RELATIVE in-place turn (positive = CCW/left) computed on the robot
from the encoder arc.  Its ``deg`` field is entered in degrees but sent in
centidegrees, producing ``RT <rel_cdeg>``.
"""

from __future__ import annotations

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
    """
    label: str
    params: list[ParamSpec]
    cdeg_fields: list[str]
    optional_zero_fields: list[str]


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
            {"name": "heading", "type": int, "min": -18000, "max": 18000, "default": 9000, "unit": "cdeg"},
            {"name": "eps",     "type": int, "min": 0,      "max": 18000, "default": 0,    "unit": "cdeg", "optional": True},
        ],
        # No cdeg_fields — TURN takes centidegrees directly (heading and eps are already in cdeg)
        "optional_zero_fields": ["eps"],
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
    * Fields listed in ``spec["cdeg_fields"]`` are multiplied by 100 before
      formatting (degree → centidegree conversion).
    * Fields listed in ``spec["optional_zero_fields"]`` are omitted from the
      output when their value is 0.
    * The label is always the first token.
    """
    label = spec["label"]
    cdeg_fields: set[str] = set(spec.get("cdeg_fields", []) or [])
    optional_zero_fields: set[str] = set(spec.get("optional_zero_fields", []) or [])

    tokens: list[str] = [label]

    for param in spec["params"]:
        name = param["name"]
        raw = values.get(name, param.get("default", 0))

        # Apply centidegree conversion if this field uses degrees on the UI.
        if name in cdeg_fields:
            wire_val = int(round(float(raw) * 100))
        else:
            wire_val = int(round(float(raw)))

        # Skip optional fields that are at their "omit" value (0).
        if name in optional_zero_fields and wire_val == 0:
            continue

        # TURN eps is formatted as eps=<val>; all other params are positional.
        if label == "TURN" and name == "eps":
            tokens.append(f"eps={wire_val}")
        else:
            tokens.append(str(wire_val))

    return " ".join(tokens)
