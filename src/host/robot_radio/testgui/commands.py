"""robot_radio.testgui.commands — command-row schema (empty, 128-004) and
the wire-string builder that used to be schema-driven from it.

``COMMANDS`` used to be a list of ``CommandSpec`` dicts describing seven
firmware motion verbs (``S``/``T``/``D``/``R``/``TURN``/``RT``/``G``), each
rendered as a parameter-field row in a hidden GUI panel
(``__main__.py``'s ``_build_command_row()``, deleted 128-004). It is now
EMPTY: five of the seven verbs (``S``/``T``/``R``/``TURN``/``G``) never had
a binary-plane arm on the current wire at all (see ``testgui/
binary_bridge.py``'s own module docstring for the full accounting), and the
remaining two (``D``/``RT``) were already superseded by the Managed/
Unmanaged preset-button panel (stakeholder 2026-07-17: "I don't need full
parameters, I just need buttons") before this ticket -- the schema-driven
panel itself was already hidden (``cmd_rows_widget.setVisible(False)``),
never shown to an operator. ``build_wire_string()`` is kept as a small,
independently-tested pure function (a ``CommandSpec``-shaped dict in,
a wire string out) in case a future command row needs it again -- it no
longer has any live caller.

No PySide6 imports here — this module is usable in headless tests without a
display server or a Qt application instance.
"""

from __future__ import annotations

import re
from typing import Any, TypedDict

# 115-009 (gut S1's own test-sweep/green-bar ticket) / testgui-motion-paths-
# dead-after-move-cutover (2026-07-22 revival): `robot_radio.planner.tour`
# went dormant for a while (its own module body raised AttributeError at
# import time, referencing `telemetry_pb2.ACK_STATUS_DONE`, part of the
# depth-3 ack ring 115-003's frame-v2 rewrite deleted) and has since been
# ported onto protocol v4's Move/single-ack-slot shape (see that module's
# own file header) -- it imports cleanly again, so `TOUR_1`/`TOUR_2` below
# resolve to the real 13/15-leg geometry, not the empty-list fallback. This
# module is NOT itself a tour/turn module -- COMMANDS/build_wire_string/
# goto_distance/goto_reached below are plain, unrelated command-schema/
# geometry helpers -- so the guard stays regardless (defense against a
# FUTURE `planner.tour` breakage taking the whole module down); only
# `TOURS` (below) would degrade to empty if it ever fired again.
try:
    from robot_radio.planner.tour import (
        PIPELINED_EXECUTION,
        SQUARE_EXECUTION,
        TOUR_1,
        TOUR_2,
        TOUR_SQUARE,
        TourExecution,
    )
except (ImportError, AttributeError):
    TOUR_1 = TOUR_2 = TOUR_SQUARE = []
    TourExecution = None  # type: ignore[assignment]
    PIPELINED_EXECUTION = SQUARE_EXECUTION = None  # type: ignore[assignment]


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
# EMPTY (128-004) -- see this module's own docstring. The seven-row table
# that used to live here (S/T/D/R/TURN/RT/G, each range-audited against
# docs/protocol-v2.md §10 at sprint 085 ticket 001) drove a GUI panel that
# was already hidden before this ticket and translated verbs that have no
# binary-plane arm on the current wire (``testgui/binary_bridge.py``'s own
# module docstring has the full accounting). ``CommandSpec``/``ParamSpec``
# stay declared above for ``build_wire_string()``'s type signature.

COMMANDS: list[CommandSpec] = []


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
    "Tour 2": TOUR_2,
    "Square": TOUR_SQUARE,
}

#: How each named tour must be DRIVEN (label → ``TourExecution``). Separate
#: from ``TOURS`` because the geometry alone does not determine the result:
#: "Square" only reproduces ticket 134-004's measured 6.3 mm closure when it
#: is driven rest-to-rest with a PASSIVE dwell (see ``planner.tour``'s own
#: ``TourExecution`` docstring). Every entry in ``TOURS`` has an entry here;
#: ``tour_execution()`` below is the accessor callers should use.
TOUR_EXECUTION: "dict[str, TourExecution]" = {
    "Tour 1": PIPELINED_EXECUTION,
    "Tour 2": PIPELINED_EXECUTION,
    "Square": SQUARE_EXECUTION,
}


def tour_execution(name: str) -> "TourExecution | None":
    """The ``TourExecution`` for tour *name*, defaulting to the historical
    pipelined execution for any tour without an explicit entry. Returns
    ``None`` only if ``planner.tour`` itself failed to import (the guard at
    the top of this module), in which case there is no tour to run anyway."""
    return TOUR_EXECUTION.get(name, PIPELINED_EXECUTION)


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
