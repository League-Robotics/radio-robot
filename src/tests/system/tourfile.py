"""tourfile -- the system-test tour-file parser.

A tour file is a text version of the protocol: one directive per line, a
script of the moves (and checks) a system-test run performs. Grammar
(system-test-minimal-system-test issue, 2026-07-31):

    # comment                                blank lines ignored
    TWIST vx=150 [vy=0] [omega=45] (time=|dist=|angle=) [timeout=]
    WHEELS left=100 right=100 (time=|dist=|angle=) [timeout=]
    SPLINE file=tag_tour.path.json [speed=150] [lookahead=150] [laps=1]
           [tol=120] [interval=0.12]
    STOP [dwell=1.0]
    DWELL 0.5
    DBG wedge left 1500
    MARK leg1a                               sugar for DBG mark leg1a
    SEND STATUS [data]
    EXPECT '.type=="status" and .payload.ready' [timeout=1.0]
    CAMFIX x=0 y=0 radius=50 [heading=90 tol=5]

Human units in the file: mm, mm/s, deg, deg/s, s. Parsed steps are
normalized to the codebase's internal units (rad for angles, s kept for
times -- the executor converts to wire ms). Unknown directives and unknown
key=value keys raise immediately: a tour author gets a clear error, never
silent misparsing (same defense-in-depth stance as planner/tour.py's
parse_tour()).

Timeout default: 3x the expected duration, floor 2 s -- the same rule
planner/tour.py's _MOVE_TIMEOUT_FACTOR/_MOVE_MIN_TIMEOUT encode.
"""

from __future__ import annotations

import math
import shlex
from dataclasses import dataclass, field
from pathlib import Path
from typing import Union

_TIMEOUT_FACTOR = 3.0
_MIN_TIMEOUT = 2.0  # [s]

_DEG = math.pi / 180.0


class TourParseError(ValueError):
    """A tour-file line failed to parse; carries file/line context."""

    def __init__(self, source: str, line_no: int, message: str):
        super().__init__(f"{source}:{line_no}: {message}")
        self.source = source
        self.line_no = line_no


@dataclass(frozen=True)
class MoveStep:
    """One bounded motion -- maps 1:1 onto a protocol-v5 Move."""

    line_no: int
    variant: str  # "twist" | "wheels"
    v_x: float = 0.0  # [mm/s] twist
    v_y: float = 0.0  # [mm/s] twist (accepted, ignored on differential)
    omega: float = 0.0  # [rad/s] twist
    v_left: float = 0.0  # [mm/s] wheels
    v_right: float = 0.0  # [mm/s] wheels
    stop_kind: str = "time"  # "time" | "dist" | "angle"
    stop_value: float = 0.0  # [s] time / [mm] dist / [rad] angle; positive
    timeout: float = _MIN_TIMEOUT  # [s]


@dataclass(frozen=True)
class StopStep:
    """Planned stop (the wire STOP verb) + optional at-rest dwell."""

    line_no: int
    dwell: float = 0.0  # [s]


@dataclass(frozen=True)
class DwellStep:
    """Host-side pause; no command sent."""

    line_no: int
    seconds: float  # [s]


@dataclass(frozen=True)
class DbgStep:
    """Send DBG:<text> down the wire (fault injection / annotation)."""

    line_no: int
    text: str  # everything after DBG, verbatim ("wedge left 1500")


@dataclass(frozen=True)
class SendStep:
    """Send an arbitrary cleartext verb (SEND STATUS, SEND PING)."""

    line_no: int
    verb: str
    data: str = ""  # optional payload after the verb


@dataclass(frozen=True)
class ExpectStep:
    """Assert a JSONL record matching `query` arrives between the previous
    directive's issue time and `timeout`."""

    line_no: int
    query: str
    timeout: float = 2.0  # [s]


@dataclass(frozen=True)
class SplineStep:
    """Follow a stored spline path with pure pursuit.

    The path itself lives in a .path.json next to the tour (splinefile.py);
    the tour only says which one, how fast, and how far ahead to aim. A
    SPLINE is one directive because pure pursuit is a closed-loop follow, not
    a sequence of bounded Moves -- the runner streams twists until the path
    is consumed."""

    line_no: int
    path: str                      # file name, resolved next to the tour
    speed: float = 150.0           # [mm/s]
    lookahead: float = 150.0       # [mm]
    laps: int = 1
    tol: float = 120.0             # [mm] max allowed cross-track before failing
    interval: float = 0.12         # [s] pure-pursuit re-issue period


@dataclass(frozen=True)
class CamfixStep:
    """Independent position validation: camera fix (playfield) or plant
    true pose (sim); robot centre must be within `radius` of (x, y)."""

    line_no: int
    x: float  # [mm]
    y: float  # [mm]
    radius: float  # [mm]
    heading: float | None = None  # [rad] optional yaw assertion
    tol: float = 5.0 * _DEG  # [rad]


Step = Union[MoveStep, StopStep, DwellStep, DbgStep, SendStep, ExpectStep,
             CamfixStep, SplineStep]


@dataclass(frozen=True)
class Tour:
    name: str
    source: str  # path or "<text>"
    text: str  # the verbatim file content (recorded in run_meta)
    steps: tuple[Step, ...] = field(default_factory=tuple)


def _parse_kv(source: str, line_no: int, tokens: list[str],
              allowed: dict[str, float | None]) -> dict[str, float]:
    """Parse key=value tokens; every key must be in `allowed`. Returns only
    the keys present. Values are floats."""
    out: dict[str, float] = {}
    for tok in tokens:
        if "=" not in tok:
            raise TourParseError(source, line_no,
                                 f"expected key=value, got {tok!r}")
        key, _, raw = tok.partition("=")
        if key not in allowed:
            raise TourParseError(
                source, line_no,
                f"unknown key {key!r} (allowed: {', '.join(sorted(allowed))})")
        if key in out:
            raise TourParseError(source, line_no, f"duplicate key {key!r}")
        try:
            out[key] = float(raw)
        except ValueError:
            raise TourParseError(source, line_no,
                                 f"bad number for {key}: {raw!r}") from None
    return out


def _one_stop_condition(source: str, line_no: int,
                        kv: dict[str, float]) -> tuple[str, float]:
    given = [k for k in ("time", "dist", "angle") if k in kv]
    if len(given) != 1:
        raise TourParseError(
            source, line_no,
            f"exactly one stop condition of time=/dist=/angle= required, "
            f"got {given or 'none'}")
    kind = given[0]
    value = kv[kind]
    if value <= 0.0:
        raise TourParseError(source, line_no,
                             f"stop condition {kind}= must be positive")
    # Normalize: angle deg -> rad. time stays [s], dist stays [mm].
    if kind == "angle":
        value *= _DEG
    return kind, value


def _default_timeout(step_kind: str, stop_kind: str, stop_value: float,
                     speed: float) -> float:  # [s]
    """3x expected duration, floor 2 s. `speed` is the relevant magnitude
    ([mm/s] for dist, [rad/s] for angle); 0 means unknowable -> floor."""
    if stop_kind == "time":
        expected = stop_value
    elif speed > 0.0:
        expected = stop_value / speed
    else:
        expected = 0.0
    return max(_MIN_TIMEOUT, expected * _TIMEOUT_FACTOR)


def _parse_twist(source: str, line_no: int, tokens: list[str]) -> MoveStep:
    kv = _parse_kv(source, line_no, tokens,
                   {"vx": None, "vy": None, "omega": None, "time": None,
                    "dist": None, "angle": None, "timeout": None})
    stop_kind, stop_value = _one_stop_condition(source, line_no, kv)
    v_x = kv.get("vx", 0.0)
    v_y = kv.get("vy", 0.0)
    omega = kv.get("omega", 0.0) * _DEG  # file deg/s -> rad/s
    if v_x == 0.0 and v_y == 0.0 and omega == 0.0:
        raise TourParseError(source, line_no,
                             "TWIST needs at least one of vx=/vy=/omega=")
    speed = abs(v_x) if stop_kind == "dist" else abs(omega)
    timeout = kv.get("timeout") or _default_timeout("twist", stop_kind,
                                                    stop_value, speed)
    return MoveStep(line_no=line_no, variant="twist", v_x=v_x, v_y=v_y,
                    omega=omega, stop_kind=stop_kind, stop_value=stop_value,
                    timeout=timeout)


def _parse_wheels(source: str, line_no: int, tokens: list[str]) -> MoveStep:
    kv = _parse_kv(source, line_no, tokens,
                   {"left": None, "right": None, "time": None, "dist": None,
                    "angle": None, "timeout": None})
    if "left" not in kv or "right" not in kv:
        raise TourParseError(source, line_no,
                             "WHEELS needs both left= and right=")
    stop_kind, stop_value = _one_stop_condition(source, line_no, kv)
    speed = (abs(kv["left"]) + abs(kv["right"])) / 2.0
    timeout = kv.get("timeout") or _default_timeout("wheels", stop_kind,
                                                    stop_value, speed)
    return MoveStep(line_no=line_no, variant="wheels", v_left=kv["left"],
                    v_right=kv["right"], stop_kind=stop_kind,
                    stop_value=stop_value, timeout=timeout)


def _parse_camfix(source: str, line_no: int, tokens: list[str]) -> CamfixStep:
    kv = _parse_kv(source, line_no, tokens,
                   {"x": None, "y": None, "radius": None, "heading": None,
                    "tol": None})
    for req in ("x", "y", "radius"):
        if req not in kv:
            raise TourParseError(source, line_no, f"CAMFIX needs {req}=")
    if kv["radius"] <= 0.0:
        raise TourParseError(source, line_no, "CAMFIX radius= must be positive")
    heading = kv["heading"] * _DEG if "heading" in kv else None
    tol = kv.get("tol", 5.0) * _DEG
    return CamfixStep(line_no=line_no, x=kv["x"], y=kv["y"],
                      radius=kv["radius"], heading=heading, tol=tol)


def parse_tour_text(text: str, *, name: str, source: str = "<text>") -> Tour:
    steps: list[Step] = []
    for line_no, raw in enumerate(text.splitlines(), start=1):
        # Strip a trailing comment, then whitespace. shlex handles the
        # EXPECT quoted-query case; comments inside quotes survive.
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        try:
            tokens = shlex.split(line, comments=True)
        except ValueError as exc:
            raise TourParseError(source, line_no, f"unbalanced quotes: {exc}")
        if not tokens:
            continue
        verb, rest = tokens[0], tokens[1:]
        if verb == "TWIST":
            steps.append(_parse_twist(source, line_no, rest))
        elif verb == "WHEELS":
            steps.append(_parse_wheels(source, line_no, rest))
        elif verb == "STOP":
            kv = _parse_kv(source, line_no, rest, {"dwell": None})
            steps.append(StopStep(line_no=line_no, dwell=kv.get("dwell", 0.0)))
        elif verb == "DWELL":
            if len(rest) != 1:
                raise TourParseError(source, line_no, "DWELL takes one value")
            steps.append(DwellStep(line_no=line_no, seconds=float(rest[0])))
        elif verb == "DBG":
            if not rest:
                raise TourParseError(source, line_no, "DBG needs a subcommand")
            steps.append(DbgStep(line_no=line_no, text=" ".join(rest)))
        elif verb == "MARK":
            if not rest:
                raise TourParseError(source, line_no, "MARK needs a label")
            steps.append(DbgStep(line_no=line_no,
                                 text="mark " + " ".join(rest)))
        elif verb == "SEND":
            if not rest:
                raise TourParseError(source, line_no, "SEND needs a verb")
            steps.append(SendStep(line_no=line_no, verb=rest[0],
                                  data=" ".join(rest[1:])))
        elif verb == "EXPECT":
            if not rest:
                raise TourParseError(source, line_no, "EXPECT needs a query")
            kv_tokens = [t for t in rest[1:] if "=" in t]
            if len(kv_tokens) != len(rest) - 1:
                raise TourParseError(
                    source, line_no,
                    "EXPECT takes one quoted query then optional timeout=")
            kv = _parse_kv(source, line_no, kv_tokens, {"timeout": None})
            steps.append(ExpectStep(line_no=line_no, query=rest[0],
                                    timeout=kv.get("timeout", 2.0)))
        elif verb == "SPLINE":
            # file= is the one non-numeric value in the grammar, so it is
            # split out before _parse_kv (which coerces every value to float).
            path_val = None
            numeric = []
            for tok in rest:
                if tok.startswith("file="):
                    path_val = tok.partition("=")[2]
                else:
                    numeric.append(tok)
            if not path_val:
                raise TourParseError(source, line_no, "SPLINE needs file=")
            kv = _parse_kv(source, line_no, numeric,
                           {"speed": None, "lookahead": None,
                            "laps": None, "tol": None, "interval": None})
            steps.append(SplineStep(
                line_no=line_no, path=path_val,
                speed=float(kv.get("speed", 150.0)),
                lookahead=float(kv.get("lookahead", 150.0)),
                laps=int(kv.get("laps", 1)),
                tol=float(kv.get("tol", 120.0)),
                interval=float(kv.get("interval", 0.12))))
        elif verb == "CAMFIX":
            steps.append(_parse_camfix(source, line_no, rest))
        else:
            raise TourParseError(
                source, line_no,
                f"unknown directive {verb!r} -- known: TWIST WHEELS STOP "
                f"DWELL DBG MARK SEND EXPECT CAMFIX")
    if not steps:
        raise TourParseError(source, 0, "tour has no steps")
    return Tour(name=name, source=source, text=text, steps=tuple(steps))


def parse_tour_file(path: str | Path) -> Tour:
    p = Path(path)
    return parse_tour_text(p.read_text(), name=p.stem, source=str(p))
