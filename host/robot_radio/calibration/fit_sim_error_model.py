"""fit_sim_error_model.py — host-side sim-error-model fit tooling (069-008).

The sprint's acceptance vehicle for "the simulator and the real robot must
be tunable to behave identically" (see
``clasi/sprints/069-.../architecture-update.md``, Design Rationale
Decision 5). Regresses the deterministic/bias-shaped subset of the
``SIMSET`` registry (``source/commands/SimCommands.cpp``'s ``kSimRegistry[]``)
against a recorded trajectory using ``scipy.optimize.least_squares``
(bounded, Trust Region Reflective), minimizing summed squared position +
wrapped-heading residual across the three TLM poses (``pose=``, ``otos=``,
``encpose=`` — see ``host/robot_radio/robot/protocol.py``'s ``TLMFrame``).

097-003 NOTE (flagged, not silently patched): this module's ``_iter_run()``
reads ``encpose`` from ``sim.send_command("SNAP")``'s TEXT reply via
``robot_radio.robot._legacy_tlm_text.parse_historical_tlm_line()`` (a
frozen copy of the text-plane TLM parser 097-003 retired from
``protocol.py``), NOT via ``NezhaProtocol``'s binary telemetry delivery
that ticket converted every other internal consumer onto. Two independent,
structural reasons block the binary move for this file specifically: (1)
``_iter_run()`` drives an in-process ``Sim()`` ctypes wrapper
(``tests/_infra/sim/firmware.py``), never a ``SerialConnection`` — there is
no ``_binary_tlm_queue`` on this path; (2) even if there were, ``encpose``
is structurally ABSENT from ``telemetry.proto``'s ``Telemetry`` message
(096-001 Decision 6 trimmed it to fit the binary envelope's 186-byte
budget), so ``TLMFrame.from_pb2()`` can never populate it regardless of
transport — this is the ONE consumer 097-003's architecture research found
that structurally depends on the field the binary plane dropped. See
``robot_radio.robot._legacy_tlm_text``'s own module docstring for the full
reasoning and the other three consumers sharing this same conservative
treatment.

Recording format
-----------------
JSONL, transport-agnostic — one JSON object per line, one of two shapes:

    {"t": <ms>, "cmd": "<wire command text>"}
        One per issued command (e.g. ``"T 200 200 2000"``).

    {"t": <ms>, "encpose": [x, y, h],
                "otos":    [x, y, h],
                "pose":    [x, y, h]}
        One per sampled TLM frame; each ``[x, y, h]`` triple is
        (mm, mm, cdeg). Any of the three pose keys may be
        omitted/null if that field was absent from the sampled frame (e.g.
        ``encpose=`` on older firmware, or ``otos=``/``pose=`` before OTOS
        fusion is enabled).

This sprint's own recorder (``record_sim_run``) produces this shape from a
``Sim()`` instance ONLY (``tests/_infra/sim/firmware.py``). A HIL recorder
producing the identical shape from a live serial/relay connection is a
follow-up task's concern — see architecture-update.md Open Question 1.

Candidate parameter set
------------------------
``DEFAULT_CANDIDATE_KEYS`` is the deterministic/bias-shaped ``SIMSET``
subset: ``bodyRotScrub``, ``bodyLinScrub``, ``trackwidthMm``,
``motorOffsetL``, ``motorOffsetR``, ``encScaleErrL``, ``encScaleErrR``,
``otosLinScaleErr``, ``otosAngScaleErr``, ``otosLinDriftMmS``,
``otosYawDriftDegS``. This deliberately EXCLUDES the noise-sigma keys
(``encNoiseL``/``R``, ``otosLinNoise``, ``otosYawNoise``) — these are
Gaussian sigma/variance terms (see ``pwGaussianNoise()`` in
``source/hal/sim/PhysicsWorld.cpp``) that add scatter AROUND a mean
trajectory rather than biasing it; a least-squares fit against a single
recorded run's mean trajectory cannot usefully estimate a variance
parameter (that needs repeated-trial dispersion statistics, not a
trajectory-agreement residual).

``encSlipL``/``encSlipR`` bias-vs-variance finding: inspection of
``PhysicsWorld::update()``'s sub-step A' —
``deltaL = noisyL * dt_s * (1 + _encScaleErrL) * (1 - _encSlipL)`` — shows
``encSlipL``/``R`` is a second DETERMINISTIC multiplicative bias factor on
the reported encoder delta, structurally identical in kind to
``encScaleErrL``/``R`` (both are pure biases; the actual Gaussian variance
term is ``encNoiseL``/``R``'s additive ``pwGaussianNoise(...)`` draw earlier
in the same sub-step). So ``encSlipL``/``R`` is NOT excluded from
``DEFAULT_CANDIDATE_KEYS`` for a variance-vs-bias reason — it is simply not
part of ticket 069-008's literal named default list. Both ``fit_sim_error_model()``
and the CLI accept an arbitrary ``candidate_keys`` sequence (see
``DEFAULT_BOUNDS`` for its bounds), so a caller who wants ``encSlipL``/``R``
fit can pass it explicitly.

Scope boundary
--------------
The real-hardware Tour-1 record→fit→replay demo (the issue's ultimate
acceptance) is explicitly OUT OF SCOPE for this module and this sprint —
deferred to a follow-up HIL task (architecture-update.md Open Question 1).
This module is validated SIM-TO-SIM only this sprint: inject known
``SIMSET`` values into one sim, record, fit against fresh sim replays,
confirm recovery within tolerance (``tests/simulation/unit/
test_fit_sim_error_model.py``).

scipy is imported LAZILY (inside ``fit_sim_error_model()``) so importing
this module never hard-fails when scipy is not installed — see that
function's docstring for the install instructions surfaced in the
``ImportError`` message.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterable, Optional, Sequence

import numpy as np

from robot_radio.robot.protocol import TLMFrame

# ---------------------------------------------------------------------------
# Candidate parameter set, bounds, and no-op ("neutral") values
# ---------------------------------------------------------------------------

#: (lower, upper) bound per SIMSET key. Scrub keys mirror clampScrub()'s
#: (0, 1] physical range (PhysicsWorld.cpp) with a small positive floor
#: instead of 0 (a true 0 is a division-by-zero/sign-flip pathology, same
#: reasoning as clampScrub() itself). The remaining bounds are generous
#: ranges around each knob's no-op default, sized using the non-default
#: magnitudes ``tests/simulation/system/test_069_knob_telemetry_sweep.py``
#: already validated as clearly observable on this plant (e.g.
#: ``trackwidthMm=200`` against a ~128mm default, ``otosLinScaleErr=0.10``).
DEFAULT_BOUNDS: dict[str, tuple[float, float]] = {
    "bodyRotScrub": (0.05, 1.0),
    "bodyLinScrub": (0.05, 1.0),
    "trackwidthMm": (60.0, 250.0),
    "motorOffsetL": (0.5, 1.5),
    "motorOffsetR": (0.5, 1.5),
    "encScaleErrL": (-0.3, 0.3),
    "encScaleErrR": (-0.3, 0.3),
    "encSlipL": (-0.3, 0.3),
    "encSlipR": (-0.3, 0.3),
    "otosLinScaleErr": (-0.3, 0.3),
    "otosAngScaleErr": (-0.3, 0.3),
    "otosLinDriftMmS": (-30.0, 30.0),
    "otosYawDriftDegS": (-20.0, 20.0),
}

#: Default candidate set: the deterministic/bias-shaped SIMSET subset named
#: by ticket 069-008's acceptance criteria. See the module docstring for why
#: the noise-sigma keys (and encSlipL/R) are not in this default list.
DEFAULT_CANDIDATE_KEYS: tuple[str, ...] = (
    "bodyRotScrub",
    "bodyLinScrub",
    "trackwidthMm",
    "motorOffsetL",
    "motorOffsetR",
    "encScaleErrL",
    "encScaleErrR",
    "otosLinScaleErr",
    "otosAngScaleErr",
    "otosLinDriftMmS",
    "otosYawDriftDegS",
)

#: No-op ("nothing configured") value per key — used as the least_squares
#: initial guess (x0) when the caller doesn't supply one.
NEUTRAL_VALUE: dict[str, float] = {
    "bodyRotScrub": 1.0,
    "bodyLinScrub": 1.0,
    "trackwidthMm": 128.0,
    "motorOffsetL": 1.0,
    "motorOffsetR": 1.0,
    "encScaleErrL": 0.0,
    "encScaleErrR": 0.0,
    "encSlipL": 0.0,
    "encSlipR": 0.0,
    "otosLinScaleErr": 0.0,
    "otosAngScaleErr": 0.0,
    "otosLinDriftMmS": 0.0,
    "otosYawDriftDegS": 0.0,
}

#: Absolute finite-difference step per SIMSET key, used by
#: ``fit_sim_error_model()``'s own bounds-aware Jacobian (see
#: ``_make_jacobian``) — deliberately NOT ``scipy.optimize.least_squares``'s
#: built-in ``diff_step`` (a RELATIVE step, ``h = diff_step * abs(x)``).
#: Relative stepping silently collapses to scipy's tiny (~1.5e-8) fallback
#: absolute step whenever a candidate value is exactly (or passes through)
#: zero — true for every error-shaped key here (``encScaleErrL/R``,
#: ``otos*ScaleErr``, ``otos*DriftMmS/DegS``) — and that fallback step is far
#: below the wire protocol's resolution (``_simset_line``'s ``%.6g``
#: formatting, and the firmware's integer-mm/centidegree TLM), so the
#: resulting finite-difference Jacobian column is measured, empirically, as
#: EXACTLY ZERO: the perturbed SIMSET value round-trips to a bit-identical
#: trajectory. least_squares then treats that parameter as having zero
#: sensitivity and never moves it from its initial guess. An absolute step,
#: sized well above both the wire-formatting and telemetry quantization
#: floors, avoids this regardless of the parameter's current value.
JAC_ABS_STEP: dict[str, float] = {
    "bodyRotScrub": 0.02,
    "bodyLinScrub": 0.02,
    "trackwidthMm": 2.0,
    "motorOffsetL": 0.02,
    "motorOffsetR": 0.02,
    "encScaleErrL": 0.01,
    "encScaleErrR": 0.01,
    "encSlipL": 0.01,
    "encSlipR": 0.01,
    "otosLinScaleErr": 0.01,
    "otosAngScaleErr": 0.01,
    "otosLinDriftMmS": 1.0,
    "otosYawDriftDegS": 0.5,
}


# ---------------------------------------------------------------------------
# Recording: drive a Sim() through a maneuver, emit JSONL-shaped records
# ---------------------------------------------------------------------------

def default_maneuver() -> tuple[list[tuple[int, str]], int]:
    """A short, deterministic two-phase maneuver: drive straight, then turn.

    Exercises TRANSLATION-shaped error terms (``bodyLinScrub``,
    ``encScaleErrL/R``, ``otosLinScaleErr``, ``otosLinDriftMmS``) during the
    straight phase and ROTATION-shaped terms (``bodyRotScrub``,
    ``trackwidthMm``, ``otosAngScaleErr``, ``otosYawDriftDegS``) during the
    turn phase, so every key in ``DEFAULT_CANDIDATE_KEYS`` has an
    observable window in the recording. Command choice and duration mirror
    the magnitudes ``tests/simulation/system/test_069_knob_telemetry_sweep.py``
    already validated as clearly observable (``T 200 200`` straight drive;
    ``RT 9000``).

    Returns ``(commands, total_duration)``: ``commands`` is a list of
    ``(t, wire_command)`` pairs, ``t`` [ms] relative to the start of the
    run; ``total_duration`` [ms] is long enough to cover both phases.
    """
    commands = [
        (0, "T 200 200 2000"),
        (2500, "RT 9000"),
    ]
    total_duration = 10500  # [ms]
    return commands, total_duration


def _iter_run(
    sim: Any,
    commands: Iterable[tuple[int, str]],
    total_duration: int,  # [ms]
    sample_period: int,  # [ms]
):
    """Drive *sim* through *commands*, yielding one ``(t, sent, frame)`` per
    sample tick.

    *commands* is a list of ``(t, wire_command)`` pairs [ms] (need not be
    pre-sorted). At each sample tick ``t`` (0, sample_period, ... up to
    and including ``total_duration``), every not-yet-sent command due at or
    before ``t`` is sent (in ascending-``t`` order, via ``sim.send_command``)
    BEFORE that tick's ``SNAP``; ``sent`` is the list of command strings
    sent at this tick (usually empty). ``frame`` is the parsed ``TLMFrame``
    from the ``SNAP`` reply, or ``None`` if it didn't parse as TLM.

    Shared by ``record_sim_run`` (keeps the ``cmd``/pose JSONL records) and
    ``replay_samples`` (keeps only the pose samples) so the two can never
    drift apart on tick/command timing.

    097-003: stays on the text-plane SNAP reply, via a frozen local copy of
    the retired parser -- see this module's own header note for why.
    """
    # local: keep import graph shallow, matches the pre-097-003 import style.
    from robot_radio.robot._legacy_tlm_text import parse_historical_tlm_line

    pending = sorted(commands, key=lambda c: c[0])
    idx = 0
    t = 0
    while True:
        sent: list[str] = []
        while idx < len(pending) and pending[idx][0] <= t:
            sim.send_command(pending[idx][1])
            sent.append(pending[idx][1])
            idx += 1
        reply = sim.send_command("SNAP")
        frame = parse_historical_tlm_line(reply)
        yield t, sent, frame
        if t >= total_duration:
            break
        step = min(sample_period, total_duration - t)
        sim.tick_for(step)
        t += step


def record_sim_run(
    sim: Any,
    commands: Iterable[tuple[int, str]],
    total_duration: int,  # [ms]
    sample_period: int = 100,  # [ms]
) -> list[dict[str, Any]]:
    """Drive *sim* (an already-constructed, already-configured ``Sim()``)
    through *commands* and return a JSONL-shaped list of records (see the
    module docstring's Recording Format).

    *sim* is expected to already be in whatever state the caller wants
    recorded from (watchdog extended, OTOS model/fusion enabled, and any
    ``SIMSET`` injection already applied — this function neither configures
    nor mutates sim error parameters itself, it only drives and observes).
    """
    records: list[dict[str, Any]] = []
    for t, sent, frame in _iter_run(sim, commands, total_duration, sample_period):
        for cmd in sent:
            records.append({"t": t, "cmd": cmd})
        if frame is None:
            continue
        rec: dict[str, Any] = {"t": t}
        if frame.encpose is not None:
            rec["encpose"] = list(frame.encpose)
        if frame.otos is not None:
            rec["otos"] = list(frame.otos)
        if frame.pose is not None:
            rec["pose"] = list(frame.pose)
        if len(rec) > 1:
            records.append(rec)
    return records


def replay_samples(
    sim: Any,
    commands: Iterable[tuple[int, str]],
    total_duration: int,  # [ms]
    sample_period: int = 100,  # [ms]
) -> dict[int, TLMFrame]:
    """Like ``record_sim_run`` but returns ``{t: TLMFrame}`` samples (t [ms])
    directly (no JSONL shaping) — the form ``fit_sim_error_model()`` needs
    for residual computation.
    """
    samples: dict[int, TLMFrame] = {}
    for t, _sent, frame in _iter_run(sim, commands, total_duration, sample_period):
        if frame is not None:
            samples[t] = frame
    return samples


# ---------------------------------------------------------------------------
# JSONL I/O
# ---------------------------------------------------------------------------

def save_jsonl(records: Sequence[dict[str, Any]], path: Path) -> None:
    """Write *records* to *path*, one JSON object per line."""
    with open(path, "w") as f:
        for rec in records:
            f.write(json.dumps(rec))
            f.write("\n")


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    """Read a JSONL recording back into a list of dicts (inverse of ``save_jsonl``)."""
    records: list[dict[str, Any]] = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            records.append(json.loads(line))
    return records


def split_recording(
    records: Iterable[dict[str, Any]],
) -> tuple[list[tuple[int, str]], dict[int, TLMFrame]]:
    """Split a loaded recording into ``(commands, pose_samples)``.

    ``commands`` is a list of ``(t, wire_command)`` pairs (t [ms]). ``pose_samples``
    is ``{t: TLMFrame}`` (t [ms]) — reconstructed directly onto the canonical
    ``TLMFrame`` dataclass (``host/robot_radio/robot/protocol.py``), not a
    second hand-rolled parser: the JSONL pose records already carry
    structured ``[x, y, h]`` lists (they were produced by the text-plane TLM
    parser in the first place, see ``record_sim_run``/``_iter_run``), so
    there is no wire text left to re-parse here.
    """
    commands: list[tuple[int, str]] = []
    pose_samples: dict[int, TLMFrame] = {}
    for rec in records:
        t = int(rec["t"])
        if "cmd" in rec:
            commands.append((t, str(rec["cmd"])))
            continue
        frame = TLMFrame(t=t)
        if rec.get("encpose") is not None:
            frame.encpose = tuple(rec["encpose"])  # type: ignore[assignment]
        if rec.get("otos") is not None:
            frame.otos = tuple(rec["otos"])  # type: ignore[assignment]
        if rec.get("pose") is not None:
            frame.pose = tuple(rec["pose"])  # type: ignore[assignment]
        pose_samples[t] = frame
    return commands, pose_samples


# ---------------------------------------------------------------------------
# Residual computation (position + wrapped-heading, across all three poses)
# ---------------------------------------------------------------------------

def _wrap_angle(deg: float) -> float:
    """Wrap an angle in degrees to (-180, 180]."""
    d = math.fmod(deg + 180.0, 360.0)
    if d <= 0.0:
        d += 360.0
    return d - 180.0


def _pose_residual(recorded: tuple[int, int, int], replayed: tuple[int, int, int]) -> list[float]:
    """[dx, dy, dh] (mm, mm, deg) between two ``(x, y, heading)`` poses (mm, mm, cdeg).

    Heading is wrapped to (-180, 180] before differencing — an unwrapped
    difference near the +/-180 deg boundary would otherwise dominate the
    residual spuriously (headings are circular quantities).
    """
    dx = float(replayed[0] - recorded[0])
    dy = float(replayed[1] - recorded[1])
    dh = _wrap_angle((replayed[2] - recorded[2]) / 100.0)
    return [dx, dy, dh]


def _residual_vector(
    recorded_samples: dict[int, TLMFrame],
    replayed_samples: dict[int, TLMFrame],
) -> list[float]:
    """Flattened residual vector across every recorded timestamp and every
    one of the three TLM poses (``pose``, ``otos``, ``encpose``) present in
    BOTH the recorded sample and the corresponding replayed sample.
    """
    out: list[float] = []
    for t, rec_frame in recorded_samples.items():
        rep_frame = replayed_samples.get(t)
        if rep_frame is None:
            continue
        for attr in ("pose", "otos", "encpose"):
            rec_pose = getattr(rec_frame, attr)
            rep_pose = getattr(rep_frame, attr)
            if rec_pose is None or rep_pose is None:
                continue
            out.extend(_pose_residual(rec_pose, rep_pose))
    return out


# ---------------------------------------------------------------------------
# SIMSET wire helpers
# ---------------------------------------------------------------------------

def _simset_line(params: dict[str, float]) -> str:
    # %.6g, not Python's default float repr: SimCommands.cpp's parseSimSet
    # copies each "key=value" pair into a fixed 31-char (+NUL) buffer
    # (ArgParse.h: "sval bounded to 31+NUL") and silently TRUNCATES anything
    # longer -- Python's default repr can emit up to ~17 significant digits
    # in scientific notation (e.g. numerical-Jacobian perturbations near
    # zero), which truncates mid-exponent and comes back `ERR badval`. Six
    # significant digits keeps every "key=value" pair (even the longest key,
    # `otosYawDriftDegS`, 16 chars) safely under the 31-char cap while still
    # giving the finite-difference Jacobian (see `JAC_ABS_STEP`/
    # `_make_jacobian`) far more resolution than it needs.
    return "SIMSET " + " ".join(f"{k}={v:.6g}" for k, v in params.items())


def apply_params(sim: Any, params: dict[str, float]) -> str:
    """Apply *params* to *sim* as ONE batched ``SIMSET k1=v1 k2=v2 ...``
    command (never per-key round trips — mirrors
    ``host/robot_radio/testgui/transport.py``'s ``_apply_profile_to_sim()``,
    ticket 007's TestGUI panel). Returns the raw reply string. No-ops
    (returns ``""``) for an empty *params* dict.
    """
    if not params:
        return ""
    reply = sim.send_command(_simset_line(params))
    if not reply.upper().startswith("OK"):
        raise RuntimeError(f"SIMSET failed applying {params!r}: {reply!r}")
    return reply


def push_params(conn: Any, params: dict[str, float]) -> str:
    """Push *params* to a live connection as ONE batched ``SIMSET`` command.

    *conn* must expose either ``send_command(line) -> str`` (the
    ``firmware.Sim`` ctypes wrapper / ``SimTransport`` convention) or
    ``command(line) -> str`` (the generic ``Transport`` convention used
    elsewhere in ``host/robot_radio/testgui/transport.py``) — whichever is
    present is used, ``send_command`` preferred. Raises ``TypeError`` if
    *conn* exposes neither. Raises ``ValueError`` for an empty *params*
    dict (nothing to push).
    """
    if not params:
        raise ValueError("push_params: empty params dict")
    line = _simset_line(params)
    if hasattr(conn, "send_command"):
        return conn.send_command(line)
    if hasattr(conn, "command"):
        return conn.command(line)
    raise TypeError(
        f"push_params: {type(conn).__name__} exposes neither "
        f"send_command() nor command()"
    )


class WriteLineAdapter:
    """Adapts a ``write_line()``/``read_available()`` connection (e.g.
    ``robot_radio.calibration._conn_helpers.RelaySerial``/``DirectSerial``)
    to the ``send_command(line) -> str`` convention ``push_params`` expects.

    Real-hardware pushing is out of scope for this sprint's validation (see
    module docstring), but ``push_params`` is written generically enough
    that a future HIL follow-up needs only this adapter, not a code change
    to ``push_params`` itself.
    """

    def __init__(self, ser: Any, timeout: float = 2.0) -> None:
        self._ser = ser
        self._timeout = timeout

    def send_command(self, line: str) -> str:
        self._ser.write_line(line)
        lines = self._ser.read_available(timeout=self._timeout)
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# Parameter file I/O
# ---------------------------------------------------------------------------

def save_param_file(path: Path, params: dict[str, float]) -> None:
    """Write *params* as ``{"<SIMSET key>": <fitted value>, ...}`` JSON."""
    path.write_text(json.dumps(params, indent=2) + "\n")


def load_param_file(path: Path) -> dict[str, float]:
    """Read a parameter file written by ``save_param_file``/``fit_sim_error_model``."""
    return {k: float(v) for k, v in json.loads(path.read_text()).items()}


# ---------------------------------------------------------------------------
# The fit itself
# ---------------------------------------------------------------------------

@dataclass
class FitResult:
    """Result of ``fit_sim_error_model()``."""
    params: dict[str, float]
    cost: float
    success: bool
    message: str
    nfev: int


def fit_sim_error_model(
    records: Sequence[dict[str, Any]],
    sim_factory: Callable[[], Any],
    candidate_keys: Sequence[str] = DEFAULT_CANDIDATE_KEYS,
    bounds: Optional[dict[str, tuple[float, float]]] = None,
    x0: Optional[dict[str, float]] = None,
    total_duration: Optional[int] = None,  # [ms]
    sample_period: int = 100,  # [ms]
    **least_squares_kwargs: Any,
) -> FitResult:
    """Fit *candidate_keys* SIMSET values against a recorded trajectory.

    For every candidate parameter vector ``scipy.optimize.least_squares``
    proposes, a FRESH sim is created via ``sim_factory()`` (a zero-argument
    callable, e.g. ``lambda: Sim()`` plus whatever baseline setup — watchdog
    extension, OTOS model/fusion enable — the caller's recording needs; see
    ``tests/simulation/unit/test_fit_sim_error_model.py`` for the pattern),
    the candidate values are applied as one ``SIMSET`` command, the SAME
    command sequence from *records* is replayed into it, and the resulting
    trajectory is compared against the recording's at every matching
    timestamp (position + wrapped-heading residual, across ``pose=``/
    ``otos=``/``encpose=`` — see ``_residual_vector``).

    *records* is a loaded JSONL recording (``load_jsonl()``'s return value,
    or an equivalent in-memory list of the same dict shape). *sample_period*
    MUST match the period the recording was made with (``record_sim_run()``'s
    own *sample_period*) — replay timestamps are generated the same way
    recording timestamps were (0, sample_period, 2*sample_period, ...),
    and the residual is only computed where a recorded and a replayed
    timestamp coincide exactly.

    *bounds* defaults to ``DEFAULT_BOUNDS``; *x0* (initial guess, by SIMSET
    key) defaults to ``NEUTRAL_VALUE`` for each candidate key, clipped into
    that key's bounds. *total_duration* defaults to the recording's own last
    sampled timestamp.

    The Jacobian is estimated with this module's own bounds-aware, ABSOLUTE
    per-key finite-difference step (``JAC_ABS_STEP`` / ``_make_jacobian``),
    NOT ``scipy.optimize.least_squares``'s built-in ``diff_step`` — see
    ``JAC_ABS_STEP``'s docstring comment for why a RELATIVE step is
    unusable here (it silently collapses to a wire-invisible perturbation
    for every candidate key whose useful range straddles zero).

    Extra keyword arguments are forwarded to ``scipy.optimize.least_squares``
    (e.g. ``max_nfev``, ``xtol``).

    Raises ``ImportError`` (with install instructions) if scipy is not
    installed — scipy is a `calibrate` dependency-group extra, not a base
    host dependency; see the module docstring.

    Raises ``ValueError`` if the recording has no pose samples to fit
    against.
    """
    try:
        from scipy.optimize import least_squares  # noqa: PLC0415
    except ImportError as exc:
        raise ImportError(
            "fit_sim_error_model() requires scipy (scipy.optimize.least_squares). "
            "Install it via the 'calibrate' dependency group: "
            "`uv sync --group calibrate` (or `uv add scipy --group calibrate`)."
        ) from exc

    bounds = bounds or DEFAULT_BOUNDS
    commands, recorded_samples = split_recording(records)
    if not recorded_samples:
        raise ValueError("recording contains no pose samples to fit against")
    if total_duration is None:
        total_duration = max(recorded_samples)

    lb = [bounds[k][0] for k in candidate_keys]
    ub = [bounds[k][1] for k in candidate_keys]
    x0_vec = []
    for k, lo, hi in zip(candidate_keys, lb, ub):
        guess = (x0 or {}).get(k, NEUTRAL_VALUE.get(k, 0.5 * (lo + hi)))
        x0_vec.append(min(max(guess, lo), hi))

    def _residuals(x: Sequence[float]) -> np.ndarray:
        params = dict(zip(candidate_keys, x))
        sim = sim_factory()
        try:
            apply_params(sim, params)
            replayed = replay_samples(sim, commands, total_duration, sample_period)
        finally:
            _close_sim(sim)
        return np.asarray(_residual_vector(recorded_samples, replayed), dtype=float)

    jac = _make_jacobian(_residuals, candidate_keys, lb, ub)

    result = least_squares(
        _residuals, x0_vec, jac=jac, bounds=(lb, ub), method="trf", **least_squares_kwargs,
    )
    fitted = {k: float(v) for k, v in zip(candidate_keys, result.x)}
    return FitResult(
        params=fitted,
        cost=float(result.cost),
        success=bool(result.success),
        message=str(result.message),
        nfev=int(result.nfev),
    )


def _make_jacobian(
    residual_fn: Callable[[Sequence[float]], np.ndarray],
    candidate_keys: Sequence[str],
    lb: Sequence[float],
    ub: Sequence[float],
) -> Callable[[Sequence[float]], np.ndarray]:
    """Build a bounds-aware, ABSOLUTE-step finite-difference Jacobian
    callable for ``scipy.optimize.least_squares``'s ``jac=`` argument.

    For each candidate column, steps FORWARD by ``JAC_ABS_STEP[key]`` if
    that stays within the upper bound, else BACKWARD if that stays within
    the lower bound, else shrinks the step to 40% of the (narrower-than-a-
    full-step) bound span — mirrors the direction-flipping
    ``scipy.optimize._numdiff.approx_derivative`` does internally for
    bounded relative-step differencing, just with an absolute step instead
    (see ``JAC_ABS_STEP`` for why absolute, not relative).
    """
    def _jac(x: Sequence[float]) -> np.ndarray:
        x = list(x)
        r0 = residual_fn(x)
        J = np.zeros((len(r0), len(x)))
        for i, key in enumerate(candidate_keys):
            h = JAC_ABS_STEP.get(key, 0.01)
            if x[i] + h <= ub[i]:
                xf = list(x)
                xf[i] = x[i] + h
                J[:, i] = (residual_fn(xf) - r0) / h
            elif x[i] - h >= lb[i]:
                xb = list(x)
                xb[i] = x[i] - h
                J[:, i] = (r0 - residual_fn(xb)) / h
            else:
                # Bound span narrower than a full step (e.g. a caller-supplied
                # bounds dict with a tight range) -- shrink to what's left.
                h2 = max((ub[i] - lb[i]) * 0.4, 1e-9)
                xf = list(x)
                xf[i] = min(x[i] + h2, ub[i])
                actual_h = xf[i] - x[i]
                if actual_h > 0:
                    J[:, i] = (residual_fn(xf) - r0) / actual_h
        return J

    return _jac


def _close_sim(sim: Any) -> None:
    exit_fn = getattr(sim, "__exit__", None)
    if callable(exit_fn):
        exit_fn(None, None, None)
        return
    close_fn = getattr(sim, "close", None)
    if callable(close_fn):
        close_fn()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _repo_root() -> Path:
    # host/robot_radio/calibration/fit_sim_error_model.py -> repo root is 3
    # parents up (calibration -> robot_radio -> host -> repo root).
    return Path(__file__).resolve().parents[3]


def _default_sim_factory() -> Callable[[], Any]:
    """Build a sim-factory usable by the CLI's ``record``/``fit`` modes.

    Lazily imports ``firmware.Sim`` (``tests/_infra/sim/firmware.py``),
    inserting that directory onto ``sys.path`` first (mirrors
    ``tests/conftest.py``'s own sys.path setup) — this is dev-repo-only
    tooling, never shipped as part of the installed ``robot_radio`` package,
    so the import is deferred to first CLI use rather than module import
    time.
    """
    sim_dir = _repo_root() / "tests" / "_infra" / "sim"
    if str(sim_dir) not in sys.path:
        sys.path.insert(0, str(sim_dir))
    from firmware import Sim  # noqa: PLC0415

    def _factory() -> Any:
        s = Sim()
        s.send_command("SET sTimeout=60000")
        s.enable_otos_model()
        s.set_otos_fusion(True)
        return s

    return _factory


def _parse_kv_string(s: str) -> dict[str, float]:
    out: dict[str, float] = {}
    for part in s.split(","):
        part = part.strip()
        if not part:
            continue
        k, _, v = part.partition("=")
        out[k.strip()] = float(v)
    return out


def _build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="fit_sim_error_model",
        description="Sim-error-model host fit tooling (069-008): record a "
                     "Sim() maneuver, fit SIMSET parameters against it, and "
                     "push a fitted parameter file back over the wire.",
    )
    sub = p.add_subparsers(dest="mode", required=True)

    rec_p = sub.add_parser(
        "record", help="Drive a Sim() through the default maneuver, emit a JSONL recording."
    )
    rec_p.add_argument("--out", required=True, type=Path, help="Output JSONL path.")
    rec_p.add_argument(
        "--inject", default=None,
        help="Optional 'key=value,key=value' SIMSET params to inject before "
             "recording (for sim-to-sim testing of this tool itself).",
    )
    rec_p.add_argument("--total", type=int, default=None)
    rec_p.add_argument("--sample-period", type=int, default=100)

    fit_p = sub.add_parser("fit", help="Fit SIMSET params against a recording.")
    fit_p.add_argument("--recording", required=True, type=Path, help="Input JSONL path.")
    fit_p.add_argument("--out", required=True, type=Path, help="Output fitted-parameter JSON path.")
    fit_p.add_argument(
        "--params", default=None,
        help="Comma-separated candidate SIMSET keys (default: the "
             "deterministic/bias-shaped subset, DEFAULT_CANDIDATE_KEYS).",
    )
    fit_p.add_argument("--total", type=int, default=None)
    fit_p.add_argument("--sample-period", type=int, default=100)

    push_p = sub.add_parser(
        "push", help="Push a fitted parameter file to a live connection as one SIMSET command."
    )
    push_p.add_argument("--params-file", required=True, type=Path)
    group = push_p.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "--sim", action="store_true",
        help="Push to a fresh, throwaway Sim() (smoke test / demo only).",
    )
    group.add_argument("--port", default=None, help="Serial port for a real connection.")
    push_p.add_argument(
        "--direct", action="store_true",
        help="Treat --port as a direct robot connection (no relay handshake).",
    )

    return p


def main(argv: Optional[list[str]] = None) -> int:
    args = _build_arg_parser().parse_args(argv)

    if args.mode == "record":
        inject = _parse_kv_string(args.inject) if args.inject else {}
        sim = _default_sim_factory()()
        try:
            if inject:
                apply_params(sim, inject)
            commands, default_total_duration = default_maneuver()
            total_duration = args.total if args.total is not None else default_total_duration
            records = record_sim_run(sim, commands, total_duration, args.sample_period)
        finally:
            _close_sim(sim)
        save_jsonl(records, args.out)
        print(f"Wrote {len(records)} records to {args.out}")
        return 0

    if args.mode == "fit":
        records = load_jsonl(args.recording)
        candidate_keys = (
            tuple(k.strip() for k in args.params.split(",")) if args.params
            else DEFAULT_CANDIDATE_KEYS
        )
        result = fit_sim_error_model(
            records,
            sim_factory=_default_sim_factory(),
            candidate_keys=candidate_keys,
            total_duration=args.total,
            sample_period=args.sample_period,
        )
        save_param_file(args.out, result.params)
        status = "converged" if result.success else "DID NOT CONVERGE"
        print(f"Fit {status}: cost={result.cost:.6f} nfev={result.nfev} ({result.message})")
        for k, v in result.params.items():
            print(f"  {k} = {v:.4f}")
        print(f"Wrote fitted parameters to {args.out}")
        return 0 if result.success else 1

    if args.mode == "push":
        params = load_param_file(args.params_file)
        if args.sim:
            conn = _default_sim_factory()()
        else:
            from robot_radio.calibration._conn_helpers import make_serial_conn  # noqa: PLC0415
            ser = make_serial_conn(args.port, args.direct)
            conn = WriteLineAdapter(ser)
        reply = push_params(conn, params)
        print(f"SIMSET reply: {reply}")
        return 0

    return 1  # pragma: no cover — argparse `required=True` prevents this


if __name__ == "__main__":
    raise SystemExit(main())
