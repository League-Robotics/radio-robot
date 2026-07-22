"""src/tests/tools/one_step_ahead.py -- sprint 117 ticket 006: a pure-Python,
independently-testable reference implementation of the SAME zero-order-hold
(ZOH) one-step-ahead prediction math ``App::StateEstimator`` ships in C++
(ticket 002, ``src/firm/app/state_estimator.{h,cpp}``).

This is NOT a wrapper calling into the C++ estimator -- a genuinely separate
reimplementation, so a cross-check against it (ticket 007's notebook; ticket
005's own optional stretch replay harness) is a real independent check, not
the estimator agreeing with itself. The formula mirrored here, verbatim, is
``state_estimator.cpp``'s own ``wheelAt()``/``bodyAt()``::

    age = (t - basisTime) / 1000.0          # [ms] -> [s]
    distance = basis.distance + basis.velocity * age    # wheel stream
    heading  = basis.heading  + basis.omega  * age       # body heading stream

Both streams (a wheel's position/velocity/time, or the body's heading/
omega/time) are the SAME shape -- one quantity, its own rate, and a
timestamp -- so this module has ONE generic pure function
(``one_step_ahead_walk()``) operating on parallel ``(times, positions,
velocities)`` sequences, used identically for either stream. No pandas/numpy
dependency (plain Python, matching ``robot_radio.robot.clock_sync``'s own
no-external-dependency precedent -- see that module's ``_fit_skew()`` doc
comment) -- ticket 007's notebook has not been built yet, so there is no
established pandas dependency to align with; this module stays dependency-free
regardless.

Leave-one-out walk (this ticket's own vocabulary, matching ticket 005's
identical usage): for each sample ``k`` (``k = 1 .. N-1``), predict its own
value from the IMMEDIATELY PRECEDING sample's basis (``k-1``) extrapolated
forward to sample ``k``'s own time -- sample ``k``'s own position/velocity
are used ONLY as the "actual" value the prediction is checked against, never
as part of computing that prediction (excludes exactly one sample -- itself
-- from its own basis). Sample ``0`` has no preceding basis and produces no
residual; an ``N``-sample stream therefore walks to exactly ``N-1``
residuals. A stream of 0 or 1 samples produces an empty walk (documented, not
an error -- there is nothing to leave out).

Monotonicity: ``times`` must be non-decreasing (``times[k] >= times[k-1]``),
mirroring ``state_estimator.h``'s own ``wheelAt()``/``bodyAt()`` precondition
("t is at or after the queried peer's own basisTime"). A STRICTLY decreasing
timestamp is REJECTED (``ValueError``) rather than silently producing a
negative age -- a captured TLM stream should never actually go backward in
time; if one does, that is a data problem worth surfacing, not smoothing
over silently.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


# ---------------------------------------------------------------------------
# Core ZOH one-step-ahead walk
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Residual:
    """One leave-one-out prediction outcome for sample ``k`` (``k >= 1``).

    ``time`` is sample ``k``'s OWN timestamp (the instant being predicted
    TO, not the basis instant it was predicted FROM) -- the natural key for
    grouping residuals by pattern phase (``group_rms_by_phase()`` below)."""

    time: float        # [ms] the predicted-to sample's own timestamp
    predicted: float    # basis.position + basis.velocity * age
    actual: float        # the predicted-to sample's own recorded position
    residual: float      # actual - predicted, same unit as position/actual


def one_step_ahead_walk(times: "Sequence[float]", positions: "Sequence[float]",
                        velocities: "Sequence[float]") -> "list[Residual]":
    """Leave-one-out, one-step-ahead ZOH walk over one stream -- see this
    module's own header for the full contract (formula, leave-one-out
    definition, monotonicity precondition, empty/single-sample handling).

    ``times``/``positions``/``velocities`` are parallel sequences (index
    ``i`` is one reading: ``(times[i], positions[i], velocities[i])``) of
    equal length -- ``velocities[i]`` is the reading's own rate, HELD
    CONSTANT under ZOH extrapolation from that reading's own basis, exactly
    like ``WheelEstimate``/``BodyEstimate``'s own ``velocity``/``omega``
    field (``state_estimator.h``).

    Raises ``ValueError`` if the three sequences have unequal length, or if
    any ``times[k] < times[k-1]`` (non-monotonic -- see module header).
    """
    n = len(times)
    if len(positions) != n or len(velocities) != n:
        raise ValueError(
            f"one_step_ahead_walk(): times/positions/velocities must have equal length, "
            f"got {n}/{len(positions)}/{len(velocities)}")

    for k in range(1, n):
        if times[k] < times[k - 1]:
            raise ValueError(
                f"one_step_ahead_walk(): non-monotonic timestamps at index {k} "
                f"(times[{k - 1}]={times[k - 1]!r} > times[{k}]={times[k]!r}) -- a captured "
                f"stream must never go backward in time")

    walk: "list[Residual]" = []
    for k in range(1, n):
        age = (times[k] - times[k - 1]) / 1000.0  # [ms] -> [s], matches state_estimator.cpp's own age math
        predicted = positions[k - 1] + velocities[k - 1] * age
        actual = positions[k]
        walk.append(Residual(time=times[k], predicted=predicted, actual=actual,
                             residual=actual - predicted))
    return walk


# ---------------------------------------------------------------------------
# RMS + phase grouping (AC: "RMS grouping helpers by pattern phase")
# ---------------------------------------------------------------------------

def rms(values: "Sequence[float]") -> float:
    """Root-mean-square of *values*. ``0.0`` for an empty sequence
    (documented -- an empty bucket has no error to report, not a
    divide-by-zero)."""
    if not values:
        return 0.0
    return (sum(v * v for v in values) / len(values)) ** 0.5


@dataclass(frozen=True)
class Phase:
    """One named time window (both ends INCLUSIVE), in the SAME clock domain
    as the walk's own ``Residual.time`` -- e.g. a captured CSV's own robot-
    clock ``now``/``enc_left_time``/``otos_time`` column, per this module's
    own header note on why phase boundaries should come from that clock, not
    a host wall-clock schedule."""

    label: str
    start: float  # [ms] inclusive
    end: float    # [ms] inclusive


def group_rms_by_phase(walk: "Sequence[Residual]",
                       phases: "Sequence[Phase]") -> "dict[str, float]":
    """Buckets each ``walk`` residual into whichever ``phases`` window
    contains its OWN ``time`` (the predicted-to sample's timestamp -- "this
    residual belongs to whatever pattern phase was active when the sample it
    predicts was taken"), then returns ``{phase.label: rms(residuals in that
    phase)}``.

    A phase with ZERO residuals landing in its window is OMITTED from the
    returned dict entirely (not reported as ``0.0``) -- a 0.0 RMS would
    misleadingly read as "perfect tracking" rather than "no data". Overlapping
    phase windows are not rejected (a caller's own responsibility to avoid,
    if it matters for their analysis) -- a residual whose time falls in more
    than one window is counted in EVERY window it falls in.
    """
    buckets: "dict[str, list[float]]" = {p.label: [] for p in phases}
    for r in walk:
        for p in phases:
            if p.start <= r.time <= p.end:
                buckets[p.label].append(r.residual)

    return {label: rms(residuals) for label, residuals in buckets.items() if residuals}


# ---------------------------------------------------------------------------
# CSV-row convenience extraction -- matches tlm_log.py's own CSV_FIELDNAMES
# column names exactly (src/tests/bench/tlm_log.py), so a caller (ticket
# 007's notebook, or a script reading estimator_capture.py's own output) can
# hand this module a plain list of CSV DictReader rows with no glue code of
# its own. Pure (no I/O) -- these functions accept already-parsed rows (str
# values from csv.DictReader are converted to float here); reading the CSV
# file itself is the caller's job.
# ---------------------------------------------------------------------------

def _to_float(value) -> "float | None":
    """``None``/empty-string safe float conversion -- a CSV cell tlm_log.py's
    own ``frame_to_row()`` wrote as ``None`` (e.g. ``otos_*`` when
    ``otos_present`` was clear) round-trips through ``csv.DictReader`` as
    ``""``; both convert to ``None`` here, never raise."""
    if value is None or value == "":
        return None
    return float(value)


def wheel_stream_from_rows(rows: "Sequence[dict]", side: str) -> "tuple[list[float], list[float], list[float]]":
    """Extract ``(times, positions, velocities)`` for one wheel
    (``side="left"`` or ``"right"``) from a sequence of CSV row dicts shaped
    like ``tlm_log.CSV_FIELDNAMES`` (``enc_left_position``/
    ``enc_left_velocity``/``enc_left_time``, or the ``_right_`` equivalents).
    Rows whose encoder reading is absent (``None``/empty -- should not
    happen for ``enc_*``, which is unconditionally present per
    ``frame_to_row()``'s own doc comment, but handled defensively) are
    skipped rather than raising."""
    if side not in ("left", "right"):
        raise ValueError(f'wheel_stream_from_rows(): side must be "left" or "right", got {side!r}')

    times: "list[float]" = []
    positions: "list[float]" = []
    velocities: "list[float]" = []
    for row in rows:
        t = _to_float(row.get(f"enc_{side}_time"))
        pos = _to_float(row.get(f"enc_{side}_position"))
        vel = _to_float(row.get(f"enc_{side}_velocity"))
        if t is None or pos is None or vel is None:
            continue
        times.append(t)
        positions.append(pos)
        velocities.append(vel)
    return times, positions, velocities


# tlm_log.py's ``pose_theta``/``twist_omega`` columns are NOT radians --
# ``robot_radio.robot.protocol.TLMFrame.from_pb2()`` wire-scales both to
# compact integers before ``frame_to_row()`` ever sees them:
# ``pose_theta`` = ``round(heading_rad * _ANGLE_SCALE)`` [cdeg], and
# ``twist_omega`` = ``round(omega_radps * 1000.0)`` [mrad/s]. Recomputed here
# from first principles (``degrees(1.0) * 100`` == ``protocol.py``'s own
# ``_ANGLE_SCALE = 5729.5779513``) rather than importing that module's
# private constant -- this is fixed unit-conversion math, not a project
# tuning value that could drift. Converting back to radians/rad-per-second
# here is what makes this module's output directly comparable to ticket
# 002's own C++ formula (which operates in radians throughout) -- getting
# this wrong would silently produce heading residuals ~5730x too large.
_CDEG_PER_RAD = math.degrees(1.0) * 100.0  # [cdeg/rad] matches protocol.py's own _ANGLE_SCALE
_MRAD_PER_RADPS = 1000.0                    # [mrad/s per rad/s]


def heading_stream_from_rows(rows: "Sequence[dict]") -> "tuple[list[float], list[float], list[float]]":
    """Extract ``(times, headings, omegas)`` for the body-heading stream
    from a sequence of CSV row dicts -- ``pose_theta`` (Odometry's own
    dead-reckoned heading) and ``twist_omega`` (the SAME cycle's fused
    body-frame angular rate, ``BodyKinematics::forward()``), timestamped by
    the row's own ``now`` column (the primary frame's own collect/emit
    time -- ``pose``/``twist`` are staged the SAME cycle ``now`` is stamped,
    ``robot_loop.cpp``'s own ``updateTlm()``/pace-block ordering). Rows
    where ``pose``/``twist`` are absent (a secondary-only frame -- see
    ``TLMFrame``'s own doc comment) are skipped.

    Converts ``pose_theta`` [cdeg] -> [rad] and ``twist_omega`` [mrad/s] ->
    [rad/s] on the way out (see the module-level ``_CDEG_PER_RAD``/
    ``_MRAD_PER_RADPS`` comment above) -- the returned ``headings``/
    ``omegas`` are in the SAME radian units ``one_step_ahead_walk()``'s own
    formula (and ticket 002's C++ source) uses throughout, ready to pass
    straight in with no further conversion."""
    times: "list[float]" = []
    headings: "list[float]" = []
    omegas: "list[float]" = []
    for row in rows:
        t = _to_float(row.get("now"))
        heading_cdeg = _to_float(row.get("pose_theta"))
        omega_mradps = _to_float(row.get("twist_omega"))
        if t is None or heading_cdeg is None or omega_mradps is None:
            continue
        times.append(t)
        headings.append(heading_cdeg / _CDEG_PER_RAD)
        omegas.append(omega_mradps / _MRAD_PER_RADPS)
    return times, headings, omegas
