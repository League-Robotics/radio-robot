"""src/tests/tools/turn_events.py -- turn-prediction campaign, Phase A:
quantify the firmware's own angle-stop overshoot / stop-detection lag
DIRECTLY from a captured tlm_log-shaped telemetry stream, independent of
``one_step_ahead.py``'s general ZOH-prediction machinery.

Where ``one_step_ahead.py`` answers "how good is a ZOH prediction at lead
X, in general", this module answers a narrower, domain-specific question
straight from measured data: for EACH commanded 90-degree (or any other
magnitude) turn in a capture, how long after the MEASURED heading crossed
the commanded threshold did the robot's own angular rate actually settle
back to ~0, and how many degrees past the threshold did it travel during
that window? Those two numbers (``lag``, ``overshoot``) are exactly the
"constant ~150-180ms stop-detection lag" this campaign's own diagnosis
names -- this module is what turns raw telemetry into a measured
distribution of them, rather than a single hand-derived estimate.

This module is pure computation over already-parsed ``(times, headings,
omegas)`` streams (the SAME shape ``one_step_ahead.heading_stream_from_
rows()`` already produces) plus a manifest of which turns were issued when
-- see ``src/tests/bench/turn_prediction_capture.py`` for the capture side
that produces both. No I/O, no sim/hardware dependency, independently
testable with hand-fed arrays.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class TurnEvent:
    """One measured turn's own crossing/settle timing and overshoot --
    see this module's own header for the definitions. ``direction`` is
    +1.0 (CCW, positive omega) or -1.0 (CW, negative omega); every angle/
    heading field below is expressed as a MAGNITUDE along that direction
    (i.e. already sign-corrected), so ``overshoot`` is positive for the
    expected "traveled further than commanded" case and negative for an
    (unexpected, but not asserted-against) undershoot."""

    label: str
    move_id: int
    direction: float
    target: float           # [rad] commanded |delta heading|
    heading_start: float    # [rad] measured heading at the turn's own activation
    t_cross: float           # [ms] first instant measured heading reaches `target`
    t_settle: float          # [ms] first instant afterward omega has settled (see find_turn_events())
    lag: float                # [ms] t_settle - t_cross
    overshoot: float          # [rad] signed-corrected |heading(t_settle) - heading_start| - target


def find_turn_events(
    times: "Sequence[float]", headings: "Sequence[float]", omegas: "Sequence[float]",
    manifest: "Sequence[dict]", *,
    settle_omega: float = 0.05,   # [rad/s] |omega| at/below this counts as "settled"
    settle_dwell: float = 80.0,  # [ms] omega must stay settled for at least this long, continuously
    search_margin: float = 3000.0,  # [ms] search window past the LAST turn's own issue instant
) -> "list[TurnEvent]":
    """``times``/``headings``/``omegas`` are the SAME parallel, already
    unit-converted (rad, rad/s) arrays ``one_step_ahead.heading_stream_
    from_rows()`` returns. ``manifest`` is ``turn_prediction_capture.py``'s
    own per-turn bookkeeping (a list of dicts; entries with
    ``kind != "turn"`` -- e.g. a trailing Tour 1 summary row -- are
    ignored), IN ISSUE ORDER, each carrying ``label``/``move_id``/
    ``omega``/``target_rad``/``issue_now_ms`` (the robot-clock instant the
    capture script drained immediately before injecting this turn's own
    Move -- a lower bound on when it actually activated).

    Each turn's own search window is ``[issue_now_ms, next_turn's own
    issue_now_ms)`` (or ``+search_margin`` for the last turn) -- bounding
    the search this way, from the capture script's own issue-order
    bookkeeping, is what keeps consecutive turns' telemetry from bleeding
    into each other; it does NOT depend on any completion-ack timing (this
    module never reads ``ack_corr``/``ack_err`` at all -- it works
    entirely off measured heading/omega, independent of whatever the
    firmware's own stop-condition decision was, which is the whole point
    of an INDEPENDENT verification).

    Within a window: ``heading_start`` is the heading at the window's own
    first sample. ``t_cross`` is the first instant the measured heading,
    signed by the turn's own direction, has moved `target` past
    `heading_start`. ``t_settle`` is the first instant AT OR AFTER
    `t_cross` where `|omega| <= settle_omega` holds CONTINUOUSLY for at
    least `settle_dwell` ms (a single instantaneous zero-crossing of a
    still-oscillating rate does not count -- this is deliberately the same
    "leaky/decaying, not one-miss-resets" caution
    `executor.cpp`'s own dwell-completion logic used, see that module's
    comment, applied here to a measurement rather than a control decision).

    A turn whose window never reaches `target` (`t_cross` not found) or
    never settles afterward (`t_settle` not found) is SKIPPED -- omitted
    from the returned list entirely, not reported with a placeholder --
    matching `group_rms_by_phase()`'s own "no data, not zero" convention
    (`one_step_ahead.py`). This should not happen for a well-formed 90-
    degree-turn capture; if it does, the caller's own row count for that
    turn is one fewer than the manifest's turn count, which is itself the
    diagnostic (rather than raising and losing every OTHER turn's data)."""
    turns = [m for m in manifest if m.get("kind") == "turn"]
    events: "list[TurnEvent]" = []

    for idx, spec in enumerate(turns):
        window_start = spec["issue_now_ms"]
        window_end = (turns[idx + 1]["issue_now_ms"] if idx + 1 < len(turns)
                     else window_start + search_margin)

        window_idx = [i for i, t in enumerate(times) if window_start <= t < window_end]
        if not window_idx:
            continue

        heading_start = headings[window_idx[0]]
        direction = math.copysign(1.0, spec["omega"])
        target = spec["target_rad"]

        t_cross = None
        cross_pos = None
        for pos, i in enumerate(window_idx):
            if direction * (headings[i] - heading_start) >= target:
                t_cross = times[i]
                cross_pos = pos
                break
        if t_cross is None:
            continue

        after = window_idx[cross_pos:]
        t_settle = None
        heading_at_settle = None
        for pos, i in enumerate(after):
            if abs(omegas[i]) > settle_omega:
                continue
            t0 = times[i]
            dwell_ok = True
            for j in after[pos:]:
                if times[j] - t0 >= settle_dwell:
                    break
                if abs(omegas[j]) > settle_omega:
                    dwell_ok = False
                    break
            if dwell_ok:
                t_settle = t0
                heading_at_settle = headings[i]
                break
        if t_settle is None:
            continue

        overshoot = direction * (heading_at_settle - heading_start) - target
        events.append(TurnEvent(
            label=spec.get("label", f"turn[{idx}]"), move_id=spec["move_id"],
            direction=direction, target=target, heading_start=heading_start,
            t_cross=t_cross, t_settle=t_settle, lag=t_settle - t_cross,
            overshoot=overshoot))

    return events


def summarize_lag_and_overshoot(events: "Sequence[TurnEvent]") -> dict:
    """Plain-number distribution summary (min/max/mean/count, both for
    ``lag`` [ms] and ``overshoot`` [deg]) -- the notebook's own conclusion
    cell reads this rather than re-deriving it from the raw ``events``
    list. Returns an all-``None``/zero-count dict for an empty ``events``
    (documented, not an error -- mirrors ``one_step_ahead.rms()``'s own
    empty-input contract)."""
    if not events:
        return {"count": 0, "lag_ms": None, "overshoot_deg": None}

    lags = [e.lag for e in events]
    overshoots_deg = [math.degrees(e.overshoot) for e in events]
    return {
        "count": len(events),
        "lag_ms": {
            "min": min(lags), "max": max(lags), "mean": sum(lags) / len(lags),
        },
        "overshoot_deg": {
            "min": min(overshoots_deg), "max": max(overshoots_deg),
            "mean": sum(overshoots_deg) / len(overshoots_deg),
        },
    }
