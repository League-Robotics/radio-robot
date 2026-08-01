"""signals -- extract named, golden-comparable series from a run dataset.

One signal per golden image (umbrella issue: a combined plot cannot be
compared, because a divergence in one line contaminates the frame). Every
extractor consumes the JSONL record list and returns (x, y) arrays.

Time base: the robot's own clock (payload.t, [ms] -> [s], zeroed at the
first telemetry frame) -- host wall time never enters a golden signal.

Commanded-velocity caveat (recorded here, per the wheel-controller issue's
`cmd_vel` wire-gap note): the commanded step series is anchored at command
SEND time mapped onto the robot clock. One-leg lookahead sends a move
before it activates, so command edges lead actuation by up to one leg;
the exact per-cycle commanded value lands on the wire with the
cmdAccel/blackboard work and can replace this extractor then.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np


def load_dataset(path: str | Path) -> list[dict]:
    with open(path, encoding="utf-8") as f:
        return [json.loads(line) for line in f]


def dataset_meta(recs: list[dict]) -> dict:
    for r in recs:
        if r["type"] == "run_meta":
            return r["payload"]
    raise ValueError("dataset has no run_meta record")


def _tlm(recs: list[dict]) -> list[dict]:
    return [r for r in recs if r["type"] == "tlm"
            and r["payload"].get("t") is not None]


def extract_signals(recs: list[dict], *, track_width: float  # [mm]
                    ) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    tlm = _tlm(recs)
    if not tlm:
        return {}
    t0 = tlm[0]["payload"]["t"]  # [ms]

    def t_of(rec: dict) -> float:  # [s]
        return (rec["payload"]["t"] - t0) / 1000.0

    out: dict[str, tuple[np.ndarray, np.ndarray]] = {}

    def series(name: str, pairs: list[tuple[float, float]]) -> None:
        if pairs:
            arr = np.asarray(pairs, dtype=float)
            out[name] = (arr[:, 0], arr[:, 1])

    series("wheel_speed_left",
           [(t_of(r), r["payload"]["enc_left"]["velocity"]) for r in tlm
            if "enc_left" in r["payload"]])
    series("wheel_speed_right",
           [(t_of(r), r["payload"]["enc_right"]["velocity"]) for r in tlm
            if "enc_right" in r["payload"]])
    series("xy_trace",
           [(r["payload"]["pose"]["x"], r["payload"]["pose"]["y"])
            for r in tlm if "pose" in r["payload"]])
    series("heading_t",
           [(t_of(r), r["payload"]["pose"]["heading"]) for r in tlm
            if "pose" in r["payload"]])
    series("x_t", [(t_of(r), r["payload"]["pose"]["x"]) for r in tlm
                   if "pose" in r["payload"]])
    series("y_t", [(t_of(r), r["payload"]["pose"]["y"]) for r in tlm
                   if "pose" in r["payload"]])
    series("cycle_period_t",
           [(t_of(r), r["payload"]["cycle_period"] / 1000.0) for r in tlm
            if r["payload"].get("cycle_period")])  # [ms]
    series("cycle_busy_t",
           [(t_of(r), r["payload"]["cycle_busy"] / 1000.0) for r in tlm
            if r["payload"].get("cycle_busy") is not None])  # [ms]

    # Commanded wheel speeds: step series from tx MOVE/STOP records, send
    # time mapped host->robot clock via the nearest telemetry pair (see the
    # module docstring's lookahead caveat).
    host_t = np.asarray([r["t_host"] for r in tlm])
    robot_t = np.asarray([t_of(r) for r in tlm])
    cmd_l: list[tuple[float, float]] = []
    cmd_r: list[tuple[float, float]] = []
    for r in recs:
        if r["type"] != "cmd" or r["verb"] not in ("MOVE", "STOP", "ESTOP"):
            continue
        idx = int(np.searchsorted(host_t, r["t_host"]))
        t = float(robot_t[min(idx, len(robot_t) - 1)])
        p = r["payload"]
        if r["verb"] in ("STOP", "ESTOP"):
            v_l = v_r = 0.0
        elif p.get("v_left") is not None:
            v_l, v_r = p["v_left"], p["v_right"]
        else:
            v_x = p.get("v_x", 0.0)
            omega = p.get("omega", 0.0)  # [rad/s]
            v_l = v_x - omega * track_width / 2.0
            v_r = v_x + omega * track_width / 2.0
        cmd_l.append((t, v_l))
        cmd_r.append((t, v_r))
    series("cmd_wheel_left", cmd_l)
    series("cmd_wheel_right", cmd_r)
    return out


# Per-signal default tolerances for a first bless with no spread runs --
# replaced by suggest_tolerance() the moment N no-change runs exist.
# Canonical, FIXED plot domains -- (x_low, x_high, y_low, y_high).
#
# Axes are pinned per signal, never derived from the run's own extent
# (stakeholder, 2026-08-01). Autoscaling makes two runs incomparable: a short
# run and a long one render to visually similar images at different scales, so
# a difference in the robot shows up as no difference in the picture, and a
# difference in the picture may be nothing but a rescale.
#
# The time axis spans the LONGEST tour, not a typical one. 0..15 s was sized
# from the circle (12 s) and silently truncated the square (33 s): more than
# half of every time series fell off-plot, and since the band is only drawn
# where the trace is, the gate stopped covering the run at 15 s. Out-of-domain
# data is now a loud failure (assert_within_domain) rather than a quiet crop --
# but the domain still has to be big enough in the first place.
#
# Velocity axes are SYMMETRIC rather than 0..500. The stakeholder specified
# 0..500 for the forward case; symmetric costs half the vertical resolution but
# a reverse leg clipped off-plot would leave an empty band and a gate that
# cannot fail. Silently hiding data is the worse trade -- revisit if every tour
# is forward-only.
CANONICAL_DOMAIN = {
    #                     x_low  x_high  y_low   y_high
    "wheel_speed_left":  (0.0,  40.0,  -500.0,  500.0),   # [s] [mm/s]
    "wheel_speed_right": (0.0,   40.0,  -500.0,  500.0),
    "cmd_wheel_left":    (0.0,   40.0,  -500.0,  500.0),
    "cmd_wheel_right":   (0.0,   40.0,  -500.0,  500.0),
    "x_t":               (0.0,   40.0, -1000.0, 1000.0),   # [s] [mm]
    "y_t":               (0.0,   40.0, -1000.0, 1000.0),
    "heading_t":         (0.0,   40.0,    -7.0,    7.0),   # [s] [rad]
    "cycle_period_t":    (0.0,   40.0,     0.0,  100.0),   # [s] [ms]
    "cycle_busy_t":      (0.0,   40.0,     0.0,  100.0),
    # Square domain: the only signal whose axes share a unit, so it is the only
    # one where equal x/y extents are meaningful.
    "xy_trace":       (-1000.0, 1000.0, -1000.0, 1000.0),  # [mm] [mm]
}

# Per-axis acceptance tolerance -- each in ITS OWN axis's units.
#
# The x entry is SECONDS for every time series and MILLIMETRES for xy_trace.
# That distinction is the whole point: the previous single scalar was applied
# to both axes, so "15" meant 15 seconds horizontally on a 12-second run and
# the band swallowed the plot (96.8% coverage on wheel_speed_left, 100% on the
# cycle-timing signals). See golden_trace.AxisTolerance.
AXIS_TOLERANCE = {
    "wheel_speed_left":  (0.20, 15.0),   # [s] [mm/s]
    "wheel_speed_right": (0.20, 15.0),
    "cmd_wheel_left":    (0.20, 10.0),   # [s] [mm/s]
    "cmd_wheel_right":   (0.20, 10.0),
    "x_t":               (0.20, 15.0),   # [s] [mm]
    "y_t":               (0.20, 15.0),
    "heading_t":         (0.20, 0.06),   # [s] [rad]
    "cycle_period_t":    (0.20, 5.0),    # [s] [ms]
    "cycle_busy_t":      (0.20, 5.0),
    "xy_trace":          (15.0, 15.0),   # [mm] [mm] -- same unit both axes
}

#: Fallback when a signal is not in the table above. Kept so an unlisted signal
#: still renders, but it is a smell: add the signal to both tables instead.
DEFAULT_AXIS_TOLERANCE = (0.20, 10.0)


def assert_within_domain(name: str, x, y) -> "list[str]":
    """Report any sample outside the signal's canonical domain.

    Silent cropping is the failure this prevents: matplotlib happily draws a
    clipped trace, and because the acceptance band is only generated where the
    trace is, a truncated plot produces a band that simply stops -- so the gate
    quietly ceases to cover the tail of the run. Measured 2026-08-01: the square
    tour (33 s) against a 15 s domain lost more than half its samples this way,
    and every plot still looked plausible.

    Returns a list of human-readable complaints; empty means fully in-domain.
    """
    import numpy as np

    dom = CANONICAL_DOMAIN.get(name)
    if dom is None:
        return [f"{name}: no canonical domain"]
    x_lo, x_hi, y_lo, y_hi = dom
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    if x.size == 0:
        return []
    out = []
    n_x = int(np.count_nonzero((x < x_lo) | (x > x_hi)))
    n_y = int(np.count_nonzero((y < y_lo) | (y > y_hi)))
    if n_x:
        out.append(f"{name}: {n_x}/{x.size} samples outside x domain "
                   f"[{x_lo:g}, {x_hi:g}] (data {x.min():g}..{x.max():g})")
    if n_y:
        out.append(f"{name}: {n_y}/{y.size} samples outside y domain "
                   f"[{y_lo:g}, {y_hi:g}] (data {y.min():g}..{y.max():g})")
    return out
