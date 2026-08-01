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
DEFAULT_TOLERANCE = {
    "wheel_speed_left": 15.0,   # [mm/s]
    "wheel_speed_right": 15.0,
    "cmd_wheel_left": 10.0,
    "cmd_wheel_right": 10.0,
    "xy_trace": 15.0,           # [mm]
    "heading_t": 0.06,          # [rad]
    "x_t": 15.0,                # [mm]
    "y_t": 15.0,
    "cycle_period_t": 5.0,      # [ms]
    "cycle_busy_t": 5.0,
}
