"""src/tests/sim/turn_windage_sweep.py -- predict where the BIG HUMP of a
managed turn lands, and fit a windage that sizes it to land on target
(stakeholder 2026-07-18: "do the big hump and stop; predict how far the big
hump is going").

The "big hump" is the main jerk-limited pivot profile. In the shipped
config it OVERSHOOTS: the plant arrives at the target still carrying angular
velocity and coasts PAST it (peak excursion), then the PD rings it back down
with the "little humps" the stakeholder wants gone. The landing that matters
is the PEAK EXCURSION -- where the plant's own angular velocity first
returns to zero after the velocity peak (stakeholder: "the first minimum
after the big maximum ... that's the number of degrees you actually
turned"), measured BEFORE any ring/top-up correction.

Method
------
For every commanded angle 3..360 deg (3-deg steps) at pivot cruise rates
yaw_rate_max 1..4 rad/s, run a managed Move(delta_heading) in the
full-compute sim (fresh boot per run -- see ANGLES), and record:

  - big_hump_deg: TRUE heading (SimPlant ground truth) at the peak excursion
    -- the first cycle the commanded-direction angular velocity crosses back
    through zero after its peak. THE prediction target.
  - final_deg: true heading once the Move's completion event fires (what the
    ring/top-up correction layer eventually settles on).

The fit predicts the overshoot
    over(angle, rate) = big_hump_deg - angle
so a caller can size the planned pivot to land the big hump on target. A
SECOND sweep commands angle - over_pred(angle, rate) and re-measures the
big-hump landing -- landing back on `angle` is the proof the predictor
inverts (the firmware still rings toward the biased command afterward; that
is the separate completion-at-peak change, not what this validates).

Outputs (consumed by src/tests/notebooks/turn_windage.ipynb):
  src/tests/notebooks/out/turn_windage_baseline.csv
  src/tests/notebooks/out/turn_windage_compensated.csv
  src/tests/notebooks/out/turn_windage_fit.json

Run:  uv run python src/tests/sim/turn_windage_sweep.py  (a few minutes)
Progress goes to stderr; the sim's own HOST_BUILD encoder trace floods
stdout, so pipe stdout to /dev/null.
"""
from __future__ import annotations

import base64
import csv
import json
import math
import pathlib
import sys

import numpy as np

from robot_radio.io.sim_loop import SimLoop
from robot_radio.testgui.transport import _sim_lib_path
from robot_radio.robot.pb2 import config_pb2, envelope_pb2, telemetry_pb2

OUT_DIR = pathlib.Path(__file__).resolve().parents[1] / "notebooks" / "out"

RATES = (1.0, 2.0, 3.0, 4.0)   # [rad/s] yaw_rate_max sweep
# 3-deg steps (120 angles x 4 rates = 480 runs): each run gets a FRESH
# SimLoop/boot -- reusing one loop across many moves was found (2026-07-18)
# to accumulate executor state that degrades the rotational solve within
# ~5 consecutive pivots (culminating in ACK_STATUS_SOLVE_FAIL -- filed as a
# real firmware finding, see the session log); a fresh boot per run keeps
# every sample independent at the cost of coarser angle resolution.
ANGLES = range(3, 361, 3)       # [deg] commanded delta_heading sweep
MAX_CYCLES = 500                # 25s sim per run -- generous bound

TRACK = 128.0


def config_line(**gains) -> str:
    delta = envelope_pb2.ConfigDelta(motor=config_pb2.MotorConfigPatch(**gains))
    env = envelope_pb2.CommandEnvelope(corr_id=7, config=delta)
    return "*B" + base64.b64encode(env.SerializeToString()).decode("ascii")


def log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


def run_one(rate: float, command_deg: float) -> dict:
    """One managed pivot on a FRESH boot; returns first-rest / final headings
    [deg] (see ANGLES' own comment for why fresh-per-run)."""
    loop = SimLoop(track_width=TRACK, lib_path=_sim_lib_path())
    loop.connect(start_tick_thread=False)
    try:
        loop.inject_command(config_line(kp=0.002, ki=0.0, kff=0.002, i_max=0.0, kaw=0.0))
        loop.set_yaw_rate_max(rate)
        loop.step(3)
        loop.drain_pending_tlm()
        return _run_move(loop, command_deg)
    finally:
        loop.disconnect()


def _run_move(loop: SimLoop, command_deg: float) -> dict:
    """Record the full (heading, per-cycle rate) trace, then detect the big-
    hump peak excursion offline (robust to dither)."""
    sign = 1.0 if command_deg >= 0 else -1.0
    move_id = loop.move(distance=0.0, delta_heading=math.radians(command_deg), v_max=0.0)

    h_prev = 0.0
    unwrapped = 0.0
    heads: list[float] = []   # [rad] unwrapped heading per cycle
    rates: list[float] = []   # [rad/cycle] signed per-cycle delta (angular velocity proxy)
    terminal = None

    for cycle in range(MAX_CYCLES):
        loop.step(1)
        h = loop.get_true_pose()["h"]
        dh = h - h_prev
        if dh > math.pi:
            dh -= 2 * math.pi
        elif dh < -math.pi:
            dh += 2 * math.pi
        unwrapped += dh
        h_prev = h
        heads.append(unwrapped)
        rates.append(dh)

        for f in loop.drain_pending_tlm():
            for ack in (f.acks or ()):
                if ack.corr_id == move_id and ack.status != telemetry_pb2.ACK_STATUS_OK:
                    terminal = ack
        if terminal is not None:
            break

    status = telemetry_pb2.AckStatus.Name(terminal.status) if terminal else "NONE"
    if terminal is None:
        loop.stop()
        loop.step(20)
        loop.drain_pending_tlm()

    big_hump = _peak_excursion(heads, rates, sign)
    return {
        "big_hump_deg": math.degrees(big_hump) if big_hump is not None else float("nan"),
        "final_deg": math.degrees(unwrapped),
        "status": status,
        "cycles": cycle + 1,
    }


def _peak_excursion(heads: list[float], rates: list[float], sign: float):
    """Heading [rad] at the end of the big hump = the peak excursion: the
    first cycle the commanded-direction angular velocity crosses back through
    zero after its peak. Returns None if the trace never gets moving."""
    if not rates:
        return None
    sv = [sign * r for r in rates]           # commanded-direction rate
    vp = max(range(len(sv)), key=lambda i: sv[i])
    if sv[vp] <= 0.0:
        return None                          # never moved in the commanded dir
    # First post-peak cycle where the commanded-direction rate is <= 0 -- the
    # plant has stopped advancing toward target (peak excursion at vp..i-1).
    for i in range(vp + 1, len(sv)):
        if sv[i] <= 0.0:
            return heads[i - 1]
    # No turnaround (pure undershoot that never rings): the last sample is the
    # furthest it got.
    return heads[-1]


def sweep(name: str, windage_fn) -> list[dict]:
    """Full angle x rate sweep, one fresh boot per run. windage_fn(angle_deg,
    rate) -> commanded bias [deg] added to the wire command (0 baseline)."""
    rows: list[dict] = []
    for rate in RATES:
        for angle in ANGLES:
            bias = windage_fn(float(angle), rate)
            r = run_one(rate, float(angle) + bias)
            r.update(angle_deg=angle, rate=rate, commanded_deg=float(angle) + bias)
            rows.append(r)
            if angle % 60 == 0:
                log(f"  {name}: rate={rate} angle={angle} "
                    f"big_hump={r['big_hump_deg']:.2f} final={r['final_deg']:.2f} {r['status']}")
        log(f"{name}: rate={rate} done")
    return rows


def save_csv(path: pathlib.Path, rows: list[dict]) -> None:
    fields = ["angle_deg", "rate", "commanded_deg", "big_hump_deg", "final_deg", "status", "cycles"]
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)
    log(f"wrote {path} ({len(rows)} rows)")


def fit_windage(rows: list[dict]) -> dict:
    """over(angle, rate) = (a + b*rate) * (1 - exp(-angle/theta0)) -- the
    big-hump OVERSHOOT [deg], saturating at large angle (where the profile
    reaches cruise) and bending toward 0 at small angle. theta0
    grid-searched; (a, b) closed-form least squares per theta0."""
    ok = [r for r in rows if r["status"] == "ACK_STATUS_DONE" and not math.isnan(r["big_hump_deg"])]
    angle = np.array([r["angle_deg"] for r in ok], dtype=float)
    rate = np.array([r["rate"] for r in ok], dtype=float)
    over = np.array([r["big_hump_deg"] - r["angle_deg"] for r in ok], dtype=float)

    best = None
    for theta0 in np.arange(2.0, 120.0, 1.0):
        s = 1.0 - np.exp(-angle / theta0)
        X = np.column_stack([s, s * rate])
        coef, *_ = np.linalg.lstsq(X, over, rcond=None)
        resid = over - X @ coef
        sse = float(resid @ resid)
        if best is None or sse < best["sse"]:
            best = {"a": float(coef[0]), "b": float(coef[1]), "theta0": float(theta0),
                    "sse": sse, "rms": float(np.sqrt(sse / len(over)))}
    # Per-rate plateau means (angle >= 90) for the notebook's summary table.
    plateaus = {}
    for rt in RATES:
        m = (rate == rt) & (angle >= 90)
        plateaus[str(rt)] = float(over[m].mean()) if m.any() else float("nan")
    best["plateau_by_rate"] = plateaus
    best["n"] = len(ok)
    return best


def build_table(rows: list[dict]) -> dict:
    """The predictor proper: the measured big-hump overshoot itself, on the
    swept (angle, rate) grid -- the smooth deterministic wave a smooth
    closed-form curve can't hold (see fit_windage()'s exp model, rms ~3deg,
    vs this table's own ~1deg). Bilinear-interpolated by predict_overshoot().
    This is calibration data, the same kind rotation_gain/offset already are;
    a hardware fit would replace these numbers with a bench sweep's."""
    ok = [r for r in rows if r["status"] == "ACK_STATUS_DONE" and not math.isnan(r["big_hump_deg"])]
    angles = sorted({r["angle_deg"] for r in ok})
    table = {}
    for rt in RATES:
        by_angle = {r["angle_deg"]: r["big_hump_deg"] - r["angle_deg"] for r in ok if r["rate"] == rt}
        table[str(rt)] = [by_angle.get(a, float("nan")) for a in angles]
    return {"angle_grid": angles, "rates": list(RATES), "overshoot": table}


def predict_overshoot(table: dict, angle: float, rate: float) -> float:
    """Bilinear interpolation of the overshoot table at (angle, rate) [deg]."""
    ag = np.array(table["angle_grid"], dtype=float)
    rs = np.array(table["rates"], dtype=float)
    rate = min(max(rate, rs[0]), rs[-1])
    j = int(np.searchsorted(rs, rate))
    j = min(max(j, 1), len(rs) - 1)
    r0, r1 = rs[j - 1], rs[j]
    t = (rate - r0) / (r1 - r0) if r1 > r0 else 0.0
    o0 = float(np.interp(angle, ag, table["overshoot"][str(r0)]))
    o1 = float(np.interp(angle, ag, table["overshoot"][str(r1)]))
    return o0 * (1.0 - t) + o1 * t


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    log("=== baseline sweep (no windage) ===")
    baseline = sweep("baseline", lambda angle, rate: 0.0)
    save_csv(OUT_DIR / "turn_windage_baseline.csv", baseline)

    fit = fit_windage(baseline)
    log(f"exp fit (smooth, for reference): overshoot = ({fit['a']:.3f} + {fit['b']:.3f}*rate) "
        f"* (1 - exp(-angle/{fit['theta0']:.0f})) [deg], rms={fit['rms']:.3f}")
    (OUT_DIR / "turn_windage_fit.json").write_text(json.dumps(fit, indent=2) + "\n")

    table = build_table(baseline)
    (OUT_DIR / "turn_windage_table.json").write_text(json.dumps(table, indent=1) + "\n")
    log(f"table predictor built ({len(table['angle_grid'])} angles x {len(table['rates'])} rates)")

    def windage(angle: float, rate: float) -> float:
        # Command LESS by the predicted overshoot -> the big hump lands on
        # `angle`. (The firmware still rings toward the biased command
        # afterward -- see this module's docstring; this validates that the
        # predictor INVERTS, i.e. the big-hump landing lands back on target.)
        return -predict_overshoot(table, angle, rate)

    log("=== compensated sweep (windage applied) ===")
    compensated = sweep("compensated", windage)
    save_csv(OUT_DIR / "turn_windage_compensated.csv", compensated)

    ok_b = [r for r in baseline if r["status"] == "ACK_STATUS_DONE"]
    ok_c = [r for r in compensated if r["status"] == "ACK_STATUS_DONE"]
    eb = np.array([r["big_hump_deg"] - r["angle_deg"] for r in ok_b])
    ec = np.array([r["big_hump_deg"] - r["angle_deg"] for r in ok_c])
    log(f"baseline    big-hump overshoot: mean={np.nanmean(eb):+.3f} rms={np.sqrt(np.nanmean(eb**2)):.3f} "
        f"worst={np.nanmax(np.abs(eb)):.3f} deg (n={len(ok_b)}, timeouts={len(baseline)-len(ok_b)})")
    log(f"compensated big-hump overshoot: mean={np.nanmean(ec):+.3f} rms={np.sqrt(np.nanmean(ec**2)):.3f} "
        f"worst={np.nanmax(np.abs(ec)):.3f} deg (n={len(ok_c)}, timeouts={len(compensated)-len(ok_c)})")


if __name__ == "__main__":
    main()
