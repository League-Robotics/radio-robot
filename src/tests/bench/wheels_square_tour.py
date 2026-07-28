#!/usr/bin/env python3
"""wheels_square_tour.py -- the square tour driven ENTIRELY by plain
wheel-speed commands (no planner, no shaping, no PID): 4 x 500 mm legs at
+150/+150 mm/s, 4 x 90 deg pivots at -120/+120 mm/s, each segment a single
time-bounded WHEELS command straight to App::Drive. Every
command is ack-verified with retry (DAPLink inbound loss workaround).

Output: a two-panel chart -- the XY path integrated from the encoders
(the square itself) and commanded-vs-measured wheel speeds over time.

    uv run python src/tests/bench/wheels_square_tour.py --port /dev/cu.usbmodem2121102
"""
from __future__ import annotations

import argparse
import math
import sys
import time

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
TRACK = 128.0          # [mm]
LEG = 500.0            # [mm]
CRUISE = 150.0         # [mm/s] leg speed, both wheels
TURN_SPEED = 120.0     # [mm/s] pivot wheel speed (above the crawl boundary)
TURN_ARC = TRACK * math.pi / 4.0  # [mm] per-wheel travel for a 90 deg pivot
BOOT_WAIT = 6.0        # [s]
TAU = 0.23             # [s] plant time constant (spin-up transient)
DWELL = 0.10           # [s] armor reversal dwell (sign-flip wheel holds 0)

# Per-wheel command correction (measured steady / commanded), measured by
# the warm-up segment at the start of EVERY run -- the plant's gain
# wanders with temperature/usage state, so baked constants overcorrect
# within a session. Filled in by calibrate().
CAL = {"L": 1.0, "R": 1.0}


def meanFactor(duration):  # [s] mean/steady speed ratio incl. spin-up
    if duration <= 0:
        return 1.0
    return 1.0 - (TAU / duration) * (1.0 - math.exp(-duration / TAU))


def durationFor(distance, steady, reversal):  # [mm] [mm/s] -> [s]
    # From-rest segment: commanded window covers (distance - coast); the
    # post-expiry coast (~steady * TAU) delivers the rest.
    target = max(distance - steady * TAU, distance * 0.3)
    d = target / steady
    for _ in range(6):  # fixed point: duration -> transient factor
        eff = d - (DWELL if reversal else 0.0)
        d = target / (steady * meanFactor(eff)) + (DWELL if reversal else 0.0)
    return d


def segments() -> list[tuple[float, float, float, int]]:
    """(vLeft, vRight, duration_ms, move_id) per segment. Commands are
    calibration-corrected so the MEASURED wheel speeds hit the targets;
    durations account for the spin-up transient (and the reversal dwell on
    the sign-flipping wheel during turns)."""
    legMs = durationFor(LEG, CRUISE, reversal=False) * 1000.0
    turnMs = durationFor(TURN_ARC, TURN_SPEED, reversal=True) * 1000.0
    legL = CRUISE / CAL["L"]
    legR = CRUISE / CAL["R"]
    turnL = -TURN_SPEED / CAL["L"]
    turnR = TURN_SPEED / CAL["R"]
    out = []
    for i in range(4):
        out.append((legL, legR, legMs, 9601 + 2 * i))
        out.append((turnL, turnR, turnMs, 9602 + 2 * i))
    return out


def sendVerified(proto, vL, vR, durationMs, moveId):
    for _ in range(4):
        corr = proto.wheels(v_left=vL, v_right=vR, duration=durationMs,
                            move_id=moveId)
        ack = proto.wait_for_ack(corr, timeout=400)
        if ack is not None and ack.ok:
            return True
    print(f"WARNING: segment {moveId} never acked")
    return False


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default=DEFAULT_PORT)
    args = p.parse_args()

    conn = SerialConnection(port=args.port)
    conn.connect()
    proto = NezhaProtocol(conn)
    print(f"connected; waiting {BOOT_WAIT:.0f}s for boot")
    time.sleep(BOOT_WAIT)
    proto.read_pending_binary_tlm_frames()

    # --- Empirical prelude (still just wheel commands + arithmetic) ---
    # (a) 3.5 s straight: per-wheel steady gain from POSITION SLOPES over a
    #     robot-clock window well inside the move (immune to ack-retry
    #     latency and coast-down -- the bug in the first version).
    def capture(durationS):
        rows = []
        t0c = time.monotonic()
        while time.monotonic() - t0c < durationS:
            for f in proto.read_pending_binary_tlm_frames():
                if f.enc_left is not None:
                    rows.append((f.t, f.enc_left.position, f.enc_right.position))
            time.sleep(0.01)
        return rows

    print("calibrating straight...")
    sendVerified(proto, CRUISE, CRUISE, 3500.0, 9598)
    rows = capture(4.2)
    moving = [r for r in rows if r is not rows[0]]
    tb = rows[0][0]
    win = [r for r in rows if 1200 <= r[0] - tb <= 3200]
    dt = (win[-1][0] - win[0][0]) / 1000.0
    CAL["L"] = max(0.5, min(1.5, ((win[-1][1] - win[0][1]) / dt) / CRUISE))
    CAL["R"] = max(0.5, min(1.5, ((win[-1][2] - win[0][2]) / dt) / CRUISE))
    time.sleep(1.0)
    proto.read_pending_binary_tlm_frames()
    print(f"cal factors: L {CAL['L']:.3f}  R {CAL['R']:.3f}")

    # (b) one test pivot at the calibrated turn commands and nominal
    #     duration: measured degrees -> turn-duration scale.
    turnMsNominal = durationFor(TURN_ARC, TURN_SPEED, reversal=True) * 1000.0
    tL = -TURN_SPEED / CAL["L"]
    tR = TURN_SPEED / CAL["R"]
    print("calibrating turn...")
    rows = capture(0.1)  # drain
    sendVerified(proto, tL, tR, turnMsNominal, 9597)
    rows = capture(turnMsNominal / 1000.0 + 1.5)
    degMeasured = abs(math.degrees(
        ((rows[-1][2] - rows[0][2]) - (rows[-1][1] - rows[0][1])) / TRACK))
    turnScale = 90.0 / degMeasured if degMeasured > 10 else 1.0
    print(f"test pivot: {degMeasured:.1f} deg -> turn duration scale {turnScale:.3f}")
    time.sleep(1.0)
    proto.read_pending_binary_tlm_frames()

    # un-rotate back so the tour starts square with the test pivot undone
    sendVerified(proto, -tL, -tR, turnMsNominal * turnScale, 9596)
    time.sleep(turnMsNominal * turnScale / 1000.0 + 1.2)
    proto.read_pending_binary_tlm_frames()

    segs = segments()
    segs = [(vL, vR, dur * (turnScale if idx % 2 == 1 else 1.0), mid)
            for idx, (vL, vR, dur, mid) in enumerate(segs)]
    log = dict(t=[], velL=[], velR=[], posL=[], posR=[], cmdL=[], cmdR=[])
    marks = []  # (t, label) segment starts
    t0 = time.monotonic()
    cmdL = cmdR = 0.0

    for idx, (vL, vR, durMs, moveId) in enumerate(segs):
        label = f"leg {idx // 2 + 1}" if idx % 2 == 0 else f"turn {idx // 2 + 1}"
        marks.append((time.monotonic() - t0, label))
        sendVerified(proto, vL, vR, durMs, moveId)
        cmdL, cmdR = vL, vR
        # command window, then a rest so the next segment starts from rest
        # (matching the calibration conditions; the coast is compensated in
        # durationFor()).
        deadline = time.monotonic() + durMs / 1000.0 + 0.9
        commandEnd = deadline - 0.9
        while time.monotonic() < deadline:
            if time.monotonic() >= commandEnd:
                cmdL = cmdR = 0.0
            for f in proto.read_pending_binary_tlm_frames():
                if f.enc_left is not None:
                    log["t"].append(time.monotonic() - t0)
                    log["velL"].append(f.enc_left.velocity)
                    log["velR"].append(f.enc_right.velocity)
                    log["posL"].append(f.enc_left.position)
                    log["posR"].append(f.enc_right.position)
                    log["cmdL"].append(cmdL)
                    log["cmdR"].append(cmdR)
            time.sleep(0.005)
    # capture the final coast-down
    tail = time.monotonic() + 1.0
    while time.monotonic() < tail:
        for f in proto.read_pending_binary_tlm_frames():
            if f.enc_left is not None:
                log["t"].append(time.monotonic() - t0)
                log["velL"].append(f.enc_left.velocity)
                log["velR"].append(f.enc_right.velocity)
                log["posL"].append(f.enc_left.position)
                log["posR"].append(f.enc_right.position)
                log["cmdL"].append(0.0)
                log["cmdR"].append(0.0)
        time.sleep(0.01)
    proto.estop()
    conn.disconnect()

    # Odometry from encoder positions (host-side integration for the chart).
    xs, ys = [0.0], [0.0]
    heading = 0.0
    for i in range(1, len(log["t"])):
        dL = log["posL"][i] - log["posL"][i - 1]
        dR = log["posR"][i] - log["posR"][i - 1]
        ds = 0.5 * (dL + dR)
        dTheta = (dR - dL) / TRACK
        heading += dTheta
        xs.append(xs[-1] + ds * math.cos(heading - dTheta / 2.0))
        ys.append(ys[-1] + ds * math.sin(heading - dTheta / 2.0))

    dTotL = log["posL"][-1] - log["posL"][0]
    dTotR = log["posR"][-1] - log["posR"][0]
    path = 0.5 * (dTotL + dTotR)
    headingDeg = math.degrees((dTotR - dTotL) / TRACK)
    closure = math.hypot(xs[-1], ys[-1])
    print(f"path {path:.1f} mm (target {4 * LEG:.0f}), heading {headingDeg:.1f} deg "
          f"(target 360), closure {closure:.1f} mm")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (axPath, axVel) = plt.subplots(
        1, 2, figsize=(16, 6.5), gridspec_kw={"width_ratios": [1, 1.6]})

    axPath.plot(xs, ys, color="#1f77b4", lw=1.6)
    axPath.plot([0], [0], marker="o", color="#2ca02c", ms=8, label="start")
    axPath.plot([xs[-1]], [ys[-1]], marker="x", color="#d62728", ms=10,
                mew=2.5, label=f"end (closure {closure:.0f} mm)")
    axPath.set_aspect("equal")
    axPath.set_xlabel("x [mm]")
    axPath.set_ylabel("y [mm]")
    axPath.set_title("Path (encoder odometry)")
    axPath.legend(fontsize=9, loc="upper left")
    axPath.grid(True, alpha=0.3)

    axVel.plot(log["t"], log["cmdL"], color="#1f77b4", lw=1.0, ls="--",
               alpha=0.7, label="left commanded [mm/s]")
    axVel.plot(log["t"], log["cmdR"], color="#2ca02c", lw=1.0, ls="--",
               alpha=0.7, label="right commanded [mm/s]")
    axVel.plot(log["t"], log["velL"], color="#1f77b4", lw=1.4,
               label="left measured [mm/s]")
    axVel.plot(log["t"], log["velR"], color="#2ca02c", lw=1.4,
               label="right measured [mm/s]")
    for tm, label in marks:
        axVel.axvline(tm, color="#bbbbbb", lw=0.6)
        axVel.annotate(label, xy=(tm, 1.0), xycoords=("data", "axes fraction"),
                       fontsize=7.5, rotation=90, va="top", ha="right",
                       color="#888888")
    axVel.axhline(0, color="black", lw=0.5)
    axVel.set_xlabel("time [s]")
    axVel.set_ylabel("wheel speed [mm/s]")
    axVel.set_title("Wheel speeds -- plain wheel commands, open loop")
    axVel.legend(fontsize=8, loc="lower right")
    axVel.grid(True, alpha=0.25)

    fig.suptitle(
        f"Square tour, wheel commands only -- path {path:.0f}/{4 * LEG:.0f} mm, "
        f"heading {headingDeg:.1f}/360 deg, closure {closure:.0f} mm",
        fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    out = "src/tests/bench/wheels_square_tour.png"
    fig.savefig(out, dpi=130)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
