#!/usr/bin/env python3
"""wheels_square_tour.py -- the square tour with NO planner: every leg and
turn is a host-shaped trapezoid streamed as plain ``wheels()`` commands.

The firmware has no profile feature on this path and needs none: the host
samples each segment's trapezoid every --tick seconds and re-arms a bounded
``WHEELS`` command (the same staircase idiom velocity_profile_gate.py
proved). ``App::Drive`` actuates each step; ``Motion::Planner`` never runs.
This is the control arm against planner_square_tour.py -- same square, same
closure metric, opposite command path.

Tour: 4 x 500 mm legs + 4 x 90 deg CCW pivots, a settle dwell at every
segment boundary (rest poses bracket every segment, per
.claude/rules/playfield-testing.md's boundary-pose doctrine -- here from
telemetry pose rather than the camera, since the robot is on a stand).

Closure = distance between the first and last REST poses, plus heading
error vs the +360 deg the four turns should sweep. Pose is the firmware's
own encoder odometry off the telemetry stream; on a stand that is the only
truth available, and it is exactly what the planner tour reports too.

Usage:
    uv run python src/tests/bench/wheels_square_tour.py --port <tovez> \
        [--leg 500] [--speed 200] [--turn-speed 100] [--outdir DIR]
"""
from __future__ import annotations

import argparse
import math
import pathlib
import sys
import time

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_REPO_ROOT / "src" / "host"))

TRACK = 128.0          # [mm] tovez geometry.trackwidth (data/robots/tovez.json)
TICK = 0.10            # [s] host re-arm period
LEASE = 0.4            # [s] each command's own bounded window
RAMP_LEG = 0.75        # [s] leg trapezoid ramp
RAMP_TURN = 0.30       # [s] turn trapezoid ramp
SETTLE = 1.2           # [s] rest dwell at every segment boundary


def trapezoid(peak, ramp, duration, t):  # [mm/s] [s] [s] [s]
    if t < 0.0 or t >= duration:
        return 0.0
    if t < ramp:
        return peak * (t / ramp)
    if t > duration - ramp:
        return peak * ((duration - t) / ramp)
    return peak


def stream_segment(proto, log, v_left_peak, v_right_peak, ramp, duration):
    """One segment: both wheels follow the same trapezoid shape, scaled per
    wheel (signs included), streamed as re-armed WHEELS commands."""
    start = time.monotonic()
    step = 0
    while True:
        now = time.monotonic() - start
        if now >= duration:
            break
        shape = trapezoid(1.0, ramp, duration, now + TICK / 2.0)
        proto.wheels(v_left_peak * shape, v_right_peak * shape, LEASE * 1000.0)
        step += 1
        drain(proto, log)
        remaining = (start + step * TICK) - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
    proto.wheels(0.0, 0.0, LEASE * 1000.0)


def drain(proto, log):
    for f in proto.read_pending_binary_tlm_frames():
        if f.pose is None:
            continue
        log.append((f.t, f.pose[0], f.pose[1], f.pose[2] / 100.0,
                    float(f.enc_left.velocity) if f.enc_left else 0.0,
                    float(f.enc_right.velocity) if f.enc_right else 0.0))


def settle(proto, log):
    end = time.monotonic() + SETTLE
    while time.monotonic() < end:
        proto.wheels(0.0, 0.0, LEASE * 1000.0)
        time.sleep(TICK)
        drain(proto, log)
    return log[-1][1:4] if log else (0.0, 0.0, 0.0)  # rest pose (x, y, heading)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--leg", type=float, default=500.0)         # [mm]
    ap.add_argument("--speed", type=float, default=200.0)       # [mm/s] leg peak
    ap.add_argument("--turn-speed", type=float, default=100.0)  # [mm/s] wheel peak in a pivot
    ap.add_argument("--outdir", default=str(_REPO_ROOT / "src" / "tests" / "bench" / "output"))
    args = ap.parse_args()

    # Trapezoid durations from area: D = peak * (T - ramp)  ->  T = D/peak + ramp
    leg_T = args.leg / args.speed + RAMP_LEG                    # [s]
    arc = (math.pi / 2.0) * (TRACK / 2.0)                       # [mm] per wheel, 90 deg pivot
    turn_T = arc / args.turn_speed + RAMP_TURN                  # [s]

    from robot_radio.io.serial_conn import SerialConnection
    from robot_radio.robot.protocol import NezhaProtocol
    conn = SerialConnection(port=args.port)
    info = conn.connect()
    print(f"  connected: {args.port}  device="
          f"{(info.get('announcement') or {}).get('device_name')}")
    proto = NezhaProtocol(conn)
    log: "list[tuple]" = []
    rest_poses = []
    try:
        proto.tlmOn()
        time.sleep(1.5)
        proto.read_pending_binary_tlm_frames()
        rest_poses.append(settle(proto, log))                   # start pose
        for corner in range(4):
            stream_segment(proto, log, args.speed, args.speed, RAMP_LEG, leg_T)
            rest_poses.append(settle(proto, log))               # after leg
            # CCW pivot: left backward, right forward
            stream_segment(proto, log, -args.turn_speed, args.turn_speed,
                           RAMP_TURN, turn_T)
            rest_poses.append(settle(proto, log))               # after turn
            print(f"  corner {corner + 1}/4: pose x={rest_poses[-1][0]:7.1f} "
                  f"y={rest_poses[-1][1]:7.1f} heading={rest_poses[-1][2]:7.1f}")
    finally:
        try:
            proto.estop()
            proto.tlmOff()
        except Exception:
            pass
        conn.disconnect()

    x0, y0, h0 = rest_poses[0]
    x1, y1, h1 = rest_poses[-1]
    closure = math.hypot(x1 - x0, y1 - y0)                      # [mm]
    heading_sweep = h1 - h0                                     # [deg] target +360
    print(f"\n  CLOSURE {closure:.1f} mm   heading sweep {heading_sweep:+.1f} deg "
          f"(target +360)   {len(rest_poses)} rest poses")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, (ax_path, ax_v) = plt.subplots(1, 2, figsize=(15.0, 7.0))
    xs = [r[1] for r in log]
    ys = [r[2] for r in log]
    ax_path.plot(xs, ys, lw=1.5, color="#2a78d6")
    ax_path.plot([r[0] for r in rest_poses], [r[1] for r in rest_poses], "o",
                 color="#eb6834", ms=6, label="rest poses")
    ax_path.plot(x0, y0, "s", color="0.2", ms=9, label="start")
    ax_path.annotate(f"closure {closure:.1f} mm", xy=(x1, y1), fontsize=11,
                     xytext=(10, 10), textcoords="offset points")
    ax_path.set_aspect("equal")
    ax_path.set_xlabel("x  [mm]")
    ax_path.set_ylabel("y  [mm]")
    ax_path.set_title(f"encoder-odometry path -- closure {closure:.1f} mm, "
                      f"heading {heading_sweep:+.1f} deg")
    ax_path.grid(alpha=0.3)
    ax_path.legend(fontsize=9)

    t0 = log[0][0] if log else 0
    ts = [(r[0] - t0) / 1000.0 for r in log]
    ax_v.plot(ts, [r[4] for r in log], lw=1.2, label="left wheel", color="#2a78d6")
    ax_v.plot(ts, [r[5] for r in log], lw=1.2, label="right wheel", color="#eb6834")
    ax_v.set_xlabel("time  [s]")
    ax_v.set_ylabel("wheel speed  [mm/s]")
    ax_v.set_title("streamed WHEELS trapezoids -- 4 legs + 4 CCW pivots")
    ax_v.grid(alpha=0.3)
    ax_v.legend(fontsize=9)

    fig.suptitle("SQUARE TOUR, wheels-only (no planner) -- host-shaped "
                 "trapezoids as re-armed WHEELS commands", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    outdir = pathlib.Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    out = outdir / "wheels_square_tour.png"
    fig.savefig(out, dpi=130)
    print(f"  chart: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
