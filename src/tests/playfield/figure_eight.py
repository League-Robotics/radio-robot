"""Figure-eight: two circles, all curves, no stopping.

The ancestor of this test was `demo_figure_eight.ipynb` (a Catmull-Rom spline
driven by pure pursuit in sim, deleted by e4bffd8e). This is its playfield
form, and the point is the SAME: the robot never stops and never pivots. It
drives two tangent circles of constant curvature -- one CCW, one CW -- which
is a figure-eight, and every millimetre of it is a curve.

Each lobe is ONE command: move_twist(v, 0, omega) with a full 2*pi angle stop.
Radius is v/omega. Both lobes are enqueued back-to-back so the planner runs
them continuously, with no rest between them.

The camera records the path and guards the rails. It never steers.

    uv run python src/tests/playfield/figure_eight.py
    uv run python src/tests/playfield/figure_eight.py --laps 2 --radius 220
"""
import argparse
import math
import os
import sys
import time

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(_REPO, "src", "host"))

from robot_radio.io.serial_conn import SerialConnection          # noqa: E402
from robot_radio.robot.protocol import NezhaProtocol             # noqa: E402
from aprilcam.config import Config                               # noqa: E402
from aprilcam.client.control import DaemonControl                # noqa: E402

TAG = 100
FIELD_X, FIELD_Y = 67.15, 44.65     # [cm]
BODY_R = 9.0                        # [cm]
EMERG_X = FIELD_X - BODY_R          # [cm] genuine drive-off only
EMERG_Y = FIELD_Y - BODY_R
SPEED = 140.0                       # [mm/s]
RADIUS = 220.0                      # [mm] lobe radius; x span 4R, y span 2R


def wrap(d):
    return math.remainder(d, 2 * math.pi)


def raw(dc, cam):
    tf = dc.get_tags(cam)
    return next((t for t in tf.tags if t.id == TAG and t.world_xy), None)


def fix(dc, cam, n=13, retries=3):
    for _ in range(retries):
        xs, ys, ws = [], [], []
        for _ in range(n):
            t = raw(dc, cam)
            if t:
                xs.append(t.world_xy[0]); ys.append(t.world_xy[1]); ws.append(t.yaw)
            time.sleep(0.06)
        if xs:
            med = lambda v: sorted(v)[len(v) // 2]
            w0 = ws[0]
            return med(xs), med(ys), w0 + med([wrap(w - w0) for w in ws])
        time.sleep(0.6)
    return None


def record(proto, dc, cam, seconds, track, phase):
    """Record the path; emergency stop ONLY if the body would leave the table."""
    t0 = time.time()
    while time.time() - t0 < seconds:
        t = raw(dc, cam)
        if t:
            x, y = t.world_xy
            track.append((time.time() - t0, x, y, t.yaw, phase))
            if abs(x) > EMERG_X or abs(y) > EMERG_Y:
                proto.estop(); time.sleep(0.3); proto.estop()
                print(f"      EMERGENCY STOP at ({x:+.1f},{y:+.1f})")
                return False
        time.sleep(0.06)
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=os.environ.get("TOVEZ_PORT", "/tmp/tovez-tty"))
    ap.add_argument("--laps", type=int, default=1)
    ap.add_argument("--radius", type=float, default=RADIUS)
    ap.add_argument("--speed", type=float, default=SPEED)
    args = ap.parse_args()

    omega = args.speed / args.radius            # [rad/s]
    lobe_t = 2 * math.pi / omega                # [s] per full circle

    conn = SerialConnection(port=args.port)
    proto = NezhaProtocol(conn)
    dc = DaemonControl.connect_default(Config.load())
    cam = dc.list_cameras()[0]
    track, marks = [], []
    try:
        conn.connect(); time.sleep(1.0)
        proto.tlmOn(); time.sleep(0.8)

        # Stage at the crossing point, facing NORTH: the lobes then lie left
        # and right along the table's long axis. Camera-walked; this is setup.
        for _ in range(10):
            p = fix(dc, cam, n=7)
            if p is None:
                print("  no camera fix"); return 1
            d = math.hypot(p[0], p[1])
            if d < 4.0:
                break
            e = wrap(math.atan2(-p[1], -p[0]) - p[2])
            if abs(e) > math.radians(6):
                proto.move_twist(0, 0, 1.0 if e > 0 else -1.0,
                                 stop_angle=abs(e), timeout=15000)
                time.sleep(abs(e) / 1.0 + 2.2); continue
            hop = min(300.0, d * 10.0)
            proto.move_twist(110.0, 0, 0, stop_distance=hop, timeout=15000)
            time.sleep(hop / 110.0 + 2.0)
        p = fix(dc, cam, n=7)
        e = wrap(math.pi / 2 - p[2])
        if abs(e) > math.radians(3):
            proto.move_twist(0, 0, 1.0 if e > 0 else -1.0,
                             stop_angle=abs(e), timeout=15000)
            time.sleep(abs(e) / 1.0 + 2.2)

        start = fix(dc, cam)
        marks.append(("start", start))
        print(f"  start ({start[0]:+.2f},{start[1]:+.2f}) yaw "
              f"{math.degrees(start[2]):+.1f} (facing north)")
        print(f"  radius {args.radius:.0f} mm, speed {args.speed:.0f} mm/s, "
              f"omega {omega:.3f} rad/s, {lobe_t:.1f} s per lobe")
        print(f"  lobes span x +-{2*args.radius/10:.0f} cm, "
              f"y +-{args.radius/10:.0f} cm\n")

        for lap in range(args.laps):
            # Enqueue BOTH lobes before either finishes -- the planner runs
            # them back to back, so the crossing is continuous, not a stop.
            # replace=False is essential: move_twist defaults to replace=True,
            # which PREEMPTS the move already running. With the default, the
            # second lobe cancelled the first mid-circle and only one lobe was
            # ever driven (measured: a single CW circle, no figure at all).
            proto.move_twist(args.speed, 0.0, omega, stop_angle=2 * math.pi,
                             timeout=int(lobe_t * 1000) + 12000, replace=False)
            proto.move_twist(args.speed, 0.0, -omega, stop_angle=2 * math.pi,
                             timeout=int(lobe_t * 1000) + 12000, replace=False)
            ok = record(proto, dc, cam, 2 * lobe_t + 6.0, track, f"lap{lap+1}")
            p = fix(dc, cam)
            marks.append((f"lap{lap+1}", p))
            c = math.hypot(p[0]-start[0], p[1]-start[1]) * 10.0
            print(f"  lap {lap+1}: back at ({p[0]:+.2f},{p[1]:+.2f}) yaw "
                  f"{math.degrees(p[2]):+.1f}   closure {c:.1f} mm, "
                  f"heading {math.degrees(wrap(p[2]-start[2])):+.1f} deg"
                  f"{'' if ok else '   EMERGENCY'}")
            if not ok:
                break
    finally:
        for _ in range(2):
            try: proto.estop()
            except Exception: pass
            time.sleep(0.4)
        dc.close()

    s = marks[0][1]; e = marks[-1][1]
    closure = math.hypot(e[0]-s[0], e[1]-s[1]) * 10.0
    dyaw = math.degrees(wrap(e[2]-s[2]))
    print(f"\n  CLOSURE: {closure:.1f} mm   heading {dyaw:+.2f} deg")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(11, 8))
    cols = ["#1f77b4", "#ff7f0e", "#2ca02c", "#9467bd"]
    for i in range(args.laps):
        pts = [(x, y) for (_, x, y, _, p) in track if p == f"lap{i+1}"]
        if pts:
            ax.plot([q[0] for q in pts], [q[1] for q in pts], ".",
                    ms=3.5, color=cols[i % 4], label=f"lap {i+1} ({len(pts)} fixes)")
    th = [i * math.pi / 180 for i in range(361)]
    R = args.radius / 10.0
    ax.plot([s[0] - R + R*math.cos(t) for t in th],
            [s[1] + R*math.sin(t) for t in th], "--", color="#2ca02c", lw=1.2)
    ax.plot([s[0] + R - R*math.cos(t) for t in th],
            [s[1] + R*math.sin(t) for t in th], "--", color="#2ca02c", lw=1.2,
            label="ideal circles")
    ax.plot([s[0]], [s[1]], "s", color="#2ca02c", ms=12, label="start")
    ax.plot([e[0]], [e[1]], "X", color="#d62728", ms=14, label="end")
    ax.add_patch(plt.Rectangle((-FIELD_X, -FIELD_Y), 2*FIELD_X, 2*FIELD_Y,
                               fill=False, ec="#ccc", ls="--"))
    ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_xlabel("x [cm]"); ax.set_ylabel("y [cm]")
    ax.legend(loc="upper right", fontsize=9)
    ax.set_title(f"Figure-eight: two tangent circles, all curves, no stopping\n"
                 f"radius {args.radius:.0f} mm, {args.speed:.0f} mm/s, "
                 f"{args.laps} lap(s)    closure {closure:.1f} mm, {dyaw:+.1f} deg")
    out = os.path.join(_REPO, "src/tests/bench/output/figure_eight.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.tight_layout(); fig.savefig(out, dpi=110)
    print(f"  chart: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
