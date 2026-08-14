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
# 150, not 220: the figure must live in the MIDDLE of the table. At R=220 the
# lobes reach x = +-44 cm and the body corner +-53 cm against a 58.15 cm limit
# -- close enough to the rails that a normal tracking error puts the robot on
# one, where it wedges and the run is over. At R=150 the figure spans x +-30 cm
# (body +-39) and y +-15 cm (body +-24): comfortably interior, all margin.
RADIUS = 150.0                      # [mm] lobe radius; x span 4R, y span 2R


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
        # and right along the table's long axis. Camera-walked; setup, not
        # steering. Self-contained on purpose -- safedrive's halt line is a
        # conservative +-53.2/30.6 with a predictive margin on top, which trips
        # on an ordinary approach here and strands the run before it starts.
        # This uses the real body-on-table limit, which is what actually
        # matters.
        # First: if it STARTS outside the limit, walk it inward. The fence
        # refuses to move a robot already outside, so without this one bad
        # ending strands every subsequent run.
        for _ in range(6):
            p = fix(dc, cam, n=7)
            if p is None:
                print("  no camera fix"); return 1
            if abs(p[0]) <= EMERG_X - 2 and abs(p[1]) <= EMERG_Y - 2:
                break
            gx = -math.copysign(1.0, p[0]) if abs(p[0]) > EMERG_X - 2 else 0.0
            gy = -math.copysign(1.0, p[1]) if abs(p[1]) > EMERG_Y - 2 else 0.0
            nn = math.hypot(gx, gy) or 1.0
            gx, gy = gx / nn, gy / nn
            proj = math.cos(p[2]) * gx + math.sin(p[2]) * gy
            if abs(proj) < 0.5:
                e = wrap(math.atan2(gy, gx) - p[2])
                proto.move_twist(0, 0, 1.0 if e > 0 else -1.0,
                                 stop_angle=abs(e), timeout=15000)
                time.sleep(abs(e) / 1.0 + 2.5)
                continue
            proto.move_twist((1.0 if proj > 0 else -1.0) * 100.0, 0, 0,
                             stop_distance=200.0, timeout=12000)
            time.sleep(200 / 100.0 + 2.5)

        for _ in range(20):
            p = fix(dc, cam, n=7)
            if p is None:
                print("  no camera fix"); return 1
            r = math.hypot(p[0], p[1])
            if r < 8.0:
                break
            e = wrap(math.atan2(-p[1], -p[0]) - p[2])
            if abs(e) > math.radians(5):
                proto.move_twist(0, 0, 1.0 if e > 0 else -1.0,
                                 stop_angle=abs(e), timeout=15000)
                time.sleep(abs(e) / 1.0 + 2.5)
                continue
            hop = min(250.0, max(60.0, (r - 6.0) * 10.0))
            proto.move_twist(110.0, 0, 0, stop_distance=hop, timeout=15000)
            t0 = time.time()
            # Wait out the FULL move: the aim turn leaves a few degrees of
            # error, so the hop tracks slightly off-radial and needs several
            # iterations to converge. Cutting the wait short left the robot
            # mid-hop each time and the loop ran out of attempts far from the
            # centre.
            while time.time() - t0 < hop / 110.0 + 4.0:
                t = raw(dc, cam)
                if t and (abs(t.world_xy[0]) > EMERG_X or abs(t.world_xy[1]) > EMERG_Y):
                    proto.estop(); time.sleep(0.3); proto.estop(); break
                time.sleep(0.06)
            time.sleep(1.0)
        p = fix(dc, cam, n=7)
        if p is None or math.hypot(p[0], p[1]) > 12.0:
            print(f"  could not centre (at {p[0]:+.1f},{p[1]:+.1f}) -- refusing")
            return 1
        e = wrap(math.pi / 2 - p[2])
        if abs(e) > math.radians(3):
            proto.move_twist(0, 0, 1.0 if e > 0 else -1.0,
                             stop_angle=abs(e), timeout=15000)
            time.sleep(abs(e) / 1.0 + 2.5)

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
            # Generous window: the robot's actual omega runs below commanded,
            # so a lobe takes LONGER than 2*pi/omega. A window sized to the
            # ideal time truncates the trace and makes the lobes look smaller
            # than they are -- which is exactly how a 42%-too-LARGE radius got
            # misread as 2 cm too tight.
            ok = record(proto, dc, cam, 2 * lobe_t * 1.8 + 8.0, track, f"lap{lap+1}")
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

    # Fit each lobe: split the track at the crossing (closest approach to the
    # start after the first lobe is well underway).
    def fit_circle(pts):
        n = len(pts)
        if n < 12:
            return None
        sx=sum(p[0] for p in pts); sy=sum(p[1] for p in pts)
        sxx=sum(p[0]**2 for p in pts); syy=sum(p[1]**2 for p in pts)
        sxy=sum(p[0]*p[1] for p in pts)
        sxxx=sum(p[0]**3 for p in pts); syyy=sum(p[1]**3 for p in pts)
        sxyy=sum(p[0]*p[1]**2 for p in pts); sxxy=sum(p[0]**2*p[1] for p in pts)
        A=2*(sx*sx-n*sxx); B=2*(sx*sy-n*sxy); C=2*(sy*sy-n*syy)
        D=sxx*sx-n*sxxx+sx*syy-n*sxyy
        D2=sxx*sy-n*sxxy+syy*sy-n*syyy
        det=A*C-B*B
        if abs(det) < 1e-9:
            return None
        cx=(D*C-B*D2)/det; cy=(A*D2-B*D)/det
        rs=[math.hypot(q[0]-cx,q[1]-cy) for q in pts]
        R=sum(rs)/len(rs)
        return cx, cy, R*10.0, (sum((r-R)**2 for r in rs)/len(rs))**0.5*10.0

    tr = [(t, x, y) for (t, x, y, _, _) in track]
    if tr:
        late = [q for q in tr if q[0] > 0.35 * tr[-1][0]]
        cross = min(late, key=lambda q: math.hypot(q[1]-s[0], q[2]-s[1]))[0] if late else None
        lobeA = [(x, y) for (t, x, y) in tr if cross and t < cross]
        lobeB = [(x, y) for (t, x, y) in tr if cross and t >= cross]
        for nm, pts in (("lobe A", lobeA), ("lobe B", lobeB)):
            f = fit_circle(pts)
            if f:
                cx, cy, R, rms = f
                print(f"  {nm}: fitted R {R:6.1f} mm (commanded {args.radius:.0f}, "
                      f"ratio {R/args.radius:.3f}), fit rms {rms:.1f} mm, {len(pts)} pts")

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
