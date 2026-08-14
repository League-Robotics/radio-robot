"""Standing gate: a commanded straight leg must actually travel that far.

WHY THIS EXISTS. On 2026-08-13 the stall detector was halting essentially every
Distance move partway, because it tested `state.otos.v_x` -- a WORLD-frame
velocity component -- as if it were body forward speed, so it read ~0 whenever
the robot drove north or south. The symptoms were diagnosed, in turn, as a
planner bug, an arc bug, a Navigator bug and a sensor-scale bug, over many
hours, because nothing asserted the single most basic property of a motion
command:

    if you ask for 400 mm, you should get about 400 mm.

Concretely, what was happening while every one of those theories was being
chased: a plain 400 mm leg travelled 75 mm; 300 mm arcs stopped at 44/71/95 mm;
GO_TO legs froze mid-route pointing straight at their target. This gate would
have failed on the first run.

It deliberately checks the DUMBEST thing, in the ONE way that cannot be fooled
by the robot's own instruments: the overhead camera measures how far the robot
actually went. It also drives the SAME leg at four headings 90 degrees apart,
because the bug it was written for was heading-dependent and a single-heading
test passes straight through it.

    uv run python src/tests/bench/leg_completion_gate.py            # gauti/USB
    uv run python src/tests/bench/leg_completion_gate.py --port /dev/cu.usbmodemRELAY

Exits nonzero on any failure.
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
STOP_DIST = 5.0                     # [cm]
LEG = 300.0                         # [mm]
SPEED = 140.0                       # [mm/s]
OMEGA = 0.9                         # [rad/s]
MIN_FRACTION = 0.90                 # a leg must cover at least this much
GRACE = 0.6                         # [s] tag-loss tolerance


def wrap(d):
    return math.remainder(d, 2 * math.pi)


def halt_line():
    return FIELD_X - BODY_R - STOP_DIST, FIELD_Y - BODY_R - STOP_DIST


def raw(dc, cam):
    tf = dc.get_tags(cam)
    return next((t for t in tf.tags if t.id == TAG and t.world_xy), None)


def fix(dc, cam, n=7):
    xs, ys, ws = [], [], []
    for _ in range(n):
        t = raw(dc, cam)
        if t:
            xs.append(t.world_xy[0]); ys.append(t.world_xy[1]); ws.append(t.yaw)
        time.sleep(0.06)
    if not xs:
        return None
    med = lambda v: sorted(v)[len(v) // 2]
    w0 = ws[0]
    return med(xs), med(ys), w0 + med([wrap(w - w0) for w in ws])


def projected(hist, lead=0.45):
    """Where the robot will be in `lead` seconds at its current camera speed.

    A fence that tests only the PRESENT position cannot stop a 140 mm/s robot:
    the camera runs at ~5 Hz and the estop takes effect a beat later, so by the
    time a breach is seen the robot is already ~5-7 cm past the line. tovez was
    beached on the north rail three times this way. Projecting forward makes
    the fence trip BEFORE the line instead of after it.
    """
    if len(hist) < 3:
        return None
    t1, x1, y1 = hist[-1][0], hist[-1][1], hist[-1][2]
    t0, x0, y0 = hist[0][0], hist[0][1], hist[0][2]
    dt = t1 - t0
    if dt < 0.15:
        return None
    vx, vy = (x1 - x0) / dt, (y1 - y0) / dt          # [cm/s]
    return x1 + vx * lead, y1 + vy * lead


def settle(proto, dc, cam, seconds):
    """Wait for rest in POSITION AND HEADING; halt on tag loss or boundary."""
    t0 = time.time(); lastSeen = time.time(); hist = []
    kx, ky = halt_line()
    while time.time() - t0 < seconds:
        proto.read_pending_binary_tlm_frames()
        t = raw(dc, cam)
        if t is None:
            if time.time() - lastSeen > GRACE:
                proto.estop(); time.sleep(0.3); proto.estop()
                return False
        else:
            lastSeen = time.time(); x, y = t.world_xy
            if abs(x) > kx or abs(y) > ky:
                proto.estop(); time.sleep(0.3); proto.estop()
                return False
            pr = projected(hist)
            if pr is not None and (abs(pr[0]) > kx or abs(pr[1]) > ky):
                proto.estop(); time.sleep(0.3); proto.estop()
                return False
            hist.append((time.time(), x, y, t.yaw))
            hist = [h for h in hist if time.time() - h[0] <= 1.3]
            if time.time() - t0 > 2.2 and len(hist) >= 5:
                dx = max(h[1] for h in hist) - min(h[1] for h in hist)
                dy = max(h[2] for h in hist) - min(h[2] for h in hist)
                dyaw = max(abs(wrap(h[3] - hist[0][3])) for h in hist)
                if math.hypot(dx, dy) < 0.4 and dyaw < math.radians(2.0):
                    break
        time.sleep(0.06)
    time.sleep(0.8)
    return True


def nudgeInside(proto, dc, cam):
    """If the robot starts OUTSIDE the halt line, walk it back in.

    The guard halts any move that begins outside the line, so without this the
    robot is stuck: it cannot recentre, and every leg then scores 0 mm. Only
    motion that REDUCES the violation is allowed, and it aborts the moment the
    violation grows.
    """
    for _ in range(6):
        p = fix(dc, cam)
        if p is None:
            return False
        kx, ky = halt_line()
        over = max(abs(p[0]) - kx, abs(p[1]) - ky)
        if over <= 0:
            return True
        gx = -math.copysign(1.0, p[0]) if abs(p[0]) - kx > 0 else 0.0
        gy = -math.copysign(1.0, p[1]) if abs(p[1]) - ky > 0 else 0.0
        n = math.hypot(gx, gy) or 1.0
        gx, gy = gx / n, gy / n
        proj = math.cos(p[2]) * gx + math.sin(p[2]) * gy
        if abs(proj) < 0.35:                       # nearly sideways: face inward
            want = math.atan2(gy, gx)
            e = wrap(want - p[2])
            proto.move_twist(0.0, 0.0, OMEGA if e > 0 else -OMEGA,
                             stop_angle=abs(e), timeout=15000)
            time.sleep(abs(e) / OMEGA + 3.0)
            continue
        sign = 1.0 if proj > 0 else -1.0
        proto.move_twist(sign * 110.0, 0.0, 0.0, stop_distance=150.0, timeout=12000)
        t0 = time.time(); worst = over
        while time.time() - t0 < 150 / 110.0 + 4.0:
            t = raw(dc, cam)
            if t:
                x, y = t.world_xy
                cur = max(abs(x) - kx, abs(y) - ky)
                if cur > worst + 1.5 or abs(x) > FIELD_X - BODY_R or abs(y) > FIELD_Y - BODY_R:
                    proto.estop(); time.sleep(0.3); proto.estop(); break
                worst = min(worst, cur)
            time.sleep(0.06)
        time.sleep(1.0)
    p = fix(dc, cam)
    if p is None:
        return False
    kx, ky = halt_line()
    return abs(p[0]) <= kx and abs(p[1]) <= ky


def toCentre(proto, dc, cam):
    if not nudgeInside(proto, dc, cam):
        return False
    prev = None
    for _ in range(10):
        p = fix(dc, cam)
        if p is None:
            return False
        r = math.hypot(p[0], p[1])
        if r <= 14.0:
            return True
        err = wrap(math.atan2(-p[1], -p[0]) - p[2])
        if abs(err) > math.radians(6):
            proto.move_twist(0.0, 0.0, OMEGA if err > 0 else -OMEGA,
                             stop_angle=abs(err), timeout=15000)
            settle(proto, dc, cam, abs(err) / OMEGA + 6.0)
            continue
        if prev is not None and r > prev - 0.5:
            return False
        prev = r
        hop = min(200.0, max(60.0, (r - 14.0) * 10.0))
        proto.move_twist(120.0, 0.0, 0.0, stop_distance=hop, timeout=15000)
        settle(proto, dc, cam, hop / 120.0 + 6.0)
    # Ran out of attempts. Report FAILURE -- returning True here meant the gate
    # went on to aim and drive from wherever the robot was stranded (measured:
    # y = +30.8 cm, hard against the north boundary), so the guard aborted the
    # aim turn and the leg scored 0 mm. That looked exactly like the drivetrain
    # bug this gate exists to catch.
    p = fix(dc, cam)
    if p is None:
        return False
    return math.hypot(p[0], p[1]) <= 14.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=os.environ.get("TOVEZ_PORT", "/tmp/tovez-tty"))
    ap.add_argument("--leg", type=float, default=LEG)
    args = ap.parse_args()

    conn = SerialConnection(port=args.port)
    proto = NezhaProtocol(conn)
    dc = DaemonControl.connect_default(Config.load())
    cam = dc.list_cameras()[0]
    failures = 0
    try:
        conn.connect(); time.sleep(1.0)
        proto.tlmOn(); time.sleep(0.8)
        print(f"  leg {args.leg:.0f} mm at {SPEED:.0f} mm/s, four headings, "
              f"camera-measured (must cover >= {100*MIN_FRACTION:.0f}%)")
        for heading in (0.0, 90.0, 180.0, 270.0):
            if not toCentre(proto, dc, cam):
                print("  [FAIL] could not recentre"); failures += 1; break
            p = fix(dc, cam)
            if p is None:
                print("  [FAIL] no camera fix"); failures += 1; break
            err = wrap(math.radians(heading) - p[2])
            if abs(err) > math.radians(4):
                proto.move_twist(0.0, 0.0, OMEGA if err > 0 else -OMEGA,
                                 stop_angle=abs(err), timeout=15000)
                settle(proto, dc, cam, abs(err) / OMEGA + 6.0)
            p0 = fix(dc, cam)
            proto.move_twist(SPEED, 0.0, 0.0, stop_distance=args.leg, timeout=15000)
            settle(proto, dc, cam, args.leg / SPEED + 8.0)
            p1 = fix(dc, cam)
            if p0 is None or p1 is None:
                print(f"  [FAIL] heading {heading:3.0f}: lost fix"); failures += 1; continue
            got = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) * 10.0
            frac = got / args.leg
            ok = frac >= MIN_FRACTION
            failures += 0 if ok else 1
            print(f"  [{'PASS' if ok else 'FAIL'}] heading {heading:3.0f} deg: "
                  f"{got:6.1f} mm of {args.leg:.0f} ({100*frac:5.1f}%)")
    finally:
        for _ in range(2):
            try: proto.estop()
            except Exception: pass
            time.sleep(0.4)
        dc.close()

    print(f"\n  ==== {'PASS' if failures == 0 else str(failures) + ' FAILURE(S)'} ====")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
