"""Counter-clockwise tour of the four orange corner dots, scored by camera.

The robot navigates on ITS OWN sensors: the camera seeds a start pose once
(a surveyed start, as a real field would have), then every leg is a WORLD-frame
GO_TO. The camera never steers -- it only scores the arrival and guards the
rails.

At each dot the robot settles, and the camera takes a burst of fixes. Reported
per dot:

    error   distance from the dot to the MEAN of those fixes  [mm]
    rms     RMS distance of the individual fixes from the dot  [mm]
    spread  RMS of the fixes about their own mean -- the camera's own noise,
            so an `error` near this size is at the measurement floor

and across the tour: mean error and RMS error.

Dot positions were read off the playfield with pixel_to_world (2026-08-13).
CCW order in the camera frame (0=East, +90=North, angles increase CCW) is
NE -> NW -> SW -> SE.
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
FIELD_X, FIELD_Y = 67.15, 44.65      # [cm] table half-extents
BODY_R = 9.0                         # [cm] tag centre to body corner
# The dots sit near the corners, so the fence has to admit them: at y = 31 the
# body edge is 40 cm against a 44.65 cm table, which is fine. A tighter line
# (the 30.6 used elsewhere) would halt the robot ON the dot.
# Halt where the BODY would actually leave the table, not at an arbitrary
# tighter line: the look-ahead below already scales with measured speed, so it
# shrinks as the robot decelerates into a dot. A tighter line trips on approach
# at cruise -- measured: every leg halted, and SE was cut 243 mm short, purely
# by the fence.
# EXACTLY the body-on-table limit -- no extra margin. The dots sit at |y| = 31
# with only 4.65 cm of body clearance behind them, and the speed-scaled
# look-ahead is ~4.5 cm at cruise, so ANY additional margin makes an ordinary
# approach to a dot indistinguishable from running off, and every leg gets
# truncated (measured: 4/4 legs halted, errors inflated to 46-90 mm). This line
# still guarantees the body stays on the table; it just stops pretending there
# is room for a safety buffer that the geometry does not have.
HALT_X = FIELD_X - BODY_R            # [cm]  58.15
HALT_Y = FIELD_Y - BODY_R            # [cm]  35.65
# 100, not 140: the dots sit within ~10 cm of the rails and GO_TO overshoots
# its target at cruise (measured: SW target y=-30.2 reached -35.6, i.e. 5.4 cm
# past), which then trips the fence and truncates the NEXT leg. Slower approach
# shrinks both the overshoot and the fence's speed-scaled look-ahead.
SPEED = 100.0                        # [mm/s]
OMEGA = 0.9                          # [rad/s]
GRACE = 0.6                          # [s] tag-loss tolerance
LEAD = 0.45                          # [s] fence look-ahead

# measured on the playfield, CCW: NE -> NW -> SW -> SE
DOTS = [("NE", 51.0, 31.0), ("NW", -50.8, 31.0),
        ("SW", -51.4, -30.2), ("SE", 51.4, -30.8)]


def wrap(d):
    return math.remainder(d, 2 * math.pi)


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


def ownPose(proto, tries=12):
    """Latest raw OTOS pose (x mm, y mm, heading rad) -- GO_TO's world frame."""
    last = None
    for _ in range(tries):
        for f in proto.read_pending_binary_tlm_frames():
            if f.otos_present and f.otos:
                last = f.otos
        time.sleep(0.05)
    return None if last is None else (last[0], last[1],
                                      math.radians(last[2] / 100.0))


def burst(dc, cam, n=15):
    """A burst of fixes at rest, for scoring."""
    pts = []
    for _ in range(n):
        t = raw(dc, cam)
        if t:
            pts.append((t.world_xy[0], t.world_xy[1]))
        time.sleep(0.07)
    return pts


def projected(hist, lead=LEAD):
    if len(hist) < 3:
        return None
    t1, x1, y1 = hist[-1][0], hist[-1][1], hist[-1][2]
    t0, x0, y0 = hist[0][0], hist[0][1], hist[0][2]
    dt = t1 - t0
    if dt < 0.15:
        return None
    return x1 + (x1 - x0) / dt * lead, y1 + (y1 - y0) / dt * lead


def settle(proto, dc, cam, seconds, want_id=None):
    """Wait for the move's own completion ack; fence predictively meanwhile."""
    t0 = time.time(); lastSeen = time.time(); hist = []
    while time.time() - t0 < seconds:
        for f in proto.read_pending_binary_tlm_frames():
            for a in (getattr(f, "acks", None) or []):
                if want_id is not None and a.corr_id == want_id:
                    time.sleep(0.8)
                    return True, "ok"
        t = raw(dc, cam)
        if t is None:
            if time.time() - lastSeen > GRACE:
                proto.estop(); time.sleep(0.3); proto.estop()
                return False, "tag lost"
        else:
            lastSeen = time.time(); x, y = t.world_xy
            if abs(x) > HALT_X or abs(y) > HALT_Y:
                proto.estop(); time.sleep(0.3); proto.estop()
                return False, f"boundary ({x:+.1f},{y:+.1f})"
            hist.append((time.time(), x, y, t.yaw))
            hist = [h for h in hist if time.time() - h[0] <= 1.3]
            pr = projected(hist)
            # Outward motion only: braking INTO a dot near the line projects
            # past it for a frame or two, and tripping on that estops the
            # firmware's terminal fine-align mid-nudge -- every leg of the
            # first passing tour ended "HALTED: projected breach" AT the dot,
            # i.e. the ack never arrived and the last stage never ran.
            if pr is not None and (
                    (abs(pr[0]) > HALT_X and abs(pr[0]) > abs(x) + 0.3) or
                    (abs(pr[1]) > HALT_Y and abs(pr[1]) > abs(y) + 0.3)):
                proto.estop(); time.sleep(0.3); proto.estop()
                return False, f"projected breach ({pr[0]:+.1f},{pr[1]:+.1f})"
            if want_id is None and time.time() - t0 > 2.2 and len(hist) >= 5:
                dx = max(h[1] for h in hist) - min(h[1] for h in hist)
                dy = max(h[2] for h in hist) - min(h[2] for h in hist)
                dyaw = max(abs(wrap(h[3] - hist[0][3])) for h in hist)
                if math.hypot(dx, dy) < 0.4 and dyaw < math.radians(2.0):
                    break
        time.sleep(0.06)
    time.sleep(0.8)
    return True, "ok"


def nudgeInside(proto, dc, cam):
    """Walk a stranded robot back inside the halt line.

    Without this the run dies where it stands: the fence refuses to move a
    robot that STARTS outside the line, so one overshoot ends the tour. Only
    inward motion is allowed, and it aborts if the violation grows.
    """
    for _ in range(6):
        p = fix(dc, cam)
        if p is None:
            return False
        over = max(abs(p[0]) - HALT_X, abs(p[1]) - HALT_Y)
        if over <= 0:
            return True
        gx = -math.copysign(1.0, p[0]) if abs(p[0]) - HALT_X > 0 else 0.0
        gy = -math.copysign(1.0, p[1]) if abs(p[1]) - HALT_Y > 0 else 0.0
        n = math.hypot(gx, gy) or 1.0
        gx, gy = gx / n, gy / n
        proj = math.cos(p[2]) * gx + math.sin(p[2]) * gy
        if abs(proj) < 0.35:
            e = wrap(math.atan2(gy, gx) - p[2])
            proto.move_twist(0.0, 0.0, OMEGA if e > 0 else -OMEGA,
                             stop_angle=abs(e), timeout=15000)
            time.sleep(abs(e) / OMEGA + 3.0)
            continue
        proto.move_twist((1.0 if proj > 0 else -1.0) * 90.0, 0.0, 0.0,
                         stop_distance=140.0, timeout=12000)
        t0 = time.time(); worst = over
        while time.time() - t0 < 140 / 90.0 + 4.0:
            t = raw(dc, cam)
            if t:
                x, y = t.world_xy
                cur = max(abs(x) - HALT_X, abs(y) - HALT_Y)
                if cur > worst + 1.5 or abs(x) > FIELD_X - BODY_R or abs(y) > FIELD_Y - BODY_R:
                    proto.estop(); time.sleep(0.3); proto.estop(); break
                worst = min(worst, cur)
            time.sleep(0.06)
        time.sleep(1.0)
    p = fix(dc, cam)
    return p is not None and abs(p[0]) <= HALT_X and abs(p[1]) <= HALT_Y


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=os.environ.get("TOVEZ_PORT", "/tmp/tovez-tty"))
    ap.add_argument("--laps", type=int, default=1)
    args = ap.parse_args()

    conn = SerialConnection(port=args.port)
    proto = NezhaProtocol(conn)
    dc = DaemonControl.connect_default(Config.load())
    cam = dc.list_cameras()[0]
    results = []
    try:
        conn.connect(); time.sleep(1.0)
        proto.tlmOn(); time.sleep(0.8)

        p = fix(dc, cam, n=25)
        if p is None:
            print("  no camera fix -- refusing to start"); return 1
        proto.send_fast(f"SEED:{p[0]*10:.0f},{p[1]*10:.0f},{p[2]:.4f}")
        time.sleep(1.2)

        # Bootstrap the seed YAW by driving. The tag's orientation is the
        # noisiest part of a camera fix (+-1-2 deg run to run) and a seed-yaw
        # error rotates the whole odometry frame: 1 deg = 17 mm per metre of
        # leg, which alone busts a 10 mm budget. Displacement DIRECTION over a
        # 300 mm leg is good to ~0.1 deg, so: drive one calibration leg,
        # measure the rotation between the OTOS and camera displacements, and
        # re-seed with the corrected yaw. This is part of the surveyed start
        # (camera used once, before the tour) -- not in-run steering.
        c0 = fix(dc, cam); o0 = ownPose(proto)
        proto.move_twist(0.0, 0.0, OMEGA if wrap(-c0[2]) > 0 else -OMEGA,
                         stop_angle=abs(wrap(-c0[2])), timeout=15000) \
            if abs(wrap(-c0[2])) > math.radians(5) else None
        settle(proto, dc, cam, abs(wrap(-c0[2])) / OMEGA + 6.0)
        c0 = fix(dc, cam); o0 = ownPose(proto)
        proto.move_twist(110.0, 0.0, 0.0, stop_distance=300.0, timeout=15000)
        settle(proto, dc, cam, 300 / 110.0 + 7.0)
        c1 = fix(dc, cam); o1 = ownPose(proto)
        frameErr = 0.0
        if None not in (c0, c1, o0, o1):
            cd = ((c1[0] - c0[0]) * 10.0, (c1[1] - c0[1]) * 10.0)
            od = (o1[0] - o0[0], o1[1] - o0[1])
            if math.hypot(*cd) > 150:
                frameErr = wrap(math.atan2(od[1], od[0]) - math.atan2(cd[1], cd[0]))
        # Apply the correction to the SAME frame it was measured in: the
        # chip's current heading minus the measured rotation. Re-sampling the
        # tag yaw here and subtracting the OLD frame's error mixes two
        # independent noisy yaw samples and makes the seed WORSE (measured:
        # drift exploded to 47-119 mm that way).
        p = fix(dc, cam, n=25)
        oc = ownPose(proto)
        hNew = (oc[2] if oc is not None else p[2]) - frameErr
        proto.send_fast(f"SEED:{p[0]*10:.0f},{p[1]*10:.0f},{hNew:.4f}")
        time.sleep(1.2)
        print(f"  seeded at ({p[0]:+.1f},{p[1]:+.1f}) cm; frame rotation "
              f"{math.degrees(frameErr):+.2f} deg measured by driving, "
              f"removed from the chip's own heading\n")
        print(f"  {'dot':>4} {'target':>15} {'reached':>15} {'error':>8} {'rms':>7} {'spread':>7}   known  drift")

        gid = 700
        for lap in range(args.laps):
            for name, tx, ty in DOTS:
                if not nudgeInside(proto, dc, cam):
                    print("  stranded outside the halt line -- stopping"); break
                p0 = fix(dc, cam)
                if p0 is None:
                    print("  lost fix"); break
                # aim first: an in-place turn is safe and straightens the arc
                err = wrap(math.atan2(ty - p0[1], tx - p0[0]) - p0[2])
                if abs(err) > math.radians(5):
                    proto.move_twist(0.0, 0.0, OMEGA if err > 0 else -OMEGA,
                                     stop_angle=abs(err), timeout=15000)
                    ok, why = settle(proto, dc, cam, abs(err) / OMEGA + 6.0)
                    if not ok:
                        print(f"  {name}: aim halted -- {why}"); break
                p0 = fix(dc, cam)
                dist = math.hypot(tx - p0[0], ty - p0[1]) * 10.0
                gid += 1
                proto.read_pending_binary_tlm_frames()
                proto.go_to(tx * 10.0, ty * 10.0, frame=0, arrive=10.0,
                            timeout=int(dist / SPEED * 1000) + 20000, goto_id=gid)
                ok, why = settle(proto, dc, cam, dist / SPEED + 20.0, want_id=gid)

                # Refine on the robot's OWN sensors: it knows its residual
                # (raw OTOS pose vs the world target); anything it knows about
                # it can close itself. What is left after this is true
                # odometry drift -- invisible to the robot by definition.
                for _ in range(2):
                    op = ownPose(proto)
                    if op is None:
                        break
                    resid = math.hypot(op[0] - tx * 10.0, op[1] - ty * 10.0)
                    if resid <= 6.0:
                        break
                    gid += 1
                    proto.read_pending_binary_tlm_frames()
                    proto.go_to(tx * 10.0, ty * 10.0, frame=0, arrive=5.0,
                                timeout=int(resid / SPEED * 1000) + 12000,
                                goto_id=gid)
                    ok, why = settle(proto, dc, cam, resid / SPEED + 14.0,
                                     want_id=gid)

                op = ownPose(proto)
                pts = burst(dc, cam)
                if not pts:
                    print(f"  {name}: no fixes at the dot"); continue
                mx = sum(q[0] for q in pts) / len(pts)
                my = sum(q[1] for q in pts) / len(pts)
                error = math.hypot(mx - tx, my - ty) * 10.0
                rms = math.sqrt(sum(((q[0]-tx)**2 + (q[1]-ty)**2)
                                    for q in pts) / len(pts)) * 10.0
                spread = math.sqrt(sum(((q[0]-mx)**2 + (q[1]-my)**2)
                                       for q in pts) / len(pts)) * 10.0
                known = (math.hypot(op[0] - tx * 10.0, op[1] - ty * 10.0)
                         if op is not None else float("nan"))
                drift = (math.hypot(op[0] - mx * 10.0, op[1] - my * 10.0)
                         if op is not None else float("nan"))
                results.append((name, error, rms))
                print(f"  {name:>4} ({tx:+6.1f},{ty:+6.1f}) ({mx:+6.1f},{my:+6.1f}) "
                      f"{error:7.1f} {rms:6.1f} {spread:6.1f}   "
                      f"known {known:5.1f}  drift {drift:5.1f}"
                      f"{'' if ok else '   HALTED: ' + why}")
    finally:
        for _ in range(2):
            try: proto.estop()
            except Exception: pass
            time.sleep(0.4)
        dc.close()

    if results:
        n = len(results)
        mean_err = sum(r[1] for r in results) / n
        rms_err = math.sqrt(sum(r[1] ** 2 for r in results) / n)
        print(f"\n  ==== {n} dots ====")
        print(f"  mean error : {mean_err:6.1f} mm")
        print(f"  RMS  error : {rms_err:6.1f} mm")
        print(f"  worst dot  : {max(results, key=lambda r: r[1])[0]} "
              f"at {max(r[1] for r in results):.1f} mm")
    return 0


if __name__ == "__main__":
    sys.exit(main())
