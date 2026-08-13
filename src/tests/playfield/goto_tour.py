"""GO_TO accuracy tour -- every move checked against the table BEFORE issuing.

Uses safedrive, so a move that would leave the table is refused rather than
driven and then fenced. Aims at each target first (an in-place turn cannot
translate, so it is always safe), which makes the tangent arc nearly straight
and keeps it inside the keep-out box.

Targets form a rectangle well inside the keep-out (+-54.2 / +-31.6 cm), giving
GO_TO legs from ~450 mm to ~890 mm, all on the robot's own sensors. The camera
scores the result and guards the boundary; it never corrects the robot.
"""
import math, os, sys, time

_REPO = "/Volumes/Proj/proj/RobotProjects/radio-robot-elite"
sys.path.insert(0, os.path.join(_REPO, "src", "host"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol
from aprilcam.config import Config
from aprilcam.client.control import DaemonControl
import safedrive as sd

PORT = os.environ.get("TOVEZ_PORT", "/tmp/tovez-tty")
TARGETS = [(40.0, 18.0), (-40.0, 18.0), (-40.0, -18.0), (40.0, -18.0), (40.0, 18.0)]


def main():
    conn = SerialConnection(port=PORT)
    proto = NezhaProtocol(conn)
    dc = DaemonControl.connect_default(Config.load())
    cam = dc.list_cameras()[0]
    rows = []
    try:
        conn.connect(); time.sleep(1.0)
        proto.tlmOn(); time.sleep(0.8)

        ok, why = sd.toCentre(proto, dc, cam)
        print(f"  recentre: {why}")
        if not ok:
            return 1

        print(f"\n  {'target':>14} {'dist':>7} {'miss':>7} {'miss%':>7}  note")
        for tx, ty in TARGETS:
            p = sd.fix(dc, cam)
            if p is None:
                print("  no fix -- stopping"); break
            # aim at the target first: an in-place turn is always safe
            bearing = math.atan2(ty - p[1], tx - p[0])
            _, why = sd.turn(proto, dc, cam, sd.wrap(bearing - p[2]))
            if why.startswith("HALTED"):
                print(f"  aim {why}"); break

            p0 = sd.fix(dc, cam)
            if p0 is None:
                break
            dx, dy = (tx - p0[0]) * 10.0, (ty - p0[1]) * 10.0
            c, s = math.cos(p0[2]), math.sin(p0[2])
            fwd = dx * c + dy * s
            left = -dx * s + dy * c
            dist = math.hypot(fwd, left)

            p1, why = sd.goto(proto, dc, cam, fwd, left)
            if p1 is None:
                print(f"  {tx:+6.0f},{ty:+6.0f} {dist:7.0f}       -       -  {why}")
                continue
            miss = math.hypot(p1[0] - tx, p1[1] - ty) * 10.0
            rows.append((dist, miss))
            print(f"  {tx:+6.0f},{ty:+6.0f} {dist:7.0f} {miss:7.1f} {100*miss/dist:6.1f}%"
                  f"  {'' if why == 'ok' else why}")
    finally:
        for _ in range(2):
            try: proto.estop()
            except Exception: pass
            time.sleep(0.4)
        dc.close()

    if rows:
        m = sum(r[1] for r in rows) / len(rows)
        print(f"\n  GO_TO miss: mean {m:.1f} mm, max {max(r[1] for r in rows):.1f} mm "
              f"over {len(rows)} legs of {min(r[0] for r in rows):.0f}-"
              f"{max(r[0] for r in rows):.0f} mm")
        print("  (robot sensors only; camera scored and guarded, never corrected)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
