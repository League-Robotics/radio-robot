"""Measure effective track width vs curvature -- i.e. wheel scrub, directly.

    b_eff = (delta_s_right - delta_s_left) / delta_theta_true

Encoders give the two wheel path lengths (tape-calibrated to 0.3%); the camera
gives true rotation. NO firmware kinematics, NO trackwidth, NO rotational_slip
appear anywhere in this measurement, so it cannot be fooled by the values it
is checking.

Every move is TIMED, never angle-stopped: a gyro angle-stop self-corrects to
the right rotation and hides the error being measured.

Scrub should grow as curvature tightens (a pivot slides the contact patches
sideways as hard as possible; a gentle arc mostly rolls), so this sweeps from
a pure pivot out to R=400 mm, both directions. Mean of the two directions is
scrub; their difference is left/right asymmetry.
"""
import math, os, sys, time

_REPO = "/Volumes/Proj/proj/RobotProjects/radio-robot-elite"
sys.path.insert(0, os.path.join(_REPO, "src", "host"))
sys.path.insert(0, os.path.join(_REPO, "src", "tests", "playfield"))

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol
from aprilcam.config import Config
from aprilcam.client.control import DaemonControl
import safedrive as sd

PORT = os.environ.get("TOVEZ_PORT", "/tmp/tovez-tty")
GEOM = 115.0          # [mm] measured geometric track
EMERG_X, EMERG_Y = 58.1, 35.6

# (label, v [mm/s], omega [rad/s], revolutions)
CASES = [
    ("pivot   ",   0.0, 1.00, 2.0),
    ("R=100   ", 100.0, 1.00, 2.0),
    ("R=200   ", 140.0, 0.70, 2.0),
    ("R=400   ", 140.0, 0.35, 1.0),
]


def enc(proto, tries=14):
    last = None
    for _ in range(tries):
        for f in proto.read_pending_binary_tlm_frames():
            if f.enc:
                last = f.enc
        time.sleep(0.05)
    return last


def run(proto, dc, cam, v, w, revs, sign):
    dur = revs * 2 * math.pi / w
    # centre first when the arc actually translates
    if v > 0:
        sd.toCentre(proto, dc, cam)
    proto.read_pending_binary_tlm_frames()
    e0 = enc(proto)
    t = sd.raw(dc, cam)
    if e0 is None or t is None:
        return None
    yaw = t.yaw
    total = 0.0
    proto.move_wheels(*( (v - sign*w*GEOM/2, v + sign*w*GEOM/2) ),
                      stop_time=dur*1000.0, timeout=dur*1000+8000)
    t0 = time.time()
    while time.time() - t0 < dur + 1.5:
        tt = sd.raw(dc, cam)
        if tt:
            total += math.remainder(tt.yaw - yaw, 2*math.pi)
            yaw = tt.yaw
            if abs(tt.world_xy[0]) > EMERG_X or abs(tt.world_xy[1]) > EMERG_Y:
                proto.estop(); time.sleep(0.3); proto.estop()
                return None
        time.sleep(0.05)
    time.sleep(1.2)
    tt = sd.raw(dc, cam)
    if tt:
        total += math.remainder(tt.yaw - yaw, 2*math.pi)
    e1 = enc(proto)
    if e1 is None or abs(total) < 0.5:
        return None
    dsL = e1[0] - e0[0]
    dsR = e1[1] - e0[1]
    return (dsR - dsL) / total, total, dsL, dsR


def main():
    conn = SerialConnection(port=PORT); proto = NezhaProtocol(conn)
    dc = DaemonControl.connect_default(Config.load()); cam = dc.list_cameras()[0]
    rows = []
    try:
        conn.connect(); time.sleep(1.0)
        proto.tlmOn(); time.sleep(0.8)
        print(f"  geometric track = {GEOM:.0f} mm;  b_eff = (dsR-dsL)/dtheta")
        print(f"  scrub factor = GEOM/b_eff  (1.000 = no scrub)\n")
        print(f"  {'case':9} {'dir':>4} {'sweep':>8} {'b_eff':>8} {'scrub':>7}")
        for label, v, w, revs in CASES:
            per = {}
            for sign, nm in ((+1.0, "CCW"), (-1.0, "CW")):
                r = run(proto, dc, cam, v, w, revs, sign)
                if r is None:
                    print(f"  {label} {nm:>4}   -- no data / halted"); continue
                b, sweep, dsL, dsR = r
                per[nm] = b
                print(f"  {label} {nm:>4} {math.degrees(sweep):8.1f} "
                      f"{b:8.1f} {GEOM/b:7.4f}")
            if len(per) == 2:
                mean = (per["CCW"] + per["CW"]) / 2
                asym = per["CCW"] - per["CW"]
                rows.append((label, mean, asym))
                print(f"  {label}  mean b_eff {mean:6.1f}   scrub {GEOM/mean:.4f}"
                      f"   asymmetry {asym:+.1f} mm\n")
    finally:
        for _ in range(2):
            try: proto.estop()
            except Exception: pass
            time.sleep(0.4)
        dc.close()
    if rows:
        print("  ==== scrub vs curvature ====")
        for label, mean, asym in rows:
            print(f"  {label}  b_eff {mean:6.1f} mm   scrub factor {GEOM/mean:.4f}")
        spread = max(r[1] for r in rows) - min(r[1] for r in rows)
        print(f"\n  b_eff spread across curvature: {spread:.1f} mm "
              f"-> {'CURVATURE-DEPENDENT, one constant is a compromise' if spread > 4 else 'FLAT: a single constant is correct'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
