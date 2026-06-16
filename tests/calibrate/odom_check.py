#!/usr/bin/env python3
"""odom_check.py — measure ONBOARD odometry vs camera truth (READ-ONLY).

The firmware G/TURN terminate on the robot's own fused pose; if that pose is
wrong, navigation is wrong.  This drives clean probe moves and reports, for each,
what the camera (ground truth) saw vs what each onboard estimator reported:
  - encoder odometry (per-wheel mm; D resets its own accumulator)
  - raw OTOS displacement / heading
  - the FUSED pose (what G/TURN actually stop on)

It writes NOTHING — pure diagnosis.  A ratio >1 means the estimator OVER-reports
(robot stops short); <1 means it UNDER-reports (overshoot).

    uv run python tests/calibrate/odom_check.py --straight 300 --turn 90
"""
from __future__ import annotations

import argparse
import math
import sys
import time

from robot_radio.robot.nezha import Nezha
from robot_radio.robot.protocol import NezhaProtocol
from robot_radio.io.serial_conn import SerialConnection, list_serial_ports
from robot_radio.field.playfield import Playfield
from robot_radio.testkit.pose import CameraPose


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _cam_read(cam, tries: int = 4):
    """Robust camera pose read — retries (the daemon can briefly lose the tag
    right after motion).  Returns (x_cm, y_cm, yaw_rad) or None."""
    for _ in range(tries):
        try:
            return cam.read()
        except Exception:
            time.sleep(0.3)
    return None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--straight", type=int, default=300, help="straight probe mm (0=skip)")
    ap.add_argument("--turn", type=float, default=90.0, help="turn probe deg (0=skip)")
    ap.add_argument("--speed", type=int, default=120)
    ap.add_argument("--reps", type=int, default=1)
    ap.add_argument("--port", default=None)
    ap.add_argument("--robot-tag", type=int, default=100)
    ap.add_argument("--safe-x", type=float, default=40.0)
    ap.add_argument("--safe-y", type=float, default=38.0)
    a = ap.parse_args(argv)

    port = a.port or (list_serial_ports() or [None])[0]
    if port is None:
        print("no serial port found"); return 2
    conn = SerialConnection(port=port); conn.connect()
    proto = NezhaProtocol(conn); robot = Nezha(proto)
    print(f"connected: {robot.connect()}")
    pf = Playfield.open()
    cam = CameraPose(pf, tag_id=a.robot_tag)
    try:
        proto.stream_fields("enc,pose,otos")
    except Exception:
        pass

    def snap():
        t = proto.snap()
        return (t.pose if t else None), (t.otos if t else None), (t.enc if t else None)

    try:
        # ---- STRAIGHT probes (forward, measure, then open-loop return) -----
        for i in range(a.reps if a.straight else 0):
            c0 = _cam_read(cam)
            if c0 is None:
                print(f"\n[straight {i+1}] camera can't see tag — SKIP."); continue
            cx0, cy0, ch0 = c0
            ex = cx0 + (a.straight / 10.0) * math.cos(ch0)
            ey = cy0 + (a.straight / 10.0) * math.sin(ch0)
            print(f"\n[straight {i+1}] @ ({cx0:+.1f},{cy0:+.1f}) head={math.degrees(ch0):+.0f}°"
                  f" → predicted end ({ex:+.1f},{ey:+.1f})")
            if abs(ex) > a.safe_x or abs(ey) > a.safe_y:
                print(f"  predicted end outside safe box (±{a.safe_x}x±{a.safe_y}) — SKIP."
                      " Reposition/aim the robot inward."); break
            p0, o0, _ = snap()
            proto.distance(a.speed, a.speed, a.straight)   # D resets encoder
            secs = a.straight / max(a.speed, 1)
            proto.wait_for_evt_done("D", int(secs * 1000) + 4000)
            proto.stop(); time.sleep(0.8)
            p1, o1, e1 = snap()
            c1 = _cam_read(cam)
            if c1 is None:
                print("  lost tag after drive — SKIP this sample."); continue
            cx1, cy1, _ = c1

            truth = math.hypot(cx1 - cx0, cy1 - cy0) * 10.0     # mm
            encL = abs(e1[0]) if e1 else float("nan")
            encR = abs(e1[1]) if e1 else float("nan")
            enc = (encL + encR) / 2.0
            fused = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) if (p0 and p1) else float("nan")
            otos = math.hypot(o1[0] - o0[0], o1[1] - o0[1]) if (o0 and o1) else float("nan")
            print(f"  TRUTH(cam)={truth:6.1f}mm | encL={encL:6.1f} encR={encR:6.1f} "
                  f"enc={enc:6.1f} ({enc/truth:5.3f}) "
                  f"otos={otos:6.1f} ({otos/truth:5.3f}) fused={fused:6.1f} ({fused/truth:5.3f})")
            print("  (ratio>1 ⇒ over-reports ⇒ G stops SHORT by that factor)")
            # Open-loop return toward start so reps stay bounded (heading unchanged
            # by a straight drive, so reversing retraces the line — accuracy N/A).
            if i + 1 < a.reps:
                proto.drive(-a.speed, -a.speed)
                time.sleep(secs + 0.3)
                proto.stop(); time.sleep(0.6)

        # ---- TURN probes (both directions) ---------------------------------
        for sgn in ((1, -1) if a.turn else ()):
            for i in range(a.reps):
                c0 = _cam_read(cam)
                p0, o0, _ = snap()
                if c0 is None or p0 is None:
                    print("\n[turn] lost tag/snap — SKIP."); continue
                cx0, cy0, ch0 = c0
                raw = p0[2] + sgn * a.turn * 100         # onboard heading + delta, cdeg
                tgt_cdeg = int(round((raw + 18000) % 36000 - 18000))  # wrap to ±180°
                print(f"\n[turn {'CCW' if sgn>0 else 'CW'} {i+1}] cam head={math.degrees(ch0):+.0f}°"
                      f" onboard={p0[2]/100:+.0f}° → target {tgt_cdeg/100:+.0f}°")
                proto.turn(tgt_cdeg, eps_cdeg=100)
                proto.wait_for_evt_done("TURN", 12000)
                proto.stop(); time.sleep(1.2)
                p1, o1, _ = snap()
                c1 = _cam_read(cam, tries=8)
                if c1 is None or p1 is None:
                    print("  lost tag/snap after turn — SKIP."); continue
                cx1, cy1, ch1 = c1
                # All deltas wrapped to ±180° (headings cross the wrap point).
                wrap_deg = lambda d: (d + 180) % 360 - 180
                d_truth = math.degrees(_wrap(ch1 - ch0))
                d_fused = wrap_deg((p1[2] - p0[2]) / 100.0)
                d_otos = wrap_deg((o1[2] - o0[2]) / 100.0) if o0 and o1 else float("nan")
                r = (d_fused / d_truth) if d_truth else float("nan")
                print(f"  cmd Δ={sgn*a.turn:+.0f}° | TRUTH(cam)Δ={d_truth:+6.1f}° | "
                      f"fusedΔ={d_fused:+6.1f}° otosΔ={d_otos:+6.1f}°  fused/truth={r:5.3f}")
                print("  (fused≈truth ⇒ heading good; otos≈truth but fused≠truth ⇒ fusion/trackwidth bug)")
    finally:
        try: proto.stop()
        except Exception: pass
        try: conn.disconnect()
        except Exception: pass
        try: pf.close()
        except Exception: pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
