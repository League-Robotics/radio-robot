#!/usr/bin/env python
"""safe_pilot.py — drive the robot for calibration WITHOUT leaving the table.

Every move goes through one guarded primitive: read tag 100, project the FRONT
of the robot (tag + 13 cm) plus its coast along the yaw, refuse/abort the instant
that would cross the table edge. No raw drive() anywhere else. No off-table.

Per move (a constant-(L,R) command = a circular arc) we capture CLEAN at-rest
readings before and after, for camera / encoder / OTOS / fused, and derive:
  Δyaw  = heading change (deg)
  Δdist = PATH length (mm). For a circular arc, path = chord·θ / (2·sin(θ/2)),
          exact from before/after pose alone (no noisy step-integration). The
          encoder reports cumulative wheel travel = path directly.

Modes:
  --verify  : read + print tag pose. NO MOTION.
  --probe   : check telemetry frame delivery. NO MOTION.
  --arcs    : guarded battery (speeds × radii × directions); append rows to
              /tmp/arc_data.csv. Run a few times to build up data.

Frame: A1-centred world cm. Table 134.3 x 89.3 -> x[-67.15,67.15] y[-44.65,44.65].
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import time

from robot_radio.testkit import make_target
from robot_radio.field.playfield import Playfield
from robot_radio.testkit.pose import CameraPose

FIELD_W_CM, FIELD_H_CM = 134.3, 89.3
HALF_X, HALF_Y = FIELD_W_CM / 2.0, FIELD_H_CM / 2.0

FRONT_CM = 13.0
RADIUS_CM = 15.0
COAST_CM = 12.0
EDGE_MARGIN_CM = 5.0
DRIVE_SPEED = 90
SPIN_SPEED = 75
READ_TIMEOUT_S = 0.2
ARC_DUR_S = 1.6
SETTLE_S = 0.5            # post-stop wait before the 'after' read — must clear the
                         # ~0.33s coast tail (robot drifts ~50mm/10deg after stop())

TRACKWIDTH_MM = 128.0     # physical, measured (centerline-to-centerline). FIXED.
# Per-wheel calibration to the camera: encoder dist AND yaw both come from ml,mr
# (slip folded in = 1.0, firmware yaw = (dR-dL)/tw). Each arc inverts to the true
# wheel ground-travel via differential-drive kinematics:
#   dL_true = cam_dist - (tw/2)*cam_yaw_rad ;  dR_true = cam_dist + (tw/2)*cam_yaw_rad
#   ml_new = ml_cur * median(dL_true / enc_dL) ;  mr_new likewise.
# Start at the baked baseline; set to the fit result and re-run to verify.
ROT_SLIP = 1.0            # slip folded into ml/mr
ML_CAL = 0.7165           # left  mm/deg — baked baseline (the prior 0.7316 fit used
MR_CAL = 0.7077           # right mm/deg — JERKY recovery-contaminated arcs; re-fitting clean)

ARC_CSV = "/tmp/arc_data.csv"


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _geo(x0, y0, th0, x1, y1, th1):
    """Circular-arc PATH length (mm) + Δyaw (deg) from before/after pose. th rad."""
    chord = math.hypot(x1 - x0, y1 - y0)
    dth = wrap(th1 - th0)
    if abs(dth) < 0.04:
        return chord, math.degrees(dth)
    return chord * abs(dth) / (2.0 * math.sin(abs(dth) / 2.0)), math.degrees(dth)


class SafePilot:
    def __init__(self, tag_id=100):
        self.tr = make_target("production")
        self.robot = self.tr.robot
        self.robot.connect()
        self.proto = self.robot._proto
        self.pf = Playfield.open()
        self.cam = CameraPose(self.pf, tag_id=tag_id, timeout=READ_TIMEOUT_S)
        self._evt_log = []   # recovery/wedge EVTs seen during the current arc

    def drain_evts(self):
        """Pull any async EVT lines off the link and stash wedge/recovery ones.
        A recovery mid-arc re-zeros the encoder accumulator → that arc's enc_dL/dR
        are garbage and MUST be discarded from the calibration fit."""
        try:
            for ln in self.proto.read_pending_lines():
                if "i2c_recover" in ln or "enc_wedged" in ln:
                    self._evt_log.append(ln.strip())
        except Exception:
            pass

    def read(self):
        try:
            return self.cam.read()
        except Exception:
            return None

    def robust_snap(self, tries=8):
        """One clean telemetry frame with all of enc/pose/otos populated."""
        for _ in range(tries):
            f = self.proto.snap()
            if f and f.enc and f.pose and f.otos:
                return f
            time.sleep(0.04)
        return None

    # ── safety ───────────────────────────────────────────────────────────────
    def _footprint_inside(self, x, y):
        return (abs(x) <= HALF_X - RADIUS_CM - EDGE_MARGIN_CM and
                abs(y) <= HALF_Y - RADIUS_CM - EDGE_MARGIN_CM)

    def _projection_inside(self, x, y, yaw, vsign):
        lead = FRONT_CM + COAST_CM
        px = x + vsign * lead * math.cos(yaw)
        py = y + vsign * lead * math.sin(yaw)
        return abs(px) <= HALF_X - EDGE_MARGIN_CM and abs(py) <= HALF_Y - EDGE_MARGIN_CM

    def is_safe(self, c, vsign):
        return (c is not None and self._footprint_inside(c[0], c[1])
                and self._projection_inside(c[0], c[1], c[2], vsign))

    def stop(self):
        for _ in range(3):
            self.proto.drive(0, 0); self.proto.send("STOP")

    def safe_drive(self, left, right, dur):
        vsign = 1 if (left + right) > 0 else (-1 if (left + right) < 0 else 0)
        if not self.is_safe(self.read(), vsign):
            return "REFUSED"
        self.proto.drive(left, right); t0 = time.time(); reason = "done"
        while time.time() - t0 < dur:
            self.drain_evts()                 # catch any mid-arc wedge/recovery
            c = self.read()
            if c is None:
                reason = "LOST_TAG"; break
            if not self.is_safe(c, vsign):
                reason = "ABORT_EDGE"; break
            time.sleep(0.04)
        self.stop()
        return reason

    # ── in-place turn / recenter ───────────────────────────────────────────────
    def turn_to(self, target_yaw, tol=0.07, max_s=6.0):
        # In-place turn keeps the body center fixed (the radius envelope is
        # rotation-invariant), so it is ALWAYS safe — never gate it on footprint.
        t0 = time.time()
        while time.time() - t0 < max_s:
            c = self.read()
            if c is None:
                self.stop(); return False
            err = wrap(target_yaw - c[2])
            if abs(err) < tol:
                self.stop(); return True
            self.proto.drive(-SPIN_SPEED, SPIN_SPEED) if err > 0 else self.proto.drive(SPIN_SPEED, -SPIN_SPEED)
            time.sleep(0.04)
        self.stop(); return False

    def _toward_center_ok(self, c):
        """Recovery guard: facing roughly at center AND front projection on the
        actual TABLE (not the inset safe zone). Driving toward center can only
        reduce edge proximity, so this safely extracts the robot from an edge."""
        x, y, yaw = c
        if abs(wrap(math.atan2(-y, -x) - yaw)) > math.radians(45):
            return False
        lead = FRONT_CM + COAST_CM
        px, py = x + lead * math.cos(yaw), y + lead * math.sin(yaw)
        return abs(px) <= HALF_X - EDGE_MARGIN_CM and abs(py) <= HALF_Y - EDGE_MARGIN_CM

    def recenter(self, thresh=14.0):
        for _ in range(12):
            c = self.read()
            if c is None:
                return False
            if math.hypot(c[0], c[1]) < thresh:
                return True
            self.turn_to(math.atan2(-c[1], -c[0]))        # face center
            c = self.read()
            if c is None or not self._toward_center_ok(c):
                return False
            self.proto.drive(DRIVE_SPEED, DRIVE_SPEED); t0 = time.time()
            while time.time() - t0 < 1.0:
                c = self.read()
                if c is None or not self._toward_center_ok(c):
                    break
                time.sleep(0.04)
            self.stop()
        c = self.read()
        return c is not None and math.hypot(c[0], c[1]) < thresh * 1.6

    def close(self):
        self.stop()
        try:
            self.tr.conn.disconnect()
        except Exception:
            pass
        self.pf.close()


def _arc_deltas(c0, f0, c1, f1):
    """Δpath-dist (mm) + Δyaw (deg) for camera/encoder/otos/fused from before/after."""
    if not (c0 and c1 and f0 and f1):
        return None
    out = {}
    d, y = _geo(c0[0] * 10, c0[1] * 10, c0[2], c1[0] * 10, c1[1] * 10, c1[2])
    out["cam_dist"], out["cam_yaw"] = d, y
    dL, dR = f1.enc[0] - f0.enc[0], f1.enc[1] - f0.enc[1]
    out["enc_dL"], out["enc_dR"] = dL, dR
    out["enc_dist"] = (dL + dR) / 2.0
    out["enc_yaw"] = math.degrees((dR - dL) / TRACKWIDTH_MM * ROT_SLIP)
    d, y = _geo(f0.otos[0], f0.otos[1], math.radians(f0.otos[2] / 100.0),
                f1.otos[0], f1.otos[1], math.radians(f1.otos[2] / 100.0))
    out["otos_dist"], out["otos_yaw"] = d, y
    d, y = _geo(f0.pose[0], f0.pose[1], math.radians(f0.pose[2] / 100.0),
                f1.pose[0], f1.pose[1], math.radians(f1.pose[2] / 100.0))
    out["fused_dist"], out["fused_yaw"] = d, y
    return out


CSV_COLS = ["kind", "cmd_L", "cmd_R", "speed", "dur", "reason", "recovered",
            "cam_dist", "cam_yaw", "enc_dL", "enc_dR", "enc_dist", "enc_yaw",
            "otos_dist", "otos_yaw", "fused_dist", "fused_yaw"]


def probe(p):
    p.proto.send("SET tw=128", read_ms=300)
    p.proto.stream_fields("enc,pose,otos")
    p.proto.stream(150)
    time.sleep(0.5)
    p.proto.read_pending_lines()
    frames = []
    t0 = time.time()
    while time.time() - t0 < 2.0:
        for line in p.proto.read_pending_lines():
            f = p.proto.parse_tlm(line)
            if f:
                frames.append(f)
        time.sleep(0.05)
    print(f"{len(frames)} frames/2s ({len(frames)/2:.1f}Hz); last enc={frames[-1].enc if frames else None}")
    p.proto.stream(0)


def settle_probe(p, L=110, R=150):
    """Drive one guarded arc; log the camera densely THROUGH the stop and coast,
    so we can see when the tag truly settles vs where the 0.35s 'after' read lands."""
    print("SET tw   ->", p.proto.send("SET tw=128", read_ms=300).get("responses"))
    print("SET slip ->", p.proto.send(f"SET rotSlip={ROT_SLIP}", read_ms=300).get("responses"))
    p.recenter()
    if not p.is_safe(p.read(), 1):
        print("refused (not safe to start)"); return
    traj = []
    p.proto.drive(L, R); t0 = time.time(); reason = "done"
    while time.time() - t0 < ARC_DUR_S:
        c = p.read()
        if c is None:
            reason = "LOST_TAG"; break
        if not p.is_safe(c, 1):
            reason = "ABORT_EDGE"; break
        traj.append((time.time() - t0, c[0], c[1], math.degrees(c[2])))
        time.sleep(0.05)
    tstop = time.time() - t0
    p.stop()
    while time.time() - t0 < tstop + 1.6:          # keep logging through coast+settle
        c = p.read()
        if c:
            traj.append((time.time() - t0, c[0], c[1], math.degrees(c[2])))
        time.sleep(0.05)
    print(f"\ndrive ended t={tstop:.2f}s ({reason}); {len(traj)} cam samples. tail:")
    print("    t      x       y      yaw")
    for s in [s for s in traj if s[0] >= tstop - 0.25]:
        mark = ""
        if abs(s[0] - tstop) < 0.03:                  mark = "  <- stop()"
        elif abs(s[0] - (tstop + SETTLE_S)) < 0.03:   mark = f"  <- my 'after' read ({SETTLE_S}s)"
        print(f"  {s[0]:5.2f} {s[1]:+7.1f} {s[2]:+7.1f} {s[3]:+7.1f}{mark}")
    # how far does it drift AFTER the 'after' read?
    at_read = min(traj, key=lambda s: abs(s[0] - (tstop + SETTLE_S)))
    final = traj[-1]
    drift = math.hypot(final[1] - at_read[1], final[2] - at_read[2]) * 10
    dyaw = final[3] - at_read[3]
    print(f"\ndrift from {SETTLE_S}s read -> final settled: {drift:.1f} mm, {dyaw:+.1f} deg")


def collect_arcs(p):
    print("SET tw   ->", p.proto.send("SET tw=128", read_ms=300).get("responses"))
    print("SET slip ->", p.proto.send(f"SET rotSlip={ROT_SLIP}", read_ms=300).get("responses"))
    print("SET ml   ->", p.proto.send(f"SET ml={ML_CAL}", read_ms=300).get("responses"))
    print("SET mr   ->", p.proto.send(f"SET mr={MR_CAL}", read_ms=300).get("responses"))
    p.proto.stream_fields("enc,pose,otos")
    p.proto.stream(0)                        # snap on demand; no lossy continuous stream
    moves = []
    for sp in (90, 130, 170):
        moves.append(("straight", sp, sp))
        for d in (40, 90):
            moves.append(("arcL", sp - d // 2, sp + d // 2))
            moves.append(("arcR", sp + d // 2, sp - d // 2))
    for sp in (90, 130):
        moves.append(("turnL", -sp, sp))
        moves.append(("turnR", sp, -sp))

    rows = []
    n_recovered = 0
    for kind, L, R in moves:
        p.recenter()                          # start every move centered
        p._evt_log = []                       # arm per-arc wedge/recovery capture
        c0, f0 = p.read(), p.robust_snap()
        reason = p.safe_drive(L, R, ARC_DUR_S)
        time.sleep(SETTLE_S)                  # clear the coast tail before the after-read
        p.drain_evts()                        # catch a recovery during the settle tail
        c1, f1 = p.read(), p.robust_snap()
        recovered = 1 if p._evt_log else 0    # a mid-arc recovery re-zeroed enc → discard
        n_recovered += recovered
        d = _arc_deltas(c0, f0, c1, f1)
        if not d:
            print(f"  {kind:8} L={L} R={R}: {reason} (snap/cam fail)")
            continue
        row = {"kind": kind, "cmd_L": L, "cmd_R": R, "speed": (L + R) / 2.0,
               "dur": ARC_DUR_S, "reason": reason, "recovered": recovered, **d}
        rows.append(row)
        flag = "  *WEDGE/RECOVER — DISCARD*" if recovered else ""
        print(f"  {kind:8} sp={row['speed']:+5.0f}: "
              f"cam(d={d['cam_dist']:5.0f},y={d['cam_yaw']:+6.1f}) "
              f"enc(d={d['enc_dist']:5.0f},y={d['enc_yaw']:+6.1f}) "
              f"otos(d={d['otos_dist']:5.0f},y={d['otos_yaw']:+6.1f}) "
              f"fus(d={d['fused_dist']:5.0f},y={d['fused_yaw']:+6.1f}) [{reason}]{flag}")
        time.sleep(0.2)
    p.stop()
    print(f"  >>> {n_recovered}/{len(rows)} arcs hit a wedge/recovery"
          f"{' — CLEAN BATCH' if n_recovered == 0 else ' — those rows are flagged recovered=1'}")

    new = not os.path.exists(ARC_CSV)
    with open(ARC_CSV, "a", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=CSV_COLS, extrasaction="ignore")
        if new:
            w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f"appended {len(rows)} rows to {ARC_CSV}")


def _assert_no_rogue():
    """Refuse to run while a rogue driver (world_tour) is alive — it steals the relay
    port AND drives the robot, corrupting data. See check-for-rogue-driver-processes."""
    import subprocess
    out = subprocess.run(["ps", "ax"], capture_output=True, text=True).stdout
    rogue = [l.strip() for l in out.splitlines() if "world_tour" in l and "grep" not in l]
    if rogue:
        print("ABORT: a rogue driver is running — kill it before driving:")
        for l in rogue[:4]:
            print("  ", l[:110])
        raise SystemExit(1)


def main():
    _assert_no_rogue()
    ap = argparse.ArgumentParser()
    ap.add_argument("--verify", action="store_true")
    ap.add_argument("--probe", action="store_true")
    ap.add_argument("--settle", action="store_true")
    ap.add_argument("--arcs", action="store_true")
    ap.add_argument("--tw", type=float, default=None)
    args = ap.parse_args()

    p = SafePilot()
    try:
        if args.tw is not None:
            print("SET tw ->", p.proto.send(f"SET tw={int(args.tw)}", read_ms=400).get("responses"))
        c = p.read()
        print(f"tag @ ({c[0]:+.1f},{c[1]:+.1f}) yaw={math.degrees(c[2]):+.0f}" if c else "NO TAG")
        if args.probe:
            probe(p)
        if args.settle:
            settle_probe(p)
        if args.arcs:
            collect_arcs(p)
    finally:
        p.close()


if __name__ == "__main__":
    main()
