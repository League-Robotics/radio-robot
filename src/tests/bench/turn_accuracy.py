#!/usr/bin/env python3
"""Measure turn accuracy against the camera, at rest, one turn at a time.

Commands a spread of turn sizes in both directions and reports commanded vs.
camera-measured rotation. Yaw is the right thing to measure with this camera
even though its POSITION for the robot tag is inflated: a homothety preserves
angles, so the tag's orientation is unaffected by the height error.

    uv run python src/tests/bench/turn_accuracy.py
    uv run python src/tests/bench/turn_accuracy.py --angles 90 -90 45 180
"""
from __future__ import annotations

import argparse
import math
import time

RELAY_PORT = "/dev/cu.usbmodem214102"
CAMERA = "arducam-ov9782-usb-camera"
ROBOT_TAG = 100

PIVOT_OMEGA = 1.4     # [rad/s]
YAW_SIGN = +1.0       # 2026-08-13: commanded omega is now REP-103 CCW-positive,
                      # i.e. it INCREASES camera yaw. Was -1.0, a workaround for
                      # tovez's swapped drive-motor labelling (port 1 was its RIGHT
                      # wheel); fixed at source via motors.left_port/right_port.
SETTLE = 1.6          # [s]

DEFAULT_ANGLES = [90.0, -90.0, 45.0, -45.0, 180.0, 15.0, -15.0, 30.0]


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def camera_yaw(dc, samples: int = 7):
    """Median camera yaw [deg] over several frames, for jitter rejection."""
    vals = []
    for _ in range(samples * 3):
        for t in getattr(dc.get_tags(CAMERA), "tags", []):
            if t.id == ROBOT_TAG:
                vals.append(math.degrees(float(t.yaw)))
                break
        if len(vals) >= samples:
            break
        time.sleep(0.08)
    if not vals:
        return None
    # Median on the unit circle, so a wrap seam cannot corrupt it.
    base = vals[0]
    rel = sorted(wrap_deg(v - base) for v in vals)
    return wrap_deg(base + rel[len(rel) // 2])


def wait_until_idle(conn, timeout: float = 20.0) -> bool:
    """Block until the robot reports no active Move.

    A fixed sleep samples mid-turn whenever a move runs longer than expected,
    which turns a clean measurement into noise that looks like a control bug.
    flags bit 2 is kFlagActive.
    """
    conn.drain_binary_tlm()
    deadline = time.time() + timeout
    idle_seen = 0
    went_active = False
    while time.time() < deadline:
        time.sleep(0.15)
        for env in conn.drain_binary_tlm():
            t = getattr(env, "tlm", None)
            if t is None:
                continue
            active = bool(t.flags & 0x04)
            if active:
                went_active = True
                idle_seen = 0
                continue
            # Idle BEFORE the move has started is the enqueue latency, not
            # completion. Reporting it as done measures a turn that never ran.
            if not went_active:
                continue
            idle_seen += 1
            if idle_seen >= 6:      # sustained idle, not a gap between phases
                return True
    return False


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--angles", type=float, nargs="*", default=None)
    ap.add_argument("--port", default=RELAY_PORT)
    ap.add_argument("--reps", type=int, default=1)
    args = ap.parse_args()
    angles = args.angles if args.angles else DEFAULT_ANGLES

    from aprilcam.config import Config
    from aprilcam.client.control import DaemonControl
    from robot_radio.io.serial_conn import SerialConnection
    from robot_radio.robot.protocol import NezhaProtocol

    dc = DaemonControl.connect_default(Config.load())
    conn = SerialConnection(port=args.port, mode="relay")
    conn.connect()
    p = NezhaProtocol(conn)
    conn.send_cleartext("TLM:ON")
    time.sleep(0.4)
    move_id = int(time.time()) % 500000 + 600000

    status = [s for s in conn.send_cleartext("STATUS") if s.startswith("STATUS:")]
    otos_on = any(":otos=1" in s for s in status)
    print(f"OTOS present: {otos_on}   ({status[:1]})")

    rows = []
    try:
        for rep in range(args.reps):
            for want in angles:
                before = camera_yaw(dc)
                if before is None:
                    print("  lost the tag -- are the lights on?")
                    break
                move_id += 1
                omega = math.copysign(PIVOT_OMEGA, want * YAW_SIGN)
                p.move_twist(0.0, 0.0, omega, stop_angle=math.radians(abs(want)),
                             timeout=16000.0, move_id=move_id)
                if not wait_until_idle(conn):
                    print(f"  commanded {want:+7.1f}deg  ->  NEVER WENT IDLE")
                    continue
                time.sleep(SETTLE)
                after = camera_yaw(dc)
                if after is None:
                    print("  lost the tag mid-run")
                    break
                # `want` is already in world-CCW terms (YAW_SIGN is applied
                # when forming omega), so the camera delta is compared to it
                # directly. Flipping again here double-counts the convention.
                got = wrap_deg(after - before)
                err = got - want
                rows.append((want, got, err))
                print(f"  commanded {want:+7.1f}deg  ->  measured {got:+7.1f}deg   "
                      f"error {err:+6.1f}deg  ({100.0*got/want:6.1f}% of ask)")
    finally:
        try:
            p.estop()
        except Exception:
            pass
        conn.disconnect()

    if rows:
        errs = [abs(r[2]) for r in rows]
        big = [r for r in rows if abs(r[0]) >= 45.0]
        small = [r for r in rows if abs(r[0]) < 45.0]
        print()
        print(f"  n={len(rows)}  mean |error| {sum(errs)/len(errs):.2f}deg  "
              f"worst {max(errs):.2f}deg")
        for name, group in (("turns >=45deg", big), ("turns <45deg", small)):
            if group:
                e = [abs(r[2]) for r in group]
                ratio = [r[1] / r[0] for r in group]
                print(f"  {name:<14} mean |error| {sum(e)/len(e):5.2f}deg   "
                      f"mean ratio {sum(ratio)/len(ratio):.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
