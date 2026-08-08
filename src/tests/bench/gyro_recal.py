#!/usr/bin/env python
"""Park the robot and recalibrate the OTOS gyro. The 'heading seems off' tool.

Stakeholder directive 2026-08-08: "we need to have you be able to recalibrate
the gyro at will. If it seems to be off, then you go and you park the robot
and reset the gyro."

Why this exists: the gyro's bias is otherwise calibrated exactly ONCE, at
boot, from ~612ms of samples -- and whatever the chip feels in that window
becomes "zero rotation" for the whole session. A robot that boots while being
handled (battery swap, carried, set down) drives with a poisoned heading:
measured on tovez 2026-08-08, a mid-battery-swap boot produced +1.44 deg/s of
standstill drift (camera-confirmed motionless) and ~3cm/30s of position creep
(the phantom rotation sweeping the 47.8mm sensor lever arm). See
.claude/rules/hardware-bench-testing.md "The robot must be STILL when it
boots".

What it does, in order:
  1. estop until the ENCODERS confirm the robot is stopped (an estop command
     is a request; the Nezha brick latches speed and a lost zero write is
     permanent -- .claude/rules/playfield-testing.md).
  2. Measure standstill heading drift (the "is it actually off?" number).
  3. Send CALIBRATE, resending until its ack ring entry arrives -- the relay
     DROPS inbound commands outright, so a command sent once is a command
     maybe sent. err==0 means the firmware accepted it while parked;
     ERR_BUSY(10) means something was still moving; ERR_NOT_CONFIGURED(8)
     means no OTOS.
  4. Wait out the ~612ms calibration, then measure drift again and print
     PASS/FAIL. Pose and tracking survive -- no re-seed needed.

Usage:
    uv run python src/tests/bench/gyro_recal.py --port /dev/cu.usbmodemXXXX
    uv run python src/tests/bench/gyro_recal.py --port ... --measure 30

Take the port from `mbdeploy list` for this session only (ports move); the
RADIOBRIDGE port when the robot is on battery.
"""
from __future__ import annotations

import argparse
import math
import sys
import time

sys.path.insert(0, "src/tests/bench")

from robot_radio.io.serial_conn import SerialConnection  # noqa: E402
from robot_radio.robot.protocol import NezhaProtocol, TLMFrame  # noqa: E402

# [deg/s] standstill drift at or below this is a healthy gyro. A good
# calibration measures ~0.006 deg/s; the failure mode measures 0.4-1.4.
DRIFT_PASS = 0.05

ERR_NAMES = {0: "OK", 4: "ERR_FULL", 8: "ERR_NOT_CONFIGURED", 10: "ERR_BUSY"}


def newest_frame(conn) -> TLMFrame | None:
    last = None
    for env in conn.drain_binary_tlm():
        t = getattr(env, "tlm", None)
        if t is not None:
            last = TLMFrame.from_pb2(t)
    return last


def wait_pose(conn, timeout: float = 4.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        f = newest_frame(conn)
        if f is not None and f.otos_reading is not None:
            return f
        time.sleep(0.05)
    return None


def measure_drift(conn, seconds: float) -> tuple[float, float] | None:
    """Standstill (heading drift [deg/s], position creep [cm])."""
    a = wait_pose(conn)
    if a is None:
        return None
    t0 = time.time()
    time.sleep(seconds)
    b = wait_pose(conn)
    if b is None:
        return None
    dt = time.time() - t0
    # OtosReading carries the pose in native units (heading [rad], x/y [mm]);
    # TLMFrame.otos is a wire-scaled int tuple, deliberately not used here.
    dh = math.degrees(math.atan2(
        math.sin(b.otos_reading.heading - a.otos_reading.heading),
        math.cos(b.otos_reading.heading - a.otos_reading.heading)))
    dxy = math.hypot(b.otos_reading.x - a.otos_reading.x,
                     b.otos_reading.y - a.otos_reading.y) / 10.0
    return dh / dt, dxy


def halt_verified(p: NezhaProtocol, conn, tries: int = 4) -> bool:
    for _ in range(tries):
        try:
            p.estop()
        except Exception:
            pass
        time.sleep(0.3)
        a = wait_pose(conn)
        time.sleep(0.3)
        b = wait_pose(conn)
        if (a is not None and b is not None and a.enc and b.enc
                and abs(b.enc[0] - a.enc[0]) <= 3
                and abs(b.enc[1] - a.enc[1]) <= 3):
            return True
    return False


def send_calibrate_acked(p: NezhaProtocol, conn, samples: int,
                         tries: int = 8) -> int | None:
    """CALIBRATE with resend-until-ack. Returns the ack's err, or None."""
    for attempt in range(tries):
        corr = p.calibrate_imu(samples)
        t0 = time.time()
        while time.time() - t0 < 1.5:
            for env in conn.drain_binary_tlm():
                t = getattr(env, "tlm", None)
                if t is None:
                    continue
                for ack in (TLMFrame.from_pb2(t).acks or []):
                    if ack.corr_id == corr:
                        return ack.err_code
            time.sleep(0.05)
        print(f"  no ack for CALIBRATE -- resending ({attempt + 1}/{tries})")
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--samples", type=int, default=0,
                    help="gyro samples to average, 1..255; 0 = firmware "
                         "default (255, ~612ms)")
    ap.add_argument("--measure", type=float, default=20.0,
                    help="[s] drift measurement window, before and after")
    ap.add_argument("--skip-before", action="store_true",
                    help="skip the before-measurement (saves --measure "
                         "seconds when you already know it is off)")
    args = ap.parse_args()

    conn = SerialConnection(port=args.port)
    conn.connect(skip_ping=False)
    p = NezhaProtocol(conn)
    try:
        conn.send_cleartext("TLM:ON")
        time.sleep(0.6)
        conn.drain_binary_tlm()

        print("parking (estop until encoders confirm)...")
        if not halt_verified(p, conn):
            print("FAIL: could not confirm the robot stopped -- not "
                  "calibrating a moving robot")
            return 2

        if not args.skip_before:
            d = measure_drift(conn, args.measure)
            if d is None:
                print("FAIL: no telemetry")
                return 2
            print(f"before: {d[0]:+.3f} deg/s drift, {d[1]:.2f}cm creep")

        print("sending CALIBRATE...")
        err = send_calibrate_acked(p, conn, args.samples)
        if err is None:
            print("FAIL: CALIBRATE never acked -- command not delivered")
            return 2
        print(f"  ack: {ERR_NAMES.get(err, err)}")
        if err != 0:
            return 2

        time.sleep(1.5)  # 255 samples ~= 612ms, plus margin
        conn.drain_binary_tlm()

        d = measure_drift(conn, args.measure)
        if d is None:
            print("FAIL: no telemetry after calibration")
            return 2
        verdict = "PASS" if abs(d[0]) <= DRIFT_PASS else "FAIL"
        print(f"after:  {d[0]:+.3f} deg/s drift, {d[1]:.2f}cm creep  "
              f"[{verdict}: |drift| {'<=' if verdict == 'PASS' else '>'} "
              f"{DRIFT_PASS} deg/s]")
        return 0 if verdict == "PASS" else 1
    finally:
        try:
            p.estop()
        except Exception:
            pass
        conn.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
