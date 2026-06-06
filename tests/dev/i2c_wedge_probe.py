#!/usr/bin/env python3
"""i2c_wedge_probe.py — use the raw I2CW/I2CR commands to test the encoder wedge
mechanism and hunt a software recovery. STAND ONLY.

Nezha controller @ 0x10, 8-byte frames (motor left=M2, right=M1):
  move/coast : FF F9 <id> <dir> 60 <speed> F5 00
  shutdown   : FF F9 <id> 00 5F 00 F5 00     # documented to WEDGE encoder reads
  enc read   : FF F9 <id> 00 46 00 F5 00  then read 4 bytes

Sequence:
  1. drive briefly, confirm the encoder counts (baseline)
  2. fire the 0x5F shutdown at both motors via I2CW
  3. drive again — did the encoder wedge?
  4. if wedged, try candidate software recoveries and re-check
"""

from __future__ import annotations
import sys, time


def main() -> int:
    from robot_radio.io.cli import _make_robot
    from robot_radio.robot.nezha import Nezha
    from robot_radio.robot.protocol import parse_tlm

    class _A:
        port = None
        verbose = False

    robot, conn, _ = _make_robot(_A())
    proto = robot._proto
    enc = [0, 0]

    def pump(ms=25):
        for line in proto.read_lines(duration_ms=ms):
            t = parse_tlm(line)
            if t is not None and t.enc is not None:
                enc[0], enc[1] = t.enc

    def snap():
        r = proto.send("SNAP", 250)
        for line in r.get("responses", []):
            t = parse_tlm(line)
            if t is not None and t.enc is not None:
                enc[0], enc[1] = t.enc
        return (enc[0], enc[1])

    def drive_measure(secs=1.0, speed=80):
        e0 = snap()
        t0 = time.monotonic(); last = 0.0
        while time.monotonic() - t0 < secs:
            now = time.monotonic()
            if now - last >= 0.15:
                proto.drive(speed, speed); last = now
            pump(20)
        for _ in range(3):
            proto.stop(); time.sleep(0.05); pump(15)
        time.sleep(0.2)
        e1 = snap()
        return (e1[0] - e0[0], e1[1] - e0[1])

    def i2c(cmd):
        r = proto.send(cmd, 300)
        for line in r.get("responses", []):
            s = line.strip()
            if s.startswith("OK i2c") or s.startswith("I2C "):
                return s
        return str(r.get("responses", []))

    try:
        proto.send("SET sTimeout=10000", 300)
        proto.send("OI", 400)
        proto.zero_encoders()
        proto.send("STREAM fields=enc,pose,vel", 250)
        proto.stream(50)
        time.sleep(0.2); pump(60)

        print("1. baseline drive:")
        d = drive_measure(1.0, 80)
        print(f"   enc delta = {d}  ({'COUNTS' if max(abs(d[0]),abs(d[1]))>10 else 'NO MOVE?'})")
        print(f"   I2CR 10 4 = {i2c('I2CR 10 4')}")

        print("2. firing 0x5F shutdown at both motors:")
        print(f"   M2(left):  {i2c('I2CW 10 FF F9 02 00 5F 00 F5 00')}")
        print(f"   M1(right): {i2c('I2CW 10 FF F9 01 00 5F 00 F5 00')}")
        time.sleep(0.3)

        print("3. drive again after 0x5F:")
        d = drive_measure(1.0, 80)
        wedged = max(abs(d[0]), abs(d[1])) < 10
        print(f"   enc delta = {d}  ({'WEDGED ✗' if wedged else 'still counting ✓'})")
        print(f"   I2CR 10 4 = {i2c('I2CR 10 4')}")
        print(f"   DBG I2C   = {i2c('DBG I2C')}")

        if wedged:
            print("4. trying recoveries:")
            candidates = [
                ("resend move 0x60 then drive", ["I2CW 10 FF F9 02 01 60 20 F5 00",
                                                 "I2CW 10 FF F9 01 01 60 20 F5 00"]),
                ("enc-read kick 0x46",          ["I2CW 10 FF F9 02 00 46 00 F5 00",
                                                 "I2CW 10 FF F9 01 00 46 00 F5 00"]),
            ]
            for name, cmds in candidates:
                for c in cmds:
                    i2c(c)
                time.sleep(0.3)
                d = drive_measure(1.0, 80)
                ok = max(abs(d[0]), abs(d[1])) > 10
                print(f"   [{name}] → enc delta {d}  ({'RECOVERED ✓' if ok else 'still wedged'})")
                if ok:
                    print("   *** software recovery found ***")
                    break
            else:
                print("   no software recovery from these — likely needs power-cycle.")
    finally:
        try:
            for _ in range(3):
                proto.stop(); time.sleep(0.04)
            proto.stream(0); conn.disconnect()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
