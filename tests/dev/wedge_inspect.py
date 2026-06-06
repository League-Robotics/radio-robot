#!/usr/bin/env python3
"""wedge_inspect.py — probe a WEDGED Nezha chip: is it fully dead or just the
encoder value? STAND ONLY. Drives both wheels for a few seconds — WATCH THEM.

Reads several 0x10 registers via I2CR and commands a sustained spin while
streaming the encoder, so we can tell motor-control-works-but-encoder-frozen
apart from whole-controller-dead.
"""
from __future__ import annotations
import sys, time


def main() -> int:
    from robot_radio.io.cli import _make_robot
    from robot_radio.robot.nezha import Nezha
    from robot_radio.robot.protocol import parse_tlm

    class _A:
        port = None; verbose = False

    robot, conn, _ = _make_robot(_A())
    proto = robot._proto
    enc = [0, 0]

    def pump(ms=25):
        for line in proto.read_lines(duration_ms=ms):
            t = parse_tlm(line)
            if t is not None and t.enc is not None:
                enc[0], enc[1] = t.enc

    def i2c(cmd):
        r = proto.send(cmd, 300)
        for line in r.get("responses", []):
            s = line.strip()
            if s.startswith("OK i2c") or s.startswith("I2C "):
                return s
        return str(r.get("responses", []))

    try:
        proto.send("SET sTimeout=10000", 300)
        proto.send("STREAM fields=enc,vel", 250)
        proto.stream(50)
        time.sleep(0.2); pump(60)

        print("== register probe of 0x10 (chip responds at all?) ==")
        for label, reqs in [
            ("enc reg 0x46 (M2 left)",  ["I2CW 10 FF F9 02 00 46 00 F5 00"]),
            ("enc reg 0x46 (M1 right)", ["I2CW 10 FF F9 01 00 46 00 F5 00"]),
            ("speed reg 0x47 (M2)",     ["I2CW 10 FF F9 02 00 47 00 F5 00"]),
        ]:
            for c in reqs:
                i2c(c)
            print(f"   {label:24s} → {i2c('I2CR 10 4')}")

        print(f"   raw read 8 bytes        → {i2c('I2CR 10 8')}")
        print(f"   DBG I2C                 → {i2c('DBG I2C')}")

        print("\n== commanding spin S 150 150 for 5s — WATCH THE WHEELS ==")
        enc0 = (enc[0], enc[1])
        t0 = time.monotonic(); last = 0.0
        while time.monotonic() - t0 < 5.0:
            now = time.monotonic()
            if now - last >= 0.15:
                proto.drive(150, 150); last = now
            pump(20)
            if int((now - t0) * 2) != int((now - t0 - 0.05) * 2):
                pass
        print(f"   encoder during spin: start={enc0} end=({enc[0]},{enc[1]})  "
              f"delta=({enc[0]-enc0[0]},{enc[1]-enc0[1]})")
        for _ in range(3):
            proto.stop(); time.sleep(0.05)
        print("\n   >>> Did the wheels physically spin? (motor path works if yes)")
        print("   >>> encoder delta above tells us if the count moved.")

        # Vendor reset() = 0x1D ("set servo to zero"). Try it as a recovery.
        print("\n== trying vendor reset 0x1D on both motors, then drive ==")
        i2c("I2CW 10 FF F9 02 00 1D 00 F5 00")
        i2c("I2CW 10 FF F9 01 00 1D 00 F5 00")
        time.sleep(0.5); pump(40)
        enc0 = (enc[0], enc[1])
        t0 = time.monotonic(); last = 0.0
        while time.monotonic() - t0 < 3.0:
            now = time.monotonic()
            if now - last >= 0.15:
                proto.drive(150, 150); last = now
            pump(20)
        d = (enc[0] - enc0[0], enc[1] - enc0[1])
        for _ in range(3):
            proto.stop(); time.sleep(0.05)
        proto.stream(0)
        print(f"   after 0x1D reset: encoder delta = {d}  "
              f"({'RECOVERED ✓' if max(abs(d[0]),abs(d[1]))>10 else 'still frozen'})")
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
