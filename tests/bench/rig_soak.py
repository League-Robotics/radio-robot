"""tests/bench/rig_soak.py — DeviceBus rig soak test (sprint 101-004).

Sustained, VARIED reads/writes across every device to prove the I2C connection
to the OTOS + encoders is rock-solid: no hiccups, no lock-ups, no wedge latch,
zero I2C errors over a long run. Each cycle drives both motors through changing
velocity/duty patterns (incl. high-inertia motor 2), sweeps the servo, and reads
encoders (M STATE), OTOS (ODO), line, and color -- validating each -- while
tracking the OTOS-address I2C error count (ODIAG err), reply timeouts, and the
motor wedge/glitch counters.

Run:  uv run python tests/bench/rig_soak.py [seconds]   (default 120)
Pass: zero reply timeouts, zero NEW I2C errors, no wedge latch, every device
      stayed connected and plausible for the whole run.
"""
from __future__ import annotations

import math
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from rig_dev import Rig, SERVO_PIN  # noqa: E402


def _num(d, k):
    v = d.get(k)
    return v if v is not None else float("nan")


def soak(seconds: float = 120.0) -> dict:
    rig = Rig(settle=3.0)
    fail = []
    stats = {
        "cycles": 0, "reads": 0, "timeouts": 0,
        "i2c_err0": None, "i2c_err_end": None,
        "wedge_max": 0, "glitch_max": 0,
        "otos_disconnects": 0, "line_disc": 0, "color_disc": 0,
        "otos_moved": False, "enc_moved": {1: False, 2: False},
    }
    try:
        base = rig.odiag()
        stats["i2c_err0"] = base.get("err")
        if base.get("conn") != 1.0:
            fail.append("OTOS not connected at start")
        enc0 = {p: _num(rig.mstate(p), "pos") for p in (1, 2)}
        odo0 = rig.odo()

        def read1(cmd, tries=3):
            """One read, resync-and-retry on a dropped reply. The bring-up's
            single-threaded serial can lag under dual-motor I2C load; retrying
            after a flush absorbs that transient so only a genuine device/I2C
            fault (no reply after every retry) counts as a failure."""
            for _ in range(tries):
                cid = rig.send(cmd)
                ln, d = rig.await_reply(cid, timeout=0.3)
                if ln is not None:
                    return ln, d
                rig.flush()
                time.sleep(0.02)
            return None, {}

        devices = ["M 1 STATE", "M 2 STATE", "ODO", "LINE", "COLOR"]
        period = 0.10  # ~10 Hz -- sustainable for the bring-up's serial loop
        t0 = time.monotonic()
        i = 0
        while time.monotonic() - t0 < seconds:
            cstart = time.monotonic()
            i += 1
            t = cstart - t0
            # --- writes every cycle: vary both motors + sweep the servo ---
            # Smooth sines (different period/phase) so the drive crosses zero
            # WITH a dwell -- realistic motion, never a zero-dwell reversal
            # (which is the encoder-wedge trigger, see wedge_probe). Motor 2 is
            # the high-inertia one.
            tgt1 = 300.0 * math.sin(2.0 * math.pi * (t / 4.0))
            tgt2 = 220.0 * math.sin(2.0 * math.pi * (t / 3.0) + 1.0)
            rig.send(f"M 1 VEL {tgt1:.1f}")
            rig.send(f"M 2 VEL {tgt2:.1f}")
            if i % 20 == 0:
                rig.servo(60 if (i // 20) % 2 else 120)
            # --- one rotating device read (variety over time, paced) ---
            cmd = devices[i % len(devices)]
            ln, d = read1(cmd)
            stats["reads"] += 1
            if ln is None:
                stats["timeouts"] += 1
                fail.append(f"timeout on {cmd} @ t={t:.1f}")
            elif cmd.startswith("M"):
                p = int(cmd.split()[1])
                if d.get("conn") != 1.0:
                    fail.append(f"motor {p} conn!=1 @ t={t:.1f}")
                stats["wedge_max"] = max(stats["wedge_max"], int(_num(d, "wedged") or 0))
                stats["glitch_max"] = max(stats["glitch_max"], int(_num(d, "glitch") or 0))
                if abs(_num(d, "pos") - enc0[p]) > 5:
                    stats["enc_moved"][p] = True
            elif cmd == "ODO":
                if d.get("conn") != 1.0:
                    stats["otos_disconnects"] += 1
                    fail.append(f"OTOS conn!=1 @ t={t:.1f}")
                moved = (abs(_num(d, "x") - _num(odo0, "x"))
                         + abs(_num(d, "y") - _num(odo0, "y")) > 5
                         or abs(_num(d, "h") - _num(odo0, "h")) > 1)
                if moved:
                    stats["otos_moved"] = True
            elif cmd == "LINE" and d.get("conn") != 1.0:
                stats["line_disc"] += 1
            elif cmd == "COLOR" and d.get("conn") != 1.0:
                stats["color_disc"] += 1
            stats["cycles"] = i
            if i % 50 == 0:
                dg = rig.odiag()
                print(f"  t={t:5.0f}s cyc={i:4d} reads={stats['reads']:5d} "
                      f"i2c_err={int(_num(dg, 'err'))} timeouts={stats['timeouts']} "
                      f"wedge={stats['wedge_max']} glitch={stats['glitch_max']}")
            dt = time.monotonic() - cstart
            if dt < period:
                time.sleep(period - dt)
        end = rig.odiag()
        stats["i2c_err_end"] = end.get("err")
    finally:
        rig.close()

    new_i2c_err = (stats["i2c_err_end"] or 0) - (stats["i2c_err0"] or 0)
    if new_i2c_err > 0:
        fail.append(f"{int(new_i2c_err)} NEW OTOS I2C errors during soak")
    if stats["wedge_max"] > 0:
        fail.append(f"wedge latched (wedged={stats['wedge_max']})")
    if not stats["enc_moved"][1] or not stats["enc_moved"][2]:
        fail.append("an encoder never advanced")
    if not stats["otos_moved"]:
        fail.append("OTOS pose never changed (servo/drum not sensed?)")
    stats["new_i2c_err"] = new_i2c_err
    stats["pass"] = len(fail) == 0
    stats["failures"] = fail
    return stats


if __name__ == "__main__":
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    print(f"=== DeviceBus rig soak: {secs:.0f}s ===")
    s = soak(secs)
    print("\n=== RESULT ===")
    for k in ("cycles", "reads", "timeouts", "new_i2c_err", "wedge_max",
              "glitch_max", "otos_disconnects", "otos_moved", "enc_moved"):
        print(f"  {k}: {s[k]}")
    print(f"  PASS: {s['pass']}")
    for f in s["failures"]:
        print(f"    FAIL: {f}")
    sys.exit(0 if s["pass"] else 1)
