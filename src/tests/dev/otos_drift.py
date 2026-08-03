"""src/tests/bench/otos_drift.py -- OTOS heading drift over servo back-and-forth cycles.

The OTOS rides DIRECTLY on a 360deg servo: the SERVO command 0..180 maps ~linearly
to OTOS heading -pi..+pi, so SERVO 90 is the physical middle (heading ~0) and the
heading IS the servo shaft angle 1:1. This test:

  1. Drives the servo to the middle (SERVO 90) and lets it settle.
  2. Zeroes the OTOS there (ODO SETPOSE 0 0 0) -- middle == 0.
  3. Repeatedly turns it back and forth (center -> lo -> hi -> center).
  4. Each time it returns to the SAME physical center, reads the OTOS heading
     (and x/y): it should read 0, and how far it departs from 0 -- growing over
     cycles -- is the OTOS dead-reckoning drift.

Run:  uv run python src/tests/bench/otos_drift.py [cycles]   (default 40)
"""
from __future__ import annotations

import math
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from rig_dev import Rig  # noqa: E402

CENTER, LO, HI = 90, 30, 150  # [servo deg] middle, and +/-~115deg OTOS sweep (inside the +/-pi wrap)
DWELL = 0.8                    # [s] per servo move settle


def read_center(rig: Rig, n: int = 3):
    """Median OTOS heading + mean x/y from n reliable polls at rest. [rad] [mm]"""
    hs, xs, ys = [], [], []
    for _ in range(n):
        d = rig.odo()
        if "h" in d:
            hs.append(d["h"]); xs.append(d.get("x", 0.0)); ys.append(d.get("y", 0.0))
        time.sleep(0.05)
    if not hs:
        return None
    hs.sort()
    return hs[len(hs) // 2], sum(xs) / len(xs), sum(ys) / len(ys)


def drift(cycles: int = 40):
    rig = Rig(settle=3.0)
    rig.cmd("STREAM 0")  # clean polled reads, no TLM/STLM TX competing
    rig.servo(CENTER)
    time.sleep(1.5)
    rig.cmd("ODO SETPOSE 0 0 0")  # zero the OTOS at the physical middle
    time.sleep(0.4)
    z = read_center(rig)
    if z is None:
        print("FAIL: no OTOS reading at center -- is it connected? (ODIAG)")
        rig.close()
        return []
    print(f"zeroed at center (SERVO {CENTER}): h={math.degrees(z[0]):+.2f} deg "
          f"x={z[1]:+.1f} y={z[2]:+.1f}  (should be ~0)")

    rows = []
    t0 = time.monotonic()
    for c in range(1, cycles + 1):
        rig.servo(LO); time.sleep(DWELL)
        rig.servo(HI); time.sleep(DWELL)
        rig.servo(CENTER); time.sleep(DWELL + 0.3)  # extra settle before measuring
        m = read_center(rig)
        if m is None:
            continue
        t = time.monotonic() - t0
        hd = math.degrees(m[0])
        rows.append((c, t, hd, m[1], m[2]))
        print(f"  cycle {c:3d} t={t:6.0f}s  heading_drift={hd:+7.2f} deg  "
              f"x={m[1]:+7.1f} mm  y={m[2]:+7.1f} mm")

    rig.servo(CENTER)
    rig.close()

    if len(rows) >= 2:
        hd0, hdN, cyc, dur = rows[0][2], rows[-1][2], rows[-1][0], rows[-1][1]
        mx = max(rows, key=lambda r: abs(r[2]))
        print(f"\n=== OTOS DRIFT SUMMARY: {cyc} cycles over {dur:.0f}s ===")
        print(f"  heading: start {hd0:+.2f} -> end {hdN:+.2f} deg "
              f"(net {hdN - hd0:+.2f} deg, {(hdN - hd0) / cyc:+.3f} deg/cycle)")
        print(f"  max |heading drift|: {mx[2]:+.2f} deg (cycle {mx[0]})")
        print(f"  x/y at end: {rows[-1][3]:+.1f} / {rows[-1][4]:+.1f} mm")
    return rows


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 40
    print(f"=== OTOS drift test: {n} back-and-forth cycles ===")
    drift(n)
