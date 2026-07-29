#!/usr/bin/env python3
"""deadman_gate.py -- does a bounded command actually stop ON ITS OWN?

THE PLAYFIELD SAFETY GATE. protocol v5 has no session watchdog; the entire
"a robot cannot run away" guarantee rests on every command carrying its own
`duration`/`timeout` bound and the FIRMWARE honouring it with no further
host traffic (docs/protocol-v5.md Sec 5, "no deadman"). On a stand a
failure here is a curiosity. On the playfield it is a robot on the floor.

Observed 2026-07-28 and still unexplained: a WHEELS command with
duration=20000 ms kept the wheels turning for many minutes across two
subsequent script runs, and 15 ESTOP+STOP pairs plus 10 WHEELS 0,0 commands
did not stop it -- only cutting power did. The ESTOP half of that is now
explained (inbound commands were being swallowed by a periodic blind window
in the I2C interrupt mask, since fixed). The DURATION half is not: expiry
is entirely on-robot and must hold even with the link dead.

This test isolates exactly that. It arms a command and then goes SILENT --
no estop, no polling, no traffic of any kind -- and watches telemetry to
see when the wheels actually stop. Silence is the point: any host packet
could mask the very failure being tested.

    uv run python src/tests/bench/deadman_gate.py --port /dev/cu.usbmodem2121102

Robot on a stand, wheels off the ground. A hard estop runs in the finally
block regardless of outcome.
"""
from __future__ import annotations

import argparse
import math
import sys
import time

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
BOOT_WAIT = 5.0
SPEED = 200.0        # [mm/s] well clear of the dead zone
REST = 5.0           # [mm/s] "stopped"
GRACE = 1.5          # [s] allowance past the bound before calling it a failure
WATCH_EXTRA = 4.0    # [s] keep watching past the deadline to catch late stops


def armAndWatch(proto, kind: str, bound: float, moveId: int) -> dict:
    """Arm one bounded command, then stay SILENT and watch it expire.

    `bound` is [s]. Returns the observed stop time relative to arming.
    """
    proto.read_pending_binary_tlm_frames()
    if kind == "wheels":
        proto.wheels(v_left=SPEED, v_right=SPEED, duration=bound * 1000.0)
    else:  # a MOVE with a deliberately unreachable stop condition, so the
           # only thing that can end it is its own `timeout` backstop.
        proto.move_twist(v_x=SPEED, v_y=0.0, omega=0.0,
                         stop_distance=1.0e6, timeout=bound * 1000.0,
                         replace=False, move_id=moveId)

    t0 = time.monotonic()
    started = False
    stoppedAt = None
    lastMoving = None
    deadline = bound + GRACE + WATCH_EXTRA
    # FROM HERE ON: send NOTHING. Reading is passive.
    while time.monotonic() - t0 < deadline:
        for f in proto.read_pending_binary_tlm_frames():
            if f.enc_left is None:
                continue
            t = time.monotonic() - t0
            speed = max(abs(f.enc_left.velocity), abs(f.enc_right.velocity))
            if speed > 3 * REST:
                started = True
                lastMoving = t
                stoppedAt = None
            elif started and speed <= REST and stoppedAt is None:
                stoppedAt = t
        time.sleep(0.01)

    return {"kind": kind, "bound": bound, "started": started,
            "stoppedAt": stoppedAt, "lastMoving": lastMoving}


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--relay", action="store_true")
    args = p.parse_args()

    conn = SerialConnection(port=args.port,
                            mode="relay" if args.relay else None)
    conn.connect()
    proto = NezhaProtocol(conn)
    print(f"connected on {args.port}; waiting {BOOT_WAIT:.0f}s for boot")
    time.sleep(BOOT_WAIT)
    proto.read_pending_binary_tlm_frames()

    results = []
    try:
        cases = [("wheels", 1.0), ("wheels", 2.0), ("wheels", 4.0),
                 ("move", 2.0), ("move", 4.0)]
        for i, (kind, bound) in enumerate(cases):
            print(f"\n  arming {kind} bound={bound:.1f}s, then going silent...",
                  flush=True)
            r = armAndWatch(proto, kind, bound, 7000 + i)
            results.append(r)
            if not r["started"]:
                print("    WARNING: wheels never started -- inconclusive")
            elif r["stoppedAt"] is None:
                print(f"    *** STILL RUNNING at +{r['lastMoving']:.2f}s "
                      f"(bound {bound:.1f}s) ***")
            else:
                over = r["stoppedAt"] - bound
                print(f"    stopped at {r['stoppedAt']:.2f}s "
                      f"(bound {bound:.1f}s, {over:+.2f}s)")
            # Hard stop between cases so one failure cannot pollute the next.
            for _ in range(4):
                proto.estop()
                time.sleep(0.08)
            time.sleep(1.0)
    finally:
        for _ in range(6):
            proto.estop()
            time.sleep(0.08)
        conn.disconnect()

    print("\n" + "=" * 62)
    print("DEADMAN GATE")
    print("=" * 62)
    failures = 0
    for r in results:
        if not r["started"]:
            verdict = "INCONCLUSIVE (never started)"
        elif r["stoppedAt"] is None:
            verdict = "FAIL -- never stopped"
            failures += 1
        elif r["stoppedAt"] > r["bound"] + GRACE:
            verdict = f"FAIL -- {r['stoppedAt'] - r['bound']:+.2f}s late"
            failures += 1
        else:
            verdict = f"pass ({r['stoppedAt'] - r['bound']:+.2f}s)"
        print(f"  {r['kind']:6s} bound {r['bound']:4.1f}s -> {verdict}")
    print()
    if failures:
        print(f"{failures}/{len(results)} FAILED -- the on-robot bound does "
              f"NOT hold. NOT SAFE FOR THE PLAYFIELD.")
        return 1
    print("All bounds honoured with the host silent. "
          "The no-deadman guarantee holds.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
