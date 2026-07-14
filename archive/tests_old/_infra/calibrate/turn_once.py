#!/usr/bin/env python3
"""turn_once.py — drive ONE in-place TURN over the relay and report the ONBOARD
heading delta (no camera). Pair with MCP get_tags before/after for camera truth.

    uv run python tests/_infra/calibrate/turn_once.py --turn 60
"""
from __future__ import annotations

import argparse
import sys
import time

from robot_radio.robot.nezha import Nezha
from robot_radio.robot.protocol import NezhaProtocol
from robot_radio.io.serial_conn import SerialConnection, list_serial_ports


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--turn", type=float, default=60.0, help="turn delta degrees (signed)")
    ap.add_argument("--port", default=None)
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="print serial traffic: '>>' lines sent, '<<' lines received")
    a = ap.parse_args(argv)

    port = a.port or (list_serial_ports() or [None])[0]
    if port is None:
        print("no serial port"); return 2
    tx = (lambda s: print(f"  >> {s}", flush=True)) if a.verbose else None
    rx = (lambda s: print(f"  << {s}", flush=True)) if a.verbose else None
    conn = SerialConnection(port=port, on_send=tx, on_recv=rx); conn.connect()
    proto = NezhaProtocol(conn); robot = Nezha(proto)
    print("connected:", robot.connect().get("fw"))
    try:
        proto.stream_fields("enc,pose,otos")
    except Exception:
        pass

    def _snap(tries=5):
        """SNAP with retry. The relay's RAW250 framing can merge a keepalive
        '+' into the command and corrupt it (firmware replies 'ERR unknown'),
        so a single SNAP occasionally yields no TLM — just retry."""
        for _ in range(tries):
            t = proto.snap()
            if t and t.pose:
                return t
            time.sleep(0.3)
        return None

    t0 = _snap()
    h0 = t0.pose[2] if (t0 and t0.pose) else None
    o0 = t0.otos[2] if (t0 and t0.otos) else None
    if h0 is None:
        print("no onboard heading snap"); conn.disconnect(); return 1

    tgt = int(round(((h0 + a.turn * 100) + 18000) % 36000 - 18000))  # wrap ±180°
    print(f"onboard before: fused={h0/100:+.1f}° otos={o0/100 if o0 is not None else 'na'}  "
          f"→ TURN to {tgt/100:+.1f}° (Δcmd {a.turn:+.0f}°)")
    proto.turn(tgt, eps=100)
    proto.wait_for_evt_done("TURN", 12000)
    proto.stop(); time.sleep(0.8)

    t1 = _snap()
    h1 = t1.pose[2] if (t1 and t1.pose) else None
    o1 = t1.otos[2] if (t1 and t1.otos) else None
    if h1 is None:
        print("onboard after : <no SNAP reply after retries> — the turn likely "
              "completed (see 'EVT done TURN'); relay corrupted the final SNAP.")
        conn.disconnect()
        return 1
    wrap = lambda d: (d + 18000) % 36000 - 18000
    df = wrap(h1 - h0) / 100.0
    do = wrap(o1 - o0) / 100.0 if (o0 is not None and o1 is not None) else float("nan")
    print(f"onboard after : fused={h1/100:+.1f}°  Δfused={df:+.1f}°  Δotos={do:+.1f}°  (cmd {a.turn:+.0f}°)")
    conn.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
