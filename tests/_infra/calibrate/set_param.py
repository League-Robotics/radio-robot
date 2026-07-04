#!/usr/bin/env python3
"""set_param.py — SET arbitrary firmware config keys at runtime (no flash).

    uv run python tests/_infra/calibrate/set_param.py alphaYaw=1.0 yawRateMax=60
"""
from __future__ import annotations

import sys

from robot_radio.robot.nezha import Nezha
from robot_radio.robot.protocol import NezhaProtocol
from robot_radio.io.serial_conn import SerialConnection, list_serial_ports


def main(argv=None) -> int:
    argv = argv if argv is not None else sys.argv[1:]
    pairs = [a for a in argv if "=" in a]
    if not pairs:
        print("usage: set_param.py key=val [key=val ...]"); return 2
    kv = {}
    for p in pairs:
        k, v = p.split("=", 1)
        try:
            kv[k] = float(v) if ("." in v or "e" in v.lower()) else int(v)
        except ValueError:
            kv[k] = v
    port = (list_serial_ports() or [None])[0]
    if port is None:
        print("no serial port"); return 2
    conn = SerialConnection(port=port); conn.connect()
    proto = NezhaProtocol(conn); robot = Nezha(proto)
    print("connected:", robot.connect())
    # The radio intermittently corrupts a SET into 'ERR unknown' / drops the ack
    # (keepalive-merge). Retry the combined SET until the firmware echoes OK set.
    cmd = "SET " + " ".join(
        f"{k}={v:g}" if isinstance(v, float) else f"{k}={v}" for k, v in kv.items()
    )
    ok = False
    for attempt in range(6):
        r = conn.send(cmd, read_timeout=600)
        lines = r.get("responses", [])
        if any("OK set" in ln for ln in lines):
            print(f"  applied (try {attempt+1}): {[l for l in lines if 'OK set' in l]}")
            ok = True
            break
        print(f"  try {attempt+1}: {lines}")
    if not ok:
        print("  FAILED to confirm SET after retries")
    conn.disconnect()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
