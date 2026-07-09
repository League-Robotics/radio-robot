#!/usr/bin/env python3
"""Sprint 093 bench verification of the COMMAND PLANE in isolation.

main() currently runs ONLY the communication path (no control loop, no
hardware): Communicator -> CommandRouter -> CommandProcessor -> Blackboard +
reply. Because nothing ticks/drains the blackboard queues, a routed
queue-posting command ACCUMULATES on its target queue -- so the `QLEN` debug
command's `drive` count going 0->1 after an `S` is a direct proof the command
was parsed and posted to the right queue (bb.driveIn).

Uses the canonical SerialConnection + NezhaProtocol (id-correlated, pipelined
reads) -- NOT hand-rolled lock-step pyserial. Runs against a micro:bit over
direct USB; needs no motors/brick attached (main() touches no hardware).

Usage:
    uv run python tests/bench/comms_plane_verify.py [--port /dev/cu.usbmodem2121102]
"""
from __future__ import annotations

import argparse
import sys
import time

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol, parse_response


def one(proto, cmd, timeout=700, retries=6):
    """Send one command; return the first OK/ERR ParsedResponse (retry on silence).

    Retry-on-silence rides out DAPLink small-write batching (a lone reply
    awaiting a flush) -- a host-side artifact, never a flaky link/chip. Every
    command here is a pure query or an idempotent absolute-value write, so
    re-send is safe.
    """
    for attempt in range(retries):
        resp = proto.send(cmd, timeout)
        for raw in resp.get("responses", []):
            r = parse_response(raw)
            if r is not None and r.tag in ("OK", "ERR"):
                return r
        if attempt < retries - 1:
            time.sleep(0.1)
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/cu.usbmodem2121102")
    args = ap.parse_args()

    conn = SerialConnection(port=args.port)
    info = conn.connect()
    if info.get("status") != "connected":
        print(f"ERROR: connect failed: {info}")
        return 2
    ann = info.get("announcement")
    proto = NezhaProtocol(conn)

    results = []

    def check(label, cond, detail=""):
        print(f"  [{'PASS' if cond else 'FAIL'}] {label}   {detail}")
        results.append(cond)

    # 1. Communicator up + DEVICE: banner emitted by Communicator::begin()
    #    (SerialConnection classifies it out-of-band into `announcement`).
    check("Communicator up + DEVICE banner (from begin())",
          info.get("mode") == "direct" and bool(ann), f"announcement={ann}")

    # 2. System commands reply correctly.
    r = one(proto, "PING")
    check("PING -> OK pong", r is not None and r.tag == "OK" and "pong" in r.tokens, _fmt(r))
    r = one(proto, "VER")
    check("VER -> OK ver (fw/proto)", r is not None and r.tag == "OK" and "fw" in r.kv and "proto" in r.kv, _fmt(r))
    r = one(proto, "ECHO hello 42")
    check("ECHO hello 42 -> OK echo hello 42",
          r is not None and r.tag == "OK" and r.tokens[:3] == ["echo", "hello", "42"], _fmt(r))

    # 3. A queue-posting command lands on the RIGHT queue (bb.driveIn).
    #    Nothing drains it (no loop.tick), so QLEN's drive count goes 0 -> 1.
    base = one(proto, "QLEN")
    check("QLEN baseline -> drive=0", base is not None and base.kv.get("drive") == "0", _fmt(base))
    r = one(proto, "S 200 200")
    check("S 200 200 -> OK drive", r is not None and r.tag == "OK" and "drive" in r.tokens, _fmt(r))
    after = one(proto, "QLEN")
    check("QLEN after S -> drive=1 (landed on driveIn)",
          after is not None and after.kv.get("drive") == "1", _fmt(after))

    # 4. STOP posts too; motion queue stays empty (S/STOP use driveIn, not motionIn).
    r = one(proto, "STOP")
    check("STOP -> OK stop", r is not None and r.tag == "OK" and "stop" in r.tokens, _fmt(r))
    q = one(proto, "QLEN")
    check("QLEN motion stays 0 (S/STOP -> driveIn only)",
          q is not None and q.kv.get("motion") == "0", _fmt(q))

    # 5. An out-of-surface verb is rejected -> proves parse+dispatch, not just survivors.
    r = one(proto, "DEV WD 100")
    check("DEV WD 100 -> ERR unknown", r is not None and r.tag == "ERR" and "unknown" in r.tokens, _fmt(r))

    conn.disconnect()
    n = sum(results)
    print(f"\n{n}/{len(results)} checks passed")
    return 0 if all(results) else 1


def _fmt(r):
    if r is None:
        return "(no reply)"
    return f"{r.tag} {' '.join(r.tokens)} {r.kv}"


if __name__ == "__main__":
    sys.exit(main())
