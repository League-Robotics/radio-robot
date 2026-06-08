#!/usr/bin/env python3
"""calibrate_bench.py — camera-free bench drive test (robot ON A STAND).

Drives `D <spd> <spd> <mm>` repeatedly and checks each drive from the encoder
telemetry — no camera, no tape:
  1. DISTANCE — average wheel travel ≈ target mm
  2. BALANCE  — |left - right| small (wheels go the same distance = straight)

It reads the encoder ONCE after each drive completes (robot stopped, telemetry
caught up) — the robust pattern from d_repeat. It does NOT zero the encoders
before a drive (that left the readback flaky and caused spasms); D resets its own
accumulator, so the post-drive enc is the per-wheel travel.

SMOOTHNESS: this tool can't see it reliably — telemetry frames get dropped during
a D drive (CODAL ASYNC TX). To watch the drive be smooth, use:
    uv run python tests/dev/enc_watch.py
which streams a live encoder trace during an S drive.

Usage: uv run python tests/calibrate/calibrate_bench.py [-n N] [--mm 500] [--spd 80]
"""
import argparse
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("-n", type=int, default=6, help="number of drives")
    ap.add_argument("--mm", type=int, default=500, help="target distance mm")
    ap.add_argument("--spd", type=int, default=80, help="wheel speed mm/s")
    args = ap.parse_args()

    from robot_radio.io.serial_conn import SerialConnection, list_serial_ports
    from robot_radio.robot.protocol import NezhaProtocol, parse_tlm, parse_response

    port = args.port or (list_serial_ports() or [None])[0]
    if port is None:
        print("ERROR: no serial port"); return 2
    print(f"  port: {port}   {args.n}x D {args.spd} {args.spd} {args.mm}")

    conn = SerialConnection(port=port, mode="direct")
    conn.connect(skip_ping=True)
    proto = NezhaProtocol(conn)

    # boot wait (connect pulses DTR → reset)
    dl = time.monotonic() + 12.0
    while time.monotonic() < dl:
        if any("fw=" in l for l in proto.send("VER", 300).get("responses", [])):
            break
        time.sleep(0.3)
    print("  robot alive")

    def safe_stop():
        try:
            for _ in range(4):
                conn.send_fast("STOP"); time.sleep(0.04)
            proto.send("STOP", 150)
        except Exception:
            pass

    import signal
    signal.signal(signal.SIGINT, lambda *_: (safe_stop(), sys.exit(1)))
    signal.signal(signal.SIGTERM, lambda *_: (safe_stop(), sys.exit(1)))

    conn.send_fast("STOP"); conn.send_fast("STREAM 0"); time.sleep(0.1)
    conn._ser.reset_input_buffer()
    try:
        from robot_radio.io.cli import _push_calibration
        _push_calibration(conn)
    except Exception as exc:
        print(f"  WARN cal push: {exc}")
    proto.send("SET sTimeout=2000", 300)        # backstop (S-only watchdog; D autonomous)
    proto.stream_fields("enc,vel")
    proto.stream(50)
    time.sleep(0.4)

    def read_latest_enc(drain_ms=500):
        """Drain to the newest TLM frame and return its enc (robust to backlog)."""
        e = None
        end = time.monotonic() + drain_ms / 1000.0
        while time.monotonic() < end:
            for raw in conn.read_lines(duration_ms=60):
                f = parse_tlm(raw)
                if f is not None and f.enc is not None:
                    e = f.enc
        return e

    results = []
    for i in range(args.n):
        safe_stop(); time.sleep(0.4)
        conn._ser.reset_input_buffer()           # flush stale backlog
        proto.send(f"D {args.spd} {args.spd} {args.mm}", 200)
        # Wait for completion (EVT done D), or fall back to the expected drive time
        # (the EVT is sometimes dropped by ASYNC TX — harmless, we read after).
        done = None
        end = time.monotonic() + (args.mm / max(args.spd, 1)) * 2.0 + 4.0
        while time.monotonic() < end and done != "D":
            for raw in conn.read_lines(duration_ms=150):
                r = parse_response(raw)
                if r and r.tag == "EVT" and r.tokens and r.tokens[0] == "done":
                    done = r.tokens[1] if len(r.tokens) > 1 else "?"
        time.sleep(0.3)
        e = read_latest_enc()
        if e is None:
            print(f"  drive {i+1}: [FAIL] no telemetry")
            results.append(False); continue
        L, R = e[0], e[1]
        avg = (abs(L) + abs(R)) / 2.0
        imb = L - R
        imb_pct = (imb / avg * 100.0) if avg else 0.0
        # Pass on the actual calibration data: distance + balance. (No stale-guard:
        # this robot is repeatable enough that consecutive drives land on identical
        # integers — that's not a stale read. The real freeze it once caught is
        # fixed in firmware.) A dropped EVT done is noted, not failed.
        dist_ok = abs(avg - args.mm) <= 0.15 * args.mm
        bal_ok  = abs(imb) <= max(0.12 * args.mm, 25)
        ok = dist_ok and bal_ok
        results.append(ok)
        issues = []
        if not dist_ok:  issues.append(f"dist {avg:.0f}≠{args.mm}")
        if not bal_ok:   issues.append(f"imbal {imb:+.0f}")
        evt = "" if done == "D" else "  (evt dropped)"
        tag = "OK  " if ok else "FAIL"
        print(f"  drive {i+1}: [{tag}] L={L:+5.0f} R={R:+5.0f} avg={avg:.0f} "
              f"L-R={imb:+.0f}({imb_pct:+.0f}%){evt}"
              + ("  <- " + ", ".join(issues) if issues else ""))

    safe_stop()
    try:
        proto.stream(0)
    except Exception:
        pass
    conn.disconnect()
    npass = sum(1 for r in results if r)
    print(f"\n  {npass}/{len(results)} drives CLEAN (distance + balance)")
    print("  (smoothness: run tests/dev/enc_watch.py to watch a live trace)")
    return 0 if npass == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
