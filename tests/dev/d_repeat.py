#!/usr/bin/env python3
"""d_repeat.py — run D 120 120 200 N times, report per-wheel travel + imbalance.

Checks whether the D (distance) command's wheel imbalance is systematic or a
fluke. Robot ON A STAND. D resets its accumulator internally, so the post-drive
enc is the absolute per-wheel travel.
"""
import argparse, sys, time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("-n", type=int, default=5)
    ap.add_argument("--mm", type=int, default=200)
    ap.add_argument("--spd", type=int, default=120)
    args = ap.parse_args()

    from robot_radio.io.serial_conn import SerialConnection, list_serial_ports
    from robot_radio.robot.protocol import NezhaProtocol, parse_tlm, parse_response

    port = args.port or (list_serial_ports() or [None])[0]
    conn = SerialConnection(port=port, mode="direct")
    conn.connect(skip_ping=True)
    proto = NezhaProtocol(conn)

    deadline = time.monotonic() + 12.0
    while time.monotonic() < deadline:
        r = proto.send("VER", 300)
        if any("fw=" in ln for ln in r.get("responses", [])):
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
    except Exception:
        pass
    proto.zero_encoders()
    proto.send("SET sTimeout=700", 300)
    proto.stream_fields("enc,pose,vel")
    proto.stream(50)
    time.sleep(0.4)

    def read_enc(ms=400):
        e = None
        end = time.monotonic() + ms / 1000.0
        while time.monotonic() < end:
            for raw in conn.read_lines(duration_ms=50):
                f = parse_tlm(raw)
                if f is not None and f.enc is not None:
                    e = f.enc
        return e

    rows = []
    for i in range(args.n):
        conn.send_fast("STOP"); time.sleep(0.4)
        proto.send(f"D {args.spd} {args.spd} {args.mm}", 300)
        done = None
        end = time.monotonic() + 6.0
        while time.monotonic() < end and done != "D":
            for raw in conn.read_lines(duration_ms=120):
                r = parse_response(raw)
                if r and r.tag == "EVT" and r.tokens and r.tokens[0] == "done":
                    done = r.tokens[1] if len(r.tokens) > 1 else "?"
        time.sleep(0.3)
        e = read_enc()
        if e is None:
            print(f"  run {i+1}: no enc"); continue
        dL, dR = e[0], e[1]
        imb = dL - dR
        rows.append((dL, dR, imb))
        print(f"  run {i+1}: L={dL:+5d} R={dR:+5d}  L-R={imb:+5d}  "
              f"avg={(dL+dR)//2}  done={done}")

    safe_stop()
    try: proto.stream(0)
    except Exception: pass
    conn.disconnect()

    if rows:
        imbs = [r[2] for r in rows]
        print(f"\n  imbalance L-R: min={min(imbs):+d} max={max(imbs):+d} "
              f"mean={sum(imbs)/len(imbs):+.0f} mm  (target {args.mm} mm)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
