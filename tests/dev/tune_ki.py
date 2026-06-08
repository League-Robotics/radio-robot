#!/usr/bin/env python3
"""tune_ki.py — sweep velocity-loop kI, measure L/R distance bias + cruise velocity.

For each kI value (set live via SET vel.kI — no reconnect), drives D a few times and
reports the mean L-R encoder bias (the curve), the avg distance, and the per-wheel
cruise velocity (the velocity error the integral is supposed to remove). Helps find
the kI that drives the wheels to equal travel. Robot ON A STAND.

Usage: uv run python tests/dev/tune_ki.py [--mm 500] [--spd 80] [--reps 3]
                                          [--kis 0.05,0.10,0.15,0.25,0.40,0.60]
"""
import argparse
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--mm", type=int, default=500)
    ap.add_argument("--spd", type=int, default=80)
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--kis", default="0.05,0.10,0.15,0.25,0.40,0.60")
    args = ap.parse_args()

    from robot_radio.io.serial_conn import SerialConnection, list_serial_ports
    from robot_radio.robot.protocol import NezhaProtocol, parse_tlm, parse_response

    port = args.port or (list_serial_ports() or [None])[0]
    conn = SerialConnection(port=port, mode="direct")
    conn.connect(skip_ping=True)
    proto = NezhaProtocol(conn)

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
    except Exception:
        pass
    proto.send("SET sTimeout=2000", 300)
    proto.stream_fields("enc,vel")
    proto.stream(50)
    time.sleep(0.4)

    def one_drive():
        conn._ser.reset_input_buffer()
        proto.send(f"D {args.spd} {args.spd} {args.mm}", 200)
        vels = []
        latest_enc = None
        done = None
        start = time.monotonic()
        end = start + (args.mm / max(args.spd, 1)) * 2.0 + 4.0
        last_change = start
        last_enc = None
        while time.monotonic() < end and done != "D":
            newest = None
            for raw in conn.read_lines(duration_ms=40):
                f = parse_tlm(raw)
                if f is not None:
                    if f.enc is not None:
                        newest = f.enc
                    if f.vel is not None:
                        vels.append((time.monotonic() - start, f.vel[0], f.vel[1]))
                r = parse_response(raw)
                if r and r.tag == "EVT" and r.tokens and r.tokens[0] == "done":
                    done = r.tokens[1] if len(r.tokens) > 1 else "?"
            if newest is not None:
                latest_enc = newest
                if (last_enc is None or abs(newest[0] - last_enc[0]) > 1
                        or abs(newest[1] - last_enc[1]) > 1):
                    last_change = time.monotonic()
                last_enc = newest
            now = time.monotonic()
            if now - start > 1.5 and now - last_change > 0.8:
                break
        # settle + final latest enc
        time.sleep(0.3)
        t2 = time.monotonic() + 0.4
        while time.monotonic() < t2:
            for raw in conn.read_lines(duration_ms=60):
                f = parse_tlm(raw)
                if f is not None and f.enc is not None:
                    latest_enc = f.enc
        tN = vels[-1][0] if vels else 0.0
        cruise = [v for v in vels if v[0] > 0.8 and v[0] < tN - 0.5]
        vL = sum(v[1] for v in cruise) / len(cruise) if cruise else 0.0
        vR = sum(v[2] for v in cruise) / len(cruise) if cruise else 0.0

        def std(xs, m):
            return (sum((x - m) ** 2 for x in xs) / len(xs)) ** 0.5 if xs else 0.0
        jit = (std([v[1] for v in cruise], vL) + std([v[2] for v in cruise], vR)) / 2.0
        return latest_enc, vL, vR, jit

    print(f"\n  sweep kI  (D {args.spd} {args.spd} {args.mm}, {args.reps} reps; setpoint {args.spd} mm/s)\n")
    print(f"  {'kI':>6} {'biasL-R':>8} {'avgdist':>8} {'velL':>6} {'velR':>6} {'vbias':>6} {'jitter':>7}")
    print(f"  {'-'*6} {'-'*8} {'-'*8} {'-'*6} {'-'*6} {'-'*6} {'-'*7}")
    results = []
    for ki in [float(x) for x in args.kis.split(",")]:
        proto.send(f"SET vel.kI={ki}", 300)
        time.sleep(0.2)
        biases, avgs, vLs, vRs, jits = [], [], [], [], []
        for _ in range(args.reps):
            safe_stop(); time.sleep(0.4)
            enc, vL, vR, jit = one_drive()
            if enc is None:
                continue
            biases.append(enc[0] - enc[1])
            avgs.append((abs(enc[0]) + abs(enc[1])) / 2.0)
            vLs.append(vL); vRs.append(vR); jits.append(jit)
        if not biases:
            print(f"  {ki:>6.2f}   (no data)"); continue
        mb = sum(biases) / len(biases)
        ma = sum(avgs) / len(avgs)
        mvL = sum(vLs) / len(vLs)
        mvR = sum(vRs) / len(vRs)
        mj = sum(jits) / len(jits)
        results.append((ki, mb, mj))
        print(f"  {ki:>6.2f} {mb:>+8.0f} {ma:>8.0f} {mvL:>6.0f} {mvR:>6.0f} {mvL-mvR:>+6.0f} {mj:>7.1f}")

    safe_stop()
    try:
        proto.stream(0)
    except Exception:
        pass
    conn.disconnect()
    if results:
        best = min(results, key=lambda r: abs(r[1]))
        print(f"\n  lowest |bias|: kI={best[0]:.2f}  (L-R {best[1]:+.0f} mm)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
