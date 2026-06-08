#!/usr/bin/env python3
"""cmd_test.py — functional test of the drive commands via the encoders.

Exercises S / T / D / G and checks the per-wheel encoder deltas against what each
command should produce (direction + magnitude). The robot should be ON A STAND
(wheels free) — we're verifying the wheels turn the right way and amount, not that
the robot navigates a floor.

  S <l> <r>        wheel velocities (mm/s), continuous
  T <l> <r> <ms>   timed drive
  D <l> <r> <mm>   distance drive
  G <x> <y> <spd>  go-to XY (turn-to-face + drive)

Encoder = TLM enc (mm, cumulative). Needs the IRQ-guard firmware so encLMm tracks
while driving. Run: uv run python tests/dev/cmd_test.py [--port DEV]
"""
import argparse
import math
import sys
import time

TRACKWIDTH_MM = 126.0      # data/robots/tovez.json "trackwidth"
TOL = 0.30                 # ±30% magnitude tolerance (free-spin, functional check)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    args = ap.parse_args()

    from robot_radio.io.serial_conn import SerialConnection, list_serial_ports
    from robot_radio.robot.protocol import NezhaProtocol, parse_tlm, parse_response

    port = args.port or (list_serial_ports() or [None])[0]
    if port is None:
        print("ERROR: no serial port"); return 2
    print(f"  port: {port}")

    conn = SerialConnection(port=port, mode="direct")
    conn.connect(skip_ping=True)
    proto = NezhaProtocol(conn)

    # connect() pulses DTR → micro:bit resets; wait for it to finish booting
    # before sending setup, or the STREAM command is lost and no TLM comes.
    deadline = time.monotonic() + 12.0
    alive = False
    while time.monotonic() < deadline:
        r = proto.send("VER", 300)
        if any("fw=" in ln for ln in r.get("responses", [])):
            alive = True
            break
        time.sleep(0.3)
    if not alive:
        print("  FATAL: robot did not respond after connect (boot/port issue)")
        return 2
    print("  robot alive")

    def drain(ms):
        """Read for ms; return (latest_enc (l,r)|None, done_verb|None)."""
        enc = None; done = None
        deadline = time.monotonic() + ms / 1000.0
        while time.monotonic() < deadline:
            for raw in conn.read_lines(duration_ms=40):
                f = parse_tlm(raw)
                if f is not None and f.enc is not None:
                    enc = f.enc
                r = parse_response(raw)
                if r and r.tag == "EVT" and r.tokens and r.tokens[0] == "done":
                    done = r.tokens[1] if len(r.tokens) > 1 else "?"
        return enc, done

    def enc_now(ms=400):
        e = None
        deadline = time.monotonic() + ms / 1000.0
        while time.monotonic() < deadline and e is None:
            e, _ = drain(80)
        # keep reading a bit for the freshest
        e2, _ = drain(150)
        return e2 or e

    # ---- setup ----
    conn.send_fast("STOP"); conn.send_fast("STREAM 0"); time.sleep(0.1)
    conn._ser.reset_input_buffer()
    try:
        from robot_radio.io.cli import _push_calibration
        _push_calibration(conn)
    except Exception as exc:
        print(f"  WARN cal push: {exc}")
    proto.zero_encoders()
    # SHORT S-watchdog: the streaming (S) watchdog is the only auto-stop, and it
    # does NOT touch T/D/G (autonomous). 700 ms caps any S-runaway to <1 s while
    # the S keepalives (~120 ms) stay well inside it. (Was 10 s — that let the
    # motors run for 10 s after the host stopped commanding: the runaway you saw.)
    proto.send("SET sTimeout=700", 300)
    proto.stream_fields("enc,pose,vel")
    proto.stream(50)
    time.sleep(0.4)
    base = enc_now()
    print(f"  baseline enc: {base}")
    if base is None:
        print("  FATAL: no enc telemetry — wedged or wrong firmware?"); return 2

    results = []

    import signal

    def safe_stop():
        """Stop the motors as reliably as we can (multiple send_fast + a
        confirmed send). Called at the end AND on any signal."""
        try:
            for _ in range(4):
                conn.send_fast("STOP"); time.sleep(0.04)
            proto.send("STOP", 150)
        except Exception:
            pass

    def _sig(*_):
        print("\n  signal — stopping motors and exiting")
        safe_stop()
        sys.exit(1)

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    def check(label, dL, dR, expL, expR):
        def ok(d, e):
            if abs(e) < 1:      # expected ~0 → must stay small
                return abs(d) < 25
            return (d * e > 0) and (abs(d - e) <= TOL * abs(e) + 15)
        good = ok(dL, expL) and ok(dR, expR)
        results.append((label, good))
        tag = "PASS" if good else "FAIL"
        print(f"  [{tag}] {label}: dL={dL:+.0f} dR={dR:+.0f} mm  "
              f"(expect L≈{expL:+.0f} R≈{expR:+.0f})")
        return good

    def settle_start():
        # Stop and read the current enc. We do NOT zero between tests — calling
        # zero_encoders mid-sequence proved flaky (frozen/negative reads). S/T
        # use a continuous delta; D/G reset their own accumulator at drive start,
        # so for those we read the ABSOLUTE post-drive enc instead.
        conn.send_fast("STOP"); time.sleep(0.4)
        return enc_now()

    # ---- S: 120,120 for 1.5 s → ~+180 each, same direction ----
    s0 = settle_start()
    conn.send_fast("S 120 120")
    dur = 1.5; t0 = time.monotonic(); last = s0
    while time.monotonic() - t0 < dur:
        e, _ = drain(120)
        if e: last = e
        conn.send_fast("S 120 120")   # keepalive
    conn.send_fast("STOP"); time.sleep(0.3); s1 = enc_now()
    exp = 120 * dur
    check("S 120 120 (1.5s)", s1[0]-s0[0], s1[1]-s0[1], exp, exp)

    # ---- T straight: 120,120 for 1000 ms → ~+120 each ----
    t_s0 = settle_start()
    proto.send("T 120 120 1000", 300)
    _, _ = drain(0)
    enc, done = (None, None); dl = t_s0
    tend = time.monotonic() + 3.0
    while time.monotonic() < tend and done != "T":
        e, done = drain(150)
        if e: dl = e
    time.sleep(0.3); t_s1 = enc_now()
    check("T 120 120 1000 (straight)", t_s1[0]-t_s0[0], t_s1[1]-t_s0[1], 120, 120)

    # ---- T turn: 120,-120 for 1000 ms → +120 / -120 (opposite) ----
    tt0 = settle_start()
    proto.send("T 120 -120 1000", 300)
    done = None; tend = time.monotonic() + 3.0
    while time.monotonic() < tend and done != "T":
        _, done = drain(150)
    time.sleep(0.3); tt1 = enc_now()
    check("T 120 -120 1000 (turn)", tt1[0]-tt0[0], tt1[1]-tt0[1], 120, -120)

    # ---- D: 120,120,200 mm → ~+200 each (ABSOLUTE: D resets accumulator) ----
    settle_start()
    proto.send("D 120 120 200", 300)
    done = None; tend = time.monotonic() + 6.0
    while time.monotonic() < tend and done != "D":
        _, done = drain(150)
    time.sleep(0.3); d1 = enc_now()
    check("D 120 120 200", d1[0], d1[1], 200, 200)

    # ---- G: go-to (x,y) → turn-to-face + drive. Back out turn & distance. ----
    gx, gy, gspd = 200, 200, 120
    try:
        proto.otos_zero()
    except Exception:
        pass
    settle_start()
    proto.send(f"G {gx} {gy} {gspd}", 300)
    done = None; tend = time.monotonic() + 12.0
    while time.monotonic() < tend and done != "G":
        _, done = drain(200)
    conn.send_fast("STOP"); time.sleep(0.3); g1 = enc_now()
    dL, dR = g1[0], g1[1]   # ABSOLUTE: G resets its accumulator at start
    dist_est = (dL + dR) / 2.0
    turn_est_deg = math.degrees((dR - dL) / TRACKWIDTH_MM)   # +CCW
    want_dist = math.hypot(gx, gy)
    want_turn = math.degrees(math.atan2(gy, gx))             # from heading 0
    print(f"  [G ] G {gx} {gy} {gspd}  (done={done})")
    print(f"        dL={dL:+.0f} dR={dR:+.0f} mm")
    print(f"        encoder-implied: distance={dist_est:+.0f} mm  turn={turn_est_deg:+.1f}°")
    print(f"        commanded-implied (from heading 0): distance={want_dist:.0f} mm  "
          f"turn={want_turn:+.1f}°")
    print(f"        NOTE: on a stand the OTOS sees no motion, so G's pose-pursuit "
          f"may not converge — read the encoder-implied values as 'what the wheels did'.")

    safe_stop()
    try:
        proto.stream(0)
    except Exception:
        pass
    try:
        conn.disconnect()
    except Exception:
        pass

    print("\n  ==== SUMMARY ====")
    for label, good in results:
        print(f"    {'PASS' if good else 'FAIL'}  {label}")
    npass = sum(1 for _, g in results if g)
    print(f"  {npass}/{len(results)} wheel-command checks passed (+ G math above)")
    return 0 if npass == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
