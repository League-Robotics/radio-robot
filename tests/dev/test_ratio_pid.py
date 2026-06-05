#!/usr/bin/env python3
"""test_ratio_pid.py — Ratio PID validation, on-stand encoder test.

Drives the robot at 12 different speed ratios for a fixed duration on the
stand (wheels spinning freely). After each run reads encoder counts and
checks that the actual travel ratio matches the commanded ratio within
tolerance.

Usage:
    python3 tests/test_ratio_pid.py [--duration S] [--tol PCT] [--port /dev/...]

Options:
    --duration  Seconds to spin each trial (default 2.0)
    --tol       Acceptable ratio error in percent (default 5)
    --port      Serial port (default: auto-detect)
"""

from __future__ import annotations

import argparse
import sys
import time

sys.path.insert(0, "tests")
from rogo import Conn, resolve_port, sign

# (left_mms, right_mms) — 12 trials covering all sign combinations
TRIALS = [
    # ── Both forward ────────────────────────────────────────────────────────
    ( 100,  200),   # fwd, right 2× faster
    ( 200,  100),   # fwd, left  2× faster
    ( 150,  300),   # fwd, right 2× faster, higher speed
    ( 300,  150),   # fwd, left  2× faster, higher speed
    # ── Both reversed ───────────────────────────────────────────────────────
    (-100, -200),   # back, right 2× faster
    (-200, -100),   # back, left  2× faster
    (-150, -300),   # back, right 2× faster, higher speed
    (-300, -150),   # back, left  2× faster, higher speed
    # ── One wheel reversed (pivot turns) ────────────────────────────────────
    ( 100, -200),   # left fwd / right back, |right| 2× faster
    ( 200, -100),   # left fwd / right back, |left|  2× faster
    (-100,  200),   # left back / right fwd, |right| 2× faster
    (-200,  100),   # left back / right fwd, |left|  2× faster
]

WATCHDOG_MS     = 500    # firmware KSS safety timeout
KEEPALIVE_S     = 0.35   # 70% of WATCHDOG_MS
PRINT_INTERVAL  = 0.25   # live ratio print cadence (seconds)


def set_watchdog(conn: Conn, ms: int) -> None:
    lines = conn.send(f"KSS+{ms:03d}", read_ms=300)
    for line in lines:
        if f"ACK:KSS {ms}" in line:
            print(f"  KSS set to {ms} ms")
            return
    print(f"  WARNING: KSS ACK not received (got {lines})")


def parse_enc(line: str) -> tuple[int, int] | None:
    """Parse 'ENC+LLLL+RRRR' → (L, R) or None."""
    if not line.startswith("ENC"):
        return None
    rest = line[3:]
    parts: list[int] = []
    i = 0
    while i < len(rest) and len(parts) < 2:
        if rest[i] in ("+", "-"):
            neg = rest[i] == "-"
            i += 1
            start = i
            while i < len(rest) and rest[i].isdigit():
                i += 1
            if i > start:
                parts.append(-int(rest[start:i]) if neg else int(rest[start:i]))
        else:
            i += 1
    return (parts[0], parts[1]) if len(parts) == 2 else None


def run_trial(conn: Conn, left_mms: int, right_mms: int,
              duration_s: float) -> tuple[float, float]:
    scmd = f"S{sign(left_mms)}{sign(right_mms)}"
    cmd_ratio = abs(right_mms) / abs(left_mms) if left_mms != 0 else float("nan")

    # Zero encoders and let firmware update the cached value.
    conn.send("EZ", read_ms=300)
    time.sleep(0.15)

    enc_l = enc_r = 0
    deadline       = time.monotonic() + duration_s
    last_keepalive = time.monotonic()
    last_print     = 0.0
    n_stops        = 0

    conn.send_fast(scmd)

    while time.monotonic() < deadline:
        now = time.monotonic()

        if now - last_keepalive >= KEEPALIVE_S:
            conn.send_fast(scmd)
            last_keepalive = now

        for line in conn._read_lines(duration_ms=50):
            enc = parse_enc(line)
            if enc:
                enc_l, enc_r = enc
            if "SAFETY_STOP" in line:
                n_stops += 1
                conn.send_fast(scmd)
                last_keepalive = time.monotonic()

        now = time.monotonic()
        if now - last_print >= PRINT_INTERVAL:
            ratio_str = f"{abs(enc_r)/abs(enc_l):.3f}" if enc_l != 0 else "  N/A"
            elapsed   = duration_s - (deadline - now)
            print(f"    t={elapsed:.2f}s  encL={enc_l:>6}  encR={enc_r:>6}  "
                  f"ratio={ratio_str}  (cmd={cmd_ratio:.3f})", flush=True)
            last_print = now

    conn.send("X", read_ms=300)

    if n_stops:
        print(f"    (SAFETY_STOP fired {n_stops}× — check WATCHDOG_MS/KSS)")

    time.sleep(0.15)
    for line in conn.send("ENC", read_ms=300):
        enc = parse_enc(line)
        if enc:
            enc_l, enc_r = enc

    print(f"    final enc: L={enc_l}  R={enc_r}")
    return float(enc_l), float(enc_r)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port",     default=None)
    parser.add_argument("--duration", type=float, default=2.0,
                        help="Seconds per trial (default 2.0)")
    parser.add_argument("--tol",      type=float, default=5.0,
                        help="Acceptable ratio error %% (default 5)")
    args = parser.parse_args()

    port = resolve_port(args.port)
    conn = Conn(port, verbose=False)
    try:
        conn.connect()
        set_watchdog(conn, WATCHDOG_MS)

        print(f"\nRunning {len(TRIALS)} ratio trials — {args.duration:.1f}s each, "
              f"tolerance ±{args.tol:.0f}%\n")
        print(f"{'#':>2}  {'L':>5} {'R':>5}  {'cmdR':>6}  "
              f"{'encL':>8} {'encR':>8}  {'actR':>6}  {'err%':>6}  {'':6}")
        print("─" * 72)

        passed = failed = 0

        for i, (left_mms, right_mms) in enumerate(TRIALS, 1):
            cmd_ratio = abs(right_mms) / abs(left_mms) if left_mms != 0 else float("nan")
            ldir  = "F" if left_mms  > 0 else "B"
            rdir  = "F" if right_mms > 0 else "B"
            label = ldir + rdir
            print(f"{i:>2} [{label}]  L={left_mms:>+5} R={right_mms:>+5}  "
                  f"cmdR={cmd_ratio:.3f}  ", end="", flush=True)

            enc_l, enc_r = run_trial(conn, left_mms, right_mms, args.duration)

            if abs(enc_l) < 1.0:
                print(f"{'?':>8} {'?':>8}  {'?':>6}  {'?':>6}  SKIP (no travel)")
                continue

            actual_ratio = abs(enc_r) / abs(enc_l)
            err_pct      = abs(actual_ratio - cmd_ratio) / cmd_ratio * 100.0
            ok  = err_pct <= args.tol
            tag = "PASS" if ok else "FAIL"
            print(f"encL={enc_l:>+8.1f} encR={enc_r:>+8.1f}  "
                  f"actR={actual_ratio:.3f}  err={err_pct:4.1f}%  {tag}")

            if ok:
                passed += 1
            else:
                failed += 1

            time.sleep(0.5)

        print("─" * 72)
        print(f"\nResult: {passed} passed, {failed} failed out of {passed+failed} trials")

        set_watchdog(conn, 200)
        sys.exit(0 if failed == 0 else 1)

    except KeyboardInterrupt:
        print("\n[interrupted]")
        conn.send("X", read_ms=200)
    finally:
        conn.close()


if __name__ == "__main__":
    main()
