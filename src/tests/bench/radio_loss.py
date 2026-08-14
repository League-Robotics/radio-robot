"""Measure the RADIO link itself: relay -> relay, no robot involved.

Two relays on the same channel. One sends N numbered packets, the other
receives. Every packet carries its own sequence number, so loss, duplication
and reordering are all directly countable -- there is no inference and no
robot behaviour in the loop to confuse the result.

Run it on a channel no robot is using (default 7), otherwise a robot's own
telemetry and DBG output share the air and the measurement is of the bench,
not the link.

    uv run python src/tests/bench/radio_loss.py --tx PORT_A --rx PORT_B
    uv run python src/tests/bench/radio_loss.py --tx A --rx B --count 300 --gap 0.02
"""
import argparse
import re
import sys
import time

import serial


def relay_cmd(ser, cmd, wait=0.35):
    ser.write((cmd + "\n").encode()); ser.flush()
    time.sleep(wait)
    return ser.read(400)


def setup(port, channel, label):
    s = serial.Serial(port, 115200, timeout=0.2)
    time.sleep(1.2); s.reset_input_buffer()
    relay_cmd(s, "!ECHO OFF")
    relay_cmd(s, f"!C {channel}")
    relay_cmd(s, "!MODE RAW250")
    info = relay_cmd(s, "?")
    m = re.search(rb"channel:\s*(\d+)", info)
    got = int(m.group(1)) if m else -1
    print(f"  {label}: {port}  channel {got}"
          f"{'  <-- MISMATCH' if got != channel else ''}")
    relay_cmd(s, "!GO", wait=0.6)
    s.reset_input_buffer()
    return s


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tx", required=True)
    ap.add_argument("--rx", required=True)
    ap.add_argument("--channel", type=int, default=7)
    ap.add_argument("--count", type=int, default=200)
    ap.add_argument("--gap", type=float, default=0.03)   # [s] between packets
    ap.add_argument("--size", type=int, default=16)      # payload bytes
    args = ap.parse_args()

    tx = setup(args.tx, args.channel, "TX relay")
    rx = setup(args.rx, args.channel, "RX relay")
    pad = "x" * max(0, args.size - 8)

    print(f"\n  sending {args.count} packets, {args.gap*1000:.0f} ms apart, "
          f"{args.size} B payload, channel {args.channel}")
    sent = []
    for i in range(args.count):
        line = f"P{i:05d}{pad}\n"
        tx.write(line.encode()); tx.flush()
        sent.append(i)
        time.sleep(args.gap)
    time.sleep(1.5)

    buf = rx.read(200000).decode("ascii", "replace")
    got = [int(m) for m in re.findall(r"P(\d{5})", buf)]
    uniq = sorted(set(got))
    missing = [i for i in sent if i not in set(got)]
    dupes = len(got) - len(uniq)
    reordered = sum(1 for a, b in zip(got, got[1:]) if b < a)

    print(f"\n  sent      {len(sent)}")
    print(f"  received  {len(uniq)} unique  ({len(got)} raw)")
    print(f"  LOST      {len(missing)}  ({100.0*len(missing)/max(1,len(sent)):.1f}%)")
    print(f"  duplicated{dupes:>4}")
    print(f"  reordered {reordered:>4}")
    if missing:
        runs, start = [], missing[0]
        prev = missing[0]
        for m in missing[1:]:
            if m != prev + 1:
                runs.append((start, prev)); start = m
            prev = m
        runs.append((start, prev))
        burst = max(b - a + 1 for a, b in runs)
        print(f"  loss pattern: {len(runs)} bursts, longest {burst} consecutive")
        print(f"  first few missing: {missing[:12]}")
    for s in (tx, rx):
        try: s.close()
        except Exception: pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
