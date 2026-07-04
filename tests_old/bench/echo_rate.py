#!/usr/bin/env python3
"""echo_rate.py — measure the reliable data rate of the robot link via ECHO.

Sends ``ECHO <filler> #<id>`` with a unique correlation id per message; the
firmware echoes the id back (``OK echo <filler> #<id>``), so every reply is
matched to its request BY ID. This is the crucial fix over the earlier harness,
which matched strictly per-command with a timeout and DESYNCED the moment a reply
lagged — counting correct-but-late replies as "drops" and inventing a ~40% loss
that did not exist. Here a lagging reply is simply matched when it arrives.

A message counts as:
  delivered — its id came back as ``OK echo`` with the exact filler
  corrupt   — id came back but the echoed filler differed
  err       — id came back as ``ERR ...`` (e.g. queue full under a big --window)
  lost      — its id never returned within --timeout-ms of being sent (true loss)

--window sets how many requests may be in flight at once: 1 = plain request/reply
(latency-bound); higher pipelines for throughput (keep <=4 for the robot — its
command queue holds 4, beyond which it replies ERR full).

Run:
    uv run python tests/bench/echo_rate.py --port /dev/cu.usbmodemXXXX --sizes 50 200
    uv run python tests/bench/echo_rate.py --port <relay> --go --window 4
    uv run python tests/bench/echo_rate.py --port <port> --csv tests/bench/out/echo_rate.csv
"""
from __future__ import annotations

import argparse
import pathlib
import random
import re
import string
import sys
import time

import serial

ALPHABET = string.ascii_letters + string.digits
WORD_MAX = 30
MAX_WORDS = 10
_CORR = re.compile(r"#(\d+)\s*$")   # trailing corr-id on a reply line
_OK_ECHO = "OK echo "


def make_filler(n: int, rng: random.Random) -> str:
    """An n-byte filler the firmware echoes verbatim: <=10 space-joined words of
    <=30 chars (stays under the firmware's MAX_ARGS=10 / sval[32] limits)."""
    if n <= 1:
        return "x"
    nwords = min(MAX_WORDS, max(1, -(-(n + 1) // (WORD_MAX + 1))))
    content = max(nwords, n - (nwords - 1))     # chars excluding joining spaces
    base, extra = divmod(content, nwords)
    sizes = [base + (1 if i < extra else 0) for i in range(nwords)]
    return " ".join("".join(rng.choice(ALPHABET) for _ in range(s)) for s in sizes)


class Link:
    """Line-oriented serial link. Poll-based reader (returns promptly on a short
    timeout) so the pipeline can interleave sends and reads."""

    def __init__(self, port: str, baud: int = 115200):
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self._buf = b""

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def flush_input(self):
        self.ser.reset_input_buffer()
        self._buf = b""

    def send_line(self, text: str):
        self.ser.write(text.encode("utf-8") + b"\n")
        self.ser.flush()

    def set_host_baud(self, baud: int):
        # Change baud on the OPEN port (sends USB-CDC SET_LINE_CODING without a
        # DTR pulse) so the robot is NOT reset. Reopening would reset it to 115200.
        self.ser.baudrate = baud

    def poll_line(self) -> str | None:
        """Return one complete line if available, else None (after a ~10 ms poll)."""
        nl = self._buf.find(b"\n")
        if nl < 0:
            self.ser.timeout = 0.01
            chunk = self.ser.read(4096)
            if chunk:
                self._buf += chunk
            nl = self._buf.find(b"\n")
        if nl >= 0:
            line, self._buf = self._buf[:nl], self._buf[nl + 1:]
            return line.decode("utf-8", "ignore").strip()
        return None


def selftest(link, tag="999999", timeout=1.5) -> bool:
    """One id-correlated ECHO must round-trip. Returns True on success."""
    link.send_line(f"ECHO selftest_payload #{tag}")
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        ln = link.poll_line()
        if ln and ln.startswith(_OK_ECHO) and ln.rstrip().endswith(f"#{tag}"):
            return True
    return False


def run_size(link, size, duration, window, timeout_s, rng, id0):
    """Windowed, id-correlated, lag-tolerant ECHO measurement at one payload size."""
    inflight = {}                 # id -> (send_time, filler)
    rtts = []
    delivered = corrupt = err = lost = sent = 0
    nid = id0
    start = time.monotonic()
    send_deadline = start + duration

    while True:
        now = time.monotonic()
        # Fill the send window (until the duration is up).
        while now < send_deadline and len(inflight) < window:
            filler = make_filler(size, rng)
            link.send_line(f"ECHO {filler} #{nid}")
            inflight[nid] = (now, filler)
            sent += 1
            nid += 1
            now = time.monotonic()

        # Drain any available replies, matching by id.
        line = link.poll_line()
        if line is not None:
            m = _CORR.search(line)
            if m:
                rid = int(m.group(1))
                rec = inflight.pop(rid, None)
                if rec is not None:
                    t0, filler = rec
                    if line.startswith(_OK_ECHO):
                        body = _CORR.sub("", line[len(_OK_ECHO):]).strip()
                        if body == filler:
                            delivered += 1
                            rtts.append((time.monotonic() - t0) * 1000.0)
                        else:
                            corrupt += 1
                    else:
                        err += 1            # e.g. "ERR full #id"

        # Expire truly-lost ids (never returned within timeout).
        now = time.monotonic()
        for i in [i for i, (t, _) in inflight.items() if now - t > timeout_s]:
            inflight.pop(i, None)
            lost += 1

        # Done: past the send window AND nothing left in flight.
        if now >= send_deadline and not inflight:
            break

    elapsed = time.monotonic() - start
    rs = sorted(rtts)
    pct = lambda q: rs[min(len(rs) - 1, int(q * len(rs)))] if rs else float("nan")
    ok = delivered
    return {
        "size": size, "window": window, "sent": sent, "delivered": delivered,
        "corrupt": corrupt, "err": err, "lost": lost,
        "success": ok / sent if sent else 0.0,
        "elapsed": elapsed,
        "msg_per_s": ok / elapsed if elapsed else 0.0,
        "oneway_bps": ok * size / elapsed if elapsed else 0.0,
        "rtt_min": rs[0] if rs else float("nan"),
        "rtt_med": pct(0.5), "rtt_p90": pct(0.9),
        "rtt_max": rs[-1] if rs else float("nan"),
    }


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/cu.usbmodem2121102", help="serial port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--sizes", type=int, nargs="+", default=[50, 200],
                    help="payload sizes (bytes) to measure")
    ap.add_argument("--duration", type=float, default=6.0, help="seconds per size")
    ap.add_argument("--window", type=int, default=1,
                    help="max requests in flight (1=request/reply; <=4 for the robot)")
    ap.add_argument("--timeout-ms", type=int, default=1000,
                    help="a message is 'lost' only if unanswered this long (default 1000)")
    ap.add_argument("--set-baud", type=int, default=None,
                    help="after connecting at 115200, send BAUD <rate> and retune "
                         "the host (115200|230400|921600|1000000)")
    ap.add_argument("--go", action="store_true",
                    help="send '!GO' first (radio relay data-plane entry)")
    ap.add_argument("--settle", type=float, default=1.5)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--csv", type=str, default=None)
    args = ap.parse_args()

    rng = random.Random(args.seed)
    timeout_s = args.timeout / 1000.0
    sizes = [s for s in args.sizes if s > 0]

    print(f"Opening {args.port} @ {args.baud} ...")
    try:
        link = Link(args.port, args.baud)
    except serial.SerialException as exc:
        print(f"open failed: {exc}")
        return 2

    try:
        time.sleep(args.settle)
        link.flush_input()
        if args.go:
            link.send_line("!GO")
            time.sleep(0.3)
            link.flush_input()
        link.send_line("SAFE off")          # motion-free; avoids watchdog noise
        time.sleep(0.2)
        link.flush_input()

        # Self-test + liveness at the boot baud. Retry through the ~3 s post-open
        # reboot (each open pulses DTR → reset); resend the probe until it answers.
        alive = False
        for _ in range(16):
            if selftest(link, timeout=0.5):
                alive = True
                break
        if not alive:
            print("ECHO self-test FAILED — robot not answering. (right port? --go for relay?)")
            return 2
        print(f"ECHO self-test OK at {link.ser.baudrate} baud.")

        # Optional: bump the baud via the firmware BAUD command. Reply comes at
        # the OLD baud; then we retune the HOST on the open port (no reopen/reset).
        if args.set_baud and args.set_baud != link.ser.baudrate:
            target = args.set_baud
            link.send_line(f"BAUD {target}")
            ack = None
            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                ln = link.poll_line()
                if ln and "baud" in ln and str(target) in ln:
                    ack = ln
                    break
            print(f"BAUD {target} -> {ack or '(no ack)'}")
            time.sleep(0.1)                     # let the robot finish retuning
            link.set_host_baud(target)          # switch host on the OPEN port
            time.sleep(0.2)
            link.flush_input()
            if not selftest(link):
                print(f"FAILED to talk at {target} baud — the DAPLink interface "
                      f"may not support it (target nRF52 does). Reverting host to 115200.")
                link.set_host_baud(115200)
                return 2
            print(f"link verified at {target} baud ✓")
        print()

        print(f"id-correlated, lag-tolerant; window={args.window}, "
              f"{args.duration:.0f}s/size, timeout={args.timeout}ms")
        print(f"  {'size':>5} {'sent':>5} {'ok':>5} {'corr':>5} {'err':>4} {'lost':>5} "
              f"{'deliv':>6} {'msg/s':>7} {'goodput':>14} "
              f"{'rtt ms (min/med/p90/max)':>26}")
        rows = []
        for i, size in enumerate(sizes):
            st = run_size(link, size, args.duration, args.window, timeout_s, rng,
                          id0=(i + 1) * 1_000_000)
            print(f"  {size:>5} {st['sent']:>5} {st['delivered']:>5} {st['corrupt']:>5} "
                  f"{st['err']:>4} {st['lost']:>5} {st['success']:>5.0%} "
                  f"{st['msg_per_s']:>7.1f} "
                  f"{st['oneway_bps']:>8.0f} B/s ({st['oneway_bps']*8/1000:>4.1f}kbit) "
                  f"{st['rtt_min']:>5.0f}/{st['rtt_med']:>4.0f}/{st['rtt_p90']:>4.0f}/{st['rtt_max']:>4.0f}")
            rows.append(st)

        if args.csv:
            p = pathlib.Path(args.csv)
            p.parent.mkdir(parents=True, exist_ok=True)
            with p.open("w") as f:
                f.write("size,window,sent,delivered,corrupt,err,lost,success,"
                        "msg_per_s,oneway_bps,rtt_min,rtt_med,rtt_p90,rtt_max\n")
                for st in rows:
                    f.write(f"{st['size']},{st['window']},{st['sent']},{st['delivered']},"
                            f"{st['corrupt']},{st['err']},{st['lost']},{st['success']:.4f},"
                            f"{st['msg_per_s']:.2f},{st['oneway_bps']:.1f},{st['rtt_min']:.1f},"
                            f"{st['rtt_med']:.1f},{st['rtt_p90']:.1f},{st['rtt_max']:.1f}\n")
            print(f"\nwritten to {p}")
        return 0
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
