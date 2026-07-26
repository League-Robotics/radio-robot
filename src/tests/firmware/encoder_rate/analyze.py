"""analyze.py -- capture + analyze the encoder_rate bench firmware output.

Usage:
    uv run python src/tests/firmware/encoder_rate/analyze.py \
        --port /dev/cu.usbmodem2121102 [--capture capture.log]

With --port: opens the serial port, resets the target (pyocd) so the run
starts from the beginning, captures until DONE (or timeout), saves the raw
log, then analyzes. With an existing --capture file and no --port: analyzes
the saved log only.

Answers the three characterization questions (motion-planner issue §7.3):
  1. refresh period + jitter per wheel (phases A1/A2)
  2. L/R refresh-clock correlation (phase B nearest-change deltas)
  3. read-rate influence on the refresh (phase C vs A1)
"""

import argparse
import statistics
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path


def capture(port: str, outPath: Path, timeout: float) -> None:  # [s]
    import serial  # pyserial, present in the repo host env

    with serial.Serial(port, 115200, timeout=1.0) as link:
        link.reset_input_buffer()
        # Restart the program so we see the run from its banner.
        subprocess.run(["pyocd", "reset", "-t", "nrf52833"], check=True,
                       capture_output=True)
        lines = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            raw = link.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if not line:
                continue
            lines.append(line)
            print(line)
            if line == "DONE":
                break
        else:
            print("WARNING: capture timed out before DONE", file=sys.stderr)
    outPath.write_text("\n".join(lines) + "\n")
    print(f"# saved {len(lines)} lines to {outPath}")


def stats(values):
    if not values:
        return "n=0"
    ms = [v / 1000.0 for v in values]
    out = (f"n={len(ms)} mean={statistics.mean(ms):.2f} "
           f"median={statistics.median(ms):.2f} min={min(ms):.2f} "
           f"max={max(ms):.2f}")
    if len(ms) > 1:
        out += f" stdev={statistics.stdev(ms):.2f}"
    return out + " [ms]"


def histogram(values, binWidth):  # [us] bins
    counts = defaultdict(int)
    for v in values:
        counts[int(v // binWidth) * binWidth] += 1
    lines = []
    for binStart in sorted(counts):
        bar = "#" * min(60, counts[binStart])
        lines.append(f"    {binStart / 1000.0:7.1f}-"
                     f"{(binStart + binWidth) / 1000.0:6.1f} ms "
                     f"{counts[binStart]:5d} {bar}")
    return "\n".join(lines)


def analyze(logPath: Path) -> None:
    # changes[(phase, wheel)] = [(t, raw, pollsSinceChange), ...]
    changes = defaultdict(list)
    summaries = []
    for line in logPath.read_text().splitlines():
        parts = line.split(",")
        if parts[0] == "C" and len(parts) == 6:
            phase, wheel = parts[1], parts[2]
            t, raw, polls = int(parts[3]), int(parts[4]), int(parts[5])
            changes[(phase, wheel)].append((t, raw, polls))
        elif parts[0] == "N":
            summaries.append(line)

    print("\n================ encoder_rate analysis ================")
    for line in summaries:
        print(f"  {line}")

    periods = {}
    for key in sorted(changes):
        phase, wheel = key
        series = changes[key]
        if len(series) < 3:
            print(f"\n-- {phase}/{wheel}: only {len(series)} changes, skip")
            continue
        # uint32 wrap-safe diffs of consecutive change timestamps.
        diffs = [(b[0] - a[0]) & 0xFFFFFFFF
                 for a, b in zip(series, series[1:])]
        pollCounts = [c[2] for c in series[1:]]
        periods[key] = statistics.median(diffs)
        print(f"\n-- {phase}/{wheel}: change-interval {stats(diffs)}")
        print(f"   polls-per-change: median={statistics.median(pollCounts)} "
              f"min={min(pollCounts)} max={max(pollCounts)}")
        print(histogram(diffs, 5000))

    # Question 2: L/R correlation in phase B. For each L change, the delta
    # to the NEAREST R change. A shared refresh clock puts these in a tight
    # cluster at a constant offset; independent clocks spread them
    # ~uniformly over [0, period/2].
    lSeries = [c[0] for c in changes.get(("B", "L"), [])]
    rSeries = [c[0] for c in changes.get(("B", "R"), [])]
    if len(lSeries) > 3 and len(rSeries) > 3:
        deltas = []
        for t in lSeries:
            nearest = min(rSeries, key=lambda r: abs(((r - t) + 0x80000000)
                                                     & 0xFFFFFFFF)
                          - 0x80000000)
            signed = ((nearest - t + 0x80000000) & 0xFFFFFFFF) - 0x80000000
            deltas.append(signed)
        absDeltas = [abs(d) for d in deltas]
        print(f"\n-- B: L-to-nearest-R change delta {stats(absDeltas)}")
        print(histogram(absDeltas, 2000))
        period = periods.get(("B", "L"))
        if period:
            spread = (statistics.stdev(absDeltas)
                      if len(absDeltas) > 1 else 0.0)
            verdict = ("CORRELATED (shared refresh clock)"
                       if spread < period * 0.15 else
                       "INDEPENDENT (uncorrelated refresh clocks)")
            print(f"   verdict: {verdict}  "
                  f"(delta stdev {spread / 1000.0:.2f} ms vs period "
                  f"{period / 1000.0:.2f} ms)")

    # Question 3: read-rate influence -- A1 (fast poll) vs C (slow poll).
    a1 = periods.get(("A1", "L"))
    c = periods.get(("C", "L"))
    if a1 and c:
        ratio = c / a1
        verdict = ("read rate does NOT set the refresh (period unchanged)"
                   if 0.8 <= ratio <= 1.25 else
                   "read rate PERTURBS the refresh -- investigate")
        print(f"\n-- C vs A1: median period {c / 1000.0:.2f} vs "
              f"{a1 / 1000.0:.2f} ms (x{ratio:.2f}) -> {verdict}")
    print("=======================================================")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port")
    parser.add_argument("--capture",
                        default=str(Path(__file__).parent / "capture.log"))
    parser.add_argument("--timeout", type=float, default=150.0)  # [s]
    args = parser.parse_args()

    logPath = Path(args.capture)
    if args.port:
        capture(args.port, logPath, args.timeout)
    if not logPath.exists():
        sys.exit(f"no capture at {logPath} -- pass --port to record one")
    analyze(logPath)


if __name__ == "__main__":
    main()
