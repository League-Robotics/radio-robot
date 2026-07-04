#!/usr/bin/env python3
"""demo.py — live demonstration of the Nezha encoder latch and its fix.

    uv run python wedgelab/demo.py            # both: lockup, then the fix
    uv run python wedgelab/demo.py latch      # just the lockup
    uv run python wedgelab/demo.py fixed      # same motion, zero-dwell fix

Requires the WEDGELAB firmware (wedgelab/MICROBIT.hex) on the robot and the
old (latch-prone) motors on ports M1/M2. Nothing else may hold the serial
port (disconnect the VS Code serial monitor).

What it does — identical motion in both modes: cruise at 32% and reverse
direction every 0.8 s, 20 reversal cycles (~25 s of motor time per mode).
  latch : reversals written the PRODUCTION way — immediate sign flip under
          way. On susceptible motors every hot +->- flip latches the 0x46
          encoder readback (LATCH lines appear; XCHECK CHIP = the register
          itself is frozen while the wheel spins).
  fixed : the ONLY change — each reversal passes through a commanded zero
          held for 50 ms (the proven-minimum dwell; 20 ms fails). Expect
          zero latches.

Susceptibility is state-dependent: the first run after a long rest can be
clean. If `latch` reports 0, run it again — the second run latches.
"""

from __future__ import annotations

import argparse
import re
import sys

from wedgelab import DEFAULT_PORT, Lab, log_path

CYCLES = 20

SETUP = ["set nwheels 2", "set tickus 10000", "set sensors 1"]
MODES = {
    "latch": ["set resetmode 1"],                       # reversal, production-style
    "fixed": ["set resetmode 4", "set zerodwell 50"],   # reversal via 50 ms zero-dwell
}


def last_result(lab: Lab) -> int:
    # last RESULT line is in the log; re-parse from the log file tail
    for line in reversed(open(lab.log.name).read().splitlines()):
        m = re.search(r"RESULT reset n=\d+ ep=(\d+),(\d+)", line)
        if m:
            return int(m.group(1)) + int(m.group(2))
    return -1


def run_mode(lab: Lab, mode: str, cycles: int) -> int:
    print(f"\n=== {mode.upper()} mode: {cycles} reversal cycles "
          f"({'production immediate flip' if mode == 'latch' else '50 ms zero-dwell flip'}) ===")
    for cmd in SETUP + MODES[mode]:
        lab.send(cmd)
        lab.pump(0.5)
    # Susceptibility is state-dependent: after a long rest the first block
    # is often clean. LATCH mode self-warms — up to 3 blocks, stopping at
    # the first one that locks up. FIXED mode runs once (its point is the
    # absence of lockups on a WARM system, right after latch mode).
    blocks = 3 if mode == "latch" else 1
    total = 0
    for b in range(blocks):
        if b:
            print(f"--- no lockups yet (cold state) — warm-up block {b + 1} ---")
        lab.send(f"run reset {cycles}")
        lab.pump(cycles * 1.5 + 60, ("RESULT ",))
        total = last_result(lab)
        if total != 0:
            break
    lab.send("heal")
    lab.pump(30, ("(heal-end)",))
    return total


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", nargs="?", choices=["latch", "fixed", "both"],
                    default="both")
    ap.add_argument("--cycles", type=int, default=CYCLES)
    ap.add_argument("--port", default=DEFAULT_PORT)
    args = ap.parse_args()

    lab = Lab(args.port, log_path(f"demo-{args.mode}"))
    modes = ["latch", "fixed"] if args.mode == "both" else [args.mode]
    results: dict[str, int] = {}
    for m in modes:
        results[m] = run_mode(lab, m, args.cycles)

    print("\n" + "=" * 60)
    for m, n in results.items():
        if m == "latch":
            verdict = (f"LOCKED UP {n} times in {args.cycles} cycles"
                       if n > 0 else
                       "0 latches in 3 warm-up blocks — unexpected; check that "
                       "the latch-prone motors are on M1/M2")
        else:
            verdict = (f"0 latches in {args.cycles} cycles — fix holds"
                       if n == 0 else
                       f"{n} latches DESPITE the fix — investigate!")
        print(f"  {m:>6}: {verdict}")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
