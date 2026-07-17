#!/usr/bin/env python3
"""solve_time_characterize.py -- solve-time characterization for
`Motion::JerkTrajectory` (sprint 109 ticket 001; originally sprint 089
ticket 007's on-target-only script, architecture-update.md Open Question 4).

Two modes:

**HOST mode (default, `--host`)** -- compiles
`solve_time_timing_harness.cpp` together with `src/firm/motion/
jerk_trajectory.cpp` and the vendored Ruckig sources
(`src/vendor/ruckig/src/*.cpp`) under the firmware's exact build
constraints (gnu++20, -fno-exceptions -fno-rtti), runs it, and reports p50/
p99 wall-clock solve time per channel from `std::chrono::steady_clock`
samples. This is the gate this ticket (109-001) actually closes with --
see "Why HOST mode is the gate this ticket closes" below.

**ON-TARGET mode (`--on-target`)** -- the original approach: attach to an
already-running pyOCD gdbserver (`just debug`), breakpoint the
`otg_.calculate()` call site inside `solvePositionControl()`, and measure
Cortex-M4 DWT cycle-counter deltas across the call via the real serial
link. Kept for ticket 003+ (once a wire verb calls into `Motion::
JerkTrajectory` from `App::RobotLoop`/`App::Pilot`) -- see the caveat below
for why it cannot currently produce a result.

## Why HOST mode is the gate this ticket closes

This ticket (109-001) restores `Motion::JerkTrajectory` but deliberately
does not wire it into the running loop -- that is ticket 003's job (see
this ticket's own description: "Nothing in this ticket wires the solver
into the running loop"). Two independent facts follow from that:

1. `-Wl,--gc-sections` (already in the vendored codal target's linker
   flags) discards any code with no call site. Confirmed by
   `arm-none-eabi-nm build/MICROBIT | grep JerkTrajectory` returning
   **zero** symbols after this ticket's own from-scratch build -- the
   class is not merely uncalled, it is not present in the linked ELF at
   all. There is no address for a breakpoint to land on.
2. Even with hardware physically connected, ON-TARGET mode's breakpoint
   (`otg_.calculate()` inside `solvePositionControl()`) has nothing to hit.
   This is a structural fact about this ticket's own scope, independent of
   whatever micro:bit happens to be plugged in on a given session.

ON-TARGET mode's `--on-target` code path is kept, with its breakpoint
location updated to this ticket's new file layout (`src/firm/motion/
jerk_trajectory.cpp`, not the old `source/motion/jerk_trajectory.cpp`), so
a future ticket (003, once `App::Pilot`/`Motion::Executor` gives
`solvePositionControl()` a real caller) can re-run it and get a real
number -- but running it THIS ticket, on ANY hardware, cannot produce
anything but "breakpoint never hit" per point 1 above. HOST mode is this
ticket's own gate; see the ticket's completion notes for the numbers this
run actually produced, and for the honest caveat that host wall-clock
timing on an Apple Silicon/x86 development machine is not a substitute for
Cortex-M4 cycle counts -- it proves the solve is fast and bounded, not
what ticket 003's cycle-budget check will need (that check re-runs
ON-TARGET mode once the call site exists).

Usage:
    uv run python src/tests/bench/solve_time_characterize.py          # HOST (default)
    uv run python src/tests/bench/solve_time_characterize.py --on-target
"""
from __future__ import annotations

import argparse
import pathlib
import re
import shutil
import statistics
import subprocess
import sys
import time

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SOURCE_DIR = _REPO_ROOT / "src" / "firm"
_RUCKIG_INCLUDE = _REPO_ROOT / "src" / "vendor" / "ruckig" / "include"
_RUCKIG_SRC_DIR = _REPO_ROOT / "src" / "vendor" / "ruckig" / "src"
_JERK_TRAJECTORY_SRC = _SOURCE_DIR / "motion" / "jerk_trajectory.cpp"
_TIMING_HARNESS_SRC = pathlib.Path(__file__).parent / "solve_time_timing_harness.cpp"
_GDB_BATCH = pathlib.Path(__file__).parent / "solve_time_gdb_batch.gdb"

_CXX_STANDARD = "gnu++20"
_CONSTRAINT_FLAGS = ["-fno-exceptions", "-fno-rtti"]

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
CPU_CLOCK_HZ = 64_000_000  # nRF52833 max core clock


def _percentile(samples: list[float], pct: float) -> float:
    ordered = sorted(samples)
    idx = min(len(ordered) - 1, max(0, int(round(pct / 100.0 * (len(ordered) - 1)))))
    return ordered[idx]


def run_host() -> int:
    cxx = None
    for candidate in ("c++", "clang++", "g++"):
        found = shutil.which(candidate)
        if found:
            cxx = found
            break
    if cxx is None:
        print("no system C++ compiler (c++/clang++/g++) found on PATH", file=sys.stderr)
        return 2

    ruckig_srcs = sorted(_RUCKIG_SRC_DIR.glob("*.cpp"))
    if not ruckig_srcs:
        print(f"no vendored ruckig sources under {_RUCKIG_SRC_DIR}", file=sys.stderr)
        return 2

    out_dir = pathlib.Path(__file__).parent / "out"
    out_dir.mkdir(exist_ok=True)
    binary = out_dir / "solve_time_timing_harness"

    compile_cmd = [
        cxx,
        f"-std={_CXX_STANDARD}",
        *_CONSTRAINT_FLAGS,
        "-O2",
        "-I", str(_SOURCE_DIR),
        "-I", str(_RUCKIG_INCLUDE),
        "-o", str(binary),
        str(_TIMING_HARNESS_SRC),
        str(_JERK_TRAJECTORY_SRC),
        *[str(s) for s in ruckig_srcs],
    ]
    compiled = subprocess.run(compile_cmd, capture_output=True, text=True)
    if compiled.returncode != 0:
        print("compile failed:\n" + compiled.stdout + compiled.stderr, file=sys.stderr)
        return 1

    run = subprocess.run([str(binary)], capture_output=True, text=True)
    if run.returncode != 0:
        print("harness run failed:\n" + run.stdout + run.stderr, file=sys.stderr)
        return 1

    by_label: dict[str, list[float]] = {}
    for line in run.stdout.splitlines():
        parts = line.split()
        if len(parts) != 2:
            continue
        label, ns = parts
        by_label.setdefault(label, []).append(int(ns) / 1000.0)  # -> microseconds

    print("=== solve_time_characterize.py -- HOST mode ===")
    print("(host wall-clock timing, NOT Cortex-M4 cycle counts -- see this")
    print(" script's own module docstring for the caveat and why this is the")
    print(" gate 109-001 closes, not a substitute for the ON-TARGET measurement)")
    print()
    for label, samples in by_label.items():
        p50 = _percentile(samples, 50)
        p99 = _percentile(samples, 99)
        mean = statistics.fmean(samples)
        print(f"[{label}] n={len(samples)} mean={mean:.2f}us p50={p50:.2f}us "
              f"p99={p99:.2f}us max={max(samples):.2f}us")
    return 0


def run_one_on_target(conn, proto, label: str, drive_cmd: str) -> dict:
    from pathlib import Path

    log_path = Path(__file__).parent / "out" / f"solve_time_{label}.gdb.log"
    with open(log_path, "w") as logf:
        gdb = subprocess.Popen(
            ["arm-none-eabi-gdb", "-q", "--batch",
             str(_REPO_ROOT / "build" / "MICROBIT"),
             "-x", str(_GDB_BATCH)],
            stdout=logf, stderr=subprocess.STDOUT,
        )
    time.sleep(3.0)

    proto.send("STOP", read_timeout=200)
    time.sleep(0.3)
    resp = proto.send(drive_cmd, read_timeout=500)
    print(f"[{label}] dispatch: {resp}")

    try:
        gdb.wait(timeout=8.0)
    except subprocess.TimeoutExpired:
        gdb.kill()
        gdb.wait(timeout=3.0)

    log_text = log_path.read_text()
    print(f"[{label}] gdb log:\n{log_text}")

    t1_m = re.search(r"T1_CYCCNT=(\d+)", log_text)
    t2_m = re.search(r"T2_CYCCNT=(\d+)", log_text)
    hit_m = re.search(r"HIT pc=(\S+)", log_text)
    result = {"label": label, "hit_pc": hit_m.group(1) if hit_m else None}
    if t1_m and t2_m:
        t1, t2 = int(t1_m.group(1)), int(t2_m.group(1))
        delta = (t2 - t1) if t2 >= t1 else (t2 + (1 << 32) - t1)
        result["cycles"] = delta
        result["elapsed_us"] = delta / (CPU_CLOCK_HZ / 1_000_000.0)
    else:
        result["error"] = "breakpoint never hit or CYCCNT not captured"

    time.sleep(2.5)
    proto.send("STOP", read_timeout=200)
    return result


def run_on_target(port: str) -> int:
    # Deferred imports: this path needs the host package + a live serial
    # link; HOST mode (the default, and this ticket's actual gate) needs
    # neither.
    sys.path.insert(0, str(_REPO_ROOT / "src" / "host"))
    from robot_radio.io.serial_conn import SerialConnection
    from robot_radio.robot.protocol import NezhaProtocol

    print("=== solve_time_characterize.py -- ON-TARGET mode ===")
    print("NOTE (109-001): Motion::JerkTrajectory has NO call site in the")
    print("current firmware image (ticket 003 wires it in) -- "
          "`arm-none-eabi-nm build/MICROBIT | grep JerkTrajectory` returns")
    print("ZERO symbols (gc-sections discarded it). The breakpoint below")
    print("cannot be hit THIS ticket regardless of hardware state. Kept for")
    print("ticket 003+ to re-run once a real call site exists.")
    print()

    conn = SerialConnection(port=port)
    info = conn.connect()
    print(f"connected: {info}")
    if "error" in info:
        return 2
    proto = NezhaProtocol(conn)
    proto.send("DEV WD 20000", read_timeout=300)

    results = []
    try:
        results.append(run_one_on_target(conn, proto, "D_linear", "D 150 150 400"))
        time.sleep(1.0)
        results.append(run_one_on_target(conn, proto, "RT_rotational", "RT 4500"))
    finally:
        proto.send("STOP", read_timeout=300)
        proto.send("DEV STOP", read_timeout=300)
        proto.send("DEV WD 1000", read_timeout=300)
        conn.disconnect()

    print("\n=== Solve-time characterization results ===")
    for r in results:
        print(r)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--on-target", action="store_true",
                         help="Use the original pyOCD/gdb on-target path (see module "
                              "docstring for why it cannot produce a result this ticket).")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port for --on-target.")
    args = parser.parse_args()

    if args.on_target:
        return run_on_target(args.port)
    return run_host()


if __name__ == "__main__":
    sys.exit(main())
