#!/usr/bin/env python3
"""solve_time_characterize.py -- on-target Ruckig solve-time characterization
for sprint 089 ticket 007 (architecture-update.md Open Question 4).

Attaches to an ALREADY-RUNNING pyOCD gdbserver (`just debug` / `pyocd
gdbserver -t nrf52833 --persist`, started separately) via a plain `target
remote` (no `load`, no `monitor reset halt` -- the running firmware and its
live serial session are undisturbed, per `.claude/rules/debugging.md`: "The
serial link and the debugger coexist"). Enables the Cortex-M4 DWT cycle
counter, sets a breakpoint at the `otg_.calculate()` call site shared by
`Motion::JerkTrajectory::solvePositionControl()` (the position-control solve
used by D's linear channel and TURN/RT's rotational channel), and measures
elapsed CPU cycles across the call via `next` (which steps over the ENTIRE
call at full speed, not single-instruction-stepping -- DWT->CYCCNT freezes
while the core is halted for a debug read, so the T1/T2 delta reflects only
the actual running time of the `calculate()` call itself, uncontaminated by
SWD round-trip latency).

Keeps ONE serial connection open for both the D and RT characterization
runs (opening a fresh connection pulses DTR and resets the board -- see
`SerialConnection.connect()`'s own doc comment -- which would be fine
between runs but is avoided here to keep this script simple and to prove
the debug session and the serial link coexist without a reset in between,
matching the debugging rule doc's own claim).

Usage:
    uv run python src/tests/bench/solve_time_characterize.py
"""
from __future__ import annotations

import re
import subprocess
import sys
import time
from pathlib import Path

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
GDB_BATCH = Path(__file__).parent / "solve_time_gdb_batch.gdb"
CPU_CLOCK_HZ = 64_000_000   # nRF52833 max core clock


def run_one(conn: SerialConnection, proto: NezhaProtocol, label: str, drive_cmd: str) -> dict:
    log_path = Path(__file__).parent / "out" / f"solve_time_{label}.gdb.log"
    with open(log_path, "w") as logf:
        gdb = subprocess.Popen(
            ["arm-none-eabi-gdb", "-q", "--batch",
             "/Volumes/Proj/proj/RobotProjects/radio-robot-elite/build/MICROBIT",
             "-x", str(GDB_BATCH)],
            stdout=logf, stderr=subprocess.STDOUT,
        )
    # Give gdb time to attach, set the breakpoint, enable DWT, and reach
    # `continue` (blocking on the target, waiting for the breakpoint hit).
    time.sleep(3.0)

    proto.send("STOP", read_timeout=200)
    time.sleep(0.3)
    resp = proto.send(drive_cmd, read_timeout=500)
    print(f"[{label}] dispatch: {resp}")

    # Wait for gdb to hit the breakpoint, capture, and detach/quit.
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
        # DWT->CYCCNT is a free-running 32-bit counter; handle a single wrap.
        delta = (t2 - t1) if t2 >= t1 else (t2 + (1 << 32) - t1)
        result["cycles"] = delta
        result["elapsed_us"] = delta / (CPU_CLOCK_HZ / 1_000_000.0)
    else:
        result["error"] = "breakpoint never hit or CYCCNT not captured"

    # Let the drive command actually finish/settle before the next one.
    time.sleep(2.5)
    proto.send("STOP", read_timeout=200)
    return result


def main() -> int:
    conn = SerialConnection(port=DEFAULT_PORT)
    info = conn.connect()
    print(f"connected: {info}")
    if "error" in info:
        return 2
    proto = NezhaProtocol(conn)
    proto.send("DEV WD 20000", read_timeout=300)

    results = []
    try:
        # Linear channel: D's initial solveToRest() -> solvePositionControl().
        results.append(run_one(conn, proto, "D_linear", "D 150 150 400"))
        time.sleep(1.0)
        # Rotational channel: RT's initial solveToRest() -> solvePositionControl()
        # on the SAME function (a different JerkTrajectory instance/channel).
        results.append(run_one(conn, proto, "RT_rotational", "RT 4500"))
    finally:
        proto.send("STOP", read_timeout=300)
        proto.send("DEV STOP", read_timeout=300)
        proto.send("DEV WD 1000", read_timeout=300)
        conn.disconnect()

    print("\n=== Solve-time characterization results ===")
    for r in results:
        print(r)
    return 0


if __name__ == "__main__":
    sys.exit(main())
