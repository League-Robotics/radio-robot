"""wedge_latch_matrix.py — reliable encoder-latch (wedge) reproduction harness.

Drives the firmware's self-contained `DBG WEDGE` exerciser (WedgeTest.cpp —
takes over the robot: no planner, no EKF, no telemetry; motors + encoder
reads only) across a stimulus matrix, measuring TIME-TO-LATCH per
configuration.  The output is the discriminating dataset for the wedge
sprint: which knob (real PID path, write throttle, dither, shared-bus sensor
traffic) provokes the 0x46 readback latch, and how reliably.

DBG WEDGE args (positional): rate[Hz] writeMs bus[kHz] dither reg sensors realCtrl
Verdict line on latch: 'WEDGE-POS-FROZEN'.  Any serial byte aborts a run.

Between trials the encoder is re-primed at rest (`ZERO enc` — the documented
un-latcher) so a latched register does not bias the next trial.

Usage:
    uv run python tests/bench/wedge_latch_matrix.py [--port PORT]
        [--trial-timeout 120] [--trials 2] [--cells production,unthrottled,...]

Robot must be ON THE STAND (motors will spin).

KNOWN CAVEAT (2026-07-03): WedgeTest can emit WEDGE-LATCHED within ~2 ticks
of start (glitch-never-recovered verdict against a stale baseline) — review
its verdict init before trusting sub-second latch times.  For day-to-day
reproduction prefer tests/bench/wedge_latch_repro.py (D-decel cycles,
firmware EVT ground truth, measured 0.63 episodes/leg on 2026-07-03).
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

import serial

_REPO = pathlib.Path(__file__).resolve().parents[2]
OUTDIR = _REPO / "tests" / "bench" / "out"
OUTDIR.mkdir(parents=True, exist_ok=True)

# name -> (rate, writeMs, bus, dither, reg, sensors, realCtrl)
CELLS: dict[str, tuple[int, int, int, int, int, int, int]] = {
    # Full production-faithful path: real velocity PID, 40ms write throttle,
    # sensors sharing the bus.  KB hypothesis: THE reproducer.
    "production":   (50, 40, 400, 3, 0x46, 1, 1),
    # Same but without the shared-bus sensor traffic.
    "no-sensors":   (50, 40, 400, 3, 0x46, 0, 1),
    # Real PID with the write throttle REMOVED (write every tick) — the
    # decel-throttle-bypass hypothesis says stop/reversal writes escaping the
    # throttle are the trigger; writeMs=0 makes every tick look like that.
    "unthrottled":  (50, 0, 400, 3, 0x46, 1, 1),
    # Raw fixed-PWM path (sprint-015 flavor; historically did NOT wedge).
    "raw-dither":   (50, 40, 400, 3, 0x46, 1, 0),
    # Production at 100 kHz bus (the main.cpp mitigation speed).
    "prod-100khz":  (50, 40, 100, 3, 0x46, 1, 1),
}


def read_line(ser: serial.Serial) -> str:
    return ser.readline().decode("utf-8", "ignore").strip()


def drain(ser: serial.Serial, secs: float) -> list[str]:
    t0 = time.monotonic()
    lines = []
    while time.monotonic() - t0 < secs:
        ln = read_line(ser)
        if ln:
            lines.append(ln)
    return lines


def reprime(ser: serial.Serial) -> None:
    """Re-prime a possibly-latched register at rest (ZERO enc atomic reset)."""
    ser.write(b"ZERO enc\n")
    ser.flush()
    drain(ser, 1.5)


def run_trial(ser: serial.Serial, cell: str, params, timeout_s: float,
              log) -> dict:
    rate, write_ms, bus, dither, reg, sensors, real = params
    cmd = f"DBG WEDGE {rate} {write_ms} {bus} {dither} {reg} {sensors} {real}\n"
    ser.reset_input_buffer()
    ser.write(cmd.encode())
    ser.flush()
    t0 = time.monotonic()
    verdict = None
    status_lines = 0
    last_status = ""
    while time.monotonic() - t0 < timeout_s:
        ln = read_line(ser)
        if not ln:
            continue
        u = ln.upper()
        if "WEDGE-" in u and ("FROZEN" in u or "LATCHED" in u or "FW-STUCK" in u):
            verdict = ln
            break
        if ln.startswith(("tick", "rate", "loop")) or "writes" in ln:
            status_lines += 1
            last_status = ln
    elapsed = time.monotonic() - t0
    if verdict is None:
        # Abort the exerciser: any serial byte stops it.
        ser.write(b"\n")
        ser.flush()
        drain(ser, 2.0)
        log(f"  [{cell}] NO latch in {timeout_s:.0f}s  (status lines: {status_lines}, last: {last_status[:60]!r})")
        return {"cell": cell, "latched": False, "t_s": None}
    log(f"  [{cell}] LATCHED after {elapsed:.1f}s — {verdict[:90]}")
    drain(ser, 2.0)  # let 'wedge end' flush
    return {"cell": cell, "latched": True, "t_s": round(elapsed, 1), "verdict": verdict}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/cu.usbmodem2121102")
    ap.add_argument("--trial-timeout", type=float, default=120.0)
    ap.add_argument("--trials", type=int, default=2)
    ap.add_argument("--cells", default=",".join(CELLS.keys()))
    args = ap.parse_args()

    def log(m):
        print(f"{time.strftime('%H:%M:%S')} {m}", flush=True)

    results = []
    ser = serial.Serial(args.port, 115200, timeout=0.5)
    try:
        drain(ser, 3.0)  # boot banner after DTR reset
        ser.write(b"PING\n")
        ser.flush()
        pong = drain(ser, 1.5)
        log(f"connected: {[l for l in pong if 'pong' in l][:1]}")

        for cell in args.cells.split(","):
            cell = cell.strip()
            if cell not in CELLS:
                log(f"  (unknown cell {cell!r} — skipped)")
                continue
            for n in range(args.trials):
                log(f"=== {cell} trial {n+1}/{args.trials} "
                    f"(rate,writeMs,bus,dither,reg,sensors,real = {CELLS[cell]}) ===")
                reprime(ser)
                time.sleep(1.0)
                results.append(run_trial(ser, cell, CELLS[cell],
                                         args.trial_timeout, log))
    finally:
        try:
            ser.write(b"\n")     # abort any running exerciser
            ser.flush()
            time.sleep(0.5)
            ser.write(b"X\n")    # motors off
            ser.flush()
            ser.close()
        except Exception:
            pass
        out = OUTDIR / "wedge_latch_matrix.json"
        out.write_text(json.dumps(results, indent=1))
        log(f"wrote {out}")

    # Summary table.
    log("---- summary (time-to-latch, s) ----")
    for cell in CELLS:
        rs = [r for r in results if r["cell"] == cell]
        if not rs:
            continue
        ts = [r["t_s"] for r in rs if r["latched"]]
        log(f"  {cell:<14} latched {len(ts)}/{len(rs)}  times={ts}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
