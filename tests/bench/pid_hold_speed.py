#!/usr/bin/env python3
"""pid_hold_speed.py — coupled-rig PID disturbance-rejection test (ticket 077-006).

Bench setup: two motors mechanically linked by a shared shaft (ports 3 and 4
by default — the coupled bench rig; running one loads the other, see the
sprint-077 issue's "A motor is instantiated on a port" note). This script
holds a `DEV M <pid-port> VEL` target on one motor while stepping the OTHER
motor through a load-duty schedule (assist → freewheel → drag → reverse) via
`DEV M <load-port> DUTY`, and checks that the held motor's embedded velocity
PID (`docs/protocol-v2.md` §16) actually rejects the disturbance rather than
just coasting.

PASS conditions (reported per load step and overall):
  - measured velocity on --pid-port stays within --tolerance of the target
    once each step has settled (the last --settle-time seconds of the step);
  - applied duty on --pid-port rises (monotonically, allowing a small
    epsilon) across the assist → freewheel → drag → reverse schedule — proof
    the PID is actually working harder against a bigger disturbance, not
    coasting at a fixed duty.

Logs every sample to --csv as (t, target, velocity, applied, load_duty).

Safety: `DEV WD 3000` widens the serial-silence watchdog for the session;
the `finally` block always sends `DEV STOP` (neutralizes both the held motor
and the loading motor — `DEV STOP` is the global "everything off" verb) and
restores `DEV WD 1000`, on a clean run, a failure, or Ctrl-C.

Usage:
    uv run python tests/bench/pid_hold_speed.py
    uv run python tests/bench/pid_hold_speed.py --pid-port 3 --load-port 4 --target 150
"""

from __future__ import annotations

import argparse
import csv
import pathlib
import sys
import time

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol, ParsedResponse, parse_response

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
SESSION_WATCHDOG_WINDOW = 3000    # [ms]
BOOT_WATCHDOG_WINDOW = 1000       # [ms] firmware default — restored on exit
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]

# Load schedule: (label, duty [%, -100..100]) applied to --load-port while
# --pid-port holds its VEL target. Signs assume the loading motor's positive
# duty direction matches the held motor's forward direction on the coupled
# rig (assist = same direction eases the PID's job; drag/reverse = opposing
# duty makes it work harder) — verify against the actual rig wiring and
# override via --assist-duty/--drag-duty/--reverse-duty if the physical
# coupling runs the other way.
_SCHEDULE_LABELS = ("assist", "freewheel", "drag", "reverse")


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port (default {DEFAULT_PORT})")
    p.add_argument("--pid-port", type=int, default=3,
                   help="Motor port holding the VEL target (default 3)")
    p.add_argument("--load-port", type=int, default=4,
                   help="Motor port applying the load duties (default 4)")
    p.add_argument("--target", type=float, default=150.0,
                   help="VEL target on --pid-port, mm/s (default 150)")
    p.add_argument("--assist-duty", type=float, default=25.0,
                   help="Load duty for the 'assist' step, percent (default 25)")
    p.add_argument("--drag-duty", type=float, default=-25.0,
                   help="Load duty for the 'drag' step, percent (default -25)")
    p.add_argument("--reverse-duty", type=float, default=-50.0,
                   help="Load duty for the 'reverse' step, percent (default -50)")
    p.add_argument("--step-time", type=float, default=4.0,
                   help="Seconds to hold each load step (default 4)")
    p.add_argument("--settle-time", type=float, default=2.0,
                   help="Seconds at the END of each step counted as 'settled' "
                        "for the tolerance check (default 2, must be < --step-time)")
    p.add_argument("--tolerance", type=float, default=25.0,
                   help="Acceptable |measured - target| once settled, mm/s (default 25)")
    p.add_argument("--sample-period", type=float, default=0.1,
                   help="Seconds between DEV M STATE polls (default 0.1)")
    p.add_argument("--csv", default=str(_REPO_ROOT / "tests" / "bench" / "out" / "pid_hold_speed.csv"),
                   help="CSV output path")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Small helpers (same shape as dev_exercise.py's — kept local; these scripts
# are standalone CLI tools per the ticket's locked tests/bench/ layout, not a
# shared library module).
# ---------------------------------------------------------------------------

def dev_send(proto: NezhaProtocol, cmd: str, timeout: int = 500,  # [ms]
            retries: int = 6) -> ParsedResponse | None:
    """Send one DEV command, retrying on a totally silent reply.

    077-007's HITL bench pass found this bench's direct-USB CDC link
    outright, occasionally burstily drops replies (same finding as
    dev_exercise.py's dev_send() — see its docstring for the measurements
    and rationale). Without this retry, a dropped sample here just reads as
    a `None` velocity/applied point for one row of the CSV (the sampling
    loop below already tolerates the occasional single miss) — but a
    multi-sample burst-loss can blank out an entire settle window and turn
    a real PASS into a false FAIL on pure transport noise, which this
    ticket's bench pass caught. Safe to retry unconditionally: every command
    this script sends is either a pure query (STATE) or an idempotent
    absolute-value write (VEL/DUTY/WD/STOP) — re-sending an unacknowledged
    one just re-applies the same value.
    """
    for attempt in range(retries):
        resp = proto.send(cmd, timeout)
        for raw in resp.get("responses", []):
            r = parse_response(raw)
            if r is not None and r.tag in ("OK", "ERR"):
                return r
        if attempt < retries - 1:
            time.sleep(0.1)
    return None


def _kv_float(r: ParsedResponse | None, key: str) -> float | None:
    if r is None or key not in r.kv:
        return None
    try:
        return float(r.kv[key])
    except ValueError:
        return None


def main() -> int:
    args = _parse_args()
    schedule = [
        ("assist", args.assist_duty),
        ("freewheel", 0.0),
        ("drag", args.drag_duty),
        ("reverse", args.reverse_duty),
    ]
    print(f"  port: {args.port}   pid_port: {args.pid_port}   load_port: {args.load_port}"
          f"   target: {args.target:g} mm/s")

    csv_path = pathlib.Path(args.csv)
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    conn = SerialConnection(port=args.port)
    proto: NezhaProtocol | None = None
    csv_file = None
    step_results: list[dict] = []
    overall_pass = False

    try:
        info = conn.connect()
        if "error" in info:
            print(f"ERROR: connect failed: {info['error']}")
            return 2
        print(f"  connected: mode={info.get('mode')}")
        proto = NezhaProtocol(conn)

        dev_send(proto, f"DEV WD {SESSION_WATCHDOG_WINDOW}")

        csv_file = open(csv_path, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow(["t", "target", "velocity", "applied", "load_duty"])

        dev_send(proto, f"DEV M {args.pid_port} VEL {args.target}")
        t_start = time.monotonic()

        for label, duty in schedule:
            print(f"\n  step [{label}] load_duty={duty:+.0f}%")
            dev_send(proto, f"DEV M {args.load_port} DUTY {duty}")
            step_t0 = time.monotonic()
            samples: list[tuple[float, float | None, float | None]] = []
            while time.monotonic() - step_t0 < args.step_time:
                t_step = time.monotonic() - step_t0
                st = dev_send(proto, f"DEV M {args.pid_port} STATE")
                vel, applied = _kv_float(st, "vel"), _kv_float(st, "applied")
                writer.writerow([f"{time.monotonic() - t_start:.3f}", args.target, vel, applied, duty])
                samples.append((t_step, vel, applied))
                time.sleep(args.sample_period)

            settled = [s for s in samples if s[0] >= args.step_time - args.settle_time]
            settled_vels = [v for _, v, _ in settled if v is not None]
            settled_applied = [a for _, _, a in settled if a is not None]
            in_band = bool(settled_vels) and all(
                abs(v - args.target) <= args.tolerance for v in settled_vels)
            avg_vel = sum(settled_vels) / len(settled_vels) if settled_vels else None
            avg_applied = sum(settled_applied) / len(settled_applied) if settled_applied else None
            step_results.append({
                "label": label, "duty": duty, "in_band": in_band,
                "avg_vel": avg_vel, "avg_applied": avg_applied,
            })
            print(f"    settled avg_vel={avg_vel}  avg_applied={avg_applied}"
                  f"  in_band={'PASS' if in_band else 'FAIL'}")

        # Overall PASS 1: velocity held in-band across every load step.
        all_in_band = all(r["in_band"] for r in step_results)

        # Overall PASS 2: applied duty rose (non-decreasing, small epsilon)
        # across the schedule — proof the PID is doing real work against a
        # bigger disturbance, not coasting at a fixed duty.
        applied_seq = [r["avg_applied"] for r in step_results]
        rising = (all(a is not None for a in applied_seq)
                  and all(applied_seq[i + 1] >= applied_seq[i] - 0.02
                          for i in range(len(applied_seq) - 1)))

        print(f"\n  {'PASS' if all_in_band else 'FAIL'}: velocity held within "
              f"±{args.tolerance:g} mm/s across all load steps")
        print(f"  {'PASS' if rising else 'FAIL'}: applied duty rose with load "
              f"({[f'{a:.3f}' if a is not None else 'None' for a in applied_seq]})")
        print(f"\n  CSV: {csv_path}")

        overall_pass = all_in_band and rising

    except KeyboardInterrupt:
        print("\n  interrupted — stopping motors...")
        overall_pass = False
    finally:
        if csv_file is not None:
            csv_file.close()
        if proto is not None:
            try:
                dev_send(proto, "DEV STOP")
            except Exception as exc:
                print(f"  WARN: DEV STOP failed during cleanup: {exc}")
            try:
                dev_send(proto, f"DEV WD {BOOT_WATCHDOG_WINDOW}")
            except Exception as exc:
                print(f"  WARN: DEV WD restore failed during cleanup: {exc}")
        if conn.is_open:
            conn.disconnect()

    return 0 if overall_pass else 1


if __name__ == "__main__":
    sys.exit(main())
