#!/usr/bin/env python3
"""pid_hold_speed.py — coupled-rig PID disturbance-rejection test (ticket 077-006/077-007).

Bench setup: two motors on the coupled bench rig (ports 3 and 4 by default)
whose shafts/wheels are in FRICTION contact — not a positive (geared/belted)
drive. Stakeholder clarification (077-007 HITL pass, 2026-07-04): friction
coupling only transmits load when BOTH motors are spinning — a stopped
motor's stiction defeats the friction contact outright (it just slips), so
the earlier "drive one, watch the other sit still" probe was the wrong test
for this rig. The real effect only shows with --pid-port already holding a
velocity: changing --load-port's speed changes how much the friction contact
drags on (or eases) the held motor, which shows up as a shift in the held
motor's APPLIED DUTY (the embedded PID needs more or less duty to hold the
same velocity), not as a change to the held motor's OWN measured velocity
(that's supposed to stay put — the PID's job).

Empirically measured on the bench 2026-07-04 (--pid-port 3 holding 150 mm/s,
--load-port 4 stepped through +50/+25/0/-25/-50 duty): motor 3's applied
duty fell MONOTONICALLY as motor 4's duty went from +50 (same rotational
sign as motor 3's forward direction) down to -50 (reverse) —
0.380 -> 0.370 -> 0.367 -> 0.320 -> 0.230 — while motor 3's velocity stayed
in a tight band (149-157 mm/s) throughout. That is: on THIS rig, same-sign
duty on the loading motor is the heavier friction load and reverse duty is
the lighter one (very likely a contact-geometry artifact of how the two
wheels are mounted — two wheels touching at their rims need OPPOSITE
rotational signs to have matching, low-slip contact-point velocity, so
"same sign" here means more relative slip, i.e. more friction drag; this
script does not need to know why, only that the direction is monotonic and
repeatable). The default --load-duties schedule below is ordered from that
measured HEAVIEST setting to the LIGHTEST, and the "applied duty tracks
load" PASS check asserts the monotonically-falling direction actually
measured — override --load-duties (and swap PASS's expected direction, see
`_applied_tracks_load()`) if a different rig's contact geometry runs the
other way.

PASS conditions (reported per load step and overall):
  - measured velocity on --pid-port stays within --tolerance of the target
    once each step has settled (the last --settle-time seconds of the step)
    — the PID is actually holding the target, not drifting with the load;
  - applied duty on --pid-port tracks the load schedule (monotonically,
    allowing a small epsilon, in the measured heaviest-to-lightest
    direction) — proof the PID is doing real work against a changing
    friction disturbance, not coasting at a fixed duty.

Logs every sample to --csv as (t, target, velocity, applied, load_duty).

Safety: `DEV WD 3000` widens the serial-silence watchdog for the session;
the `finally` block always sends `DEV STOP` (neutralizes both the held motor
and the loading motor — `DEV STOP` is the global "everything off" verb) and
restores `DEV WD 1000`, on a clean run, a failure, or Ctrl-C.

Usage:
    uv run python tests/bench/pid_hold_speed.py
    uv run python tests/bench/pid_hold_speed.py --pid-port 3 --load-port 4 --target 150
    uv run python tests/bench/pid_hold_speed.py --load-duties 50,25,0,-25,-50
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

# Default load schedule (--load-port duty, percent), ordered from the
# measured HEAVIEST friction load on --pid-port to the LIGHTEST — see the
# module docstring for the 2026-07-04 bench measurement this ordering is
# based on. Each entry is labeled by its duty value directly (no
# assist/drag/reverse naming — those assumed a positive-drive relationship
# this friction rig does not have).
_DEFAULT_LOAD_DUTIES = (50.0, 25.0, 0.0, -25.0, -50.0)


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
    p.add_argument("--load-duties", type=str,
                   default=",".join(str(d) for d in _DEFAULT_LOAD_DUTIES),
                   help="Comma-separated --load-port duty schedule, percent "
                        "-100..100 (default '50,25,0,-25,-50' — measured "
                        "heaviest-to-lightest friction load order on this "
                        "rig; see module docstring)")
    p.add_argument("--step-time", type=float, default=10.0,
                   help="Seconds to hold each load step (default 10 — wide "
                        "enough to absorb dev_send()'s worst-case retry "
                        "budget, ~6.6 s at its defaults, without starving "
                        "the settled window; see 077-007 bench notes)")
    p.add_argument("--settle-time", type=float, default=5.0,
                   help="Seconds at the END of each step counted as 'settled' "
                        "for the tolerance check (default 5, must be < --step-time)")
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


def _applied_tracks_load(applied_seq: list[float | None]) -> bool:
    """True if applied duty falls monotonically across the load schedule.

    The default --load-duties schedule is ordered heaviest-to-lightest per
    the 2026-07-04 bench measurement (see module docstring), so a working
    PID should need monotonically LESS applied duty as the schedule
    progresses. Allows a small epsilon for sample noise (same slack as the
    old assist/drag/reverse check this replaces).
    """
    return (all(a is not None for a in applied_seq)
            and all(applied_seq[i + 1] <= applied_seq[i] + 0.02
                    for i in range(len(applied_seq) - 1)))


def main() -> int:
    args = _parse_args()
    load_duties = [float(tok) for tok in args.load_duties.split(",") if tok.strip()]
    schedule = [(f"m4_duty={d:+.0f}", d) for d in load_duties]
    print(f"  port: {args.port}   pid_port: {args.pid_port}   load_port: {args.load_port}"
          f"   target: {args.target:g} mm/s   load_duties: {load_duties}")

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
        # Let the PID settle onto the target before the FIRST load step so
        # its own startup transient isn't mistaken for a load-response —
        # friction coupling needs both motors already spinning (see
        # docstring), and this pause is exactly that "both running" precondition.
        time.sleep(2.0)

        for label, duty in schedule:
            print(f"\n  step [{label}] load_duty={duty:+.0f}%")
            dev_send(proto, f"DEV M {args.load_port} DUTY {duty}")
            step_t0 = time.monotonic()
            samples: list[tuple[float, float | None, float | None]] = []
            while time.monotonic() - step_t0 < args.step_time:
                st = dev_send(proto, f"DEV M {args.pid_port} STATE")
                # t_step is captured AFTER dev_send() returns, not before —
                # dev_send() can now block for several seconds riding out a
                # transport retry (077-007), and a pre-call timestamp would
                # stamp a late-arriving sample as if it were taken at the top
                # of the step, silently excluding it from the "settled"
                # window below even though it was really read near/after the
                # step's end. Confirmed live: a step-5 run showed a valid
                # applied=0.22 sample recorded at global t=41.7s (long after
                # the step nominally started) reporting settled=None because
                # the pre-call t_step had frozen at ~0.3s during the retry.
                t_step = time.monotonic() - step_t0
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

        # Overall PASS 2: applied duty tracks the load schedule (monotonic,
        # measured heaviest-to-lightest direction — see _applied_tracks_load()) —
        # proof the PID is doing real work against a changing friction
        # disturbance, not coasting at a fixed duty.
        applied_seq = [r["avg_applied"] for r in step_results]
        tracks_load = _applied_tracks_load(applied_seq)

        print(f"\n  {'PASS' if all_in_band else 'FAIL'}: velocity held within "
              f"±{args.tolerance:g} mm/s across all load steps")
        print(f"  {'PASS' if tracks_load else 'FAIL'}: applied duty tracks load "
              f"({[f'{a:.3f}' if a is not None else 'None' for a in applied_seq]})")
        print(f"\n  CSV: {csv_path}")

        overall_pass = all_in_band and tracks_load

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
