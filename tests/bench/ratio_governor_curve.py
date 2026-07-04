#!/usr/bin/env python3
"""ratio_governor_curve.py — ratio-governor acceptance test (ticket 077-006/077-007).

Two test modes, selected by --disturb-port:

**Primary protocol (077-007, stakeholder-specified) — --disturb-port set
(default 4):** the Drivetrain is bound to a pair that is NOT the friction-
coupled rig itself — default `DEV DT PORTS 2 3` (port 2: an otherwise
unloaded wheel; port 3: the wheel friction-coupled to port 4 — see
pid_hold_speed.py's docstring for the coupling physics). An unequal curve
is commanded (`DEV DT WHEELS <left> <right>`), then port 4 (independent of
the drivetrain — `DEV M 4 DUTY ...`, never `DEV DT`) is stepped through
--load-schedule with a dwell per step. This varies the friction drag on
wheel 3 ONLY — an asymmetric disturbance the paired wheel 2 never feels —
which is exactly the scenario `Drivetrain::governRatio()` exists for: if
wheel 3 sags below its own target under load, the governor should scale
BOTH targets down together so the 2:3 ratio holds, rather than only wheel
3 drifting off while wheel 2 runs its target unaffected.

077-007 found and fixed a real bug this protocol exposed: `DEV M 4 DUTY
...` (port 4, NOT in the bound pair) was unconditionally dropping
drivetrain authority (`DevLoopState::drivetrainActive`), regardless of
which port was targeted — silently killing the governor the instant the
load step ran. Fixed in `source/commands/dev_commands.cpp` (`isBoundPort()`
gates the authority drop to the actually-bound ports only) — see
`docs/protocol-v2.md` §16's Authority section.

PASS (primary protocol): the measured wheel-2:wheel-3 velocity ratio stays
within --tolerance of the commanded ratio across EVERY load step (not just
one settled window) — governed runs (`--sync-gain` 0.5-1.0) should hold
this; ungoverned (`--sync-gain 0`) is the negative control and may drift
when a step bogs wheel 3 down.

**Legacy mode (077-006) — --disturb-port 0:** the original single-curve
test, unchanged: bind the Drivetrain directly to the coupled pair itself
(e.g. `DEV DT PORTS 3 4`) and command an unequal `DEV DT WHEELS <left>
<right>` curve with no separate disturbance step — the coupling between
the SAME two commanded wheels is the only load. Useful for a quick sanity
check; the primary protocol above is the one that isolates an asymmetric
disturbance and is the acceptance-gate default.

Negative control (`--sync-gain 0`) / governed run (`--sync-gain 0.5`-`1.0`):
sends `DEV DT CFG sync_gain=<value>` before the curve is commanded (see
`docs/protocol-v2.md` §16's `DEV DT CFG`, added this ticket to close the
gap that `sync_gain` boots at 0 with no other live setter) and echoes the
firmware-confirmed applied value. Omitting `--sync-gain` sends no CFG and
uses whatever `sync_gain` the firmware currently has configured.

Logs every sample to --csv. Ends with `DEV M <disturb-port> NEUTRAL`,
`DEV DT STOP`, and `DEV STOP` regardless of outcome.

Usage:
    # primary protocol (default): DT on 2/3, disturbance from 4
    uv run python tests/bench/ratio_governor_curve.py --sync-gain 0.8
    uv run python tests/bench/ratio_governor_curve.py --sync-gain 0      # negative control
    uv run python tests/bench/ratio_governor_curve.py --left 200 --right 120 \\
        --load-schedule 40,0,-20,-40

    # legacy single-curve mode on the coupled pair itself
    uv run python tests/bench/ratio_governor_curve.py --disturb-port 0 \\
        --dt-left 3 --dt-right 4 --left 200 --right 80
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

# Default disturbance schedule (--disturb-port duty, percent) — dwelled in
# order, per the stakeholder-specified protocol.
_DEFAULT_LOAD_SCHEDULE = (40.0, 0.0, -20.0, -40.0)


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port (default {DEFAULT_PORT})")
    p.add_argument("--dt-left", type=int, default=2,
                   help="DEV DT PORTS left (default 2 — an unloaded wheel; "
                        "primary protocol's Drivetrain pair)")
    p.add_argument("--dt-right", type=int, default=3,
                   help="DEV DT PORTS right (default 3 — friction-coupled to "
                        "--disturb-port)")
    p.add_argument("--disturb-port", type=int, default=4,
                   help="Motor port driven independently (DEV M, never DEV DT) "
                        "through --load-schedule to disturb --dt-right's "
                        "friction load. 0 disables (legacy single-curve mode "
                        "— bind --dt-left/--dt-right directly to the coupled "
                        "pair instead, e.g. 3 4).")
    p.add_argument("--load-schedule", type=str,
                   default=",".join(str(d) for d in _DEFAULT_LOAD_SCHEDULE),
                   help="Comma-separated --disturb-port duty schedule, percent "
                        "-100..100 (default '40,0,-20,-40')")
    p.add_argument("--left", type=float, default=200.0,
                   help="Commanded --dt-left wheel target, mm/s (default 200)")
    p.add_argument("--right", type=float, default=120.0,
                   help="Commanded --dt-right wheel target, mm/s (default 120)")
    p.add_argument("--run-time", type=float, default=6.0,
                   help="Legacy mode only: seconds to hold the curve and sample (default 6)")
    p.add_argument("--step-time", type=float, default=8.0,
                   help="Primary protocol: seconds to hold each load-schedule "
                        "step (default 8 — wide enough to absorb dev_send()'s "
                        "worst-case retry budget without starving the settled "
                        "window; see 077-007 bench notes)")
    p.add_argument("--settle-time", type=float, default=4.0,
                   help="Seconds at the END of --run-time (legacy) or each "
                        "load step (primary) counted as 'settled' for the "
                        "ratio check (default 4, must be < --run-time/--step-time)")
    p.add_argument("--tolerance", type=float, default=0.25,
                   help="Acceptable |measured_ratio - commanded_ratio| / commanded_ratio, "
                        "fractional (default 0.25 = 25%%)")
    p.add_argument("--sample-period", type=float, default=0.1,
                   help="Seconds between state polls (default 0.1)")
    p.add_argument("--sync-gain", type=float, default=None,
                   help="Sends 'DEV DT CFG sync_gain=<value>' before commanding the "
                        "curve. Pass 0 for the negative control (governor off) or "
                        "e.g. 0.5-1.0 for the governed comparison run. Omit to "
                        "send no CFG and use whatever sync_gain the firmware "
                        "currently has configured.")
    p.add_argument("--csv",
                   default=str(_REPO_ROOT / "tests" / "bench" / "out" / "ratio_governor_curve.csv"),
                   help="CSV output path")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Small helpers (kept local per tests/bench/'s locked, standalone-script layout).
# ---------------------------------------------------------------------------

def dev_send(proto: NezhaProtocol, cmd: str, timeout: int = 500,  # [ms]
            retries: int = 6) -> ParsedResponse | None:
    """Send one DEV command, retrying on a totally silent reply.

    See dev_exercise.py's dev_send() docstring — 077-007's HITL bench pass
    found this bench's direct-USB CDC link occasionally, burstily drops
    replies outright; a multi-sample burst-loss can blank out this script's
    settled-sample window and turn a real PASS into a false FAIL on pure
    transport noise. Safe to retry unconditionally: every command here is a
    pure query (STATE) or an idempotent absolute-value write (WHEELS/PORTS/
    CFG/WD/STOP/DUTY/NEUTRAL).
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


def _ratio_check(measured_left: float | None, measured_right: float | None,
                 commanded_ratio: float, tolerance: float) -> tuple[bool, str]:
    if measured_left is None or measured_right is None or abs(measured_right) < 1e-6:
        return False, "insufficient settled samples (or zero measured_right) to judge"
    measured_ratio = measured_left / measured_right
    rel_err = abs(measured_ratio - commanded_ratio) / commanded_ratio
    held = rel_err <= tolerance
    return held, (f"commanded={commanded_ratio:.3f} measured={measured_ratio:.3f} "
                  f"rel_err={rel_err:.3f} tol={tolerance:.3f}")


def _settled_avg(samples: list[tuple[float, float | None, float | None]],
                 window: float, span: float) -> tuple[float | None, float | None]:
    settled = [s for s in samples if s[0] >= span - window]
    lefts = [l for _, l, _ in settled if l is not None]
    rights = [r for _, _, r in settled if r is not None]
    avg_l = sum(lefts) / len(lefts) if lefts else None
    avg_r = sum(rights) / len(rights) if rights else None
    return avg_l, avg_r


def main() -> int:
    args = _parse_args()
    commanded_ratio = args.left / args.right if args.right else float("inf")
    load_schedule = [float(tok) for tok in args.load_schedule.split(",") if tok.strip()]
    primary_mode = args.disturb_port and args.disturb_port > 0
    print(f"  port: {args.port}   DEV DT PORTS {args.dt_left} {args.dt_right}"
          f"   left={args.left:g} right={args.right:g} mm/s"
          f"   commanded_ratio={commanded_ratio:.3f}"
          f"   sync_gain={'unset (no live command)' if args.sync_gain is None else f'{args.sync_gain:g} (requested)'}")
    if primary_mode:
        print(f"  primary protocol: disturb_port={args.disturb_port}  load_schedule={load_schedule}")
    else:
        print("  legacy single-curve mode (--disturb-port 0)")

    csv_path = pathlib.Path(args.csv)
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    conn = SerialConnection(port=args.port)
    proto: NezhaProtocol | None = None
    csv_file = None
    overall_pass = False
    gain_label = "unset"

    try:
        info = conn.connect()
        if "error" in info:
            print(f"ERROR: connect failed: {info['error']}")
            return 2
        print(f"  connected: mode={info.get('mode')}")
        proto = NezhaProtocol(conn)

        dev_send(proto, f"DEV WD {SESSION_WATCHDOG_WINDOW}")

        # Live sync_gain control (077-007): DEV DT CFG did not exist when this
        # script was first written — now it does, so --sync-gain actually
        # sets the governor instead of merely labeling the run. Sent before
        # DEV DT PORTS/WHEELS since sync_gain lives on the single shared
        # Drivetrain instance, not per bound pair.
        if args.sync_gain is not None:
            cfg_resp = dev_send(proto, f"DEV DT CFG sync_gain={args.sync_gain}")
            print(f"  {cfg_resp.raw if cfg_resp else '(no reply)'}")
            applied = _kv_float(cfg_resp, "sync_gain")
            gain_label = f"{applied:g} (confirmed)" if applied is not None else f"{args.sync_gain:g} (unconfirmed — CFG reply lost)"
        else:
            gain_label = "unset (firmware's currently-configured value, unknown to this run)"

        csv_file = open(csv_path, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow(["t", "sync_gain_label", "commanded_left", "commanded_right",
                          "measured_left", "measured_right", "dt_active", "disturb_duty"])

        bind_resp = dev_send(proto, f"DEV DT PORTS {args.dt_left} {args.dt_right}")
        print(f"  {bind_resp.raw if bind_resp else '(no reply)'}")

        curve_resp = dev_send(proto, f"DEV DT WHEELS {args.left} {args.right}")
        print(f"  {curve_resp.raw if curve_resp else '(no reply)'}")

        # Let the drivetrain settle onto the curve before disturbing it —
        # both wheels need to actually be spinning for the friction coupling
        # to transmit anything (see pid_hold_speed.py's docstring).
        time.sleep(2.0)

        t_start = time.monotonic()

        if primary_mode:
            step_results: list[dict] = []
            for duty in load_schedule:
                print(f"\n  step [disturb_port={args.disturb_port} duty={duty:+.0f}%]")
                dev_send(proto, f"DEV M {args.disturb_port} DUTY {duty}")
                step_t0 = time.monotonic()
                samples: list[tuple[float, float | None, float | None]] = []
                while time.monotonic() - step_t0 < args.step_time:
                    dt_state = dev_send(proto, "DEV DT STATE")
                    left_state = dev_send(proto, f"DEV M {args.dt_left} STATE")
                    right_state = dev_send(proto, f"DEV M {args.dt_right} STATE")
                    # Timestamp captured AFTER the (possibly retried) reads —
                    # see pid_hold_speed.py's identical fix (077-007 Defect 3).
                    t_step = time.monotonic() - step_t0
                    m_left = _kv_float(left_state, "vel")
                    m_right = _kv_float(right_state, "vel")
                    dt_active = dt_state.kv.get("active") if dt_state is not None else None
                    writer.writerow([f"{time.monotonic() - t_start:.3f}", gain_label,
                                     args.left, args.right, m_left, m_right, dt_active, duty])
                    samples.append((t_step, m_left, m_right))
                    time.sleep(args.sample_period)

                avg_l, avg_r = _settled_avg(samples, args.settle_time, args.step_time)
                held, detail = _ratio_check(avg_l, avg_r, commanded_ratio, args.tolerance)
                step_results.append({"duty": duty, "held": held, "detail": detail,
                                     "avg_l": avg_l, "avg_r": avg_r})
                print(f"    settled avg measured left={avg_l} right={avg_r}"
                      f"  {'PASS' if held else 'FAIL'}: {detail}")

            overall_pass = all(r["held"] for r in step_results)
            print(f"\n  {'PASS' if overall_pass else 'FAIL'}: measured ratio held commanded "
                  f"ratio within tolerance across ALL {len(step_results)} load steps")
            for r in step_results:
                print(f"    duty={r['duty']:+.0f}%  {'PASS' if r['held'] else 'FAIL'}  {r['detail']}")
            print(f"\n  CSV: {csv_path}")

        else:
            # Legacy single-curve mode (077-006): unchanged behavior — one
            # continuous run-time window, one settled check at the end.
            samples = []
            while time.monotonic() - t_start < args.run_time:
                dt_state = dev_send(proto, "DEV DT STATE")
                left_state = dev_send(proto, f"DEV M {args.dt_left} STATE")
                right_state = dev_send(proto, f"DEV M {args.dt_right} STATE")
                t = time.monotonic() - t_start
                m_left = _kv_float(left_state, "vel")
                m_right = _kv_float(right_state, "vel")
                dt_active = dt_state.kv.get("active") if dt_state is not None else None
                writer.writerow([f"{t:.3f}", gain_label, args.left, args.right,
                                 m_left, m_right, dt_active, ""])
                samples.append((t, m_left, m_right))
                time.sleep(args.sample_period)

            avg_l, avg_r = _settled_avg(samples, args.settle_time, args.run_time)
            held, detail = _ratio_check(avg_l, avg_r, commanded_ratio, args.tolerance)
            print(f"\n  settled avg measured left={avg_l} right={avg_r} mm/s")
            print(f"  {'PASS' if held else 'FAIL'}: measured ratio holds commanded "
                  f"ratio within tolerance — {detail}")
            print(f"\n  CSV: {csv_path}")
            overall_pass = held

    except KeyboardInterrupt:
        print("\n  interrupted — stopping motors...")
        overall_pass = False
    finally:
        if csv_file is not None:
            csv_file.close()
        if proto is not None:
            if primary_mode:
                try:
                    dev_send(proto, f"DEV M {args.disturb_port} NEUTRAL B")
                except Exception as exc:
                    print(f"  WARN: disturb-port NEUTRAL failed during cleanup: {exc}")
            try:
                dev_send(proto, "DEV DT STOP")
            except Exception as exc:
                print(f"  WARN: DEV DT STOP failed during cleanup: {exc}")
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
