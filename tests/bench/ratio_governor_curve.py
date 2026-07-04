#!/usr/bin/env python3
"""ratio_governor_curve.py — coupled-rig ratio-governor curve test (ticket 077-006).

Bench setup: the coupled rig, two motors mechanically linked (ports 3 and 4
by default — running one loads the other, same rig as pid_hold_speed.py).
Binds the Drivetrain to that pair (`DEV DT PORTS 3 4`), then commands an
unequal-wheel-target "curve" (`DEV DT WHEELS <left> <right>`) so the
mechanical coupling drags the faster-commanded wheel down. The ratio (sync)
governor (`Subsystems::Drivetrain::governRatio()`, `docs/protocol-v2.md`
§16) is supposed to scale BOTH targets down together so the measured
wheel-speed ratio holds the commanded ratio, rather than letting the
under-loaded wheel run away.

PASS: sampled `DEV DT STATE` (commanded, pre-governor targets) + per-motor
`DEV M <n> STATE` (measured velocity) show the measured left/right ratio
within --tolerance of the commanded ratio, once settled.

Negative control (`--sync-gain 0`): the ticket calls for "a governor-off run
for the drift control comparison". As of ticket 077-005 there was no wire
command to set `DrivetrainConfig.sync_gain` live (the DEV DT family was
`PORTS`/`VW`/`WHEELS`/`NEUTRAL`/`STATE`/`STOP` only), and `source/main.cpp`
boots the Drivetrain with `sync_gain` left at its zero default — i.e. **the
governor is OFF by default on this firmware**
(`Drivetrain::governRatio()` returns immediately when `sync_gain <= 0`).
Ticket 077-007's HITL bench pass added the missing live setter,
`DEV DT CFG sync_gain=<value>` (and `trackwidth=<value>`) — see
`docs/protocol-v2.md` §16 and `source/commands/dev_commands.cpp`'s
`handleDevDtCfg`. `--sync-gain`, when given, now sends that command before
the curve is commanded (echoing the firmware's confirmed applied value in
the console banner and CSV header) — pass `0` for the negative control and
a value in the governor's useful range (e.g. `0.5`-`1.0`) for the governed
comparison run. Omitting `--sync-gain` sends no CFG at all and uses
whatever `sync_gain` the firmware currently has configured (boot default
0 = ungoverned, unless a previous command this session changed it) — this
preserves the original "no live control" behavior for a bare run.

Logs every sample to --csv. Ends with `DEV DT STOP` (+ `DEV STOP` for
belt-and-suspenders) regardless of outcome.

Usage:
    uv run python tests/bench/ratio_governor_curve.py
    uv run python tests/bench/ratio_governor_curve.py --left 200 --right 80
    uv run python tests/bench/ratio_governor_curve.py --sync-gain 0     # negative control (governor off)
    uv run python tests/bench/ratio_governor_curve.py --sync-gain 0.8   # governed run
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


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port (default {DEFAULT_PORT})")
    p.add_argument("--dt-left", type=int, default=3, help="DEV DT PORTS left (default 3)")
    p.add_argument("--dt-right", type=int, default=4, help="DEV DT PORTS right (default 4)")
    p.add_argument("--left", type=float, default=200.0,
                   help="Commanded left wheel target, mm/s (default 200)")
    p.add_argument("--right", type=float, default=80.0,
                   help="Commanded right wheel target, mm/s (default 80 — a 2.5:1 curve)")
    p.add_argument("--run-time", type=float, default=6.0,
                   help="Seconds to hold the curve and sample (default 6)")
    p.add_argument("--settle-time", type=float, default=2.0,
                   help="Seconds at the END of --run-time counted as 'settled' "
                        "for the ratio check (default 2, must be < --run-time)")
    p.add_argument("--tolerance", type=float, default=0.25,
                   help="Acceptable |measured_ratio - commanded_ratio| / commanded_ratio, "
                        "fractional (default 0.25 = 25%%)")
    p.add_argument("--sample-period", type=float, default=0.1,
                   help="Seconds between state polls (default 0.1)")
    p.add_argument("--sync-gain", type=float, default=None,
                   help="Sends 'DEV DT CFG sync_gain=<value>' before commanding the "
                        "curve (see module docstring). Pass 0 for the negative "
                        "control (governor off) or e.g. 0.5-1.0 for the governed "
                        "comparison run. Omit to send no CFG and use whatever "
                        "sync_gain the firmware currently has configured.")
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
    CFG/WD/STOP).
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
    commanded_ratio = args.left / args.right if args.right else float("inf")
    print(f"  port: {args.port}   DEV DT PORTS {args.dt_left} {args.dt_right}"
          f"   left={args.left:g} right={args.right:g} mm/s"
          f"   commanded_ratio={commanded_ratio:.3f}"
          f"   sync_gain={'unset (no live command)' if args.sync_gain is None else f'{args.sync_gain:g} (requested)'}")

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
        # script was first written (see module docstring) — now it does, so
        # --sync-gain actually sets the governor instead of merely labeling
        # the run. Sent before DEV DT PORTS/WHEELS since sync_gain lives on
        # the single shared Drivetrain instance, not per bound pair.
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
                          "measured_left", "measured_right", "dt_active"])

        bind_resp = dev_send(proto, f"DEV DT PORTS {args.dt_left} {args.dt_right}")
        print(f"  {bind_resp.raw if bind_resp else '(no reply)'}")

        curve_resp = dev_send(proto, f"DEV DT WHEELS {args.left} {args.right}")
        print(f"  {curve_resp.raw if curve_resp else '(no reply)'}")

        t0 = time.monotonic()
        samples: list[tuple[float, float | None, float | None]] = []
        while time.monotonic() - t0 < args.run_time:
            t = time.monotonic() - t0
            dt_state = dev_send(proto, "DEV DT STATE")
            left_state = dev_send(proto, f"DEV M {args.dt_left} STATE")
            right_state = dev_send(proto, f"DEV M {args.dt_right} STATE")
            m_left = _kv_float(left_state, "vel")
            m_right = _kv_float(right_state, "vel")
            dt_active = dt_state.kv.get("active") if dt_state is not None else None
            writer.writerow([f"{t:.3f}", gain_label, args.left, args.right,
                              m_left, m_right, dt_active])
            samples.append((t, m_left, m_right))
            time.sleep(args.sample_period)

        settled = [s for s in samples if s[0] >= args.run_time - args.settle_time]
        ratios = []
        for _, l, r in settled:
            if l is not None and r is not None and abs(r) > 1e-6:
                ratios.append(l / r)
        avg_ratio = sum(ratios) / len(ratios) if ratios else None
        avg_left = sum(l for _, l, _ in settled if l is not None) / max(
            1, len([l for _, l, _ in settled if l is not None]))
        avg_right = sum(r for _, _, r in settled if r is not None) / max(
            1, len([r for _, _, r in settled if r is not None]))

        if avg_ratio is None or commanded_ratio in (0.0, float("inf")):
            ratio_held = False
            detail = "insufficient settled samples (or zero/inf commanded ratio) to judge"
        else:
            rel_err = abs(avg_ratio - commanded_ratio) / commanded_ratio
            ratio_held = rel_err <= args.tolerance
            detail = (f"commanded={commanded_ratio:.3f} measured={avg_ratio:.3f} "
                      f"rel_err={rel_err:.3f} tol={args.tolerance:.3f}")

        print(f"\n  settled avg measured left={avg_left:.1f} right={avg_right:.1f} mm/s")
        print(f"  {'PASS' if ratio_held else 'FAIL'}: measured ratio holds commanded "
              f"ratio within tolerance — {detail}")
        print(f"\n  CSV: {csv_path}")

        overall_pass = ratio_held

    except KeyboardInterrupt:
        print("\n  interrupted — stopping motors...")
        overall_pass = False
    finally:
        if csv_file is not None:
            csv_file.close()
        if proto is not None:
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
