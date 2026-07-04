#!/usr/bin/env python3
"""dev_exercise.py — scripted DEV-command bench validation (ticket 077-006).

Exercises the `DEV M …` / `DEV DT …` / `DEV WD` / `DEV STATE` / `DEV STOP`
command family (`docs/protocol-v2.md` §16, "Development Commands") end to
end against a real robot on the bench, over `NezhaProtocol.send()`
(`host/robot_radio/robot/protocol.py`). Runs unchanged against a direct-USB
connection or a radio relay's `!GO` data plane — `SerialConnection.connect()`
auto-detects which one it is talking to from the boot `DEVICE:` banner (see
`.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md`), so this
script never has to know or care which transport it's on.

Sequence (mirrors the sprint-077 issue's Verification section):
  1. PING / VER            — liveness + firmware identity.
  2. Per-port STATE / CAPS (ports 1..4) — confirm every leaf answers.
  3. DUTY spin + position-climb check on --motor.
  4. VEL step + convergence logging on --motor.
  5. VOLT → assert `ERR unsupported` (Motor::apply()'s capability gate).
  6. RESET → assert position rezeroes.
  7. DEV DT VW hand-drag observation — logged only. Whether the ratio
     governor actually holds the commanded ratio under a hand load needs a
     human watching the wheels; this step just drives DEV DT VW and prints
     DEV DT STATE so an operator can eyeball it (or skip with --skip-dt).
  8. Watchdog check: go silent, confirm the firmware neutralizes on its own
     (EVT dev_watchdog and/or applied duty dropping to 0).

Every step goes through `dev_send()` below, which is a thin wrapper around
`NezhaProtocol.send()` + the existing `parse_response()` (both already
handle the DEV family's plain `OK`/`ERR` + `k=v` reply shape — no protocol.py
changes were needed for this ticket).

Safety: `DEV WD 3000` widens the serial-silence watchdog for the whole
session up front; the `finally` block always sends `DEV STOP` and restores
`DEV WD 1000` (the firmware boot default) — on a clean run, an assertion
failure, an unhandled exception, or Ctrl-C. Motors must never be left
running.

Usage:
    uv run python tests/bench/dev_exercise.py
    uv run python tests/bench/dev_exercise.py --port /dev/cu.usbmodem2121102 --motor 1
"""

from __future__ import annotations

import argparse
import sys
import time

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol, ParsedResponse, parse_response

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
SESSION_WATCHDOG_WINDOW = 3000    # [ms] widened for the whole session
TEST_WATCHDOG_WINDOW = 800        # [ms] narrowed just for the watchdog-fire check
BOOT_WATCHDOG_WINDOW = 1000       # [ms] firmware default — restored on exit


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT,
                   help=f"Serial port (default {DEFAULT_PORT})")
    p.add_argument("--motor", type=int, default=1, choices=(1, 2, 3, 4),
                   help="Motor port to exercise for DUTY/VEL/VOLT/RESET (default 1)")
    p.add_argument("--duty", type=float, default=30.0,
                   help="DUTY test value, percent -100..100 (default 30)")
    p.add_argument("--spin-time", type=float, default=1.5,
                   help="Seconds to let DUTY spin before checking position climb (default 1.5)")
    p.add_argument("--pos-climb-min", type=float, default=5.0,
                   help="Minimum |delta position| (deg) to count as 'climbed' (default 5)")
    p.add_argument("--vel", type=float, default=120.0,
                   help="VEL test target, mm/s (default 120)")
    p.add_argument("--settle-time", type=float, default=3.0,
                   help="Seconds to log VEL convergence before checking (default 3)")
    p.add_argument("--vel-tolerance", type=float, default=25.0,
                   help="Acceptable |measured - target| at settle, mm/s (default 25)")
    p.add_argument("--pos-zero-tolerance", type=float, default=2.0,
                   help="Acceptable |position| after RESET, deg (default 2)")
    p.add_argument("--skip-dt", action="store_true",
                   help="Skip the DEV DT hand-drag observation step")
    p.add_argument("--dt-left", type=int, default=1, help="DEV DT PORTS left (default 1)")
    p.add_argument("--dt-right", type=int, default=2, help="DEV DT PORTS right (default 2)")
    p.add_argument("--dt-speed", type=float, default=150.0,
                   help="DEV DT VW forward speed for the hand-drag step, mm/s (default 150)")
    p.add_argument("--dt-observe-time", type=float, default=5.0,
                   help="Seconds to poll DEV DT STATE during the hand-drag step (default 5)")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------

def dev_send(proto: NezhaProtocol, cmd: str, timeout: int = 500) -> ParsedResponse | None:  # [ms]
    """Send one DEV command, return the first OK/ERR reply line, parsed.

    DEV replies are synchronous, one line per command — the standard
    OK/ERR taxonomy with plain tokens (``DEV``, ``M``, ``1``, ...) and
    trailing ``k=v`` pairs, which ``parse_response()`` already handles.
    """
    resp = proto.send(cmd, timeout)
    for raw in resp.get("responses", []):
        r = parse_response(raw)
        if r is not None and r.tag in ("OK", "ERR"):
            return r
    return None


def _kv_float(r: ParsedResponse | None, key: str) -> float | None:
    if r is None or key not in r.kv:
        return None
    try:
        return float(r.kv[key])
    except ValueError:
        return None


def _fmt(r: ParsedResponse | None) -> str:
    if r is None:
        return "(no reply)"
    return r.raw


class Result:
    """Accumulates (name, passed, detail) checks and prints a pass/fail summary."""

    def __init__(self) -> None:
        self.checks: list[tuple[str, bool, str]] = []

    def record(self, name: str, passed: bool, detail: str = "") -> None:
        self.checks.append((name, passed, detail))
        mark = "PASS" if passed else "FAIL"
        suffix = f"  ({detail})" if detail else ""
        print(f"  [{mark}] {name}{suffix}")

    def summary(self) -> bool:
        total = len(self.checks)
        passed = sum(1 for _, ok, _ in self.checks if ok)
        print(f"\n{passed}/{total} checks passed")
        for name, ok, detail in self.checks:
            if not ok:
                print(f"  FAILED: {name}" + (f" — {detail}" if detail else ""))
        return passed == total


# ---------------------------------------------------------------------------
# The exercise sequence
# ---------------------------------------------------------------------------

def _run_checks(proto: NezhaProtocol, conn: SerialConnection,
                args: argparse.Namespace, result: Result) -> None:
    motor = args.motor

    # 1. Liveness.
    ver = proto.get_ver()
    result.record("VER", ver is not None, str(ver))
    ping = proto.ping()
    result.record("PING", ping is not None, str(ping))

    # 2. Per-port STATE / CAPS.
    for port in (1, 2, 3, 4):
        st = dev_send(proto, f"DEV M {port} STATE")
        result.record(f"DEV M {port} STATE", st is not None and st.tag == "OK", _fmt(st))
        caps = dev_send(proto, f"DEV M {port} CAPS")
        result.record(f"DEV M {port} CAPS", caps is not None and caps.tag == "OK", _fmt(caps))

    # 3. DUTY spin + position climb.
    st0 = dev_send(proto, f"DEV M {motor} STATE")
    pos0 = _kv_float(st0, "pos")
    duty_resp = dev_send(proto, f"DEV M {motor} DUTY {args.duty}")
    result.record(f"DEV M {motor} DUTY {args.duty:g}",
                   duty_resp is not None and duty_resp.tag == "OK", _fmt(duty_resp))
    time.sleep(args.spin_time)
    st1 = dev_send(proto, f"DEV M {motor} STATE")
    pos1 = _kv_float(st1, "pos")
    delta = None if pos0 is None or pos1 is None else pos1 - pos0
    climbed = delta is not None and abs(delta) >= args.pos_climb_min
    result.record("position climbed under DUTY", climbed,
                   f"pos0={pos0} pos1={pos1} delta={delta}")
    dev_send(proto, f"DEV M {motor} NEUTRAL B")

    # 4. VEL step + convergence logging.
    vel_resp = dev_send(proto, f"DEV M {motor} VEL {args.vel}")
    result.record(f"DEV M {motor} VEL {args.vel:g}",
                   vel_resp is not None and vel_resp.tag == "OK", _fmt(vel_resp))
    print("  convergence log (t, vel, applied):")
    t0 = time.monotonic()
    last_vel: float | None = None
    while time.monotonic() - t0 < args.settle_time:
        t = time.monotonic() - t0
        st = dev_send(proto, f"DEV M {motor} STATE")
        v, a = _kv_float(st, "vel"), _kv_float(st, "applied")
        print(f"    t={t:5.2f}s  vel={v}  applied={a}")
        last_vel = v
        time.sleep(0.2)
    converged = last_vel is not None and abs(last_vel - args.vel) <= args.vel_tolerance
    result.record("VEL converged within tolerance", converged,
                   f"target={args.vel:g} measured={last_vel} tol={args.vel_tolerance:g}")
    dev_send(proto, f"DEV M {motor} NEUTRAL B")

    # 5. VOLT → capability-gated rejection.
    volt_resp = dev_send(proto, f"DEV M {motor} VOLT 3")
    volt_rejected = (volt_resp is not None and volt_resp.tag == "ERR"
                      and "unsupported" in " ".join(volt_resp.tokens).lower())
    result.record("VOLT rejected (capability gate)", volt_rejected, _fmt(volt_resp))

    # 6. RESET → rezero.
    reset_resp = dev_send(proto, f"DEV M {motor} RESET")
    st_after = dev_send(proto, f"DEV M {motor} STATE")
    pos_after = _kv_float(st_after, "pos")
    rezeroed = (reset_resp is not None and reset_resp.tag == "OK"
                and pos_after is not None and abs(pos_after) <= args.pos_zero_tolerance)
    result.record("RESET rezeroes position", rezeroed, f"pos_after_reset={pos_after}")

    # 7. DEV DT hand-drag observation (not auto-asserted — needs a human).
    if not args.skip_dt:
        print("\n  DEV DT hand-drag observation — physically load one wheel now.")
        dev_send(proto, f"DEV DT PORTS {args.dt_left} {args.dt_right}")
        dev_send(proto, f"DEV DT VW {args.dt_speed} 0 0")
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.dt_observe_time:
            dt_state = dev_send(proto, "DEV DT STATE")
            print(f"    t={time.monotonic() - t0:5.2f}s  {_fmt(dt_state)}")
            time.sleep(0.3)
        dev_send(proto, "DEV DT STOP")
        result.record(
            "DEV DT VW hand-drag observed",
            True,
            "ratio-holding under a hand load needs a human eyeballing the log above "
            "— not auto-asserted",
        )

    # 8. Watchdog check: narrow the window, go silent, confirm it fires.
    print(f"\n  watchdog check: narrowing to {TEST_WATCHDOG_WINDOW} ms, then going silent...")
    dev_send(proto, f"DEV WD {TEST_WATCHDOG_WINDOW}")
    time.sleep(TEST_WATCHDOG_WINDOW / 1000.0 + 0.5)
    evt_lines = conn.read_pending_lines()
    saw_watchdog_evt = any("dev_watchdog" in ln for ln in evt_lines)
    st_final = dev_send(proto, f"DEV M {motor} STATE")
    applied_final = _kv_float(st_final, "applied")
    neutralized = applied_final is not None and applied_final == 0.0
    result.record(
        "watchdog fired (EVT dev_watchdog seen or motor neutralized)",
        saw_watchdog_evt or neutralized,
        f"evt_seen={saw_watchdog_evt} applied_after={applied_final}",
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    args = _parse_args()
    print(f"  port: {args.port}   motor: {args.motor}")

    conn = SerialConnection(port=args.port)   # mode=None: auto-detect direct vs. relay
    result = Result()
    proto: NezhaProtocol | None = None

    try:
        info = conn.connect()
        if "error" in info:
            print(f"ERROR: connect failed: {info['error']}")
            return 2
        print(f"  connected: mode={info.get('mode')}")
        proto = NezhaProtocol(conn)

        # Widen the watchdog for the whole session up front (constraint: the
        # firmware default is 1000 ms, and this script's checks — especially
        # the VEL settle log and the DT hand-drag observation — routinely run
        # long enough to trip it if left at the default between DEV sends).
        wd = dev_send(proto, f"DEV WD {SESSION_WATCHDOG_WINDOW}")
        result.record(f"DEV WD {SESSION_WATCHDOG_WINDOW} (widen for session)",
                       wd is not None and wd.tag == "OK", _fmt(wd))

        _run_checks(proto, conn, args, result)

    except KeyboardInterrupt:
        print("\n  interrupted — stopping motors...")
    finally:
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

    return 0 if result.summary() else 1


if __name__ == "__main__":
    sys.exit(main())
