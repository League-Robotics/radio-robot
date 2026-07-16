#!/usr/bin/env python3
"""motion_command_verify.py — on-stand command-surface bench verification (ticket 088-009).

Robot is mounted on a stand with the wheels off the ground (see
`.claude/rules/hardware-bench-testing.md`), so it is safe to spin the wheels
freely. Verifies, over the real serial link (auto-detects direct-USB vs. the
radio relay's `!GO` plane from the boot `DEVICE:` banner, via
`SerialConnection`):

  1. Device announcement — `HELLO` re-emits `DEVICE:NEZHA2:robot:<name>:<serial>`
     (ticket 088-005), and `connect()` itself banner-classifies the transport.
  2. Liveness / identity — PING, VER, ID, HELP (the full registered verb list,
     ticket 088-003).
  3. Config — GET a key, SET it (`SET key=value`), poll GET until the two-plane
     Configurator applies it (ticket 088: config propagates within ~1 tick).
  4. Motion verbs FUNCTION via encoders — for D / T / S / RT, command the verb
     and read per-motor encoder deltas (`DEV M <n> STATE pos=`): the two drive
     ports both move for a straight command, and move OPPOSITE for a spin.

Note on `fwd_sign` (ticket 088-002): the encoder path scales by `fwd_sign`, so
encoders read positive for a "forward" command with EITHER polarity — the
wheel-direction fix (port 2 = -1) makes both wheels physically drive forward,
which encoders alone cannot distinguish. This tool proves the verbs FUNCTION and
that straight/spin encoder *relationships* are correct; a human eyeball on the
wheels confirms the absolute forward direction.

Streaming `S` may show a net-negative encoder delta because the velocity loop
briefly reverse-spins at an abrupt STOP (the separate terminal-overshoot issue,
`clasi/issues/rt-open-loop-overshoot-*`); the verb still drives the wheels, which
is what this check asserts (movement, sign-agnostic for straight verbs).

Safety: widens the serial-silence watchdog for the motor runs and always sends
STOP + DEV STOP + restores `DEV WD 1000` in a finally. Motors are never left
running.

Usage:
    uv run python src/tests/bench/motion_command_verify.py
    uv run python src/tests/bench/motion_command_verify.py --port /dev/cu.usbmodem2121102
"""
from __future__ import annotations

import argparse
import sys
import time

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol, parse_response

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
RUN_WATCHDOG_WINDOW = 5000     # [ms] widened for the motor runs
BOOT_WATCHDOG_WINDOW = 1000    # [ms] firmware default — restored on exit


def _args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT)
    return p.parse_args()


class Result:
    def __init__(self) -> None:
        self.checks: list[tuple[str, bool, str]] = []

    def record(self, name: str, ok: bool, detail: str = "") -> None:
        self.checks.append((name, ok, detail))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))

    def ok(self) -> bool:
        passed = sum(1 for _, k, _ in self.checks if k)
        print(f"\n==== {passed}/{len(self.checks)} checks passed ====")
        for name, k, d in self.checks:
            if not k:
                print(f"  FAILED: {name}" + (f" — {d}" if d else ""))
        return passed == len(self.checks)


def raw_lines(proto: NezhaProtocol, cmd: str, timeout: int = 600, retries: int = 6) -> list[str]:
    """Return the raw reply lines, retrying on total silence (direct-USB CDC
    intermittently drops replies — a documented, non-firmware bench artifact)."""
    for attempt in range(retries):
        resp = proto.send(cmd, timeout)
        lines = resp.get("responses", [])
        if lines:
            return lines
        if attempt < retries - 1:
            time.sleep(0.1)
    return []


def reply(proto: NezhaProtocol, cmd: str, **kw):
    """First parsed OK/ERR/ID/CFG reply (banner DEVICE lines excluded — use raw_lines)."""
    for raw in raw_lines(proto, cmd, **kw):
        r = parse_response(raw)
        if r is not None and r.tag in ("OK", "ERR", "ID", "CFG"):
            return r
    return None


def pos_of(proto: NezhaProtocol, motor: int) -> float | None:
    r = reply(proto, f"DEV M {motor} STATE")
    if r is None or "pos" not in r.kv:
        return None
    try:
        return float(r.kv["pos"])
    except ValueError:
        return None


def run_motion(proto: NezhaProtocol, result: Result, name: str, line: str,
               settle: float = 2.0, straight: bool = True) -> None:
    reply(proto, "STOP")
    time.sleep(1.2)            # let any prior reverse-spin/coast fully settle
    reply(proto, "ZERO enc")
    time.sleep(0.3)
    a1, a2 = pos_of(proto, 1), pos_of(proto, 2)
    r = reply(proto, line)
    t0 = time.monotonic()
    while time.monotonic() - t0 < settle:
        pos_of(proto, 1)   # polling feeds the serial-silence watchdog
        pos_of(proto, 2)
    reply(proto, "STOP")
    time.sleep(1.2)            # settle the terminal reverse-spin before the final read
    b1, b2 = pos_of(proto, 1), pos_of(proto, 2)
    if None in (a1, a2, b1, b2):
        result.record(name, False, f"encoder read failed; reply={r.raw if r else None!r}")
        return
    d1, d2 = b1 - a1, b2 - a2
    ok_reply = r is not None and r.tag == "OK"
    moved = abs(d1) > 2 and abs(d2) > 2
    detail = f"reply={r.raw if r else None!r} d_port1={d1:.1f} d_port2={d2:.1f}"
    if straight:
        result.record(name, ok_reply and moved, detail + " (both move)")
    else:
        opp = (d1 > 2 and d2 < -2) or (d1 < -2 and d2 > 2)
        result.record(name, ok_reply and moved and opp, detail + " (opposite signs)")


def run(proto: NezhaProtocol, conn: SerialConnection, result: Result) -> None:
    # 1. Announcement — the authoritative check is main()'s connect() classifying
    # mode=direct: SerialConnection sends HELLO, receives the DEVICE:NEZHA2:robot:
    # banner (ticket 088-005), and classifies the robot as a direct device off
    # field 1. DEVICE: lines are consumed out-of-band by the banner path, so they
    # do NOT appear in a command-response list; we still fire HELLO here to exercise
    # the re-announce handler, and surface any banner that reaches the pending stream.
    proto.send("HELLO")
    time.sleep(0.3)
    banner = next((ln for ln in conn.read_pending_lines()
                   if ln.startswith("DEVICE:NEZHA2:robot:")), None)
    if banner is not None:
        result.record("HELLO re-announce banner (pending stream)", True, banner)

    # 2. Liveness / identity.
    result.record("PING", proto.ping() is not None)
    ver = proto.get_ver()
    result.record("VER", ver is not None and "fw" in ver, str(ver))
    idr = reply(proto, "ID")
    result.record("ID", idr is not None and idr.tag == "ID", idr.raw if idr else "(none)")
    helpr = proto.get_help()
    has = helpr is not None and all(v in helpr.split()
                                    for v in ("S", "T", "D", "STOP", "SET", "GET", "HELLO"))
    result.record("HELP full verb list", bool(has), helpr or "(none)")

    # 3. Config GET / SET(key=value) / GET-until-applied.
    g0 = reply(proto, "GET tw")
    if g0 is not None and "tw" in g0.kv:
        cur = float(g0.kv["tw"])
        newv = 111.0 if cur != 111.0 else 122.0
        s = reply(proto, f"SET tw={newv:g}")
        applied = False
        for _ in range(8):        # poll until the Configurator applies it (~1 tick)
            time.sleep(0.25)
            g1 = reply(proto, "GET tw")
            if g1 is not None and "tw" in g1.kv and abs(float(g1.kv["tw"]) - newv) < 0.5:
                applied = True
                break
        result.record("SET/GET (tw key=value, applied)",
                      s is not None and s.tag == "OK" and applied,
                      f"set={s.raw if s else None!r} applied={applied}")
        reply(proto, f"SET tw={cur:g}")   # restore
    else:
        result.record("SET/GET (tw key=value, applied)", False,
                      f"GET tw -> {g0.raw if g0 else None!r}")

    # 4. Motion verbs via encoders.
    reply(proto, f"DEV WD {RUN_WATCHDOG_WINDOW}")
    run_motion(proto, result, "D  distance-drive drives both wheels", "D 150 150 120")
    run_motion(proto, result, "T  timed-drive drives both wheels", "T 150 150 900")
    run_motion(proto, result, "S  streaming-drive drives both wheels", "S 150 150")
    run_motion(proto, result, "RT relative-turn spins wheels opposite", "RT 45", straight=False)


def main() -> int:
    args = _args()
    conn = SerialConnection(port=args.port)   # mode=None -> auto-detect direct vs relay
    result = Result()
    proto: NezhaProtocol | None = None
    try:
        info = conn.connect()
        if "error" in info:
            print(f"connect failed: {info['error']}")
            return 2
        # connect() sends HELLO, reads the DEVICE:NEZHA2:robot: banner (088-005),
        # and classifies the robot as "direct" off field 1 — the end-to-end
        # announcement proof through production host code.
        result.record("device announcement (connect banner-classify -> direct)",
                      info.get("mode") == "direct", f"mode={info.get('mode')}")
        proto = NezhaProtocol(conn)
        run(proto, conn, result)
    except KeyboardInterrupt:
        print("\n interrupted — stopping motors...")
    finally:
        if proto is not None:
            for c in ("STOP", "DEV STOP", f"DEV WD {BOOT_WATCHDOG_WINDOW}"):
                try:
                    reply(proto, c)
                except Exception as exc:  # noqa: BLE001
                    print(f"  WARN cleanup {c!r}: {exc}")
            print("  [safety] STOP + DEV STOP + DEV WD 1000 restored.")
        if conn.is_open:
            conn.disconnect()
    return 0 if result.ok() else 1


if __name__ == "__main__":
    sys.exit(main())
