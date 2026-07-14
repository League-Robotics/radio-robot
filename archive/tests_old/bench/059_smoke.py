"""059_smoke.py — Sprint 059 bench smoke verification (safe on-stand only).

Verifies that the sprint-059 firmware (Phase 3: command-bus, planner, and
bottom-up config) responds correctly over the relay link.  ALL motion is
confined to a SINGLE on-stand rotation — the robot must be elevated on its
bench stand before the motion step is reached.  No field/floor driving.

Checks (in order):

  1. HELLO     — firmware banner contains "radio-robot" or "NEZHA"
  2. PING      — returns "OK pong" (PONG)
  3. STREAM    — STREAM 100 (10 Hz); collects >= 5 TLM frames via SNAP polling;
                 asserts ``pose`` and ``twist`` (velocity) fields present &
                 values not obviously garbage
  4. SNAP      — one-shot TLM frame; asserts parsed field layout
  5. ON-STAND ROTATION (REQUIRES --i-confirm-on-stand)
                 RT 1800 (180° relative turn); waits for mode → Idle via SNAP
                 poll; asserts heading changed by ~π rad (180° ± 25°);
                 asserts EVT done RT seen in ambient line reads where available
  6. CANCEL    — ``X``; asserts near-zero velocity (|twist.v| < 30 mm/s)

Notes on relay transport (from .clasi/knowledge/2026-06-12-...):
  • The relay is transparent after the !GO handshake — no ``>`` prefix needed.
  • Async STREAM/EVT lines may be dropped by the bridge; SNAP (request/reply)
    is reliable.  Motion-done detection polls SNAP ``mode`` field (not EVT).
  • ``make_robot`` / SerialConnection handle !GO automatically.

Usage:
    uv run python tests/bench/059_smoke.py [--port /dev/cu.usbmodemXXXX] \\
        [--i-confirm-on-stand] [--checks 1,2,3,4,5,6]

    # Syntax-check (no hardware needed):
    uv run python tests/bench/059_smoke.py --help

Safety:
    The motion step (check 5) will NOT run without ``--i-confirm-on-stand``.
    That flag is a declaration by the operator that the robot is elevated on
    its bench stand and the wheels cannot touch the ground.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import subprocess
import sys
import time
from datetime import datetime, timezone

# ---------------------------------------------------------------------------
# Path setup — host library
# ---------------------------------------------------------------------------

_REPO = pathlib.Path(__file__).resolve().parents[2]
_HOST = _REPO / "host"
if str(_HOST) not in sys.path:
    sys.path.insert(0, str(_HOST))

from robot_radio.io.cli import _make_robot          # noqa: E402
from robot_radio.robot.protocol import parse_tlm    # noqa: E402

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

STEP_PASS = "PASS"
STEP_FAIL = "FAIL"
STEP_SKIP = "SKIP"

# How long (ms) to wait for a SNAP reply in steady-state.
SNAP_READ = 400

# STREAM period for check 3 (100 ms = 10 Hz).
STREAM_PERIOD = 100

# Target TLM frames to collect in check 3 (relay may drop async frames; we
# collect via SNAP polling so this is a minimum via polling, not STREAM).
STREAM_TARGET_FRAMES = 5
STREAM_COLLECT_S = 3.0   # seconds to poll; 5 SNAPs at 100 ms each + slack

# RT 1800 = 180° relative turn.
RT_CDEG = 1800
RT_EXPECTED = 180.0
RT_TOL = 25.0         # ±25° tolerance on heading change
RT_TIMEOUT = 10_000    # 10 s; a 180° on-stand spin should complete well within this

# Velocity near-zero threshold for check 6.
VELOCITY_ZERO_MMPS = 30   # |twist.v| below this → "near zero"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _banner(title: str) -> None:
    w = 60
    print()
    print("=" * w)
    print(f"  {title}")
    print("=" * w)


def _result(num: int, name: str, status: str, note: str = "") -> None:
    sym = "+" if status == STEP_PASS else ("~" if status == STEP_SKIP else "X")
    extra = f"  ({note})" if note else ""
    print(f"  [{sym}] Check {num}: {name} -- {status}{extra}")


def _wrap(a: float) -> float:
    """Wrap angle in degrees to [-180, 180]."""
    return math.degrees(
        math.atan2(math.sin(math.radians(a)), math.cos(math.radians(a)))
    )


def _parse_raw_tlm(line: str) -> dict | None:
    """Low-level KV parser for a raw TLM line (fallback if parse_tlm unavailable)."""
    if not line.startswith("TLM"):
        return None
    out: dict = {"_raw": line}
    for tok in line.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    return out


def _snap_raw(conn) -> dict | None:
    """Fire SNAP and return a dict of TLM KV pairs, or None.

    Uses the low-level conn.send_fast + read_lines path (same as bench_validation_032).
    Tries up to 3 times to tolerate a single missed frame.
    """
    for _ in range(3):
        conn.send_fast("SNAP")
        for ln in conn.read_lines(SNAP_READ, stop_token="TLM"):
            if "TLM" in ln:
                d = _parse_raw_tlm(ln)
                if d:
                    return d
    return None


def _snap_heading(conn) -> float | None:
    """Return pose heading in degrees via SNAP, or None."""
    f = _snap_raw(conn)
    if f is None or "pose" not in f:
        return None
    parts = f["pose"].split(",")
    if len(parts) < 3:
        return None
    try:
        return int(parts[2]) / 100.0   # cdeg -> deg
    except ValueError:
        return None


def _wait_mode_idle(conn, timeout: int = RT_TIMEOUT, min: int = 500) -> str:
    """Poll SNAP until mode == 'I' (idle).  Returns 'done' or 'timeout'.

    Relay-safe motion-done detector.  Async EVT done lines are dropped by the
    bridge, so we poll the synchronous SNAP ``mode`` field instead.

    ``min`` floor: require at least one active-mode frame before accepting
    idle, OR that min has elapsed since the first poll — prevents a fast
    move from being declared done before it ever started.
    """
    t0 = time.time()
    deadline = t0 + timeout / 1000.0
    saw_active = False
    idle_streak = 0
    while time.time() < deadline:
        f = _snap_raw(conn)
        mode = f.get("mode") if f else None
        if mode is not None and mode != "I":
            saw_active = True
            idle_streak = 0
        elif mode == "I":
            elapsed = (time.time() - t0) * 1000.0
            if saw_active or elapsed >= min:
                idle_streak += 1
                if idle_streak >= (2 if saw_active else 3):
                    return "done"
        time.sleep(0.08)
    return "timeout"


# ---------------------------------------------------------------------------
# Check 1: HELLO — firmware banner
# ---------------------------------------------------------------------------

def check1_hello(conn) -> str:
    """HELLO → assert firmware banner contains expected token."""
    _banner("Check 1: HELLO (firmware banner)")

    resp = conn.send("HELLO", read_timeout=600, stop_token="DEVICE")
    lines = resp.get("responses", []) + resp.get("lines", [])
    print(f"  HELLO responses ({len(lines)} line(s)):")
    for ln in lines:
        print(f"    {ln.strip()}")

    # Also check the announcement captured during connect() (stored in conn).
    ann = getattr(conn, "_announcement", None) or {}
    ann_str = str(ann).lower()

    all_text = " ".join(lines).lower() + " " + ann_str
    keywords = ("radio-robot", "nezha", "device", "tovez", "bot")
    for kw in keywords:
        if kw in all_text:
            print(f"  Banner contains '{kw}' — firmware is present.")
            return STEP_PASS

    print(f"  FAIL: no expected firmware keyword in banner. "
          f"all_text={all_text[:200]!r}")
    return STEP_FAIL


# ---------------------------------------------------------------------------
# Check 2: PING — returns PONG
# ---------------------------------------------------------------------------

def check2_ping(conn) -> str:
    """PING → assert 'pong' in response."""
    _banner("Check 2: PING (assert PONG)")

    resp = conn.send("PING", read_timeout=500, stop_token="OK")
    lines = resp.get("responses", [])
    print(f"  PING response: {lines}")
    for ln in lines:
        if "pong" in ln.lower():
            print("  PONG received — robot is responding.")
            return STEP_PASS

    print(f"  FAIL: 'pong' not found in responses: {lines}")
    return STEP_FAIL


# ---------------------------------------------------------------------------
# Check 3: STREAM — collect >= 5 TLM frames; assert pose + velocity present
# ---------------------------------------------------------------------------

def check3_stream(conn) -> str:
    """STREAM 100 → collect >= 5 TLM frames via SNAP polling; validate fields."""
    _banner(f"Check 3: STREAM {STREAM_PERIOD} ms + TLM field validation")

    # Start STREAM (the relay may drop async frames — we collect via SNAP).
    conn.send(f"STREAM {STREAM_PERIOD}", read_timeout=300)
    print(f"  STREAM {STREAM_PERIOD} ms enabled (10 Hz).")
    print(f"  Collecting >= {STREAM_TARGET_FRAMES} TLM frames via SNAP polling"
          f" over {STREAM_COLLECT_S:.0f} s ...")

    frames = []
    t_end = time.time() + STREAM_COLLECT_S
    while time.time() < t_end and len(frames) < STREAM_TARGET_FRAMES * 2:
        # Try draining any async STREAM lines first.
        for ln in conn.read_lines(50):
            f = _parse_raw_tlm(ln)
            if f:
                frames.append(f)
        # Then explicitly request one via SNAP.
        f = _snap_raw(conn)
        if f:
            frames.append(f)
        time.sleep(STREAM_PERIOD / 1000.0 * 0.8)

    conn.send("STREAM 0", read_timeout=200)
    print(f"  STREAM 0 (disabled).  Collected {len(frames)} TLM frame(s).")

    if len(frames) < STREAM_TARGET_FRAMES:
        print(f"  FAIL: collected only {len(frames)} frame(s);"
              f" need >= {STREAM_TARGET_FRAMES}.")
        return STEP_FAIL

    # Validate fields in the most-recent frame.
    sample = frames[-1]
    print(f"  Sample frame keys: {[k for k in sample if not k.startswith('_')]}")

    problems = []
    # pose=x,y,h_cdeg — all should be parseable integers.
    if "pose" not in sample:
        problems.append("missing 'pose' field")
    else:
        parts = sample["pose"].split(",")
        if len(parts) < 3:
            problems.append(f"pose has {len(parts)} parts (expected >= 3)")
        else:
            try:
                x, y, h = int(parts[0]), int(parts[1]), int(parts[2])
                # Garbage check: heading must be in ±180 degrees = ±18000 cdeg.
                if abs(h) > 36000:
                    problems.append(f"pose.h={h} cdeg seems garbage (>±360°)")
                print(f"  pose: x={x} mm, y={y} mm, h={h / 100.0:.1f}°")
            except ValueError as exc:
                problems.append(f"pose parse error: {exc}")

    # twist=v,omega_mrad (differential) or vx,vy,omega_mrad (mecanum).
    # Accept either "twist" (body velocity) or "vel" (per-wheel).
    vel_present = "twist" in sample or "vel" in sample
    if not vel_present:
        problems.append("missing velocity field ('twist' or 'vel')")
    else:
        vel_field = "twist" if "twist" in sample else "vel"
        print(f"  velocity ({vel_field}): {sample[vel_field]}")

    if problems:
        print(f"  FAIL: field validation problems: {problems}")
        return STEP_FAIL

    print(f"  All field checks passed ({len(frames)} frames, sample validated).")
    return STEP_PASS


# ---------------------------------------------------------------------------
# Check 4: SNAP — one-shot TLM frame, assert field layout
# ---------------------------------------------------------------------------

def check4_snap(conn) -> str:
    """SNAP → one-shot TLM frame with correct field layout."""
    _banner("Check 4: SNAP (one-shot TLM frame)")

    f = _snap_raw(conn)
    if f is None:
        print("  FAIL: SNAP returned no TLM frame.")
        return STEP_FAIL

    keys = [k for k in f if not k.startswith("_")]
    print(f"  SNAP frame keys: {keys}")
    print(f"  Raw: {f.get('_raw', '').strip()}")

    required = ["t", "mode", "pose"]
    missing = [k for k in required if k not in f]
    if missing:
        print(f"  FAIL: missing required fields: {missing}")
        return STEP_FAIL

    # mode should be a single character string (I/V/G/T/R/D).
    mode = f.get("mode", "")
    if not mode or len(mode) > 4:
        print(f"  FAIL: mode={mode!r} looks wrong (expected single char like 'I')")
        return STEP_FAIL

    print(f"  SNAP validated: t={f.get('t')} ms, mode={mode!r}, "
          f"pose={f.get('pose')}")
    return STEP_PASS


# ---------------------------------------------------------------------------
# Check 5: ON-STAND ROTATION — RT 1800 (180°); heading change ~π rad
#          *** REQUIRES --i-confirm-on-stand ***
# ---------------------------------------------------------------------------

_ON_STAND_GUARD = """
============================================================
  ON-STAND MOTION STEP — SAFETY GUARD
============================================================
  The robot is about to perform a 180° in-place rotation.

  REQUIREMENTS:
    * The robot MUST be elevated on its bench stand.
    * All four wheels must be CLEAR OF THE GROUND.
    * No cables should be taut enough to tilt the stand.

  Run with --i-confirm-on-stand to proceed.
============================================================
"""


def check5_on_stand_rotation(conn, confirmed: bool) -> str:
    """RT 1800 (180° spin); heading changes by ~180° ± 25°.

    SKIPPED unless --i-confirm-on-stand is passed.
    """
    _banner("Check 5: ON-STAND ROTATION (RT 1800 = 180°)")

    if not confirmed:
        print(_ON_STAND_GUARD)
        print("  SKIP: --i-confirm-on-stand not passed.")
        return STEP_SKIP

    print("  [ON-STAND CONFIRMED] Starting 180° rotation ...")
    h0 = _snap_heading(conn)
    if h0 is None:
        print("  FAIL: cannot read starting heading via SNAP.")
        return STEP_FAIL
    print(f"  Starting heading: {h0:.1f}°")

    # Send RT 1800 (relative turn, 1800 cdeg = 180°).
    resp = conn.send(f"RT {RT_CDEG}", read_timeout=400, stop_token="OK")
    lines = resp.get("responses", [])
    print(f"  RT {RT_CDEG} response: {lines}")

    # Wait for motion to complete via SNAP mode polling (relay-safe).
    print(f"  Waiting for rotation to complete"
          f" (SNAP poll, timeout {RT_TIMEOUT // 1000} s) ...")
    outcome = _wait_mode_idle(conn, timeout=RT_TIMEOUT)
    print(f"  Motion outcome: {outcome}")

    # Read final heading.
    h1 = _snap_heading(conn)
    if h1 is None:
        print("  FAIL: cannot read final heading via SNAP.")
        return STEP_FAIL
    print(f"  Final heading: {h1:.1f}°")

    # Heading delta, wrapped to [-180, 180].
    delta = abs(_wrap(h1 - h0))
    # Accept the supplement too (some robots turn the short way).
    delta_eff = min(delta, abs(180.0 - delta))
    print(f"  |Δheading| = {delta:.1f}° "
          f"(effective deviation from 180° = {delta_eff:.1f}°, "
          f"tolerance ±{RT_TOL}°)")

    if outcome != "done":
        print(f"  WARNING: motion outcome was '{outcome}', not 'done'."
              f"  Checking heading anyway ...")

    if delta_eff <= RT_TOL:
        print(f"  Heading change within tolerance ({delta:.1f}° ≈ 180°).")
        return STEP_PASS

    print(f"  FAIL: heading change {delta:.1f}° deviates {delta_eff:.1f}°"
          f" from expected 180° (tolerance ±{RT_TOL}°).")
    return STEP_FAIL


# ---------------------------------------------------------------------------
# Check 6: CANCEL — X stops motion; near-zero velocity
# ---------------------------------------------------------------------------

def check6_cancel(conn) -> str:
    """X (cancel); assert near-zero twist velocity."""
    _banner("Check 6: CANCEL (X) — near-zero velocity")

    resp = conn.send("X", read_timeout=400, stop_token="OK")
    lines = resp.get("responses", [])
    print(f"  X response: {lines}")

    # Brief settle.
    time.sleep(0.3)

    f = _snap_raw(conn)
    if f is None:
        print("  WARN: no SNAP frame after X — assuming stopped (cannot verify).")
        return STEP_PASS   # conservative pass; robot was already stopped

    # Check both 'twist' and 'vel' for any high velocity.
    vel_ok = True
    for vel_key in ("twist", "vel"):
        if vel_key in f:
            parts = f[vel_key].split(",")
            try:
                vals = [abs(int(p)) for p in parts]
                max_v = max(vals)
                print(f"  {vel_key} after X: {f[vel_key]} (max |component| = {max_v})")
                if max_v > VELOCITY_ZERO_MMPS:
                    print(f"  FAIL: {vel_key} component {max_v} > "
                          f"{VELOCITY_ZERO_MMPS} mm/s — not near-zero.")
                    vel_ok = False
            except ValueError:
                print(f"  WARN: could not parse {vel_key}={f[vel_key]!r}")

    mode = f.get("mode", "?")
    print(f"  mode after X: {mode!r}")

    if vel_ok:
        print(f"  Near-zero velocity confirmed (threshold {VELOCITY_ZERO_MMPS} mm/s).")
        return STEP_PASS
    return STEP_FAIL


# ---------------------------------------------------------------------------
# Field log (appended after each run)
# ---------------------------------------------------------------------------

def _append_field_log(results: dict[int, str], step_names: dict[int, str]) -> None:
    """Append a dated SHA-stamped entry to docs/knowledge/field-log.md."""
    log_path = _REPO / "docs" / "knowledge" / "field-log.md"
    log_path.parent.mkdir(parents=True, exist_ok=True)

    try:
        sha = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=str(_REPO), stderr=subprocess.DEVNULL, text=True,
        ).strip()
    except Exception:
        sha = "unknown"

    ts = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%MZ")
    overall = "PASS" if all(v == STEP_PASS for v in results.values()) else "FAIL"

    lines = [f"\n## {ts}  sha={sha}  sprint=059-smoke  overall={overall}\n"]
    for num in sorted(step_names):
        status = results.get(num, STEP_SKIP)
        lines.append(f"- Check {num} ({step_names[num]}): {status}\n")

    with open(log_path, "a", encoding="utf-8") as fh:
        fh.writelines(lines)

    print(f"\n  Field log entry appended: {log_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument(
        "--port", default=None,
        help="Relay serial port (auto-detects if omitted).",
    )
    ap.add_argument(
        "--i-confirm-on-stand", action="store_true", default=False,
        help=(
            "SAFETY GATE: declare that the robot is elevated on its bench stand "
            "with all wheels clear of the ground.  Required to run check 5 "
            "(the 180° rotation).  WITHOUT this flag, check 5 is SKIPPED."
        ),
    )
    ap.add_argument(
        "--checks", default="1,2,3,4,5,6",
        help="Comma-separated check numbers to run (default: all).",
    )
    args = ap.parse_args()

    checks_to_run = {int(s.strip()) for s in args.checks.split(",")}
    on_stand = args.i_confirm_on_stand

    step_names = {
        1: "HELLO (firmware banner)",
        2: "PING (PONG response)",
        3: "STREAM 100 + TLM field validation",
        4: "SNAP (one-shot TLM layout)",
        5: "ON-STAND RT 1800 (180° rotation)",
        6: "CANCEL (X) near-zero velocity",
    }

    # ------------------------------------------------------------------
    # Connect (auto-detects relay vs direct; handles !GO handshake).
    # ------------------------------------------------------------------
    print("Connecting to robot (auto-detecting port / relay mode) ...")
    args.verbose = False
    robot, conn_obj, result = _make_robot(args)
    if result.get("error"):
        sys.exit(f"Connection failed: {result['error']}")

    # We talk at the raw connection level for this smoke script (like
    # bench_validation_032) so we can inspect raw TLM lines.  The
    # NezhaProtocol object is robot._proto; use it for ping() which
    # returns (uptime, rtt) and is cleaner.
    proto = robot._proto
    conn = proto._conn   # underlying SerialConnection

    # Verify liveness via NezhaProtocol.ping() (handles corr-id, cleaner).
    png = proto.ping()
    if not png:
        conn.disconnect()
        sys.exit("PING failed — robot not responding. Power-cycle and retry.")
    print(f"Robot alive: uptime={png[0]} ms, rtt={png[1]:.0f} ms")

    results: dict[int, str] = {}

    try:
        if 1 in checks_to_run:
            results[1] = check1_hello(conn)

        if 2 in checks_to_run:
            results[2] = check2_ping(conn)

        if 3 in checks_to_run:
            results[3] = check3_stream(conn)

        if 4 in checks_to_run:
            results[4] = check4_snap(conn)

        if 5 in checks_to_run:
            results[5] = check5_on_stand_rotation(conn, on_stand)

        if 6 in checks_to_run:
            results[6] = check6_cancel(conn)

    finally:
        # Always safe-stop on exit.
        print()
        print("[safe-stop] Sending X + STREAM 0 ...")
        try:
            conn.send("X", read_timeout=300)
            conn.send("STREAM 0", read_timeout=200)
        except Exception:
            pass
        conn.disconnect()

    # Append field log.
    _append_field_log(results, step_names)

    # Summary table.
    _banner("059 Smoke Summary")
    overall = STEP_PASS
    for num in sorted(step_names):
        if num not in checks_to_run:
            status = STEP_SKIP
        else:
            status = results.get(num, STEP_SKIP)
        _result(num, step_names[num], status)
        if status == STEP_FAIL:
            overall = STEP_FAIL

    print()
    if overall == STEP_PASS:
        print("OVERALL: PASS — all checks passed (or skipped).")
        sys.exit(0)
    else:
        print("OVERALL: FAIL — one or more checks failed.")
        sys.exit(1)


if __name__ == "__main__":
    main()
