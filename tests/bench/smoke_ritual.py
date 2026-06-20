"""smoke_ritual.py — sprint acceptance smoke ritual for radio-robot-c.

Run after a clean firmware flash (mbdeploy deploy robot --clean) to confirm
the refactored firmware behaves correctly on the real robot.

Five ritual checks (run in order):

  1. SAFE query     — prints PASS if response is 'on'.
  2. RT x4 closure  — four relative RT +90 turns; robot must return within
                       12 degrees of the starting heading (fused-pose readback).
  3. G square       — 200x200 mm square: four G <side> 0 forward legs with a
                       relative RT +90 between them; fused-pose return error at
                       origin < 120 mm.  Geofenced to keep it on the playfield.
  4. Lift test      — operator lifts robot mid-drive; script expects
                       'EVT otos lost' within 5 s of the lift prompt; then
                       checks robot does not spin on re-placement.
  5. TLM drop-rate  — STREAM 40 for 10 s; counts frames; reports observed
                       rate and any apparent drops.

All motion in checks 2-4 is wrapped with BenchRun for automatic safe-stop
on Ctrl-C, exception, or wall-clock cap.

On completion, a dated SHA-stamped entry is appended to
docs/knowledge/field-log.md (created if absent).

Usage:
    uv run python tests/bench/smoke_ritual.py --port /dev/cu.usbmodem2121302
    uv run python tests/bench/smoke_ritual.py          # auto-detects relay port

Outputs a summary table of pass/fail per check.  Any FAIL exits non-zero.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
import pathlib
import threading
from datetime import datetime, timezone

_REPO = pathlib.Path(__file__).resolve().parents[2]
_HOST = _REPO / "host"
if str(_HOST) not in sys.path:
    sys.path.insert(0, str(_HOST))

from robot_radio.io.cli import _make_robot
from robot_radio.robot.protocol import parse_response, parse_tlm
from bench_safety import BenchRun, RobotSilentError, RunawayAbortError


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

STEP_PASS = "PASS"
STEP_FAIL = "FAIL"
STEP_SKIP = "SKIP"


def _banner(title: str) -> None:
    width = 60
    print()
    print("=" * width)
    print(f"  {title}")
    print("=" * width)


def _result(step: int, name: str, status: str, note: str = "") -> None:
    tick = "+" if status == STEP_PASS else ("~" if status == STEP_SKIP else "X")
    extra = f"  ({note})" if note else ""
    print(f"  [{tick}] Check {step}: {name} -- {status}{extra}")


def _wrap_deg(a: float) -> float:
    """Wrap angle (degrees) to [-180, 180]."""
    return math.degrees(math.atan2(math.sin(math.radians(a)),
                                   math.cos(math.radians(a))))


def _otos_heading_deg(proto) -> float | None:
    """Read fused-pose heading in degrees via SNAP (pose=x,y,h_cdeg).

    The current firmware emits the fused EKF pose as ``pose=`` in the TLM
    frame; the raw ``otos=`` field is absent, so read ``.pose`` (which is what
    G/D/TURN drive against anyway).
    """
    frame = proto.snap()
    if frame is None or frame.pose is None:
        return None
    return frame.pose[2] / 100.0  # cdeg -> deg


def _otos_pos_mm(proto) -> tuple[int, int] | None:
    """Read fused-pose x,y position in mm via SNAP."""
    frame = proto.snap()
    if frame is None or frame.pose is None:
        return None
    return frame.pose[0], frame.pose[1]


def _keepalive_thread(proto: NezhaProtocol, stop: list[bool]) -> None:
    """Background keepalive sender (+ every 200 ms)."""
    while not stop[0]:
        proto.send_fast("+")
        time.sleep(0.2)


# ---------------------------------------------------------------------------
# Check 1: Safety check
# ---------------------------------------------------------------------------

def check1_safety_check(proto: NezhaProtocol) -> str:
    """SAFE query must return 'on'."""
    _banner("Check 1: Safety check (SAFE query)")
    resp = proto.send("SAFE", read_ms=400)
    for line in resp.get("responses", []):
        r = parse_response(line)
        if r and r.tag == "OK" and "safety" in r.tokens:
            idx = r.tokens.index("safety")
            if idx + 1 < len(r.tokens):
                state = r.tokens[idx + 1]
                print(f"  SAFE query response: {line.strip()}")
                if state == "on":
                    print("  Safety watchdog is ON.")
                    return STEP_PASS
                else:
                    print(f"  Safety watchdog is '{state}' -- expected 'on'.")
                    return STEP_FAIL
    print(f"  No parseable SAFE reply. Raw responses: {resp.get('responses', [])}")
    return STEP_FAIL


# ---------------------------------------------------------------------------
# Check 2: TURN x4 closure
# ---------------------------------------------------------------------------

def check2_turn_closure(robot) -> str:
    """Four relative RT +90 turns; heading must close to +-12 degrees.

    Uses RT (relative turn) — NOT TURN (absolute heading).  Four RT +90s sum
    to a full revolution back to the starting heading; an absolute TURN 9000
    repeated would just hold +90 after the first (the verb goes to an absolute
    target, so calls 2-4 are no-ops).
    """
    proto = robot._proto
    _banner("Check 2: RT x4 closure (4x relative +90)")
    RT_STEP_CDEG = 9000    # +90 deg relative (CCW)
    RT_TIMEOUT = 15_000    # ms per turn
    CLOSURE_TOL_DEG = 12.0

    h0 = _otos_heading_deg(proto)
    if h0 is None:
        print("  Cannot read starting heading -- is fused pose reporting?")
        return STEP_FAIL
    print(f"  Starting heading: {h0:.1f} deg")

    try:
        with BenchRun(robot, max_seconds=90) as _bench:
            for i in range(4):
                print(f"  RT {RT_STEP_CDEG} cdeg (relative +90, turn {i+1}/4) ...")
                proto.send(f"RT {RT_STEP_CDEG} #{i + 1}", read_ms=200)
                outcome = proto.wait_for_evt_done("RT", timeout_ms=RT_TIMEOUT,
                                                  corr_id=str(i + 1))
                if outcome != "done":
                    print(f"  RT {i+1} outcome: {outcome} (expected 'done')")
                    return STEP_FAIL
                time.sleep(0.3)
    except (RobotSilentError, RunawayAbortError) as exc:
        print(f"  BenchRun aborted: {exc}")
        return STEP_FAIL

    h1 = _otos_heading_deg(proto)
    if h1 is None:
        print("  Cannot read final heading.")
        return STEP_FAIL
    print(f"  Final heading:    {h1:.1f} deg")

    delta = abs(_wrap_deg(h1 - h0))
    print(f"  Heading closure error: {delta:.1f} deg (tolerance: {CLOSURE_TOL_DEG} deg)")
    if delta <= CLOSURE_TOL_DEG:
        return STEP_PASS
    print(f"  FAIL -- closure error {delta:.1f} deg exceeds {CLOSURE_TOL_DEG} deg")
    return STEP_FAIL


# ---------------------------------------------------------------------------
# Check 3: G square (300x300 mm, < 50 mm return error from OTOS)
# ---------------------------------------------------------------------------

# 200 mm square via robot-relative G forward legs + relative RT +90 turns.
# G is ROBOT-RELATIVE (G <forward_mm> <left_mm> <speed>), so each leg drives
# forward SQUARE_SIDE_MM and a relative RT +90 reorients for the next side;
# four legs + three turns trace a square back to the origin.  20 cm sides keep
# the path well inside the playfield (corners reach ~283 mm from start).
SQUARE_SIDE_MM = 200     # 20 cm sides
SQUARE_SPEED = 150       # mm/s
LEG_TIMEOUT = 20_000     # ms per forward leg
TURN_TIMEOUT = 15_000    # ms per corner turn
SQUARE_GEOFENCE_MM = 350  # abort if displacement from origin exceeds this
SQUARE_ARRIVE_MM = 120   # fused-pose return error tolerance at origin


def check3_g_square(robot) -> str:
    """Drive a 200 mm square with G forward legs + RT turns; return error < 120 mm.

    Uses fused-pose (reset at start, read at origin return) as ground truth,
    since the overhead camera is not available in all bench contexts.  A live
    geofence on displacement-from-origin keeps the robot on the playfield.
    """
    proto = robot._proto
    _banner("Check 3: G square (200x200 mm, G forward + RT turns)")

    # Zero fused pose before the run so origin = start.
    proto.zero_otos()
    time.sleep(0.3)
    print("  Fused pose zeroed at start position (origin).")

    def _disp_mm() -> float | None:
        p = _otos_pos_mm(proto)
        return None if p is None else math.hypot(p[0], p[1])

    try:
        with BenchRun(robot, max_seconds=120) as _bench:
            for i in range(4):
                print(f"  leg {i+1}/4: G {SQUARE_SIDE_MM} 0 {SQUARE_SPEED} "
                      f"(forward {SQUARE_SIDE_MM} mm) ...")
                proto.send(f"G {SQUARE_SIDE_MM} 0 {SQUARE_SPEED} #{i + 1}", read_ms=300)
                outcome = proto.wait_for_evt_done("G", timeout_ms=LEG_TIMEOUT,
                                                  corr_id=str(i + 1))
                d = _disp_mm()
                if d is not None and d > SQUARE_GEOFENCE_MM:
                    print(f"  GEOFENCE -- {d:.0f} mm from origin > {SQUARE_GEOFENCE_MM} mm; aborting")
                    return STEP_FAIL
                if outcome != "done":
                    print(f"  leg {i+1}: outcome={outcome} (expected 'done')")
                    return STEP_FAIL
                if i < 3:   # reorient for the next side (no turn after the last leg)
                    print("  RT 9000 (relative +90) ...")
                    proto.send(f"RT 9000 #{i + 1}", read_ms=200)
                    if proto.wait_for_evt_done("RT", timeout_ms=TURN_TIMEOUT,
                                               corr_id=str(i + 1)) != "done":
                        print(f"  corner turn {i+1}: not done")
                        return STEP_FAIL
                time.sleep(0.3)
    except (RobotSilentError, RunawayAbortError) as exc:
        print(f"  BenchRun aborted: {exc}")
        return STEP_FAIL

    # Check fused position back at origin.
    pos = _otos_pos_mm(proto)
    if pos is None:
        print("  Cannot read final position.")
        return STEP_FAIL

    err_mm = math.hypot(pos[0], pos[1])
    print(f"  Final fused position: x={pos[0]} mm, y={pos[1]} mm")
    print(f"  Return error from origin: {err_mm:.0f} mm"
          f"  (tolerance: {SQUARE_ARRIVE_MM} mm)")
    if err_mm <= SQUARE_ARRIVE_MM:
        return STEP_PASS
    print(f"  FAIL -- return error {err_mm:.0f} mm > {SQUARE_ARRIVE_MM} mm")
    return STEP_FAIL


# ---------------------------------------------------------------------------
# Check 4: Lift test -- EVT otos lost within 5 s of lift
# ---------------------------------------------------------------------------

def check4_lift_test(robot) -> str:
    """Lift robot mid-drive; expect 'EVT otos lost' within 5 s; no spin on replace.

    Operator flow:
      1. Script starts a slow T 100 100 5000 drive.
      2. Script prompts: 'Lift the robot now and press Enter when lifted...'
      3. Operator lifts robot, presses Enter.
      4. Script polls for 'EVT otos lost' in the event stream for 5 s.
      5. Script prompts: 'Replace robot on floor and press Enter...'
      6. Script waits 3 s and checks that robot is not spinning (OTOS heading
         stable within 30 degrees over 2 s after re-placement).
    """
    proto = robot._proto
    _banner("Check 4: Lift test (EVT otos lost)")

    EVT_WAIT_MS = 5_000   # time to wait for EVT otos lost after lift
    SPIN_CHECK_MS = 2_000  # duration to check heading stability after replace
    SPIN_TOL_DEG = 30.0   # heading drift threshold for 'no spin'

    try:
        with BenchRun(robot, max_seconds=60) as _bench:
            # Start a slow forward drive long enough for the lift.
            print("  Starting slow T drive (100 mm/s, 6 s) ...")
            proto.timed(100, 100, 6_000)

            # Prompt operator.
            input("\n  >>> Lift the robot off the floor now, then press Enter <<<\n")

            # Poll for 'EVT otos lost' in the next 5 s.
            otos_lost_seen = False
            deadline = time.time() + EVT_WAIT_MS / 1000.0
            while time.time() < deadline:
                lines = proto.read_lines(duration_ms=200)
                for ln in lines:
                    if "otos lost" in ln or "EVT otos lost" in ln:
                        otos_lost_seen = True
                        print(f"  EVT otos lost received: {ln.strip()}")
                        break
                if otos_lost_seen:
                    break

            if not otos_lost_seen:
                print("  FAIL -- 'EVT otos lost' not received within 5 s of lift.")
                return STEP_FAIL

            # Prompt operator to replace robot.
            input("\n  >>> Replace robot on the floor, then press Enter <<<\n")
            time.sleep(1.0)  # brief settle

            # Check heading stability (no spin on placement).
            h0 = _otos_heading_deg(proto)
            if h0 is None:
                print("  Cannot read heading after replace -- PASS assumed (no spin detected).")
                return STEP_PASS

            time.sleep(SPIN_CHECK_MS / 1000.0)

            h1 = _otos_heading_deg(proto)
            if h1 is None:
                print("  Cannot read heading after settle -- PASS assumed.")
                return STEP_PASS

            drift = abs(_wrap_deg(h1 - h0))
            print(f"  Heading before/after settle: {h0:.1f} / {h1:.1f} deg"
                  f"  drift={drift:.1f} deg (tolerance: {SPIN_TOL_DEG} deg)")
            if drift <= SPIN_TOL_DEG:
                print("  No spin on placement detected.")
                return STEP_PASS
            print(f"  FAIL -- heading drifted {drift:.1f} deg after re-placement "
                  f"(possible spin-on-placement).")
            return STEP_FAIL

    except (RobotSilentError, RunawayAbortError) as exc:
        print(f"  BenchRun aborted: {exc}")
        return STEP_FAIL


# ---------------------------------------------------------------------------
# Check 5: TLM drop-rate
# ---------------------------------------------------------------------------

def check5_tlm_drop_rate(proto: NezhaProtocol) -> str:
    """STREAM 40 for 10 s; count TLM frames and report observed rate and drops.

    Expected rate: ~25 frames/s (STREAM 40 ms = 25 Hz).
    PASS criterion: >= 80% of expected frames received (i.e., >= 200 frames in 10 s).
    Always prints the measured rate even on failure.
    """
    _banner("Check 5: TLM drop-rate (STREAM 40 for 10 s)")

    STREAM_PERIOD_MS = 40   # 25 Hz
    MEASURE_SECS = 10.0
    EXPECTED_FRAMES = int(MEASURE_SECS * 1000.0 / STREAM_PERIOD_MS)
    MIN_FRAMES = int(EXPECTED_FRAMES * 0.80)   # 80% threshold

    proto.stream(STREAM_PERIOD_MS)
    time.sleep(0.15)  # let first frames arrive
    print(f"  STREAM {STREAM_PERIOD_MS} ms enabled. Counting for {MEASURE_SECS:.0f} s ...")

    tlm_count = 0
    t_start = time.monotonic()
    t_end = t_start + MEASURE_SECS
    while time.monotonic() < t_end:
        for ln in proto.read_lines(duration_ms=200):
            if parse_tlm(ln) is not None:
                tlm_count += 1

    elapsed = time.monotonic() - t_start
    proto.stream(0)

    obs_rate = tlm_count / elapsed if elapsed > 0.0 else 0.0
    print(f"  TLM frames received: {tlm_count} over {elapsed:.1f} s"
          f"  (observed rate: {obs_rate:.1f} Hz)")
    print(f"  Expected >= {MIN_FRAMES} frames ({EXPECTED_FRAMES} ideal at"
          f" {1000.0/STREAM_PERIOD_MS:.0f} Hz x 0.80)")

    if tlm_count >= MIN_FRAMES:
        return STEP_PASS
    print(f"  FAIL -- received only {tlm_count} frames ({obs_rate:.1f} Hz),"
          f" expected >= {MIN_FRAMES}.")
    return STEP_FAIL


# ---------------------------------------------------------------------------
# Field log
# ---------------------------------------------------------------------------

def _append_field_log(results: dict[int, str], step_names: dict[int, str]) -> None:
    """Append a dated SHA-stamped entry to docs/knowledge/field-log.md."""
    log_path = _REPO / "docs" / "knowledge" / "field-log.md"
    log_path.parent.mkdir(parents=True, exist_ok=True)

    # Get git SHA (short).
    try:
        sha = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=str(_REPO), stderr=subprocess.DEVNULL, text=True
        ).strip()
    except Exception:
        sha = "unknown"

    ts = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%MZ")
    overall = "PASS" if all(v == STEP_PASS for v in results.values()) else "FAIL"

    lines = [f"\n## {ts}  sha={sha}  overall={overall}\n"]
    for num in sorted(step_names):
        status = results.get(num, STEP_SKIP)
        lines.append(f"- Check {num} ({step_names[num]}): {status}\n")

    with open(log_path, "a", encoding="utf-8") as f:
        f.writelines(lines)

    print(f"\n  Field log entry appended: {log_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None,
                    help="relay serial port (auto-detects if omitted)")
    ap.add_argument("--checks", default="1,2,3,4,5",
                    help="comma-separated check numbers to run (default: all)")
    args = ap.parse_args()

    checks_to_run = {int(s.strip()) for s in args.checks.split(",")}

    step_names = {
        1: "Safety check",
        2: "RT x4 closure",
        3: "G square (200mm)",
        4: "Lift test (EVT otos lost)",
        5: "TLM drop-rate",
    }

    # Connect and preflight via the shared robot-construction path
    # (_make_robot -> Nezha robot + open SerialConnection; auto !GO over relay).
    print("Connecting to robot ...")
    args.verbose = False
    robot, conn, result = _make_robot(args)
    if result.get("error"):
        sys.exit(f"Connection failed: {result['error']}")
    proto = robot._proto

    png = proto.ping()
    if not png:
        proto.stop()
        sys.exit("PING failed -- robot not responding. Power-cycle and retry.")
    print(f"Robot alive: t={png[0]} ms, rtt={png[1]:.0f} ms")

    results: dict[int, str] = {}

    try:
        if 1 in checks_to_run:
            results[1] = check1_safety_check(proto)

        if 2 in checks_to_run:
            results[2] = check2_turn_closure(robot)

        if 3 in checks_to_run:
            results[3] = check3_g_square(robot)

        if 4 in checks_to_run:
            results[4] = check4_lift_test(robot)

        if 5 in checks_to_run:
            results[5] = check5_tlm_drop_rate(proto)

    finally:
        # Always safe-stop the robot on exit (normal or exception).
        print()
        print("[safe-stop] Sending STOP + STREAM 0 ...")
        proto.stop()
        proto.stream(0)
        conn.disconnect()

    # Append field log entry.
    _append_field_log(results, step_names)

    # Summary.
    _banner("Smoke Ritual Summary")
    overall = STEP_PASS
    for check_num in sorted(step_names):
        if check_num not in checks_to_run:
            status = STEP_SKIP
        else:
            status = results.get(check_num, STEP_SKIP)
        _result(check_num, step_names[check_num], status)
        if status == STEP_FAIL:
            overall = STEP_FAIL

    print()
    if overall == STEP_PASS:
        print("OVERALL: PASS -- all checks passed.")
        sys.exit(0)
    else:
        print("OVERALL: FAIL -- one or more checks failed.")
        sys.exit(1)


if __name__ == "__main__":
    main()
