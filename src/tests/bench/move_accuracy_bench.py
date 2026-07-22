#!/usr/bin/env python3
"""move_accuracy_bench.py -- stakeholder acceptance-table bench script
(2026-07-22): "verify that you say 'drive 500mm' and it drives for 500mm
... 'drive for 2 seconds' and it drives for 2 seconds. You did not test
this."

This is NOT a protocol-correctness check (that is `move_protocol_bench.py`
-- queue/ack/timing semantics). This script measures real-world ACCURACY
against a stopwatch and a tape measure, using the SAME calibration-push
mechanism the TestGUI's `_push_robot_calibration()` uses
(`calibration.push.calibration_commands()` fed through
`testgui.binary_bridge.translate_command()`), so a clean pass here is
direct evidence the TestGUI's own robot-select calibration push actually
lands on real hardware -- the root cause fixed alongside this script (see
`testgui/binary_bridge.py`'s `_handle_set_patch()`).

What this script does, in order:
  1. Connects, does a passive bus-health read (no drive commands) --
     encoders/OTOS-presence/motor-bus-connectivity flags, matching
     `.claude/rules/hardware-bench-testing.md`'s "sensors are alive" gate.
  2. Pushes `tovez_nocal.json` (uncalibrated) and drives ONE 500mm forward
     leg -- the "before" half of the calibration A/B.
  3. Pushes `tovez.json` (calibrated) via `calibration_commands()` +
     `binary_bridge.translate_command()` -- the EXACT path the TestGUI's
     robot-select handler uses -- and reports applied/rejected/nodev
     counts. This is the regression check for the GUI calibration-push fix.
  4. Drives another 500mm forward leg under calibrated gains -- the
     "after" half of the A/B.
  5. Reads 5s of IDLE telemetry (no drive commands in flight) and reports
     whether encoder POSITIONS advance (real creep) or only `vel` reads
     nonzero while positions hold (estimator/measurement noise floor).
  6. Runs the full distance/time/turn acceptance table under calibrated
     gains: 100/300/500/700mm forward + 500mm reverse (2 trials each),
     move_twist(150, stop_time=2000) (2 trials), and 90-degree turns both
     directions (2 trials each).

Every distance/turn number is measured from `TLMFrame.enc`/`TLMFrame.pose`
(encoder odometry -- always on the wire, no OTOS/camera dependency, per
`docs/protocol-v4.md` sec 8.1). Completion is detected the SAME way the
TestGUI's `_await_move_completion()` now does: poll for the Move's own
completion ack (`ack_corr == move_id`, `ack_fresh`), not just the enqueue
ack -- see that method's own docstring (`testgui/transport.py`) for why
the two are different acks on the same wire slot.

Usage:
    uv run python src/tests/bench/move_accuracy_bench.py
    uv run python src/tests/bench/move_accuracy_bench.py --port /dev/cu.usbmodem2121102
    uv run python src/tests/bench/move_accuracy_bench.py --skip-ab --skip-creep
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

from robot_radio.calibration.push import calibration_commands
from robot_radio.config.robot_config import load_robot_config
from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol, TLMFrame
from robot_radio.testgui import binary_bridge

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
ACK_TIMEOUT = 500          # [ms] wait_for_ack() bound for a command's enqueue ack
CRUISE_SPEED = 150.0       # [mm/s] task-specified acceptance-table speed
YAW_RATE = 2.0             # [rad/s] matches transport.py's own _UNMANAGED_YAW_RATE
COMPLETION_MARGIN_S = 0.75  # [s] matches transport.py's _COMPLETION_POLL_MARGIN_S
COMPLETION_POLL_S = 0.03   # [s]

_REPO_ROOT = Path(__file__).resolve().parents[3]
_TOVEZ_JSON = _REPO_ROOT / "data" / "robots" / "tovez.json"
_TOVEZ_NOCAL_JSON = _REPO_ROOT / "data" / "robots" / "tovez_nocal.json"

# Move.id space disjoint from SerialConnection's own small sequential
# corr_id counter -- see transport.py's _MOVE_ID_BASE for the same
# discipline applied in the TestGUI itself.
_NEXT_MOVE_ID = 1 << 24


def _next_move_id() -> int:
    global _NEXT_MOVE_ID
    _NEXT_MOVE_ID += 1
    return _NEXT_MOVE_ID


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------


@dataclass
class Trial:
    label: str
    commanded: float
    measured: float
    unit: str
    extra: str = ""

    @property
    def error_pct(self) -> float:
        if self.commanded == 0:
            return float("nan")
        return 100.0 * (self.measured - self.commanded) / abs(self.commanded)

    def line(self) -> str:
        return (f"  {self.label:<28} commanded={self.commanded:+.1f}{self.unit} "
                f"measured={self.measured:+.1f}{self.unit} "
                f"error={self.error_pct:+.1f}%" + (f"  ({self.extra})" if self.extra else ""))


@dataclass
class Report:
    trials: list[Trial] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    def add(self, trial: Trial) -> None:
        self.trials.append(trial)
        print(trial.line())

    def note(self, text: str) -> None:
        self.notes.append(text)
        print(f"  NOTE: {text}")


# ---------------------------------------------------------------------------
# Telemetry helpers
# ---------------------------------------------------------------------------


def _drain(proto: NezhaProtocol) -> list[TLMFrame]:
    return proto.read_pending_binary_tlm_frames()


def _latest_baseline(proto: NezhaProtocol, attempts: int = 5,
                     settle_s: float = 0.08) -> "tuple[tuple[int, int] | None, tuple[int, int, int] | None]":
    """Drain up to `attempts` times (short settle between each) and return
    the most recent (enc, pose) pair seen -- BOTH read from the SAME set of
    drained frames each pass, so capturing one never steals the frame the
    other needed (the bug a two-separate-single-purpose-drain version of
    this had: enc's own retry-drain could consume the only buffered frame,
    leaving pose's own retry with an emptied queue and a real chance of
    missing a fresh push within its own single settle beat -- observed on
    the bench as intermittent "NO POSE DATA" turn trials)."""
    enc = None
    pose = None
    for _ in range(attempts):
        for f in _drain(proto):
            if f.enc is not None:
                enc = f.enc
            if f.pose is not None:
                pose = f.pose
        if enc is not None and pose is not None:
            break
        time.sleep(settle_s)
    return enc, pose


def _dispatch_and_await_completion(
    proto: NezhaProtocol, *, v_x: float = 0.0, v_y: float = 0.0, omega: float = 0.0,
    stop_time: "float | None" = None, stop_distance: "float | None" = None,
    stop_angle: "float | None" = None, timeout: float,
) -> dict:
    """Send ONE move_twist(), wait for its enqueue ack, then poll telemetry
    for its completion ack -- the exact two-ack sequence
    `_HardwareTransport._dispatch_managed_move()`/`_await_move_completion()`
    (testgui/transport.py) now use. Returns a dict: enqueue_ok, completed,
    fault_move_timeout, elapsed_s (dispatch -> completion ack),
    enc_before, enc_after, first_motion_s (elapsed from dispatch to the
    first frame whose enc differs from enc_before -- None if it never
    moved), pose_before, pose_after (heading in degrees).
    """
    move_id = _next_move_id()
    enc_before, pose_before = _latest_baseline(proto)

    t_dispatch = time.monotonic()
    corr_id = proto.move_twist(
        v_x=v_x, v_y=v_y, omega=omega, stop_time=stop_time,
        stop_distance=stop_distance, stop_angle=stop_angle,
        timeout=timeout, replace=True, move_id=move_id)
    ack = proto.wait_for_ack(corr_id, timeout=ACK_TIMEOUT)
    enqueue_ok = ack is not None and ack.ok

    bound = t_dispatch + (timeout / 1000.0) + COMPLETION_MARGIN_S
    enc_after = enc_before
    pose_after = pose_before
    first_motion_s = None
    completed = False
    fault = False
    while time.monotonic() < bound:
        for f in _drain(proto):
            if f.enc is not None:
                enc_after = f.enc
                if (first_motion_s is None and enc_before is not None
                        and f.enc != enc_before):
                    first_motion_s = time.monotonic() - t_dispatch
            if f.pose is not None:
                pose_after = f.pose
            if f.ack_fresh and f.ack_corr == move_id:
                completed = True
                fault = bool(f.fault_move_timeout)
                if f.enc is not None:
                    enc_after = f.enc
                if f.pose is not None:
                    pose_after = f.pose
        if completed:
            break
        time.sleep(COMPLETION_POLL_S)
    elapsed_s = time.monotonic() - t_dispatch

    return dict(
        enqueue_ok=enqueue_ok, completed=completed, fault_move_timeout=fault,
        elapsed_s=elapsed_s, enc_before=enc_before, enc_after=enc_after,
        first_motion_s=first_motion_s, pose_before=pose_before, pose_after=pose_after)


def _enc_delta(result: dict) -> "tuple[float, float, float] | None":
    """(mean, left, right) mm delta, or None if either encoder reading is
    missing."""
    b, a = result["enc_before"], result["enc_after"]
    if b is None or a is None:
        return None
    dl = a[0] - b[0]
    dr = a[1] - b[1]
    return (dl + dr) / 2.0, dl, dr


def _heading_delta_deg(result: dict) -> "float | None":
    b, a = result["pose_before"], result["pose_after"]
    if b is None or a is None:
        return None
    return (a[2] - b[2]) / 100.0  # cdeg -> deg


# ---------------------------------------------------------------------------
# Config push (mirrors __main__.py's _push_robot_calibration() exactly --
# calibration_commands() -> binary_bridge.translate_command(), the fixed
# path)
# ---------------------------------------------------------------------------


def push_robot_config(proto: NezhaProtocol, cfg_path: Path, report: Report) -> dict:
    cfg = load_robot_config(cfg_path)
    cmds = calibration_commands(cfg)
    n_bad = 0
    n_nodev = 0
    rejected: list[str] = []
    for cmd, read_timeout in cmds:
        reply = binary_bridge.translate_command(proto, cmd)
        upper = (reply or "").upper()
        if "NODEV" in upper:
            n_nodev += 1
        elif "ERR" in upper:
            n_bad += 1
            rejected.append(f"{cmd!r} -> {reply!r}")
    n_applied = len(cmds) - n_bad - n_nodev
    print(f"  [{cfg.robot_name}] pushed {n_applied}/{len(cmds)} calibration values"
          + (f" ({n_nodev} nodev)" if n_nodev else "")
          + (f" ({n_bad} REJECTED)" if n_bad else ""))
    for r in rejected:
        print(f"    REJECTED: {r}")
    return dict(robot_name=cfg.robot_name, total=len(cmds), applied=n_applied,
                nodev=n_nodev, rejected=n_bad, rejected_detail=rejected)


# ---------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------


def _outcome(result: dict) -> str:
    """One-word-ish outcome label folding in the enqueue ack too -- a
    NO-COMPLETION-ACK trial is only really ambiguous ("lost completion ack"
    vs "Move never even got in") when the enqueue itself is also known bad;
    surface that distinction rather than hiding it."""
    if not result["enqueue_ok"]:
        return "ENQUEUE-REJECTED/TIMEOUT"
    if result["completed"] and not result["fault_move_timeout"]:
        return "completed"
    if result["fault_move_timeout"]:
        return "TIMEOUT-FAULT"
    return "NO-COMPLETION-ACK"


def run_distance_trial(proto: NezhaProtocol, mm: float, report: Report, label: str) -> None:
    speed = CRUISE_SPEED
    v_x = math.copysign(speed, mm)
    expected_s = abs(mm) / speed
    timeout = max(2000.0, expected_s * 1000.0 * 3.0)
    result = _dispatch_and_await_completion(
        proto, v_x=v_x, stop_distance=abs(mm), timeout=timeout)
    delta = _enc_delta(result)
    outcome = _outcome(result)
    if delta is None:
        report.add(Trial(label, mm, 0.0, "mm", extra=f"{outcome}, NO ENCODER DATA"))
        return
    mean, dl, dr = delta
    signed_mean = math.copysign(abs(mean), mm)
    report.add(Trial(label, mm, signed_mean, "mm",
                     extra=f"{outcome}, L={dl:+.0f} R={dr:+.0f} |L-R|={abs(dl-dr):.0f}mm, "
                           f"elapsed={result['elapsed_s']*1000:.0f}ms"))


def run_time_trial(proto: NezhaProtocol, stop_time_ms: float, report: Report, label: str) -> None:
    timeout = stop_time_ms + 1500.0
    result = _dispatch_and_await_completion(
        proto, v_x=CRUISE_SPEED, stop_time=stop_time_ms, timeout=timeout)
    outcome = _outcome(result)
    delta = _enc_delta(result)
    dist_text = f"traveled={delta[0]:.0f}mm" if delta else "no encoder data"
    first_motion = result["first_motion_s"]
    if first_motion is None:
        report.note(f"{label}: first-motion never observed within the poll window")
        elapsed_from_motion_ms = float("nan")
    else:
        elapsed_from_motion_ms = (result["elapsed_s"] - first_motion) * 1000.0
    report.add(Trial(label, stop_time_ms, elapsed_from_motion_ms, "ms",
                     extra=f"{outcome}, {dist_text}, "
                           f"first_motion={None if first_motion is None else f'{first_motion*1000:.0f}ms'}"))


def run_turn_trial(proto: NezhaProtocol, deg: float, report: Report, label: str) -> None:
    omega = math.copysign(YAW_RATE, deg)
    angle = math.radians(abs(deg))
    expected_s = angle / YAW_RATE
    timeout = max(2000.0, expected_s * 1000.0 * 3.0)
    result = _dispatch_and_await_completion(
        proto, omega=omega, stop_angle=angle, timeout=timeout)
    outcome = _outcome(result)
    hd = _heading_delta_deg(result)
    if hd is None:
        report.add(Trial(label, deg, 0.0, "deg", extra=f"{outcome}, NO POSE DATA"))
        return
    report.add(Trial(label, deg, hd, "deg",
                     extra=f"{outcome}, elapsed={result['elapsed_s']*1000:.0f}ms"))


def idle_creep_check(proto: NezhaProtocol, duration_s: float, report: Report) -> None:
    print(f"\n== Rest-creep check: {duration_s:.0f}s idle telemetry ==")
    frames: list[TLMFrame] = []
    deadline = time.monotonic() + duration_s
    _drain(proto)
    while time.monotonic() < deadline:
        for f in _drain(proto):
            if f.enc is not None:
                frames.append(f)
        time.sleep(0.05)
    if len(frames) < 2:
        report.note("rest-creep check: not enough frames captured -- inconclusive")
        return
    enc0, enc_n = frames[0].enc, frames[-1].enc
    d_left = enc_n[0] - enc0[0]
    d_right = enc_n[1] - enc0[1]
    vels = [f.vel for f in frames if f.vel is not None]
    vel_left_mean = sum(v[0] for v in vels) / len(vels) if vels else float("nan")
    vel_right_mean = sum(v[1] for v in vels) / len(vels) if vels else float("nan")
    print(f"  frames captured: {len(frames)}")
    print(f"  encoder positions: L {enc0[0]}->{enc_n[0]} (d={d_left:+d}mm), "
          f"R {enc0[1]}->{enc_n[1]} (d={d_right:+d}mm)")
    print(f"  mean reported velocity: L={vel_left_mean:+.1f}mm/s R={vel_right_mean:+.1f}mm/s")
    if abs(d_left) <= 2 and abs(d_right) <= 2:
        report.note(f"REST-CREEP VERDICT: NOISE FLOOR -- positions held (L{d_left:+d}/R{d_right:+d}mm "
                    f"over {duration_s:.0f}s) despite vel reading L={vel_left_mean:+.1f}/"
                    f"R={vel_right_mean:+.1f}mm/s")
    else:
        report.note(f"REST-CREEP VERDICT: REAL CREEP -- positions advanced L{d_left:+d}/R{d_right:+d}mm "
                    f"over {duration_s:.0f}s (mean vel L={vel_left_mean:+.1f}/R={vel_right_mean:+.1f}mm/s)")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def _args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--skip-ab", action="store_true", help="skip the nocal-vs-calibrated 500mm A/B")
    p.add_argument("--skip-creep", action="store_true", help="skip the 5s idle rest-creep check")
    p.add_argument("--trials", type=int, default=2, help="trials per distance/turn condition")
    return p.parse_args()


def main() -> int:
    args = _args()
    report = Report()

    conn = SerialConnection(port=args.port)
    info = conn.connect()
    if info.get("status") != "connected":
        print(f"ERROR: connect failed: {info}")
        return 2
    proto = NezhaProtocol(conn)
    print(f"connected: {info}")

    try:
        # --- passive sensor/bus-health read ---------------------------------
        print("\n== Bus-health / sensor-alive read (no drive commands) ==")
        frames = []
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            frames.extend(_drain(proto))
            time.sleep(0.05)
        if frames:
            f = frames[-1]
            print(f"  flags=0x{f.flags:x} conn_left={f.conn_left} conn_right={f.conn_right} "
                  f"otos_present={f.otos_present} enc={f.enc} pose={f.pose}")
            if not (f.conn_left and f.conn_right):
                print("  WARNING: motor bus not fully connected -- results below may be unreliable")
        else:
            print("  WARNING: no telemetry frames observed in 1s")

        # --- A/B: nocal vs calibrated, 500mm forward ------------------------
        if not args.skip_ab:
            print("\n== A/B: tovez_nocal vs tovez (calibrated), 500mm forward ==")
            push_robot_config(proto, _TOVEZ_NOCAL_JSON, report)
            for i in range(args.trials):
                run_distance_trial(proto, 500.0, report, f"nocal 500mm fwd #{i+1}")

        # --- push calibration (the fixed GUI path) --------------------------
        print("\n== Calibration push: tovez.json (calibrated), via calibration_commands() "
              "+ binary_bridge.translate_command() -- the TestGUI's own fixed path ==")
        cal_result = push_robot_config(proto, _TOVEZ_JSON, report)
        report.note(f"calibration push: {cal_result['applied']}/{cal_result['total']} applied, "
                    f"{cal_result['rejected']} rejected, {cal_result['nodev']} nodev")

        if not args.skip_ab:
            for i in range(args.trials):
                run_distance_trial(proto, 500.0, report, f"CAL 500mm fwd #{i+1}")

        # --- rest-creep check -------------------------------------------------
        if not args.skip_creep:
            idle_creep_check(proto, 5.0, report)

        # --- acceptance table (calibrated gains) -----------------------------
        print("\n== Acceptance table (calibrated gains, CRUISE_SPEED="
              f"{CRUISE_SPEED:.0f}mm/s) ==")
        for mm in (100.0, 300.0, 500.0, 700.0):
            for i in range(args.trials):
                run_distance_trial(proto, mm, report, f"D {mm:+.0f}mm #{i+1}")
        for i in range(args.trials):
            run_distance_trial(proto, -500.0, report, f"D -500mm (reverse) #{i+1}")

        for i in range(args.trials):
            run_time_trial(proto, 2000.0, report, f"time-stop 2000ms #{i+1}")

        for i in range(args.trials):
            run_turn_trial(proto, 90.0, report, f"turn +90deg #{i+1}")
        for i in range(args.trials):
            run_turn_trial(proto, -90.0, report, f"turn -90deg #{i+1}")

    finally:
        try:
            proto.stop()
        except Exception:
            pass
        conn.disconnect()

    print("\n==== Summary ====")
    for t in report.trials:
        print(t.line())
    for n in report.notes:
        print(f"NOTE: {n}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
