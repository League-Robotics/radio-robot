"""calibrate_turns — interactive OTOS angular scale calibration.

Core logic extracted from ``host/calibrate_angular.py``.  No argparse or
sys.exit; the caller handles CLI setup and calls ``calibrate_turns()``.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Optional

from robot_radio.calibration.helpers import (
    deep_merge,
    int8_to_scale,
    mean_stdev,
    resolve_save_path,
    save_config,
    scale_to_int8,
)
from robot_radio.robot._legacy_tlm_text import parse_historical_tlm_line

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

TURN_SPEED = 100         # [mm/s] wheel speed for spin
DEFAULT_ANGLE = 360.0    # [deg] target spin angle per trial
OTOS_FW_MIN_SCALE = 0.872    # int8 = -128
OTOS_FW_MAX_SCALE = 1.127    # int8 = +127
BAUD = 115200


# ---------------------------------------------------------------------------
# Angular-specific math helpers
# ---------------------------------------------------------------------------

def compute_new_angular_scale(
    target: float,
    otos: float,
    current_scale: float,
) -> tuple[float, int]:
    """Compute recommended new OTOS angular scale.  target/otos in [deg].

    Formula: new_scale = (target / otos) * current_scale.
    Returns (new_scale_float, new_scale_int8).
    Clamps to firmware representable range.
    """
    if abs(otos) < 1.0:
        return current_scale, scale_to_int8(current_scale)
    ratio = target / otos
    raw = ratio * current_scale
    clamped = max(OTOS_FW_MIN_SCALE, min(OTOS_FW_MAX_SCALE, raw))
    return clamped, scale_to_int8(clamped)


def heading_delta(before: int, after: int) -> float:  # [cdeg] in, [deg] out
    """Compute signed heading change in degrees from two centi-degree readings.

    Handles wrap-around at ±18000 cdeg (±180°).
    Returns positive for CCW (firmware convention: CCW positive).
    """
    delta = after - before
    while delta > 18000:
        delta -= 36000
    while delta <= -18000:
        delta += 36000
    return delta / 100.0


# ---------------------------------------------------------------------------
# Config helpers specific to angular calibration
# ---------------------------------------------------------------------------

def load_current_angular_scale(config_path: Path) -> float:
    """Read otos_angular_scale from robot config JSON, default 1.0."""
    try:
        data = json.loads(config_path.read_text())
        return float(data.get("calibration", {}).get("otos_angular_scale", 1.0))
    except Exception:
        return 1.0


def save_angular_calibration_to_config(
    path: Path,
    angular_scale: float,
    rotation_gain: Optional[float] = None,
    rotation_gain_neg: Optional[float] = None,
) -> None:
    """Write angular calibration fields into robot config JSON."""
    cal: dict = {"otos_angular_scale": round(angular_scale, 6)}
    if rotation_gain is not None:
        cal["rotation_gain"] = round(rotation_gain, 6)
    if rotation_gain_neg is not None:
        cal["rotation_gain_neg"] = round(rotation_gain_neg, 6)
    save_config(path, {"calibration": cal})


# ---------------------------------------------------------------------------
# Serial wire helpers
# ---------------------------------------------------------------------------

def _send_and_wait(ser, cmd: str, want_prefix: str, timeout: float = 5.0) -> list[str]:
    ser.write_line(cmd)
    collected: list[str] = []
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        lines = ser.read_available(timeout=0.1)
        for line in lines:
            collected.append(line)
            clean = line.strip().lstrip("<# ").strip()
            if clean.startswith(want_prefix):
                return collected
    return collected


def _snap_pose(ser, timeout: float = 3.0) -> Optional[tuple[int, int, int]]:
    # 097-003: this script talks to the robot via _conn_helpers.RelaySerial/
    # DirectSerial (a raw pyserial wrapper, deliberately not SerialConnection
    # -- see _conn_helpers.py's own header), so there is no _binary_tlm_queue
    # to source a TLMFrame from here. Stays on the text plane (still live --
    # only a later ticket, 097-008, retires the firmware's text SNAP/STREAM
    # handlers) via the frozen historical parser; see
    # robot_radio.robot._legacy_tlm_text's own module docstring. Same
    # reasoning applies to _stream_tlm_until_evt()/_interactive_adjust()
    # below, and to this file's own "STREAM 20"/"STREAM 0" text commands.
    lines = _send_and_wait(ser, "SNAP", "TLM", timeout=timeout)
    for line in lines:
        frame = parse_historical_tlm_line(line)
        if frame is not None and frame.pose is not None:
            return frame.pose
    return None


def _wait_evt_done(ser, verb: str, timeout: float = 60.0) -> bool:
    target = f"EVT done {verb}"
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        lines = ser.read_available(timeout=0.2)
        for line in lines:
            clean = line.strip().lstrip("<# ").strip()
            if clean.startswith(target):
                return True
            if clean.startswith("EVT safety_stop"):
                return False
    return False


def _stream_tlm_until_evt(
    ser, verb: str, timeout: float
) -> tuple[list[tuple[int, int, int]], bool]:
    target = f"EVT done {verb}"
    poses: list[tuple[int, int, int]] = []
    deadline = time.monotonic() + timeout
    done = False
    while time.monotonic() < deadline:
        lines = ser.read_available(timeout=0.1)
        for line in lines:
            frame = parse_historical_tlm_line(line)
            if frame is not None and frame.pose is not None:
                poses.append(frame.pose)
            clean = line.strip().lstrip("<# ").strip()
            if clean.startswith(target):
                done = True
                break
            if clean.startswith("EVT safety_stop"):
                break
        if done:
            break
    return poses, done


# ---------------------------------------------------------------------------
# Interactive arrow-key adjustment
# ---------------------------------------------------------------------------

def _interactive_adjust(ser, current_heading: int, target: float,
                        nudge_speed: int = 80, nudge_duration: int = 60) -> int:  # [cdeg], [deg], [mm/s], [ms]
    """Interactive arrow-key heading adjustment. Returns final heading (cdeg)."""
    if not sys.stdin.isatty():
        print("  (stdin is not a TTY — skipping interactive adjustment)")
        return current_heading

    import select
    import termios
    import tty
    import os as _os

    heading = current_heading

    def _read_key(fd: int) -> str:
        ch = _os.read(fd, 1).decode("utf-8", errors="replace")
        if ch == "\x1b":
            if select.select([fd], [], [], 0.1)[0]:
                rest = _os.read(fd, 2).decode("utf-8", errors="replace")
                if rest == "[D":
                    return "left"
                if rest == "[C":
                    return "right"
            return "esc"
        if ch == "\x03":
            return "ctrl-c"
        if ch in ("\r", "\n"):
            return "enter"
        return ch

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    def _show():
        err = heading / 100.0 - target  # [deg]
        hint = " <- " if err > 0 else " -> "
        print(f"\r  heading={heading / 100:+.2f}  "
              f"err={err:+.2f}  [<- CW  -> CCW  Enter=done]{hint}   ",
              end="", flush=True)

    try:
        tty.setraw(fd)
        _show()
        while True:
            if select.select([fd], [], [], 0.02)[0]:
                k = _read_key(fd)
                if k == "enter":
                    break
                if k in ("esc", "ctrl-c"):
                    break
                if k == "left":
                    ser.write_line(f"T -{nudge_speed} {nudge_speed} {nudge_duration}")
                elif k == "right":
                    ser.write_line(f"T {nudge_speed} -{nudge_speed} {nudge_duration}")
                time.sleep(nudge_duration / 1000.0 + 0.1)
            lines = ser.read_available(timeout=0.05)
            for line in lines:
                frame = parse_historical_tlm_line(line)
                if frame is not None and frame.pose is not None:
                    heading = frame.pose[2]
                    _show()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print()

    return heading


# ---------------------------------------------------------------------------
# Core interactive calibration logic
# ---------------------------------------------------------------------------

def calibrate_turns(
    ser,
    config_path: Optional[Path],
    target: float = DEFAULT_ANGLE,  # [deg]
    speed: int = TURN_SPEED,        # [mm/s]
    dry_run: bool = False,
) -> None:
    """Run interactive angular scale calibration.

    *ser* must expose ``write_line(text)``, ``read_available(timeout)`` and
    ``close()`` — satisfied by ``_RelaySerial`` or ``_DirectSerial`` from the
    entry-point script, or any compatible mock.

    *config_path* is the Path to the active robot JSON (may be None).

    All output goes to stdout/stderr.  Raises ``SystemExit`` only on
    unrecoverable errors (already handled by the entry point).
    """
    current_scale = 1.0
    if config_path and config_path.exists():
        current_scale = load_current_angular_scale(config_path)
        print(f"  Config: {config_path}")
    else:
        print("  WARNING: No robot config found — using otos_angular_scale = 1.0")
    current_int8 = scale_to_int8(current_scale)
    print(f"  Current otos_angular_scale = {current_scale:.4f}  (int8={current_int8:+d})")

    # Ping & set angular scalar
    print("\n  Checking link (PING)...")
    lines = _send_and_wait(ser, "PING", "OK pong", timeout=3.0)
    if not any("pong" in ln for ln in lines):
        print("  WARNING: no PING reply — robot may not be reachable.")
    else:
        print("  Robot responding.")

    print(f"  Setting OA {current_int8:+d} (scale={current_scale:.4f}) on hardware...")
    _send_and_wait(ser, f"OA {current_int8}", "OK", timeout=2.0)

    print(f"\n  Target spin angle: {target:.1f}  Speed: {speed} mm/s")
    print("  Aim the robot's marker at a reference point on the wall/floor.")
    print("  Trials alternate CCW / CW.  Use <- -> to nudge back to the mark.")
    print("  Press Enter to spin each trial, 'q' to finish.\n")

    samples: list[tuple[int, float]] = []  # (direction_sign, otos) [deg]
    direction = +1  # start CCW

    try:
        while True:
            n = len(samples)
            label = "CCW" if direction > 0 else "CW"
            print(f"[Trial {n + 1}]  ({n} samples)  next: {label}  "
                  "— Enter to spin, 'q' to finish")
            try:
                raw = input().strip()
            except EOFError:
                break
            if raw.lower() in ("q", "quit", "exit"):
                break

            _send_and_wait(ser, "ZERO enc pose", "OK", timeout=2.0)
            time.sleep(0.15)

            pose_before = _snap_pose(ser, timeout=2.0)
            heading_before = pose_before[2] if pose_before else 0  # [cdeg]

            _send_and_wait(ser, "STREAM 20", "OK", timeout=2.0)
            time.sleep(0.1)

            trackwidth = 126.0  # [mm]
            duration = int((target / 360.0) * math.pi * trackwidth / speed * 1000)  # [ms]
            duration = max(500, min(15000, duration))

            l_speed = direction * speed
            r_speed = -direction * speed
            print(f"  Spinning {label} for ~{duration}ms ({target:.0f})...")

            spin_cmd = f"T {l_speed} {r_speed} {duration}"
            _send_and_wait(ser, spin_cmd, "OK", timeout=2.0)
            timeout_s = duration / 1000.0 + 5.0
            poses_during, done_flag = _stream_tlm_until_evt(ser, "T", timeout=timeout_s)

            if not done_flag:
                print("  WARNING: Did not receive EVT done T — spin may have been incomplete.")

            _send_and_wait(ser, "STREAM 0", "OK", timeout=2.0)
            time.sleep(0.2)

            pose_after = _snap_pose(ser, timeout=3.0)
            heading_after = pose_after[2] if pose_after else (
                poses_during[-1][2] if poses_during else heading_before
            )  # [cdeg]

            otos_raw = heading_delta(heading_before, heading_after)  # [deg]
            print(f"  OTOS heading: before={heading_before / 100:.2f}  "
                  f"after={heading_after / 100:.2f}  delta={otos_raw:+.2f}")
            print(f"  Target: {direction * target:+.1f}  "
                  f"Error: {otos_raw - direction * target:+.2f}")

            print(f"\n  Nudge the robot back to the starting mark with <- -> keys.")
            print(f"  Press Enter when aligned.")
            heading_adjusted = _interactive_adjust(
                ser, heading_after,
                target=direction * target,
                nudge_speed=80, nudge_duration=50,
            )
            ser.write_line("STOP")
            time.sleep(0.1)

            adjusted = heading_delta(heading_before, heading_adjusted)  # [deg]
            err = adjusted - direction * target  # [deg]
            print(f"  Adjusted: otos={adjusted:+.2f}  "
                  f"target={direction * target:+.1f}  "
                  f"err={err:+.2f}")

            if abs(adjusted) < 10.0:
                print("  WARNING: adjusted heading < 10 — possible misread. Discarded.")
            else:
                samples.append((direction, abs(adjusted)))
                print(f"  Sample {len(samples)} recorded: "
                      f"dir={label}  adjusted={abs(adjusted):.2f}")

            direction = -direction

    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.write_line("STOP")
            ser.write_line("STREAM 0")
            time.sleep(0.2)
        except Exception:
            pass

    # Statistics
    print("\n" + "=" * 60)
    print(f"Samples collected: {len(samples)}")
    if len(samples) < 2:
        print("Need >= 2 samples — not enough data.")
        return

    ccw = [d for (sign, d) in samples if sign > 0]
    cw  = [d for (sign, d) in samples if sign < 0]

    print(f"\n{'#':>3}  {'dir':>4}  {'otos':>9}  {'ratio':>7}  {'err':>7}")
    ratios: list[float] = []
    for i, (sign, d) in enumerate(samples, 1):
        label = "CCW" if sign > 0 else "CW"
        ratio = target / d if d > 0 else 0.0
        ratios.append(ratio)
        print(f"{i:>3}  {label:>4}  {d:>9.2f}  {ratio:>7.4f}  "
              f"{d - target:>+7.2f}")

    mean_all, std_all = mean_stdev(ratios)
    mean_ccw, std_ccw = mean_stdev(ccw)
    mean_cw,  std_cw  = mean_stdev(cw)

    new_scale_raw = mean_all * current_scale
    new_scale_clamped = max(OTOS_FW_MIN_SCALE, min(OTOS_FW_MAX_SCALE, new_scale_raw))
    new_int8 = scale_to_int8(new_scale_clamped)
    rounded_scale = int8_to_scale(new_int8)

    rot_gain = target / mean_ccw if mean_ccw > 0 else None
    rot_gain_neg = target / mean_cw if mean_cw > 0 else None

    print(f"\nRatio statistics (target / otos, deg):")
    print(f"  Overall: mean={mean_all:.4f}  stdev={std_all:.4f}  (n={len(ratios)})")
    if ccw:
        print(f"  CCW otos: mean={mean_ccw:.2f}  stdev={std_ccw:.2f}  (n={len(ccw)})")
    if cw:
        print(f"  CW  otos: mean={mean_cw:.2f}  stdev={std_cw:.2f}  (n={len(cw)})")

    print(f"\nRecommended values:")
    print(f"  otos_angular_scale = {current_scale:.4f} x {mean_all:.4f} = {new_scale_raw:.4f}")
    print(f"  clamped = {new_scale_clamped:.4f}  ->  int8={new_int8:+d}  -> {rounded_scale:.4f}")
    if rot_gain is not None:
        print(f"  rotation_gain     (CCW) = {rot_gain:.4f}")
    if rot_gain_neg is not None:
        print(f"  rotation_gain_neg (CW)  = {rot_gain_neg:.4f}")

    print(f"\n  To set manually in data/robots/<robot>.json:")
    print(f'    "otos_angular_scale": {rounded_scale:.4f}')
    if rot_gain is not None:
        print(f'    "rotation_gain": {round(rot_gain, 4)}')
    if rot_gain_neg is not None:
        print(f'    "rotation_gain_neg": {round(rot_gain_neg, 4)}')

    if dry_run:
        print("\n  --dry-run: config NOT updated.")
        return

    if config_path is None:
        print("\n  No config path found — cannot save. Update manually.")
        return

    print(f"\n  Save to {config_path}? [Y/n] ", end="", flush=True)
    try:
        ans = input().strip()
    except EOFError:
        ans = ""
    if ans.lower() in ("", "y", "yes"):
        save_angular_calibration_to_config(
            config_path, rounded_scale,
            rotation_gain=rot_gain,
            rotation_gain_neg=rot_gain_neg,
        )
        print(f"  Saved to {config_path}")
    else:
        print("  Not saved.")
