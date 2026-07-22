#!/usr/bin/env python3
"""src/tests/bench/tlm_log.py -- ticket 008 (sprint 115, gut-to-minimal-
firmware S1): stream v2 `Telemetry` frames (`protocol.py`'s `TLMFrame`,
115-003/007) to a CSV, one row per frame.

Per sprint.md's Architecture Decision 2 ("the frame is the dataset"): with
every on-chip measurement ring deleted (ticket 002) and every-cycle
telemetry emission in place (ticket 005), this is the SOLE dataset-
construction path this program now has -- the CSV log this script writes
reconstructs any time window for later analysis (sprint 117's estimator
work depends on this tool's own output existing and being correct).
Narrowly scoped: stream frames, write rows. No analysis logic belongs here
-- that is explicitly a future sprint's job (sprint.md's own module
boundary for this file). `frame_to_row()` is a straight, field-for-field
transcription of `TLMFrame` -- the SAME units each field already carries
(see `protocol.py`'s `TLMFrame`/`EncoderReading`/`OtosReading` docstrings
for the unit of each); no conversion, no derivation.

Connection setup follows `twist_drive.py`'s established P4-era pattern
(`SerialConnection` + `NezhaProtocol`, `mode=None` auto-detects direct USB
vs. the radio relay from the boot banner) -- NOT `relay_telemetry_rate.py`'s
(stale: it calls `proto.ping()`/`proto.stream()`, both P4-pruned verbs per
`protocol.py`'s own module docstring; not a pattern to follow).

`stream_to_csv()` is deliberately duck-typed on a frame source exposing
`.read_pending_binary_tlm_frames() -> list[TLMFrame]` (the `FrameSource`
Protocol below) rather than pinned to `NezhaProtocol` -- `robot_radio.io.
sim_loop.SimLoop` exposes the identical method (this project's "one Sim
object" convention: tests and this tool drive frames through the SAME
shape, never a second parallel path), so the exact function that logs a
real bench session also logs a `SimLoop` session with no adapter. `main()`'s
own CLI wires up only the real serial path (this ticket's own acceptance
criterion); a `SimLoop`-backed capture is exercised directly by this
ticket's own sim-mode verification and by `test_tlm_log.py`.

Usage:
    uv run python src/tests/bench/tlm_log.py
    uv run python src/tests/bench/tlm_log.py --port /dev/cu.usbmodem2121102 --duration 60
    uv run python src/tests/bench/tlm_log.py --relay --port /dev/cu.usbmodem2121302 --csv out/drive_session.csv
"""
from __future__ import annotations

import argparse
import csv
import pathlib
import sys
import time
from typing import Protocol

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol, TLMFrame

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
DEFAULT_CSV = _REPO_ROOT / "src" / "tests" / "bench" / "out" / "tlm_log.csv"

POLL_INTERVAL = 0.02      # [s] pending-frame drain period -- well under the 20ms primary
                          # cycle (protocol.py's kPrimaryPeriod mirror), so no frame waits
                          # a full poll behind
PROGRESS_INTERVAL = 1.0  # [s] stdout progress print period


class FrameSource(Protocol):
    """The exact slice of `NezhaProtocol`'s public surface `stream_to_csv()`
    depends on -- a `Protocol` (structural, duck-typed), mirroring
    `planner.executor.TwistTransport`'s own precedent. A real `NezhaProtocol`
    satisfies this as-is; so does `robot_radio.io.sim_loop.SimLoop` (see
    module docstring) -- no adapter needed for either."""

    def read_pending_binary_tlm_frames(self) -> "list[TLMFrame]": ...


# ---------------------------------------------------------------------------
# CSV row shape -- pure, frame-in/row-dict-out (ticket 008's own testing
# plan: testable without a real or simulated connection). CSV_FIELDNAMES is
# the single source of truth for column order; frame_to_row() always returns
# a dict with exactly these keys.
# ---------------------------------------------------------------------------

CSV_FIELDNAMES: "tuple[str, ...]" = (
    "now", "seq", "mode", "flags",
    "flag_otos_present", "flag_otos_connected", "flag_active",
    "flag_conn_left", "flag_conn_right", "flag_ack_fresh",
    "flag_fault_i2c_safety_net", "flag_fault_wedge_latch",
    "flag_fault_i2c_nak_timeout", "flag_fault_malformed_frame",
    "flag_fault_move_timeout",
    "flag_event_deadman_expired", "flag_event_boot_ready", "flag_event_config_applied",
    "flag_line_present", "flag_color_present",
    "ack_corr", "ack_err",
    "enc_left_position", "enc_left_velocity", "enc_left_time",
    "enc_right_position", "enc_right_velocity", "enc_right_time",
    "otos_x", "otos_y", "otos_heading", "otos_v_x", "otos_v_y", "otos_omega", "otos_time",
    "pose_x", "pose_y", "pose_theta",
    "twist_v_x", "twist_omega",
    "line_ch1", "line_ch2", "line_ch3", "line_ch4",
    "color_r", "color_g", "color_b", "color_c",
)


def frame_to_row(frame: TLMFrame) -> "dict[str, object]":
    """Flatten one `TLMFrame` into a flat dict keyed by `CSV_FIELDNAMES` --
    pure (no I/O), so this is the function `test_tlm_log.py` exercises
    directly. `None` for any field the frame didn't populate (e.g. `otos_*`
    when `otos_present` is clear) -- the csv module writes a bare `None` as
    an empty field natively, giving the blank cell the acceptance criteria
    ask for with no manual NaN-substitution needed.

    `flag_*` columns are the decoded convenience booleans (every
    presence/status/fault/event bit `TLMFrame` exposes as a `@property`,
    covering every bit the hardware bench gate -- ticket 010 -- reads:
    otos presence/connectivity, per-motor bus connectivity, ack freshness,
    all four faults, all three events, line/color presence); `flags` is
    still carried too, raw, alongside them.
    """
    enc_left, enc_right = frame.enc_left, frame.enc_right
    otos = frame.otos_reading
    pose_x, pose_y, pose_theta = (
        frame.pose if frame.pose is not None else (None, None, None))
    twist_v_x, twist_omega = frame.twist if frame.twist is not None else (None, None)
    line_ch1, line_ch2, line_ch3, line_ch4 = (
        frame.line if frame.line is not None else (None, None, None, None))
    color_r, color_g, color_b, color_c = (
        frame.color if frame.color is not None else (None, None, None, None))

    return {
        "now": frame.t,
        "seq": frame.seq,
        "mode": frame.mode,
        "flags": frame.flags,
        "flag_otos_present": frame.otos_present,
        "flag_otos_connected": frame.otos_connected,
        "flag_active": frame.active,
        "flag_conn_left": frame.conn_left,
        "flag_conn_right": frame.conn_right,
        "flag_ack_fresh": frame.ack_fresh,
        "flag_fault_i2c_safety_net": frame.fault_i2c_safety_net,
        "flag_fault_wedge_latch": frame.fault_wedge_latch,
        "flag_fault_i2c_nak_timeout": frame.fault_i2c_nak_timeout,
        "flag_fault_malformed_frame": frame.fault_malformed_frame,
        "flag_fault_move_timeout": frame.fault_move_timeout,
        "flag_event_deadman_expired": frame.event_deadman_expired,
        "flag_event_boot_ready": frame.event_boot_ready,
        "flag_event_config_applied": frame.event_config_applied,
        "flag_line_present": frame.line_present,
        "flag_color_present": frame.color_present,
        "ack_corr": frame.ack_corr,
        "ack_err": frame.ack_err,
        "enc_left_position": enc_left.position if enc_left is not None else None,
        "enc_left_velocity": enc_left.velocity if enc_left is not None else None,
        "enc_left_time": enc_left.time if enc_left is not None else None,
        "enc_right_position": enc_right.position if enc_right is not None else None,
        "enc_right_velocity": enc_right.velocity if enc_right is not None else None,
        "enc_right_time": enc_right.time if enc_right is not None else None,
        "otos_x": otos.x if otos is not None else None,
        "otos_y": otos.y if otos is not None else None,
        "otos_heading": otos.heading if otos is not None else None,
        "otos_v_x": otos.v_x if otos is not None else None,
        "otos_v_y": otos.v_y if otos is not None else None,
        "otos_omega": otos.omega if otos is not None else None,
        "otos_time": otos.time if otos is not None else None,
        "pose_x": pose_x,
        "pose_y": pose_y,
        "pose_theta": pose_theta,
        "twist_v_x": twist_v_x,
        "twist_omega": twist_omega,
        "line_ch1": line_ch1,
        "line_ch2": line_ch2,
        "line_ch3": line_ch3,
        "line_ch4": line_ch4,
        "color_r": color_r,
        "color_g": color_g,
        "color_b": color_b,
        "color_c": color_c,
    }


def stream_to_csv(source: FrameSource, csv_path: "str | pathlib.Path", duration: float,
                   poll_interval: float = POLL_INTERVAL,
                   progress_interval: float = PROGRESS_INTERVAL) -> int:  # [s] [s] [s]
    """Drain `source.read_pending_binary_tlm_frames()` for `duration`
    seconds, writing one CSV row (`frame_to_row()`) per frame to `csv_path`.
    Prints elapsed-time/row-count progress to stdout every
    `progress_interval` seconds. Returns the total row count written.

    I/O wrapper only -- every row-shape decision lives in `frame_to_row()`,
    kept separate per this ticket's own implementation plan so the pure
    function is what `test_tlm_log.py` exercises, never this loop.
    """
    csv_path = pathlib.Path(csv_path)
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    row_count = 0
    start = time.monotonic()
    next_progress = start + progress_interval
    with open(csv_path, "w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDNAMES)
        writer.writeheader()

        while True:
            now = time.monotonic()
            elapsed = now - start
            if elapsed >= duration:
                break
            for frame in source.read_pending_binary_tlm_frames():
                writer.writerow(frame_to_row(frame))
                row_count += 1
            if now >= next_progress:
                print(f"  t={elapsed:6.1f}s  rows={row_count}")
                next_progress += progress_interval
            time.sleep(poll_interval)

        # Final drain -- catch anything queued between the last poll and the
        # duration deadline.
        for frame in source.read_pending_binary_tlm_frames():
            writer.writerow(frame_to_row(frame))
            row_count += 1

    return row_count


def _args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--relay", action="store_true",
                   help="port is a radio relay dongle (default: direct USB, auto-detected)")
    p.add_argument("--duration", type=float, default=60.0,  # [s]
                   help="capture window")
    p.add_argument("--csv", default=str(DEFAULT_CSV),
                   help=f"output CSV path (default {DEFAULT_CSV})")
    return p.parse_args()


def main() -> int:
    args = _args()
    mode = "relay" if args.relay else None  # None -> SerialConnection auto-detects

    conn = SerialConnection(port=args.port, mode=mode)
    info = conn.connect()
    if info.get("status") != "connected":
        print(f"ERROR: connect failed: {info}")
        return 2
    proto = NezhaProtocol(conn)
    print(f"connected: port={args.port} mode={info.get('mode')}")

    row_count = 0
    try:
        # Drop anything queued before this run started so the capture below
        # only sees fresh pushes.
        proto.read_pending_binary_tlm_frames()

        print(f"capturing {args.duration:.0f}s -> {args.csv}")
        row_count = stream_to_csv(proto, args.csv, args.duration)
    finally:
        # Guaranteed stop: motors must never be left running on an
        # exception or Ctrl-C.
        try:
            proto.stop()
        except Exception:
            pass
        conn.disconnect()

    print(f"wrote {row_count} rows to {args.csv}")
    return 0 if row_count > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
