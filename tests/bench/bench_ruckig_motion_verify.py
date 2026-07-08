#!/usr/bin/env python3
"""bench_ruckig_motion_verify.py — on-stand HITL bench verification for
sprint 089 ticket 007 (D/T/TURN/RT motion accuracy + G spot-check).

This is the sprint's REAL acceptance gate (`.claude/rules/hardware-bench-
testing.md`): sim tests (ticket 006) cannot close this sprint on their own
because the sim plant already masks the confirmed `D`/`T` reverse-spin
symptom and cannot fully validate real-world `TURN`/`RT` accuracy either
(086/087's tolerance bars were bench-tuned against real slip/stiction, not
sim's idealized physics).

Robot is mounted on a stand with the wheels off the ground — safe to spin
freely (see `.claude/rules/hardware-bench-testing.md`).

What this script checks, per verb, using the SAME captured TLM/EVT stream:

  1. No reverse encoder motion after `EVT done` (the confirmed bug: ~16mm
     for D, ~23mm for T) — measured as the post-done encoder delta.
  2. Completion-mode: the `EVT done` `reason=` token is the goal's OWN stop
     condition (`dist` for D, `heading` for TURN, `rot` for RT), never the
     `STOP_TIME` safety net (architecture-update.md (089) Decision 10).
  3. D only: peak measured wheel speed vs. the commanded 200 mm/s (the
     confirmed bug was ~292 mm/s on a commanded 200 — a ~46% overshoot;
     this is a qualitative regression check against that known bug, not a
     tight numeric spec, since no precise "existing ratio-governor/PID
     tolerance" number is codified anywhere in the repo).
  4. TURN/RT: heading accuracy against the 086/087 tolerance bars pulled
     directly from `tests/sim/unit/test_motion_commands_arc_turn.py`
     (TURN-from-zero: +-8 deg; RT: +-7 deg) — not re-derived here.
  5. Terminal near-rest replan-chatter characterization (Open Question 7):
     records the encoder/velocity trace in the settle window so the bench
     log's author (this same session) can eyeball repeated corrections.

Safety: widens `DEV WD` for the whole session and ALWAYS sends STOP + DEV
STOP + restores `DEV WD 1000` in a `finally` block. Motors are never left
running. Uses the production `robot_radio` host library (NezhaProtocol /
SerialConnection), not raw pyserial, per this project's own bench
conventions (`.clasi/knowledge/`).

Usage:
    uv run python tests/bench/bench_ruckig_motion_verify.py [--port PORT] [--mode direct|relay]
    uv run python tests/bench/bench_ruckig_motion_verify.py --only D,T,TURN,RT,G
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol, parse_response, parse_tlm

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
RUN_WATCHDOG_WINDOW = 20000    # [ms] widened for the whole bench session (DEV WD max is 60000)
BOOT_WATCHDOG_WINDOW = 1000    # [ms] firmware default — restored on exit
OUT_DIR = Path(__file__).parent / "out"

# TURN-from-zero and RT tolerance bars, pulled verbatim from
# tests/sim/unit/test_motion_commands_arc_turn.py (089-005/006 retune) —
# architecture-update.md Open Question 6 says pull, don't re-derive.
TURN_TOLERANCE_DEG = 8.0
RT_TOLERANCE_DEG = 7.0


def _wrap_deg(d: float) -> float:
    """Wrap a degree value into (-180, 180]."""
    return (d + 180.0) % 360.0 - 180.0


class Capture:
    """Records TLM frames + EVT lines for one bounded-motion run."""

    def __init__(self, label: str) -> None:
        self.label = label
        self.frames: list[tuple[float, "object"]] = []   # (host_monotonic, TLMFrame)
        self.evt_lines: list[tuple[float, str]] = []
        self.done_reason: str | None = None
        self.done_host_t: float | None = None
        self.send_host_t: float | None = None

    def to_dict(self) -> dict:
        return {
            "label": self.label,
            "n_frames": len(self.frames),
            "evt_lines": [ln for _, ln in self.evt_lines],
            "done_reason": self.done_reason,
            "frames": [
                {
                    "host_t": round(ht - (self.send_host_t or ht), 3),
                    "t": f.t,
                    "mode": f.mode,
                    "enc": f.enc,
                    "vel": f.vel,
                    "pose": f.pose,
                    "encpose": f.encpose,
                }
                for ht, f in self.frames
            ],
        }


def drain(conn: SerialConnection, cap: Capture, duration_ms: int, verb: str) -> None:
    """Drain TLM/EVT queues for duration_ms, recording into cap."""
    for line in conn.read_lines(duration=duration_ms):
        r = parse_response(line)
        if r is None:
            continue
        now = time.monotonic()
        if r.tag == "TLM":
            f = parse_tlm(line)
            if f is not None:
                cap.frames.append((now, f))
        elif r.tag == "EVT":
            cap.evt_lines.append((now, line))
            if r.tokens and r.tokens[0] == "done" and (len(r.tokens) < 2 or r.tokens[1] == verb):
                if cap.done_reason is None:
                    cap.done_reason = r.kv.get("reason")
                    cap.done_host_t = now


def run_bounded_verb(conn: SerialConnection, proto: NezhaProtocol, label: str,
                     verb: str, send_line: str, timeout_s: float = 8.0,
                     post_done_s: float = 2.5, reanchor_zero: bool = False) -> Capture:
    """Issue one bounded motion command, capture TLM+EVT through and past
    EVT done. Settles any prior coast, zeroes encoders, streams TLM at 20ms,
    sends the command, then drains until post_done_s after EVT done fires
    (or timeout_s total if it never fires)."""
    proto.send("STOP", read_timeout=200)
    time.sleep(1.3)   # let prior reverse-spin/coast fully settle before the baseline
    if reanchor_zero:
        proto.set_internal_pose(0, 0, 0)
    proto.zero_encoders()
    time.sleep(0.2)
    conn.read_pending_lines()   # drain stale queue
    proto.stream(20)
    time.sleep(0.15)   # let the port settle after STREAM's own reply before the drive command
                       # -- this USB-CDC link intermittently drops/garbles a command sent
                       # immediately after another blocking send() (documented flakiness,
                       # .clasi/knowledge/); observed as a silent RT dispatch failure without
                       # this settle gap.

    cap = Capture(label)
    cap.send_host_t = time.monotonic()
    # Use send() (not send_fast()) for the actual dispatch: it appends a corr-id and
    # retries on a corrupted/garbled command (ERR unknown), giving positive confirmation
    # the command was accepted rather than fire-and-forgetting into a flaky link.
    dispatch_resp = proto.send(send_line, read_timeout=500)
    dispatch_ok = any("OK" in ln for ln in dispatch_resp.get("responses", []))
    if not dispatch_ok:
        print(f"  WARN [{label}] no OK reply to {send_line!r}: {dispatch_resp}")

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        drain(conn, cap, 150, verb)
        if cap.done_host_t is not None and (time.monotonic() - cap.done_host_t) >= post_done_s:
            break
    proto.stream(0)
    proto.send("STOP", read_timeout=200)
    return cap


def analyze_no_reverse(cap: Capture) -> dict:
    """Post-done encoder delta: does either wheel's encoder value move
    OPPOSITE to its OWN established (start-to-done) direction after EVT done
    fires, beyond a small noise floor?

    Each wheel's "forward" sign is derived from its own net travel between
    the run's first frame and the done frame, rather than assumed +1 for
    both wheels -- required for TURN/RT, where the two wheels travel in
    OPPOSITE encoder directions for the same in-place rotation (D/T's two
    wheels happen to travel the same direction, so this reduces to the
    obvious case there too). Returns per-wheel min-delta-from-done and a
    pass/fail verdict."""
    if cap.done_host_t is None or not cap.frames:
        return {"ok": False, "reason": "no EVT done captured / no frames"}

    enc_frames = [(ht, f.enc) for ht, f in cap.frames if f.enc is not None]
    if not enc_frames:
        return {"ok": False, "reason": "no enc= frames available"}

    enc_start = enc_frames[0][1]

    # enc value at (or nearest before) the done timestamp.
    at_done = None
    for ht, enc in enc_frames:
        if ht <= cap.done_host_t + 0.05:
            at_done = enc
    if at_done is None:
        at_done = next((enc for ht, enc in enc_frames if ht >= cap.done_host_t),
                       enc_frames[-1][1])

    sign_l = 1.0 if (at_done[0] - enc_start[0]) >= 0 else -1.0
    sign_r = 1.0 if (at_done[1] - enc_start[1]) >= 0 else -1.0

    worst_l = 0.0
    worst_r = 0.0
    post = [(ht, enc) for ht, enc in enc_frames if ht >= cap.done_host_t]
    for _ht, enc in post:
        dl = (enc[0] - at_done[0]) * sign_l
        dr = (enc[1] - at_done[1]) * sign_r
        worst_l = min(worst_l, dl)
        worst_r = min(worst_r, dr)

    # Noise floor: 1.0 mm — encoder quantization/noise, far below the
    # confirmed 16-23mm bug magnitude.
    NOISE_FLOOR_MM = 1.0
    reverse_mm = max(-worst_l, -worst_r)
    ok = reverse_mm <= NOISE_FLOOR_MM
    return {
        "ok": ok,
        "enc_start": enc_start,
        "enc_at_done": at_done,
        "established_sign_l": sign_l,
        "established_sign_r": sign_r,
        "worst_post_done_delta_l_mm": round(worst_l, 2),
        "worst_post_done_delta_r_mm": round(worst_r, 2),
        "reverse_mm": round(reverse_mm, 2),
        "n_post_done_frames": len(post),
    }


def peak_speed(cap: Capture) -> float | None:
    speeds = [max(abs(f.vel[0]), abs(f.vel[1])) for _ht, f in cap.frames if f.vel is not None]
    return max(speeds) if speeds else None


def heading_at(cap: Capture, near_host_t: float | None) -> int | None:
    """pose= heading (cdeg) from the frame closest to near_host_t (or the
    last frame if near_host_t is None)."""
    frames_with_pose = [(ht, f) for ht, f in cap.frames if f.pose is not None]
    if not frames_with_pose:
        return None
    if near_host_t is None:
        return frames_with_pose[-1][1].pose[2]
    best = min(frames_with_pose, key=lambda hf: abs(hf[0] - near_host_t))
    return best[1].pose[2]


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--mode", default=None, choices=[None, "relay", "direct"])
    p.add_argument("--only", default="D,T,TURN,RT,G",
                   help="comma list of verbs to run")
    p.add_argument("--out", default=str(OUT_DIR / "bench_089_007_results.json"))
    args = p.parse_args()

    only = set(args.only.split(","))
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    conn = SerialConnection(port=args.port, mode=args.mode)
    results: dict = {"port": args.port, "checks": {}}
    proto: NezhaProtocol | None = None
    try:
        info = conn.connect()
        if "error" in info:
            print(f"connect failed: {info['error']}")
            return 2
        print(f"connected: {info}")
        results["connect_info"] = {k: v for k, v in info.items() if k != "lines"}
        proto = NezhaProtocol(conn)

        idr = proto.get_id()
        verr = proto.get_ver()
        print(f"ID: {idr}  VER: {verr}")
        results["id"] = idr
        results["ver"] = verr

        cfg = proto.get_config()
        print(f"CFG: {cfg}")
        results["cfg"] = cfg

        # Safety: widen the serial-silence watchdog for the whole session.
        r = proto.send(f"DEV WD {RUN_WATCHDOG_WINDOW}", read_timeout=300)
        print(f"DEV WD widen: {r}")

        if "D" in only:
            print("\n=== D 200 200 1000 ===")
            cap = run_bounded_verb(conn, proto, "D_200_200_1000", "D",
                                   "D 200 200 1000", timeout_s=6.0, post_done_s=2.5)
            rev = analyze_no_reverse(cap)
            pk = peak_speed(cap)
            print(f"reason={cap.done_reason}  no_reverse={rev}  peak_speed={pk}")
            results["checks"]["D"] = {
                "capture": cap.to_dict(), "no_reverse": rev,
                "peak_speed_mmps": pk, "commanded_speed_mmps": 200,
                "completion_reason": cap.done_reason,
                "completion_ok": cap.done_reason == "dist",
            }

        if "T" in only:
            print("\n=== T 200 200 1000 ===")
            cap = run_bounded_verb(conn, proto, "T_200_200_1000", "T",
                                   "T 200 200 1000", timeout_s=6.0, post_done_s=2.5)
            rev = analyze_no_reverse(cap)
            pk = peak_speed(cap)
            print(f"reason={cap.done_reason}  no_reverse={rev}  peak_speed={pk}")
            results["checks"]["T"] = {
                "capture": cap.to_dict(), "no_reverse": rev,
                "peak_speed_mmps": pk, "commanded_speed_mmps": 200,
                "completion_reason": cap.done_reason,
                # T's OWN natural stop IS time -- reason=time is correct here,
                # unlike D/TURN/RT (Decision 10 scope excludes TIMED/VELOCITY/STREAM).
                "completion_ok": cap.done_reason == "time",
            }

        if "TURN" in only:
            print("\n=== TURN 9000 (from zero) ===")
            cap = run_bounded_verb(conn, proto, "TURN_9000", "TURN",
                                   "TURN 9000", timeout_s=6.0, post_done_s=2.5,
                                   reanchor_zero=True)
            rev = analyze_no_reverse(cap)
            h_done = heading_at(cap, cap.done_host_t)
            h_deg = h_done / 100.0 if h_done is not None else None
            err_deg = abs(h_deg - 90.0) if h_deg is not None else None
            print(f"reason={cap.done_reason}  no_reverse(rot)={rev}  "
                 f"h_at_done={h_deg}  err_deg={err_deg}")
            results["checks"]["TURN"] = {
                "capture": cap.to_dict(), "no_reverse": rev,
                "heading_at_done_deg": h_deg, "target_deg": 90.0,
                "error_deg": err_deg, "tolerance_deg": TURN_TOLERANCE_DEG,
                "accuracy_ok": (err_deg is not None and err_deg <= TURN_TOLERANCE_DEG),
                "completion_reason": cap.done_reason,
                "completion_ok": cap.done_reason == "heading",
            }

        if "RT" in only:
            print("\n=== RT 9000 (relative) ===")
            # Capture the heading immediately before issuing RT as the delta baseline.
            snap = proto.snap()
            h_before = snap.pose[2] / 100.0 if (snap and snap.pose) else None
            cap = run_bounded_verb(conn, proto, "RT_9000", "RT",
                                   "RT 9000", timeout_s=6.0, post_done_s=2.5)
            rev = analyze_no_reverse(cap)
            h_done = heading_at(cap, cap.done_host_t)
            h_deg = h_done / 100.0 if h_done is not None else None
            delta_deg = _wrap_deg(h_deg - h_before) if (h_deg is not None and h_before is not None) else None
            err_deg = abs(delta_deg - 90.0) if delta_deg is not None else None
            print(f"reason={cap.done_reason}  no_reverse(rot)={rev}  "
                 f"h_before={h_before}  h_at_done={h_deg}  delta={delta_deg}  err_deg={err_deg}")
            results["checks"]["RT"] = {
                "capture": cap.to_dict(), "no_reverse": rev,
                "heading_before_deg": h_before, "heading_at_done_deg": h_deg,
                "delta_deg": delta_deg, "target_deg": 90.0,
                "error_deg": err_deg, "tolerance_deg": RT_TOLERANCE_DEG,
                "accuracy_ok": (err_deg is not None and err_deg <= RT_TOLERANCE_DEG),
                "completion_reason": cap.done_reason,
                "completion_ok": cap.done_reason == "rot",
            }

        if "G" in only:
            print("\n=== G 300 0 150 (smoke check) ===")
            proto.set_internal_pose(0, 0, 0)
            cap = run_bounded_verb(conn, proto, "G_300_0_150", "G",
                                   "G 300 0 150", timeout_s=10.0, post_done_s=1.0)
            final_pose = None
            for _ht, f in reversed(cap.frames):
                if f.pose is not None:
                    final_pose = f.pose
                    break
            dispatched = any("OK goto" in ln or True for _ht, ln in cap.evt_lines) or True
            arrived = final_pose is not None and abs(final_pose[0] - 300) < 60 and abs(final_pose[1]) < 60
            emitted_done = cap.done_reason is not None or any(
                "EVT done G" in ln for _ht, ln in cap.evt_lines)
            print(f"final_pose={final_pose}  emitted_done={emitted_done}  arrived~={arrived}")
            results["checks"]["G"] = {
                "capture": cap.to_dict(), "final_pose": final_pose,
                "emitted_done": emitted_done, "arrived_near_target": arrived,
            }

    except KeyboardInterrupt:
        print("\ninterrupted -- stopping motors...")
    finally:
        if proto is not None:
            for c in ("STOP", "DEV STOP", f"DEV WD {BOOT_WATCHDOG_WINDOW}"):
                try:
                    proto.send(c, read_timeout=300)
                except Exception as exc:  # noqa: BLE001
                    print(f"  WARN cleanup {c!r}: {exc}")
            print("  [safety] STOP + DEV STOP + DEV WD 1000 restored.")
        if conn.is_open:
            conn.disconnect()

    with open(args.out, "w") as fh:
        json.dump(results, fh, indent=2, default=str)
    print(f"\nWrote results to {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
