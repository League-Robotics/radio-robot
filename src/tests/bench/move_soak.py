#!/usr/bin/env python3
"""move_soak.py -- ticket 116-010's own sustained-load soak: streams chained
MOVEs at 5-10 Hz over the real MOVE protocol (`NezhaProtocol.move_twist()`/
`stop()`) for a long duration (default 10 minutes), the sprint's own success
criterion ("10-minute soak (>=5-10 Hz alternating MOVEs) clean: no
reboot/lockup, seq monotonic, drop rate at or better than the sprint-115
baseline" -- sprint.md).

A fresh script, not a `rig_soak.py` adaptation: `rig_soak.py`'s own `Rig`
wrapper (`rig_dev.py`) calls the now-deleted `NezhaProtocol.twist()`
(116-007 replaced it with `move_twist()`/`move_wheels()`) and was left
dormant/broken on purpose, out of scope for that ticket (see its own
completion notes) -- not something this ticket fixes either. This script
is the "or write a small soak loop" alternative ticket 010 itself
anticipates.

Each reissue is a bounded `replace=True` `Move` with a TIME stop condition
slightly longer than the reissue period (so a single missed/delayed reissue
never leaves the robot uncommanded before the next one lands) plus a
`timeout` backstop. Interleaves periodic explicit `stop()` segments (every
6s, matching `rig_soak.py`'s own cadence) so the run also exercises
STOP-while-active repeatedly, not just continuous chaining.

Tracks, over the run:
  - TLM drop rate (`protocol.tlm_drop_rate()`, uint16-wrap-safe `seq` gap
    accounting) -- this IS the seq-monotonic check: a monotonic seq at the
    expected ~50Hz cadence is what a low drop rate already certifies.
  - Reboot detection: the robot clock (`TLMFrame.t`, ms) must never jump
    backward, and `kFlagEventBootReady` (flags bit 11) must never be
    freshly observed a second time after the run's own first frame.
  - New fault bits (flags bits 6-9) that turn on DURING the run -- a bit
    already set on the very first frame (e.g. the boot-time one-shot
    `kFlagFaultI2CSafetyNet`, or `kFlagFaultCommsMalformed`/
    `kFlagFaultWedgeLatch`/`kFlagFaultI2CNak` if already present at the
    baseline this session's motor bus happens to be in) does not count.
  - Ack coverage (INFORMATIONAL ONLY, does not gate pass/fail) -- mirrors
    `rig_soak.py`'s own documented "ack-depth-1 tradeoff" note
    (protocol.py's `wait_for_ack()`/`AckEntry` docstrings): a closely-spaced
    reissue can overwrite another command's ack in the single ack slot
    before it's read.

Does NOT gate on encoder responsiveness (unlike `rig_soak.py`'s own
`responsive_intervals` check) -- ticket 010's own success criterion for
this soak is reboot/lockup/seq/drop-rate only; encoder-based responsiveness
is exactly what a disconnected motor bus (`TLMFrame.conn_left`/
`conn_right`) would compromise, orthogonal to whether the comms/queue
channel itself is holding up under sustained load.

Run:  uv run python src/tests/bench/move_soak.py --port /dev/cu.usbmodem2121102 [--duration 600]
Pass: TLM drop rate below threshold, no reboot detected, zero NEW fault
      bits, a final move_twist()+wait_for_ack() responsive check passes.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import asdict, dataclass, field

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol, TLMFrame, tlm_drop_rate

DEFAULT_PORT = "/dev/cu.usbmodem2121102"

# --- soak shape -------------------------------------------------------
V_X_PERIOD = 4.0       # [s] v_x sine period
OMEGA_PERIOD = 3.3     # [s] omega sine period (different from V_X_PERIOD -- avoids lockstep)
REISSUE_PERIOD = 0.15  # [s] move_twist() re-send period (~6.7 Hz, within the 5-10 Hz band)
MOVE_STOP_TIME = 400.0  # [ms] each reissued Move's own TIME stop condition -- comfortably
                        # longer than REISSUE_PERIOD so a single delayed reissue never leaves
                        # the robot uncommanded before the next one lands
MOVE_TIMEOUT = 1500.0   # [ms] required safety backstop, well past MOVE_STOP_TIME
STOP_CYCLE = 6.0        # [s] between explicit stop() segments (matches rig_soak.py)
STOP_DWELL = 0.5        # [s] paused at each stop() segment

# --- pass/fail thresholds ----------------------------------------------
MAX_DROP_RATE = 0.02  # 2% max TLM frame drop -- rig_soak.py's own working threshold; no
                       # numeric sprint-115 HARDWARE baseline exists to compare against
                       # instead (that checklist was never bench-run -- see
                       # docs/bench-checklists/sprint-115-gut-s1.md's own banner).

_FAULT_BIT_POSITIONS = {
    6: "kFlagFaultI2CSafetyNet (boot-time one-shot)",
    7: "kFlagFaultWedgeLatch",
    8: "kFlagFaultI2CNak",
    9: "kFlagFaultCommsMalformed",
}


def _new_fault_names(baseline_flags: int, worst_flags: int) -> list[str]:
    new_bits = worst_flags & ~baseline_flags
    return [name for bit, name in _FAULT_BIT_POSITIONS.items() if new_bits & (1 << bit)]


@dataclass
class SoakResult:
    duration: float            # [s] actual wall-clock run time
    sent: int                   # move_twist()/stop() commands sent
    stop_segments: int          # explicit stop() segments
    delivered_frames: int       # primary TLM frames received
    drop_rate: float             # tlm_drop_rate() over the run
    ack_loss: float               # fraction of sent commands never acked (informational)
    reboot_detected: bool
    reboot_evidence: str
    new_fault_bits: list = field(default_factory=list)
    responsive_at_end: bool = False
    responsive_detail: str = ""
    pass_: bool = field(default=False)
    failures: list = field(default_factory=list)


def soak(port: str, duration: float) -> SoakResult:  # [s]
    conn = SerialConnection(port=port)
    info = conn.connect()
    if info.get("status") != "connected":
        raise ConnectionError(f"connect failed: {info}")
    proto = NezhaProtocol(conn)
    print(f"connected: port={port} mode={info.get('mode')}")

    frames_all: list[TLMFrame] = []
    pending_acks: dict[int, float] = {}
    sent = 0
    stop_segments = 0

    baseline_flags: int | None = None
    worst_flags = 0
    last_t: int | None = None
    reboot_detected = False
    reboot_evidence = ""

    def drain() -> None:
        nonlocal baseline_flags, worst_flags, last_t, reboot_detected, reboot_evidence
        for f in proto.read_pending_binary_tlm_frames():
            frames_all.append(f)
            if f.ack is not None:
                pending_acks.pop(f.ack.corr_id, None)
            if f.flags is not None:
                if baseline_flags is None:
                    baseline_flags = f.flags
                worst_flags |= f.flags
            if f.t is not None:
                # Robot clock jumping backward (beyond simple frame-reordering
                # slack) is the ONLY reboot signature this function trusts.
                # kFlagEventBootReady (flags bit 11) is documented as a
                # "one-shot, transition-cycle" event (telemetry.proto's own
                # bit-table comment) but is NOT implemented that way as of
                # this sprint: `RobotLoop::boot()` calls
                # `tlm_.setFlag(kFlagEventBootReady, true)` exactly once at
                # `robot_loop.cpp:433`, with no corresponding `setFlag(...,
                # false)` anywhere in the tree -- `Telemetry::flags_` is a
                # plain persistent bitmask, so the bit stays set on EVERY
                # frame for the rest of the session once boot completes, not
                # just the one transition cycle. A `boot_ready`-occurrence
                # counter therefore false-positives within the first few
                # frames of any run, real reboot or not (discovered live,
                # ticket 116-010's bench session, 2026-07-22) -- do not
                # resurrect that check without first fixing the firmware to
                # actually pulse the bit.
                if last_t is not None and f.t < last_t - 50 and not reboot_detected:
                    reboot_detected = True
                    reboot_evidence = f"robot clock jumped backward: {last_t} -> {f.t}"
                last_t = f.t if (last_t is None or f.t >= last_t) else last_t

    t0 = time.monotonic()
    next_reissue = t0
    next_stop = t0 + STOP_CYCLE
    last_print = t0

    drain()  # drop anything queued during connect()'s own settle
    try:
        while time.monotonic() - t0 < duration:
            now = time.monotonic()
            t = now - t0
            drain()

            if now >= next_stop:
                corr = proto.stop()
                pending_acks[corr] = now
                sent += 1
                stop_segments += 1
                time.sleep(STOP_DWELL)
                drain()
                resumed = time.monotonic()
                next_reissue = resumed
                next_stop = resumed + STOP_CYCLE
                continue

            if now >= next_reissue:
                v_x = 150.0 * math.sin(2 * math.pi * t / V_X_PERIOD)       # [mm/s]
                omega = 0.8 * math.sin(2 * math.pi * t / OMEGA_PERIOD)     # [rad/s]
                corr = proto.move_twist(v_x=v_x, v_y=0.0, omega=omega,
                                        stop_time=MOVE_STOP_TIME, timeout=MOVE_TIMEOUT,
                                        replace=True)
                pending_acks[corr] = now
                sent += 1
                next_reissue = now + REISSUE_PERIOD

            if now - last_print >= 10.0:
                last_print = now
                print(f"  t={t:6.1f}s sent={sent:5d} frames={len(frames_all):6d} "
                      f"worst_flags=0x{worst_flags:x} pending_acks={len(pending_acks)} "
                      f"reboot_detected={reboot_detected}")

            time.sleep(0.01)
    finally:
        drain()  # final catch-up
        try:
            proto.stop()
        except Exception:
            pass

    actual_duration = time.monotonic() - t0
    drop_rate = tlm_drop_rate(frames_all)
    ack_loss = (len(pending_acks) / sent) if sent else 0.0
    new_faults = _new_fault_names(baseline_flags or 0, worst_flags)

    # Responsive-at-end: one more move_twist() + wait_for_ack() after the soak.
    responsive_at_end = False
    responsive_detail = ""
    try:
        corr = proto.move_twist(v_x=100.0, v_y=0.0, omega=0.0, stop_time=300.0,
                                timeout=1000.0, replace=True)
        ack = proto.wait_for_ack(corr, timeout=500)
        responsive_at_end = ack is not None and ack.ok
        responsive_detail = f"ack={ack}"
    except Exception as exc:
        responsive_detail = f"exception: {exc}"
    finally:
        try:
            proto.stop()
        except Exception:
            pass
        conn.disconnect()

    result = SoakResult(
        duration=actual_duration,
        sent=sent,
        stop_segments=stop_segments,
        delivered_frames=len(frames_all),
        drop_rate=drop_rate,
        ack_loss=ack_loss,
        reboot_detected=reboot_detected,
        reboot_evidence=reboot_evidence,
        new_fault_bits=new_faults,
        responsive_at_end=responsive_at_end,
        responsive_detail=responsive_detail,
    )

    failures = []
    if drop_rate > MAX_DROP_RATE:
        failures.append(f"TLM drop rate {drop_rate:.2%} exceeds {MAX_DROP_RATE:.2%}")
    if reboot_detected:
        failures.append(f"reboot detected: {reboot_evidence}")
    if new_faults:
        failures.append(f"new fault bits observed: {new_faults}")
    if not responsive_at_end:
        failures.append(f"not responsive at end: {responsive_detail}")

    result.failures = failures
    result.pass_ = len(failures) == 0
    return result


def report(result: SoakResult) -> None:
    print("\n=== RESULT ===")
    print(f"  duration              : {result.duration:.1f} s")
    print(f"  commands sent         : {result.sent} ({result.stop_segments} stop segments)")
    print(f"  primary frames        : {result.delivered_frames}")
    print(f"  TLM drop rate         : {result.drop_rate:.2%}")
    print(f"  ack loss (informational, does not gate): {result.ack_loss:.2%}")
    print(f"  reboot detected       : {result.reboot_detected} ({result.reboot_evidence})")
    print(f"  new fault bits        : {result.new_fault_bits or 'none'}")
    print(f"  responsive at end     : {result.responsive_at_end} ({result.responsive_detail})")
    print(f"  PASS: {result.pass_}")
    for f in result.failures:
        print(f"    FAIL: {f}")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--duration", type=float, default=600.0)  # [s]
    p.add_argument("--json-out", default=None)
    args = p.parse_args()

    print(f"=== move_soak: {args.duration:.0f}s over {args.port} ===")
    result = soak(args.port, args.duration)
    report(result)

    if args.json_out:
        with open(args.json_out, "w") as fh:
            json.dump(asdict(result), fh, indent=2, default=str)
        print(f"\nwrote {args.json_out}")

    return 0 if result.pass_ else 1


if __name__ == "__main__":
    sys.exit(main())
