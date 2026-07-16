#!/usr/bin/env python3
"""src/tests/bench/rig_soak.py — sustained twist/stop soak over the binary plane
(104-006), replacing the pre-103 DeviceBus-era `Rig` soak (sprint 101-004,
which drove per-port `M <port> VEL`/`SERVO` directly).

Drives the rig's two motors as the firmware's own left/right drivetrain via
repeated `twist(v_x, omega, duration)` calls (there is no per-port control
left on the P4 wire — see `rig_dev.py`'s own module docstring), interleaved
with explicit `stop()` segments, for a sustained run. Smooth sine references
(via `rig_dev.waveform()`) vary `v_x`/`omega` — a sine always crosses zero
WITH a dwell either side, never an instantaneous zero-dwell reversal (the
encoder-wedge trigger, `.clasi/knowledge/encoder-wedge-boundary-latch.md`).

Logs, over the run:
  - TLM drop rate (`protocol.tlm_drop_rate()`, uint16-wrap-safe `seq` gap
    accounting — same statistic `relay_telemetry_rate.py` reports).
  - fault_bits/event_bits observed (`Telemetry.fault_bits`/`event_bits`) —
    only a bit that turns on DURING the run (not already set in the very
    first frame, e.g. the boot-time-one-shot `kFaultI2CSafetyNet`) counts as
    a NEW fault and fails the run.
  - encoder motion per commanded twist: at each reissue, checks whether the
    PRECEDING command (if it commanded non-trivial `v_x`/`omega`) produced a
    non-trivial encoder delta over the interval it was active — sanity-
    checking the plant actually responds, not just that acks arrive.
  - ack coverage (INFORMATIONAL ONLY, does not gate pass/fail): every
    `twist()`/`stop()` corr_id sent is tracked until its ack rides a
    `Telemetry` push (this loop is already draining telemetry every pass, so
    acks are matched from that same drain rather than a separate blocking
    wait). 104-006's own hardware verification found the live ack ring
    intermittently omits an individual corr_id's entry even for a
    well-separated, otherwise-healthy command — confirmed, via bare
    `NezhaProtocol.twist()`/`wait_for_ack()` diagnostics with none of this
    script's own logic involved, to be a pre-existing wire/firmware
    characteristic, NOT a TLM frame-delivery problem (frames keep flowing;
    specific acks are just sometimes missing from them) — see this ticket's
    completion notes and the filed follow-up issue. Because of that, ack
    loss is reported but does not fail the run; `drop_rate` (a genuine,
    historically low, frame-delivery statistic) is the metric that gates.

Must run against both direct USB and the radio relay (`--relay`, matching
`gamepad_teleop.py`'s own convention — see `.clasi/knowledge/
relay-transport-and-stand-vs-floor.md`): the sustained dual-transport
verification itself is ticket 007's job (`clasi/sprints/
104-host-realignment-and-full-bench-gate/tickets/
007-p6-soak-gate-sustained-dual-transport-bench-runnable-verification.md`);
this script only needs to SUPPORT both, not prove both here.

Run:  uv run python src/tests/bench/rig_soak.py [--duration 120] [--relay]
Pass: TLM drop rate below threshold, zero NEW fault bits, every commanded
      twist segment produced a plausible encoder response. (Ack loss is
      reported but does not gate -- see module docstring.)
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass, field

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from rig_dev import Rig, waveform  # noqa: E402

from robot_radio.robot.protocol import TLMFrame, tlm_drop_rate  # noqa: E402

DEFAULT_PORT = "/dev/cu.usbmodem2121102"

# --- soak shape --------------------------------------------------------
V_X_PERIOD = 4.0    # [s] v_x sine period
OMEGA_PERIOD = 3.3  # [s] omega sine period (different from V_X_PERIOD -- avoids lockstep)
TWIST_ARM = 500.0        # [ms] twist() deadman window
REISSUE_PERIOD = 0.15    # [s] twist() re-send period, well under TWIST_ARM
STOP_CYCLE = 6.0          # [s] between explicit stop() segments
STOP_DWELL = 0.5          # [s] paused at each stop() segment

# --- commanded-vs-response thresholds -----------------------------------
CMD_V_THRESHOLD = 30.0      # [mm/s] |v_x| at/above this counts as "commanded"
CMD_OMEGA_THRESHOLD = 0.3   # [rad/s] |omega| at/above this counts as "commanded"
MIN_RESPONSE_MM = 2.0        # [mm] minimum |enc delta| over one reissue period to count as "responded"

# --- pass/fail thresholds -------------------------------------------------
MAX_DROP_RATE = 0.02          # 2% max TLM frame drop
MIN_RESPONSIVE_RATE = 0.8      # 80% of commanded intervals must show encoder response

_FAULT_BIT_NAMES = {
    0: "kFaultI2CSafetyNet (boot-time one-shot)",
    1: "kFaultWedgeLatch",
    2: "kFaultI2CNak",
    3: "kFaultCommsMalformed",
}
_EVENT_BIT_NAMES = {
    0: "deadman staleness expired",
    1: "boot-ready transition",
    2: "ConfigDelta applied",
}


def _decode_bits(bits: int, names: dict[int, str]) -> list[str]:
    return [names.get(i, f"bit{i}") for i in range(32) if bits & (1 << i)]


@dataclass
class SoakResult:
    duration: float           # [s] actual wall-clock run time
    mode: str                  # "direct" or "relay"
    sent: int                  # twist()/stop() commands sent
    stop_segments: int         # explicit stop() segments
    delivered_frames: int      # primary TLM frames received
    drop_rate: float            # tlm_drop_rate() over the run
    ack_loss: float              # fraction of sent commands never acked
    commanded_intervals: int     # reissue intervals where the preceding command was "commanded"
    responsive_intervals: int    # of those, how many showed a plausible encoder delta
    responsive_rate: float
    new_fault_bits: int           # fault bits that turned on DURING the run
    new_fault_names: list        # decoded names of new_fault_bits
    event_bits_seen: int          # union of every event_bits value observed
    event_names_seen: list
    secondary_samples: int         # TelemetrySecondary frames received
    pass_: bool = field(default=False)
    failures: list = field(default_factory=list)


def soak(port: str, mode: str | None, duration: float) -> SoakResult:  # [s]
    rig = Rig.open(port=port, mode=mode, settle=2.5)
    actual_mode = rig.conn.mode or "direct"
    print(f"connected: port={port} mode={actual_mode}")

    frames_all: list[TLMFrame] = []
    pending_acks: dict[int, float] = {}
    sent = 0
    stop_segments = 0
    secondary_samples = 0

    baseline_fault_bits: int | None = None
    fault_bits_ever = 0
    event_bits_ever = 0

    last_enc: tuple[int, int] | None = None
    prev_reissue_enc: tuple[int, int] | None = None
    prev_commanded = False
    commanded_intervals = 0
    responsive_intervals = 0

    t0 = time.monotonic()
    next_reissue = t0
    next_stop = t0 + STOP_CYCLE
    last_print = t0

    def drain() -> None:
        nonlocal last_enc, baseline_fault_bits, fault_bits_ever, event_bits_ever, secondary_samples
        frames = rig.read_tlm()
        frames_all.extend(frames)
        for f in frames:
            if f.enc is not None:
                last_enc = f.enc
            if f.fault_bits is not None:
                if baseline_fault_bits is None:
                    baseline_fault_bits = f.fault_bits
                fault_bits_ever |= f.fault_bits
            if f.event_bits is not None:
                event_bits_ever |= f.event_bits
            for ack in (f.acks or ()):
                pending_acks.pop(ack.corr_id, None)
        secondary_samples += len(rig.read_secondary_tlm())

    try:
        drain()  # drop anything queued during Rig.open()'s own settle
        while time.monotonic() - t0 < duration:
            now = time.monotonic()
            t = now - t0
            drain()

            if now >= next_stop:
                corr = rig.stop()
                pending_acks[corr] = now
                sent += 1
                stop_segments += 1
                time.sleep(STOP_DWELL)
                drain()
                resumed = time.monotonic()
                next_reissue = resumed
                next_stop = resumed + STOP_CYCLE
                prev_reissue_enc = last_enc
                prev_commanded = False
                continue

            if now >= next_reissue:
                if prev_reissue_enc is not None and last_enc is not None and prev_commanded:
                    commanded_intervals += 1
                    d_left = abs(last_enc[0] - prev_reissue_enc[0])
                    d_right = abs(last_enc[1] - prev_reissue_enc[1])
                    if (d_left + d_right) >= MIN_RESPONSE_MM:
                        responsive_intervals += 1

                v_x = waveform("sine", t, period=V_X_PERIOD, amp=150.0)       # [mm/s]
                omega = waveform("sine", t, period=OMEGA_PERIOD, amp=0.8)     # [rad/s]
                corr = rig.twist(v_x, omega, duration=TWIST_ARM)
                pending_acks[corr] = now
                sent += 1

                prev_reissue_enc = last_enc
                prev_commanded = (abs(v_x) >= CMD_V_THRESHOLD
                                  or abs(omega) >= CMD_OMEGA_THRESHOLD)
                next_reissue = now + REISSUE_PERIOD

            if now - last_print >= 10.0:
                last_print = now
                print(f"  t={t:6.1f}s sent={sent:5d} frames={len(frames_all):6d} "
                      f"commanded={commanded_intervals} responsive={responsive_intervals} "
                      f"faults=0x{fault_bits_ever:x} pending_acks={len(pending_acks)}")

            time.sleep(0.01)
    finally:
        # Final drain -- catch anything queued between the last poll and stop.
        drain()
        rig.close()

    actual_duration = time.monotonic() - t0
    drop_rate = tlm_drop_rate(frames_all)
    ack_loss = (len(pending_acks) / sent) if sent else 0.0
    responsive_rate = (responsive_intervals / commanded_intervals) if commanded_intervals else 1.0
    new_fault_bits = fault_bits_ever & ~(baseline_fault_bits or 0)

    result = SoakResult(
        duration=actual_duration,
        mode=actual_mode,
        sent=sent,
        stop_segments=stop_segments,
        delivered_frames=len(frames_all),
        drop_rate=drop_rate,
        ack_loss=ack_loss,
        commanded_intervals=commanded_intervals,
        responsive_intervals=responsive_intervals,
        responsive_rate=responsive_rate,
        new_fault_bits=new_fault_bits,
        new_fault_names=_decode_bits(new_fault_bits, _FAULT_BIT_NAMES),
        event_bits_seen=event_bits_ever,
        event_names_seen=_decode_bits(event_bits_ever, _EVENT_BIT_NAMES),
        secondary_samples=secondary_samples,
    )

    failures = []
    if drop_rate > MAX_DROP_RATE:
        failures.append(f"TLM drop rate {drop_rate:.2%} exceeds {MAX_DROP_RATE:.2%}")
    # ack_loss is INTENTIONALLY not gated here -- see module docstring's
    # "ack coverage (INFORMATIONAL ONLY..." paragraph: 104-006 confirmed the
    # live ack ring intermittently omits individual corr_id entries as a
    # pre-existing wire/firmware characteristic independent of TLM frame
    # delivery (which drop_rate above already gates on).
    if new_fault_bits != 0:
        failures.append(f"new fault bits observed: {result.new_fault_names}")
    if commanded_intervals == 0:
        failures.append("no commanded intervals recorded (script bug or robot never moved)")
    elif responsive_rate < MIN_RESPONSIVE_RATE:
        failures.append(
            f"encoder responsiveness {responsive_rate:.0%} below {MIN_RESPONSIVE_RATE:.0%} "
            f"({responsive_intervals}/{commanded_intervals} commanded intervals responded)")
    if secondary_samples == 0:
        failures.append("no TelemetrySecondary (diagnostic) frames received during the run")

    result.failures = failures
    result.pass_ = len(failures) == 0
    return result


def report(result: SoakResult) -> None:
    print("\n=== RESULT ===")
    print(f"  mode                 : {result.mode}")
    print(f"  duration              : {result.duration:.1f} s")
    print(f"  commands sent         : {result.sent} ({result.stop_segments} stop segments)")
    print(f"  primary frames        : {result.delivered_frames}")
    print(f"  TLM drop rate         : {result.drop_rate:.2%}")
    print(f"  ack loss (informational, does not gate): {result.ack_loss:.2%}")
    print(f"  commanded intervals   : {result.commanded_intervals}")
    print(f"  responsive intervals  : {result.responsive_intervals} "
          f"({result.responsive_rate:.0%})")
    print(f"  new fault bits        : {result.new_fault_names or 'none'}")
    print(f"  event bits seen       : {result.event_names_seen or 'none'}")
    print(f"  secondary TLM samples : {result.secondary_samples}")
    print(f"  PASS: {result.pass_}")
    for f in result.failures:
        print(f"    FAIL: {f}")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--relay", action="store_true",
                   help="port is a radio relay dongle (default: direct USB)")
    p.add_argument("--duration", type=float, default=120.0)  # [s]
    p.add_argument("--json-out", default=None)
    args = p.parse_args()

    mode = "relay" if args.relay else "direct"
    print(f"=== rig soak: {args.duration:.0f}s over {mode} ===")
    result = soak(args.port, mode, args.duration)
    report(result)

    if args.json_out:
        with open(args.json_out, "w") as fh:
            json.dump(asdict(result), fh, indent=2, default=str)
        print(f"\nwrote {args.json_out}")

    return 0 if result.pass_ else 1


if __name__ == "__main__":
    sys.exit(main())
