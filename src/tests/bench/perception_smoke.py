#!/usr/bin/env python
"""Perception smoke test -- prove the LINE and COLOUR sensors are alive AND
responsive, on the bench, before any playfield work depends on them.

Two things this checks, because only the second one is real evidence:

1. PRESENCE -- both sensors report on every telemetry frame, with plausible
   per-channel values. Cheap, and passes against a sensor that is stuck.
2. RESPONSE -- the values MOVE when the scene under them changes. A static
   reading proves nothing: a sensor reading a constant is indistinguishable
   from a sensor that has latched, and both look "present" on the wire.

So this is human-in-the-loop by design. It prints live values and asks you
to pass a white card, a dark object, and a coloured card under each sensor,
then scores each channel on the spread it actually saw.

Usage:
    uv run python src/tests/bench/perception_smoke.py --port /dev/cu.usbmodemXXXX
    uv run python src/tests/bench/perception_smoke.py --port /dev/cu.usbmodemRELAY --relay

Take the port from `mbdeploy list` FOR THIS SESSION ONLY -- port numbers move
on every re-enumeration and more than one robot shares the hub. See
.claude/rules/hardware-bench-testing.md.

Background, so a future reader does not re-derive it:

- The LINE sensor reports four raw 0-255 channels. Distinct per-channel
  values are expected (four separate photodiodes), so "all four identical"
  is itself a symptom.
- The COLOUR sensor is an APDS-9960 with NO illumination LED driven by this
  firmware -- it is ambient-only, so it reads what the room lights give it.
  Check the bench lights before blaming the sensor
  (.claude/rules/hardware-bench-testing.md: they turn off on their own).
- The colour channels are scaled against the ADC's TRUE full scale
  (1025 * (256 - ATIME) = 4100 at the shipped ATIME=252), not 65535. Before
  2026-08-08 they were packed with a bare `>> 8`, which left only 16 of 256
  codes usable and pinned r/g/b at 0 -- see Hardware::ColorSensorLeaf::
  fullScale(). If every channel reads 0 again, suspect that packing first.
"""
from __future__ import annotations

import argparse
import statistics as st
import sys
import time

sys.path.insert(0, "src/tests/bench")

from robot_radio.io.serial_conn import SerialConnection  # noqa: E402
from robot_radio.robot.protocol import TLMFrame  # noqa: E402

# A stimulus counts as a RESPONSE only if it moves a channel well clear of
# that channel's OWN measured noise, taken from the untouched baseline phase.
# Absolute thresholds were tried first and are a trap: the line sensor idles
# with ~20 counts of noise, so a flat 20-count bar scored three dead channels
# as "RESPONDED" on jitter alone (2026-08-08). Self-calibrating against the
# baseline avoids guessing a number per sensor, per surface, per light level.
RESPONSE_NOISE_MULTIPLE = 3.0
MIN_ABSOLUTE_LINE = 40   # [counts] floor, in case baseline noise is ~0
MIN_ABSOLUTE_COLOR = 4   # [counts]


class ChannelStats:
    """Running min/max/samples for one channel, with its baseline noise split
    out so a response is judged against that channel's own quiet behaviour."""

    def __init__(self, name: str) -> None:
        self.name = name
        self.values: list[int] = []
        self.baseline: list[int] = []

    def add(self, value: int, baseline: bool = False) -> None:
        self.values.append(value)
        if baseline:
            self.baseline.append(value)

    @staticmethod
    def _spread(xs: list[int]) -> int:
        return (max(xs) - min(xs)) if xs else 0

    @property
    def spread(self) -> int:
        return self._spread(self.values)

    @property
    def noise(self) -> int:
        return self._spread(self.baseline)

    def bar(self, floor: int) -> float:
        return max(float(floor), self.noise * RESPONSE_NOISE_MULTIPLE)

    def responded(self, floor: int) -> bool:
        return bool(self.values) and self.spread >= self.bar(floor)

    def row(self, floor: int) -> str:
        if not self.values:
            return f"  {self.name:<4} NO SAMPLES"
        verdict = "RESPONDED" if self.responded(floor) else "flat"
        return (f"  {self.name:<4} min={min(self.values):3d} max={max(self.values):3d} "
                f"mean={st.mean(self.values):6.1f} spread={self.spread:3d} "
                f"noise={self.noise:3d} needs>={self.bar(floor):5.1f}  {verdict}")


def drain(conn, line_ch, color_ch, baseline: bool = False) -> tuple[tuple | None, tuple | None]:
    """Pull every pending frame into the stats; return the last of each."""
    last_line = last_color = None
    for env in conn.drain_binary_tlm():
        tlm = getattr(env, "tlm", None)
        if tlm is None:
            continue
        frame = TLMFrame.from_pb2(tlm)
        if frame.line_present and frame.line:
            last_line = frame.line
            for i, v in enumerate(frame.line):
                line_ch[i].add(v, baseline)
        if frame.color_present and frame.color:
            last_color = frame.color
            for i, v in enumerate(frame.color):
                color_ch[i].add(v, baseline)
    return last_line, last_color


def prompt_phase(conn, label: str, seconds: float, line_ch, color_ch,
                 baseline: bool = False) -> None:
    """Run one prompted stimulus phase, printing live values as they change."""
    print(f"\n>>> {label}  ({seconds:.0f}s)", flush=True)
    t0 = time.time()
    last_print = 0.0
    while time.time() - t0 < seconds:
        line, color = drain(conn, line_ch, color_ch, baseline)
        now = time.time()
        if now - last_print >= 0.5 and (line or color):
            print(f"    line={line}  color={color}", flush=True)
            last_print = now
        time.sleep(0.02)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="robot USB port, or the RELAY port with --relay")
    ap.add_argument("--relay", action="store_true", help="port is the radio relay dongle")
    ap.add_argument("--phase-seconds", type=float, default=12.0)
    args = ap.parse_args()

    conn = SerialConnection(port=args.port, mode="relay" if args.relay else "direct")
    info = conn.connect(skip_ping=False)
    if args.relay and not (info.get("relay_info") or {}).get("entered_data_plane"):
        print("FAIL: relay never entered the data plane -- talking to its control plane, not the robot")
        return 2

    line_ch = [ChannelStats(n) for n in ("ch1", "ch2", "ch3", "ch4")]
    color_ch = [ChannelStats(n) for n in ("r", "g", "b", "c")]

    try:
        print("VER:", conn.send_cleartext("VER", read_timeout=1000)[:1])
        conn.send_cleartext("TLM:ON")
        time.sleep(0.6)
        conn.drain_binary_tlm()

        prompt_phase(conn, "BASELINE -- leave the sensors completely alone", args.phase_seconds / 2,
                     line_ch, color_ch, baseline=True)
        prompt_phase(conn, "Pass a WHITE card slowly under BOTH sensors", args.phase_seconds,
                     line_ch, color_ch)
        prompt_phase(conn, "Pass something DARK (black tape/card) under BOTH sensors",
                     args.phase_seconds, line_ch, color_ch)
        prompt_phase(conn, "Pass a STRONGLY COLOURED card (red, then green/blue) under the COLOUR sensor",
                     args.phase_seconds, line_ch, color_ch)
    finally:
        conn.disconnect()

    print("\n===== RESULT =====")
    print(f"LINE  ({len(line_ch[0].values)} samples)")
    for ch in line_ch:
        print(ch.row(MIN_ABSOLUTE_LINE))
    print(f"COLOUR ({len(color_ch[0].values)} samples)")
    for ch in color_ch:
        print(ch.row(MIN_ABSOLUTE_COLOR))

    line_ok = sum(1 for c in line_ch if c.responded(MIN_ABSOLUTE_LINE))
    color_ok = sum(1 for c in color_ch if c.responded(MIN_ABSOLUTE_COLOR))
    print(f"\nline channels that responded : {line_ok}/4  (need all 4)")
    print(f"colour channels that responded: {color_ok}/4  (need all 4)")

    ok = line_ok == 4 and color_ok == 4
    print("PASS: both sensors are alive and responsive" if ok else
          "FAIL: at least one channel never moved -- see the rows above")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
