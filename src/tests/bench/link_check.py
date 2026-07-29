#!/usr/bin/env python3
"""link_check.py -- is the host<->robot link actually reliable?

Answers one question and nothing else: when the robot appears to stall, is
the LINK down, or is the robot simply not moving while the link is fine?

Three independent observables, logged continuously with both host and robot
timestamps:

  1. TELEMETRY CONTINUITY -- every frame carries `seq` (wraps at 128,
     telemetry.proto's `(max) = 127`) and `now` (robot clock [ms]). These
     separate two very different failures:
       * a gap in `seq`          -> frames were LOST (link or host buffer)
       * `seq` contiguous but a
         jump in `now`           -> the FIRMWARE skipped cycles (loop stall)
     NOTE: protocol.py's own `tlm_drop_rate()` masks with 0xFFFF (wrap at
     65536) and so reports a false ~65k-frame gap on every 128-wrap. This
     script does its own mod-128 accounting instead.

  2. PING/PONG -- a cleartext round trip on a fixed cadence, giving an
     independent liveness signal that does not depend on telemetry at all.
     Sent with send_fast() and observed via the `on_recv` hook: `send()`
     CANNOT see these replies (124-005 made _handle_text_line() a no-op),
     it would always time out.

  3. COMMAND ACKS -- each move's enqueue ack (keyed by corr_id) and
     completion ack (keyed by Move.id), with the wait time for each.

Run structure: an idle baseline, then the same move traffic the square tour
generates, then idle again -- so "does motion itself break the link?" is
answered by comparing phases rather than assumed.

    uv run python src/tests/bench/link_check.py --port /dev/cu.usbmodem2121102

The robot is on a stand with wheels off the ground (see
.claude/rules/hardware-bench-testing.md) -- safe to drive. stop() runs in a
finally block so motors are never left running.
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
import time

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol

DEFAULT_PORT = "/dev/cu.usbmodem2121102"
BOOT_WAIT = 5.0          # [s]
SEQ_MODULUS = 128        # telemetry.proto Telemetry.seq (max)=127, wraps
PING_PERIOD = 0.25       # [s]
PING_TIMEOUT = 1.0       # [s] beyond this a ping counts as lost
IDLE_SECONDS = 15.0      # [s] per idle phase

# Mirror planner_square_tour.py's traffic exactly.
LEG = 500.0              # [mm]
CRUISE = 150.0           # [mm/s]
TURN = math.pi / 2       # [rad]
OMEGA = 1.2              # [rad/s]
MOVE_TIMEOUT = 30000.0   # [ms] per-move safety backstop
ACK_TIMEOUT = 1000       # [ms]


class LinkMonitor:
    """Collects telemetry continuity and ping liveness on one connection."""

    def __init__(self, conn, proto):
        self.conn = conn
        self.proto = proto
        self.frames = 0
        self.seq_gaps = []      # (host_t, prev_seq, curr_seq, missing)
        self.clock_jumps = []   # (host_t, prev_now, curr_now, delta)
        self.last_seq = None
        self.last_now = None
        self.last_frame_host_t = None
        self.frame_starved = []  # (host_t, silence) -- no frame for a while
        self.pings = []          # (host_t, rtt or None)
        self._captured = []
        self._original_on_recv = conn.on_recv
        conn.on_recv = self._capture
        self.t0 = time.monotonic()

    def _capture(self, line) -> None:
        if isinstance(line, str):
            self._captured.append((time.monotonic(), line))
        if self._original_on_recv:
            self._original_on_recv(line)

    def close(self) -> None:
        self.conn.on_recv = self._original_on_recv

    def pump(self) -> list:
        """Drain telemetry, updating continuity stats. Returns the frames."""
        frames = self.proto.read_pending_binary_tlm_frames()
        host_t = time.monotonic() - self.t0
        if frames:
            if self.last_frame_host_t is not None:
                silence = host_t - self.last_frame_host_t
                if silence > 1.0:
                    self.frame_starved.append((host_t, silence))
            self.last_frame_host_t = host_t
        for f in frames:
            self.frames += 1
            if f.seq is not None:
                if self.last_seq is not None:
                    gap = (f.seq - self.last_seq) % SEQ_MODULUS
                    if gap != 1:
                        # gap 0 = duplicate; >1 = that many frames missing
                        self.seq_gaps.append(
                            (host_t, self.last_seq, f.seq, max(gap - 1, 0)))
                self.last_seq = f.seq
            if f.t is not None:
                if self.last_now is not None:
                    delta = f.t - self.last_now
                    # One cycle is ~40-47ms; flag anything far beyond that
                    # that is NOT explained by a seq gap.
                    if delta > 200:
                        self.clock_jumps.append(
                            (host_t, self.last_now, f.t, delta))
                self.last_now = f.t
        return frames

    def ping(self) -> None:
        """Fire one PING and resolve it (or time it out) against on_recv."""
        sent = time.monotonic()
        mark = len(self._captured)
        self.conn.send_fast("PING")
        deadline = sent + PING_TIMEOUT
        while time.monotonic() < deadline:
            for at, line in self._captured[mark:]:
                if line.startswith("PONG"):
                    self.pings.append((sent - self.t0, at - sent))
                    return
            self.pump()
            time.sleep(0.01)
        self.pings.append((sent - self.t0, None))

    def idle(self, seconds: float, label: str) -> None:
        print(f"  [{label}] {seconds:.0f}s, pinging every {PING_PERIOD}s...",
              flush=True)
        end = time.monotonic() + seconds
        next_ping = time.monotonic()
        while time.monotonic() < end:
            self.pump()
            if time.monotonic() >= next_ping:
                self.ping()
                next_ping += PING_PERIOD
            time.sleep(0.005)


def moves() -> list[dict]:
    """The square tour's 8 moves, same ids and shapes."""
    out = []
    for i in range(4):
        out.append(dict(v_x=CRUISE, v_y=0.0, omega=0.0, stop_distance=LEG,
                        timeout=MOVE_TIMEOUT, replace=False,
                        move_id=9001 + 2 * i))
        out.append(dict(v_x=0.0, v_y=0.0, omega=OMEGA, stop_angle=TURN,
                        timeout=MOVE_TIMEOUT, replace=False,
                        move_id=9002 + 2 * i))
    return out


def run_moves(mon: LinkMonitor) -> list[dict]:
    """Send each move, wait for its enqueue ack, then its completion ack.

    Strictly one move in flight so every stall is unambiguous: whatever we
    are waiting for is named, and the link is being pinged the whole time.
    """
    records = []
    for spec in moves():
        move_id = spec["move_id"]
        rec = {"move_id": move_id, "enqueue_retries": 0,
               "enqueue_wait": None, "completion_wait": None,
               "pings_during": 0, "pings_lost_during": 0,
               "seq_missing_during": 0}
        t_send = time.monotonic()
        ping_mark, gap_mark = len(mon.pings), len(mon.seq_gaps)

        corr = mon.proto.move_twist(**spec)
        ack = mon.proto.wait_for_ack(corr, timeout=ACK_TIMEOUT)
        while ack is None and rec["enqueue_retries"] < 5:
            rec["enqueue_retries"] += 1
            print(f"    move {move_id}: enqueue retry "
                  f"{rec['enqueue_retries']}", flush=True)
            corr = mon.proto.move_twist(**spec)
            ack = mon.proto.wait_for_ack(corr, timeout=ACK_TIMEOUT)
        rec["enqueue_wait"] = time.monotonic() - t_send
        if ack is None:
            rec["outcome"] = "NEVER ENQUEUED"
            records.append(rec)
            continue

        # Wait for the completion ack (keyed by Move.id), pinging throughout.
        t_wait = time.monotonic()
        deadline = t_wait + (MOVE_TIMEOUT / 1000.0) + 5.0
        next_ping = time.monotonic()
        done = False
        while time.monotonic() < deadline and not done:
            for f in mon.pump():
                for entry in f.acks:
                    if entry.corr_id == move_id:
                        done = True
            if time.monotonic() >= next_ping:
                mon.ping()
                next_ping += PING_PERIOD
            time.sleep(0.005)
        rec["completion_wait"] = time.monotonic() - t_wait
        rec["outcome"] = "complete" if done else "NO COMPLETION ACK"
        rec["pings_during"] = len(mon.pings) - ping_mark
        rec["pings_lost_during"] = sum(
            1 for _, rtt in mon.pings[ping_mark:] if rtt is None)
        rec["seq_missing_during"] = sum(
            miss for _, _, _, miss in mon.seq_gaps[gap_mark:])
        print(f"    move {move_id}: enqueue {rec['enqueue_wait']:.2f}s "
              f"(retries {rec['enqueue_retries']}), "
              f"completion {rec['completion_wait']:.2f}s -> {rec['outcome']}; "
              f"pings {rec['pings_during'] - rec['pings_lost_during']}/"
              f"{rec['pings_during']} ok, {rec['seq_missing_during']} tlm frames lost",
              flush=True)
        records.append(rec)
    return records


def report(mon: LinkMonitor, records: list[dict], elapsed: float) -> int:
    print("\n" + "=" * 68)
    print("LINK REPORT")
    print("=" * 68)

    print(f"\ntelemetry: {mon.frames} frames in {elapsed:.1f}s "
          f"({mon.frames / elapsed:.1f} frames/s)")
    missing = sum(m for _, _, _, m in mon.seq_gaps)
    expected = mon.frames + missing
    loss = (missing / expected * 100.0) if expected else 0.0
    print(f"  seq gaps      : {len(mon.seq_gaps)} events, "
          f"{missing} frames missing ({loss:.2f}% loss)")
    for host_t, prev, curr, miss in mon.seq_gaps[:10]:
        print(f"      t={host_t:6.1f}s  seq {prev:3d} -> {curr:3d} "
              f"({miss} missing)")
    if len(mon.seq_gaps) > 10:
        print(f"      ... and {len(mon.seq_gaps) - 10} more")

    print(f"  robot-clock jumps >200ms: {len(mon.clock_jumps)}")
    for host_t, prev, curr, delta in mon.clock_jumps[:10]:
        print(f"      t={host_t:6.1f}s  now {prev} -> {curr} (+{delta}ms)")
    if len(mon.clock_jumps) > 10:
        print(f"      ... and {len(mon.clock_jumps) - 10} more")

    print(f"  host silence >1s: {len(mon.frame_starved)}")
    for host_t, silence in mon.frame_starved[:10]:
        print(f"      t={host_t:6.1f}s  no frames for {silence:.2f}s")

    ok = [rtt for _, rtt in mon.pings if rtt is not None]
    lost = [t for t, rtt in mon.pings if rtt is None]
    print(f"\nping: {len(mon.pings)} sent, {len(ok)} answered, "
          f"{len(lost)} lost ({len(lost) / max(len(mon.pings), 1) * 100:.1f}%)")
    if ok:
        print(f"  rtt min/median/max: {min(ok) * 1000:.0f} / "
              f"{statistics.median(ok) * 1000:.0f} / {max(ok) * 1000:.0f} ms")
    for t in lost[:10]:
        print(f"      lost at t={t:6.1f}s")
    if len(lost) > 10:
        print(f"      ... and {len(lost) - 10} more")

    print("\nmoves:")
    for r in records:
        print(f"  {r['move_id']}: enqueue {r['enqueue_wait']:.2f}s "
              f"(retries {r['enqueue_retries']}), completion "
              f"{(r['completion_wait'] or 0):.2f}s -> {r['outcome']}")

    print("\n" + "-" * 68)
    print("VERDICT")
    print("-" * 68)
    stalls = [r for r in records
              if (r["completion_wait"] or 0) > 20.0]
    link_clean = loss < 1.0 and len(lost) == 0
    if stalls:
        # The question is not "was any ping lost during the stall" -- with a
        # nonzero baseline loss rate, a 30s stall sends ~120 pings and will
        # always lose a few. It is whether loss during the stall is WORSE
        # than the run's own baseline. Only an elevated rate implicates the
        # link; a baseline rate means the link was up the whole time.
        baseline = len(lost) / max(len(mon.pings), 1)
        during = sum(r["pings_lost_during"] for r in stalls)
        during_total = sum(r["pings_during"] for r in stalls)
        stall_rate = during / max(during_total, 1)
        print(f"{len(stalls)} move(s) took >20s to complete "
              f"(MOVE_TIMEOUT is {MOVE_TIMEOUT / 1000:.0f}s).")
        print(f"  ping loss during stalls: {stall_rate * 100:.1f}%  "
              f"vs run baseline {baseline * 100:.1f}%")
        if stall_rate <= baseline * 1.5:
            print("  => Not elevated: the link was UP throughout the stalls.")
            print("     The robot is sitting in a move that never reaches "
                  "its stop")
            print("     condition and runs out to the MOVE_TIMEOUT backstop.")
            print("     This is a MOTION/stop-condition problem, NOT comms.")
        else:
            print("  => Elevated during stalls: the link is implicated.")
    elif link_clean:
        print("No stalls, no ping loss, no telemetry loss. Link is reliable.")
    else:
        print("No long stalls, but the link showed loss (see above).")
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--relay", action="store_true",
                   help="the port is the RADIO RELAY dongle")
    p.add_argument("--idle", type=float, default=IDLE_SECONDS,
                   help="[s] duration of each idle phase")
    args = p.parse_args()

    conn = SerialConnection(port=args.port,
                            mode="relay" if args.relay else None)
    conn.connect()
    proto = NezhaProtocol(conn)
    print(f"connected on {args.port}; waiting {BOOT_WAIT:.0f}s for boot")
    time.sleep(BOOT_WAIT)
    proto.read_pending_binary_tlm_frames()

    mon = LinkMonitor(conn, proto)
    t_start = time.monotonic()
    records: list[dict] = []
    try:
        print("\nphase 1: idle baseline (no motion)")
        mon.idle(args.idle, "idle-before")
        print("\nphase 2: square-tour move traffic")
        records = run_moves(mon)
        print("\nphase 3: idle after motion")
        mon.idle(args.idle, "idle-after")
    finally:
        try:
            proto.stop()
            proto.estop()
        finally:
            elapsed = time.monotonic() - t_start
            mon.close()
            rc = report(mon, records, elapsed) if records or mon.frames else 1
            conn.disconnect()
    return rc


if __name__ == "__main__":
    sys.exit(main())
