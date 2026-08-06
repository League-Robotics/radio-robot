#!/usr/bin/env python3
"""command_loss_bench.py -- ticket 135-001's own measurement: how many
inbound commands does the firmware actually lose, over a KNOWN transport at
a KNOWN rate -- replacing the unsourced "~20%" folklore figure that has
been silently informing `pathplan` (and its bench-script siblings') retry
policy since a single undated-provenance 2026-07-27 docstring.

**The folklore, stated precisely** (see this ticket's own body,
`clasi/sprints/135-go-to-navigator-in-the-motion-library/tickets/
001-measure-inbound-command-loss-serial-always-relay-conditional.md`, for
the full framing this repeats -- do not cite the number below as fact
anywhere else, only as the thing this measurement replaces): a widely-cited
"~20% inbound command loss" figure appears in `speed_map.py`,
`square_tour.py`, and `planner_square_tour.py`. It traces to one unsourced
comment written 45 minutes after the 12-deep command ring landed -- no
measurement artifact behind it, and the two surviving copies disagree on
WHERE the loss happens (one blames the DAPLink USB bridge, another the
radio relay), which is itself evidence nobody measured it. This script is
the first to actually measure it, and the first to consult the firmware's
own dropped-command signal (`App::Comms::commandsDroppedCount()`,
telemetry `flags` bit 18, `kFlagFaultCommandsDropped`) at all.

**What this script measures, precisely.** It streams N id-distinct
`move_twist(replace=True)` enqueues at a known rate over ONE transport,
then scans every telemetry frame captured during and just after the run for
each enqueue's own `corr_id` in that frame's ack ring (`TLMFrame.acks`). An
enqueue counts as LOST only if its `corr_id` never appears in ANY captured
frame's ring at all -- an ack that DID arrive but reports a rejection
(``ok=False``, e.g. a boot-window ``ERR_NOT_CONFIGURED``) is NOT "lost," it
is evidence the command reached the firmware and was answered. Both counts
(`acks_observed` = arrived at all, `acks_ok` = arrived AND accepted) are
reported so the two are never conflated.

**Distinguishing link loss from firmware ring overflow.** `App::Comms`
drops (and counts) a well-formed command only when its 12-deep ring is
already full when a new one arrives (`docs/protocol-v5.md` Sec 3.2) -- a
DIFFERENT failure mode from a byte-level loss on the wire itself, with a
different fix (a bigger ring vs. a better transport). The firmware surfaces
this as a LATCHING fault bit -- "this has happened since boot," not a live
per-run count. There is in fact no `commandsDroppedCount` integer anywhere
on the wire, only this one boolean fault bit (`flags` bit 18) -- and it is
not even decoded as a `TLMFrame` property yet (`protocol.py`'s own
module-level comment already flags bits 17/18 as "declared in telemetry.h,
not yet decoded here ... reserved for a future ticket to fill in the gap");
this script is that future ticket, at least for bit 18, reading it directly
off `TLMFrame.flags` (see `_bit18()` below) rather than waiting for a
shared decoded property. This is a real, load-bearing wire gap this
measurement surfaces, not an oversight in this script.

This script reads the bit immediately before and immediately after each
leg: if it is False both before and after, any measured loss on that leg is
link-layer by elimination (the ring never filled); if it is False before
and True after, ring overflow contributed some or all of the observed loss
DURING that leg. Because the bit LATCHES for the whole boot session, a bit
already True at the START of a leg cannot be freshly attributed to that leg
-- reported honestly as "already set before this run" rather than folded
into a manufactured delta.

Two legs run per transport: a **steady** leg (``--rate``/``--count``,
default 20/s x 200 -- chosen to sit far below the ring's own theoretical
overflow threshold: 12 slots draining every ~40ms cycle is roughly 300/s of
sustained headroom, so a clean bit-18 read at this rate is expected, and any
loss observed is attributable to the link alone) and a **burst** leg
(``--burst-count``, default 40, sent back-to-back with no inter-send delay)
specifically aimed at the ring's own overflow boundary. Together they are
what let this script's own summary say WHICH failure mode a given loss
number belongs to, not just that loss occurred.

**Relay leg is CONDITIONAL, never a blocker** (this ticket's own scoping;
`.claude/rules/hardware-bench-testing.md`): this script calls
``uv run mbdeploy list`` itself and only attempts the relay leg if a
ROLE=RADIOBRIDGE row is attached THIS session; if none is found, that leg
is reported SKIPPED (not failed), and the direct-serial numbers stand alone
as the sprint-relevant result.

Every connection this script opens is addressed by UID (``TOVEZ_UID``
below), never a remembered port -- ``mbdeploy list`` is queried live at the
start of the run, never hardcoded. `tovez` is on a stand with wheels off
the ground (safe to spin freely -- `.claude/rules/hardware-bench-testing.md`);
every leg calls ``estop()`` (never ``stop()``) when it finishes.

Usage:
    uv run python src/tests/bench/command_loss_bench.py --help
    uv run python src/tests/bench/command_loss_bench.py --port /dev/cu.usbmodem2121102
    uv run python src/tests/bench/command_loss_bench.py --count 200 --rate 20 --burst-count 40
"""
from __future__ import annotations

import argparse
import pathlib
import re
import subprocess
import sys
import time
from dataclasses import dataclass

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import AckEntry, NezhaProtocol, TLMFrame

TOVEZ_UID = "9906360200052820a8fdb5e413abb276000000006e052820"
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]

DEFAULT_COUNT = 200
DEFAULT_RATE = 20.0          # [Hz] commands/sec -- steady-state leg
DEFAULT_BURST_COUNT = 40     # back-to-back leg, no inter-send delay
ACK_GRACE_S = 1.0            # [s] extra drain after the last send, to catch a lagging ack
_MOVE_ID_BASE = 9700

# telemetry.proto Telemetry.flags bit 18 (kFlagFaultCommandsDropped) --
# App::Comms's 12-deep command ring was full when a well-formed command
# arrived (docs/protocol-v5.md Sec 3.2). NOT yet decoded as a TLMFrame
# property (protocol.py's own comment: "declared in telemetry.h, not yet
# decoded here ... reserved for a future ticket to fill in the gap") --
# read directly off the raw flags word here.
_FLAG_FAULT_COMMANDS_DROPPED = 1 << 18
# bit 9 (kFlagFaultMalformedFrame -- IS already decoded, as
# TLMFrame.fault_malformed_frame, but that property's own "never observed"
# case collapses to False, indistinguishable from "observed and clean".
# Re-read as a raw bit here (via _flag_bit(), the same helper bit 18 uses)
# so this script's own "never observed" ("None") vs "observed clean"
# ("False") distinction is consistent for BOTH bits -- a genuinely corrupt
# inbound line (bit 9) is a DIFFERENT failure mode from a full command ring
# (bit 18): telemetry.proto's own bit-table comment calls bit 9 "a
# wire/link problem (a line arrived corrupt)" vs bit 18's "firmware
# backpressure (a well-formed command arrived faster than one cycle's
# drain could route it)". Checked here as a second, already-available
# attribution signal: a burst-leg loss with BOTH bits clean throughout
# points at neither firmware failure mode -- i.e. the loss happened before
# a frame ever reached App::Comms at all (a dropped/corrupted byte on the
# wire that never assembled into a decodable line, or a host-side send-path
# limitation) -- worth flagging honestly rather than leaving unattributed.
_FLAG_FAULT_MALFORMED_FRAME = 1 << 9

_UID_RE = re.compile(r"\b([0-9a-f]{40,64})\b")
_PORT_RE = re.compile(r"(/dev/[^\s]+)")

_next_move_id_counter = [_MOVE_ID_BASE]


def _next_move_id() -> int:
    _next_move_id_counter[0] += 1
    return _next_move_id_counter[0]


# ---------------------------------------------------------------------------
# Device discovery (mbdeploy list, live -- never a remembered port; see
# .claude/rules/hardware-bench-testing.md)
# ---------------------------------------------------------------------------

@dataclass
class DeviceRow:
    uid: str
    port: "str | None"
    role: str
    name: str


def list_devices() -> "list[DeviceRow]":
    """Live `mbdeploy list` snapshot -- never `probe` (which prints the
    stale registry too, including devices that are not connected). Parsed
    the same tolerant way `motor_survey.py`'s own `enumerate_devices()`
    does: locate the UID and port tokens by regex, then split whatever
    text remains for ROLE/NAME, so column-width drift in `mbdeploy`'s own
    output doesn't break this."""
    result = subprocess.run(["uv", "run", "mbdeploy", "list"], cwd=_REPO_ROOT,
                            capture_output=True, text=True, timeout=180)
    rows: "list[DeviceRow]" = []
    for line in result.stdout.splitlines():
        uid_match = _UID_RE.search(line)
        if not uid_match:
            continue
        port_match = _PORT_RE.search(line)
        tail = line[port_match.end():] if port_match else line[uid_match.end():]
        fields = tail.split()
        role = fields[0] if fields else ""
        name = fields[1] if len(fields) > 1 else ""
        rows.append(DeviceRow(uid=uid_match.group(1),
                              port=port_match.group(1) if port_match else None,
                              role=role, name=name))
    return rows


def find_by_uid(uid: str, rows: "list[DeviceRow]") -> "DeviceRow | None":
    for row in rows:
        if row.uid == uid:
            return row
    return None


def find_relay(rows: "list[DeviceRow]") -> "DeviceRow | None":
    for row in rows:
        if row.role.upper() == "RADIOBRIDGE":
            return row
    return None


# ---------------------------------------------------------------------------
# Measurement
# ---------------------------------------------------------------------------

def _drain(proto: NezhaProtocol) -> "list[TLMFrame]":
    return proto.read_pending_binary_tlm_frames()


def _watch(proto: NezhaProtocol, duration: float) -> "list[TLMFrame]":  # [s]
    """Drain telemetry for `duration` seconds, collecting every frame."""
    frames: "list[TLMFrame]" = []
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        frames.extend(proto.read_pending_binary_tlm_frames())
        time.sleep(0.01)
    return frames


def _find_ack_entry(frames: "list[TLMFrame]", corr_id: int) -> "AckEntry | None":
    """Scan every frame's bounded ack ring for the first entry matching
    `corr_id` -- the union of every captured frame's own ring snapshot,
    not just the latest one."""
    for f in frames:
        for entry in f.acks:
            if entry.corr_id == corr_id:
                return entry
    return None


def _flag_bit(frames: "list[TLMFrame]", mask: int) -> "bool | None":
    """Last known state of the flags bit selected by `mask` across
    `frames` -- `None` if no frame with a decoded `flags` word was
    captured at all (never observed, not "known false")."""
    for f in reversed(frames):
        if f.flags is not None:
            return bool(f.flags & mask)
    return None


def _bit18(frames: "list[TLMFrame]") -> "bool | None":
    """commandsDroppedCount fault bit (ring overflow) -- see this file's
    own module docstring."""
    return _flag_bit(frames, _FLAG_FAULT_COMMANDS_DROPPED)


def _bit9(frames: "list[TLMFrame]") -> "bool | None":
    """Malformed inbound frame fault bit (wire/link corruption) -- a
    DIFFERENT failure mode from bit 18, checked here as a second
    attribution signal (see _FLAG_FAULT_MALFORMED_FRAME's own comment)."""
    return _flag_bit(frames, _FLAG_FAULT_MALFORMED_FRAME)


@dataclass
class LegResult:
    label: str
    rate: float            # [Hz] 0.0 means back-to-back (no inter-send delay)
    count: int
    acked: int              # corr_id appeared in SOME frame's ack ring at all
    acked_ok: int            # ... and that ack reported ok=True
    send_elapsed: float      # [s]
    bit18_before: "bool | None"
    bit18_after: "bool | None"
    bit9_before: "bool | None"
    bit9_after: "bool | None"

    @property
    def loss_pct(self) -> float:
        return 100.0 * (self.count - self.acked) / self.count if self.count else 0.0

    def summary_line(self) -> str:
        rate_str = "back-to-back" if self.rate <= 0 else f"{self.rate:.1f}/s"
        caveat18 = (" (ALREADY SET before this leg -- cannot attribute fresh "
                    "ring overflow to it)" if self.bit18_before else "")
        caveat9 = (" (ALREADY SET before this leg -- cannot attribute fresh "
                  "malformed-frame faults to it)" if self.bit9_before else "")
        attribution = ""
        if self.count - self.acked > 0:
            if not self.bit18_after and not self.bit9_after:
                attribution = (" -- ATTRIBUTION: neither firmware fault bit set "
                              "(ring never full, no malformed frame counted); the "
                              "loss happened before a frame reached App::Comms at "
                              "all (wire/host-send-path, not firmware backpressure)")
            elif self.bit18_after and not self.bit18_before:
                attribution = " -- ATTRIBUTION: firmware command ring overflowed during this leg"
            elif self.bit9_after and not self.bit9_before:
                attribution = " -- ATTRIBUTION: a malformed inbound frame was counted during this leg"
        return (f"[{self.label}] rate={rate_str} N={self.count} "
                f"acks_observed={self.acked}/{self.count} acks_ok={self.acked_ok} "
                f"loss={self.loss_pct:.2f}% ({self.count - self.acked}/{self.count} lost) "
                f"send_window={self.send_elapsed:.2f}s "
                f"commandsDroppedCount(flags bit18): before={self.bit18_before} "
                f"after={self.bit18_after}{caveat18} "
                f"malformedFrame(flags bit9): before={self.bit9_before} "
                f"after={self.bit9_after}{caveat9}{attribution}")


def measure_command_loss(proto: NezhaProtocol, *, label: str, count: int,
                         rate: float) -> LegResult:
    """Stream `count` id-distinct `move_twist(replace=True)` enqueues at
    `rate` commands/sec (0 = back-to-back) over `proto`'s CURRENT
    connection, then scan every captured frame's ack ring for each
    enqueue's own corr_id. See this file's own module docstring for what
    counts as "lost" and how bit 18 is used to attribute it."""
    proto.tlmOn()
    _drain(proto)
    baseline_frames = _watch(proto, 0.3)
    bit18_before = _bit18(baseline_frames)
    bit9_before = _bit9(baseline_frames)

    corrs: "list[int]" = []
    interval = (1.0 / rate) if rate > 0 else 0.0
    all_frames: "list[TLMFrame]" = list(baseline_frames)
    send_started = time.monotonic()
    for _ in range(count):
        corr = proto.move_twist(v_x=60.0, v_y=0.0, omega=0.0,
                                stop_time=2000.0, timeout=2500.0,
                                replace=True, move_id=_next_move_id())
        corrs.append(corr)
        all_frames.extend(proto.read_pending_binary_tlm_frames())
        if interval > 0:
            time.sleep(interval)
    send_elapsed = time.monotonic() - send_started

    all_frames.extend(_watch(proto, ACK_GRACE_S))

    acked = 0
    acked_ok = 0
    for corr in corrs:
        entry = _find_ack_entry(all_frames, corr)
        if entry is not None:
            acked += 1
            if entry.ok:
                acked_ok += 1

    bit18_after = _bit18(all_frames)
    if bit18_after is None:
        bit18_after = bit18_before
    bit9_after = _bit9(all_frames)
    if bit9_after is None:
        bit9_after = bit9_before

    proto.estop()  # stand safety cleanup -- estop(), never stop()

    return LegResult(label=label, rate=rate, count=count, acked=acked,
                     acked_ok=acked_ok, send_elapsed=send_elapsed,
                     bit18_before=bit18_before, bit18_after=bit18_after,
                     bit9_before=bit9_before, bit9_after=bit9_after)


def run_transport_legs(port: str, *, count: int, rate: float,
                       burst_count: int, label_prefix: str) -> "list[LegResult]":
    """Open ONE connection to `port`, run the steady-rate leg then the
    back-to-back burst leg over it, then disconnect. Legs run SEQUENTIALLY
    within one connection (bit18's own latch is a running comparison
    between them), and transports run sequentially across calls to this
    function too -- two open connections streaming into the SAME robot's
    command/ack rings at once would contaminate ring-overflow attribution
    for both. Closing a serial connection resets the robot
    (`.claude/rules/hardware-bench-testing.md`'s HUPCL note) -- accepted
    and even useful here: the NEXT transport's leg starts from a clean,
    just-booted bit18 latch."""
    conn = SerialConnection(port=port)
    info = conn.connect()
    if info.get("status") != "connected":
        print(f"ERROR: connect failed on {port}: {info}")
        return []
    proto = NezhaProtocol(conn)
    print(f"connected: port={port} mode={info.get('mode')} ready={info.get('ready')}")
    results: "list[LegResult]" = []
    try:
        results.append(measure_command_loss(
            proto, label=f"{label_prefix} (steady)", count=count, rate=rate))
        results.append(measure_command_loss(
            proto, label=f"{label_prefix} (burst)", count=burst_count, rate=0.0))
    finally:
        try:
            proto.estop()
        except Exception:
            pass
        conn.disconnect()
    return results


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=None,
                   help="tovez's direct-serial port THIS session -- if omitted, "
                        "resolved live from `mbdeploy list` by UID (never a "
                        "remembered port)")
    p.add_argument("--uid", default=TOVEZ_UID, help="hardware target UID (default: tovez)")
    p.add_argument("--count", type=int, default=DEFAULT_COUNT,
                   help=f"commands sent in the steady-rate leg (default {DEFAULT_COUNT})")
    p.add_argument("--rate", type=float, default=DEFAULT_RATE,
                   help=f"steady-rate leg send rate, commands/sec (default {DEFAULT_RATE})")
    p.add_argument("--burst-count", type=int, default=DEFAULT_BURST_COUNT,
                   help=f"commands sent back-to-back in the burst leg (default {DEFAULT_BURST_COUNT})")
    p.add_argument("--skip-relay", action="store_true",
                   help="skip the relay leg even if `mbdeploy list` shows one attached")
    args = p.parse_args()

    print("resolving devices via `uv run mbdeploy list` (never a remembered port)...")
    rows = list_devices()
    target = find_by_uid(args.uid, rows)
    if target is None:
        print(f"ERROR: UID {args.uid} not found in `mbdeploy list` -- tovez is unplugged. "
              f"Stopping (never falling back to whatever device IS present).")
        return 2
    port = args.port or target.port
    if port is None:
        print(f"ERROR: {args.uid} has no PORT in `mbdeploy list` (row: {target}).")
        return 2
    print(f"target: uid={args.uid[:16]}... port={port} role={target.role} name={target.name}")

    all_results: "list[LegResult]" = []
    all_results.extend(run_transport_legs(
        port, count=args.count, rate=args.rate, burst_count=args.burst_count,
        label_prefix="direct-serial"))

    relay_row = None if args.skip_relay else find_relay(rows)
    relay_note = ""
    if relay_row is not None and relay_row.port is not None:
        print(f"\nrelay device attached this session: {relay_row.name} "
              f"({relay_row.uid[:16]}...) @ {relay_row.port} -- running the relay leg")
        all_results.extend(run_transport_legs(
            relay_row.port, count=args.count, rate=args.rate,
            burst_count=args.burst_count, label_prefix="relay"))
    else:
        relay_note = ("SKIPPED -- no ROLE=RADIOBRIDGE device attached this session "
                      "(`mbdeploy list` showed none); not a failure, per this "
                      "ticket's own relay-conditional scoping."
                      if not args.skip_relay else
                      "SKIPPED -- --skip-relay was passed")
        print(f"\nrelay leg: {relay_note}")

    print("\n" + "=" * 78)
    print("COMMAND LOSS MEASUREMENT SUMMARY")
    for result in all_results:
        print("  " + result.summary_line())
    if relay_note:
        print(f"  [relay] {relay_note}")
    print("=" * 78)

    return 0


if __name__ == "__main__":
    sys.exit(main())
