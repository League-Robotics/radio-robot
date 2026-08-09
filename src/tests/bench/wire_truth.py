#!/usr/bin/env python3
"""src/tests/bench/wire_truth.py -- 124-012: physical-layer wire-quality
probe, promoted from the 123-era scratchpad `wire_truth.py` (single-
threaded raw pyserial + demux + decode, no queue/threads -- the
authoritative wire-quality measurement from 123's overnight bench work,
see clasi/sprints/124-.../issues/telemetry-physical-layer-corruption-and-
move-ack-observability.md). Measures BOTH the direct-USB and radio-relay
`!GO` data-plane physical paths under protocol v5 (124-005's `<COMMAND>
[':' data]'\\n'` grammar, COBS keyed on 0x0A, CRC scoped over the command
name -- see `robot_radio.io.wire_codec`'s own module docstring).

**Explicit non-goal (stated once here, and again in the ticket's own
Completion Notes so this cannot be misread later as an open
investigation): this script does NOT root-cause the physical-layer
corruption it measures.** The residual USB corruption is an ACCEPTED
condition, already proven 100% CRC-caught by 123's overnight bench work
(steady-state ~5-11%, plus irregular multi-second ~100%-loss episodes,
`unparseable=0` on every run -- i.e. framing itself never breaks, only
CRC-checked content does). If a run of this script surfaces a theory
about WHY bytes are getting corrupted, that theory does not belong in
this ticket -- file it as a fresh `clasi/issues/` item instead. This
script's only job is: capture, classify, report a number, and check it
against a stated budget.

Usage (ports are bench-specific -- confirm with `mbdeploy list`'s ROLE
column, never a stale example; run BOTH paths back-to-back, comparable
duration, same firmware build, so the two numbers are actually
comparable per the ticket's own acceptance criterion):

    uv run python src/tests/bench/wire_truth.py \\
        --port /dev/cu.usbmodem2121102 --label direct-usb --duration 120

    uv run python src/tests/bench/wire_truth.py \\
        --port /dev/cu.usbmodem2121302 --label relay --relay --duration 120

Prints a human-readable PASS/FAIL report against the stated budget
(below) to stdout; `--json-out` additionally writes a machine-readable
summary ticket 013's own bench gate can read directly rather than
re-parsing stdout.

Methodology (single-threaded, no queue/threads):
-------------------------------------------------
Connecting and running the `!ECHO OFF`/`!MODE RAW250`/`!GO` relay
handshake (or the plain HELLO/PING classify for direct USB) is proven,
already-tested logic -- this script reuses `SerialConnection.connect()`
for exactly that step rather than hand-rolling it a second time. Once
connected, it immediately calls the (internal, but already a documented
bench-script pattern -- see `relay_telemetry_rate.py`'s own
`instrument_malformed_counter()`) `SerialConnection._stop_reader()` to
tear down the background reader thread and its bounded queues, then
takes over `SerialConnection._ser` directly for the actual capture: one
tight loop, `ser.read()` -> `ByteStreamDemuxer.feed()` -> classify, no
thread hand-off, no queue, matching the 123-era probe's own methodology
exactly for the part that matters (the corruption is a property of the
raw bytes on the wire, not of anything `SerialConnection`'s own
threading/queueing might add or hide).

Classification deliberately does NOT reuse `SerialConnection.
_handle_binary_reply()` / `wire_codec.decode_frame()` wholesale --
instead it walks the same primitives (`cobs_decode()`, `crc16_*()`) one
step at a time so it can distinguish, the way 123's methodology did:

  - ``cobs_malformed`` (== the historical "unparseable" bucket): the
    COBS framing itself failed to decode -- the strongest form of
    corruption (byte-level framing, not just content, broke). 123 found
    this is ALWAYS zero; a nonzero count here is a genuine regression,
    not the accepted condition.
  - ``crc_mismatch`` (== the historical "corrupted" bucket): COBS
    decoded cleanly (framing intact) but the trailing CRC-16 does not
    match the payload -- content-level bit corruption, CRC-caught, the
    ~5-11% USB baseline this ticket is measuring against.
  - ``unrecognized_verb``: the line's own ``<COMMAND>`` prefix (before
    the first ``':'``) does not name a registered verb at all --
    corruption landed on the verb bytes themselves rather than the COBS
    body. See "The `#`/`!`/`?` counting bias" below -- this bucket is
    where that bias lives.
  - ``crc_ok_protobuf_invalid``: CRC matched but the payload does not
    parse as a well-formed ``ReplyEnvelope`` -- a CRC collision on
    corrupted bytes (~1/65536 chance per corrupted frame); expected to
    stay at/near zero, tracked for completeness.
  - ``ok``: decoded cleanly end-to-end.
  - ``cleartext``: a registered non-binary verb (``DEVICE``/``PONG``/
    ``ID``/``VER``/``HELLO``/``PING``) -- informational, excluded from
    the corruption denominator (during the capture window this script
    sends nothing itself, so almost every line should be ``TLM``).

The `#`/`!`/`?` counting bias (124-010):
-----------------------------------------
124-010 taught `Core::Comms::dispatchLine()` (firmware, inbound
command-plane) to silently drop -- BEFORE incrementing
`malformedCount_` -- any line whose first byte is `#`, `!`, or `?`
(leaked radio-relay control-plane sigils: `# entering data plane`,
`!MODE RAW250`, the relay's own `?` config query). `SerialConnection.
_handle_wire_line()` has the same carve-out for `#` specifically on the
host's own downlink path. A genuine v5 line always begins with an
uppercase verb letter, so this carve-out can never mask a real
registered-verb collision -- but it means a CORRUPTED frame whose first
byte happens, by chance, to land on one of those three sigil values
(3 of 256 possible byte values, ~1.2%) is invisible to production's own
`malformedCount_`/`malformed_frame_count` counters.

This script does NOT apply that carve-out -- every demuxed line is
classified on its own merits regardless of first byte, so a
sigil-collision lands, correctly, in `unrecognized_verb` here. What it
DOES do is separately tag how many `unrecognized_verb` (and, in
principle, any other corrupted-bucket) hits had a `#`/`!`/`?` first
byte (`sigil_biased` in the report) and print BOTH this script's own
(uncorrected, true) corruption rate AND the rate production's own
counters would show (this script's rate minus the sigil-biased count) --
so a reader comparing this script's number against a live
`malformed_frame_count` read off the robot knows to expect the latter
to run very slightly (~1.2% of the corrupted-frame population) low, not
to mistake that gap for a second corruption mechanism.

The stated loss budget (124-012's own deliverable):
-----------------------------------------------------
CRC catches 100% of corruption -- correctness is never at risk from any
corruption rate (see the issue's own "the link is safe, just lossy"
framing). The budget below is therefore an OPERATIONAL threshold (how
much loss is tolerable before higher-level behavior -- ack
observability, live telemetry for closed-loop control -- visibly
degrades), not a safety one, and it is scored against THIS script's own
uncorrected corruption rate (`crc_mismatch` + `cobs_malformed` +
`unrecognized_verb` + `crc_ok_protobuf_invalid`, divided by total lines
demuxed).

USB direct (`kUsbBudget`): 123's overnight bench work established a
~5-11% steady-state baseline (post-`SerialPort::send()` truncation fix,
commit 9b4ea538) plus irregular multi-second ~100%-loss episodes (worst
observed: ~16s). `kUsbCorruptedRateBudget = 0.15` gives ~1.4-3x margin
over that established baseline -- wide enough to absorb ordinary run-to-
run variance, tight enough that a genuine regression (e.g. a reintroduced
truncation bug) still trips it. `kUsbEpisodeDurationBudget = 5.0` [s] is
chosen against `NezhaProtocol.wait_for_ack()`'s own default 500 ms ack-
wait timeout (`protocol.py`): any episode this long WILL cause a
default-timeout ack wait to observe silence and must be understood as an
expected, budgeted event, not a bug -- but it is set well under the
worst historical 16s so a regression that makes episodes materially
worse (a link degrading further, not just noisy) is still caught.
`kUsbUnparseableBudget` is NOT a percentage -- it is exactly 0, a hard
correctness gate: 123 proved COBS framing itself never breaks, only
CRC-checked content does; any nonzero `cobs_malformed` count is a
genuine regression, not accepted-condition noise.

Radio relay (`kRelayBudget`): NO BASELINE EXISTS YET -- no hardware was
connected to this development environment (`pyocd list` found no probe,
no `/dev/cu.usbmodem*`), so these numbers are a FIRST-PASS PROPOSAL, not
a measured fact, and the stakeholder's own bench run is what actually
fills them in. The proposal's reasoning: the relay path can only ever be
equal to or worse than direct USB, physically -- USB's own nRF UART <->
KL27 DAPLink <-> USB CDC corruption is still present upstream of
whatever the relay's own UART bridge + RF hop adds, so relay corruption
is USB corruption plus (at least) one more physical hop's own noise
floor. `kRelayCorruptedRateBudget = 0.25` gives the relay path roughly
1.7x the USB budget's own ceiling (one added hop, proportionally more
tolerance, not unlimited tolerance). `kRelayEpisodeDurationBudget = 10.0`
[s] doubles the USB episode budget on the same one-more-hop reasoning.
`kRelayUnparseableBudget` stays 0 -- the "framing itself must never
break" correctness bar does not relax just because the path is longer;
if it DOES break on the relay path specifically, that is worth a fresh
`clasi/issues/` entry (RAW250 chunking is a plausible first suspect --
NOT this ticket's job to chase, per the non-goal above), not a widened
budget.

If the stakeholder's real relay measurement lands meaningfully outside
this proposal (either direction), the fix is to edit the `kRelay*`
constants below to the measured reality and note the change in ticket
013 or a follow-up issue -- these are deliberately ordinary module
constants, not something requiring a re-run of this ticket.

Sprint 123 root cause background (do not re-derive -- see
`.clasi/knowledge/` and the issue file): `SerialPort::send()`'s partial-
frame truncation bug (fixed, commit 9b4ea538) took steady-state 20s USB
drop from ~11% to ~1.3%; residual corruption above that is proven real
wire-level bit corruption (constant byte rate, constant delimiter rate,
100% content corruption during an episode), not a host/capture-harness
artifact, not truncation, not queue overflow.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from collections import Counter
from dataclasses import asdict, dataclass, field

from robot_radio.io import wire_commands
from robot_radio.io.serial_conn import SerialConnection
from robot_radio.io.wire_codec import ByteStreamDemuxer, WireFrameError, cobs_decode, crc16_init, crc16_update
from robot_radio.robot.pb2 import envelope_pb2

# --- 124-010 leaked relay control-plane sigils (see module docstring) ---
_CONTROL_PLANE_SIGILS = (b"#", b"!", b"?")

# --- corruption-bucket vocabulary (see module docstring) ----------------
_CORRUPTED_BUCKETS = (
    "cobs_malformed",
    "crc_mismatch",
    "unrecognized_verb",
    "crc_ok_protobuf_invalid",
)

# --- episode detection ----------------------------------------------------
kEpisodeWindow = 1.0          # [s] bucket width for the corruption-rate timeline
kEpisodeRateThreshold = 0.80  # fraction corrupted within a window to call it an "episode"

# --- stated loss budget (124-012's own deliverable -- see module docstring
# "The stated loss budget" section for the full reasoning behind each
# number) ------------------------------------------------------------------
kUsbCorruptedRateBudget = 0.15     # fraction of lines corrupted, steady-state
kUsbEpisodeDurationBudget = 5.0    # [s] longest tolerated near-100%-corrupted episode
kUsbUnparseableBudget = 0          # cobs_malformed count -- hard correctness gate, not a rate

kRelayCorruptedRateBudget = 0.25    # first-pass proposal -- no relay baseline exists yet
kRelayEpisodeDurationBudget = 10.0  # [s] first-pass proposal
kRelayUnparseableBudget = 0         # same hard correctness gate as USB


@dataclass(frozen=True)
class Budget:
    label: str
    corrupted_rate: float
    episode_duration: float  # [s]
    unparseable: int


kUsbBudget = Budget(label="USB direct", corrupted_rate=kUsbCorruptedRateBudget,
                     episode_duration=kUsbEpisodeDurationBudget,
                     unparseable=kUsbUnparseableBudget)
kRelayBudget = Budget(label="radio relay", corrupted_rate=kRelayCorruptedRateBudget,
                       episode_duration=kRelayEpisodeDurationBudget,
                       unparseable=kRelayUnparseableBudget)


def _crc_over_scope(command: bytes, payload: bytes) -> int:
    """Local port of `wire_codec._crc_over_scope()`'s exact composition
    (that helper is module-private, by design -- see this script's own
    module docstring "Methodology": this probe walks the primitives
    itself rather than trusting the higher-level decode path it exists
    to independently verify). Byte-for-byte identical composition:
    `crc16(COMMAND ':' payload)` when `command` is non-empty,
    `crc16(payload)` alone otherwise."""
    crc = crc16_init()
    if command:
        crc = crc16_update(crc, command)
        crc = crc16_update(crc, b":")
    return crc16_update(crc, payload)


def split_wire_line(line: bytes) -> tuple[bytes, bytes]:
    """`<COMMAND>[':' data]` split under protocol v5's uniform grammar
    (124-005: the FIRST ':' ends the command) -- a local copy of
    `serial_conn._split_wire_line()`'s rule, kept independent for the
    same reason `_crc_over_scope()` above is: this probe must not share
    a bug with the code path it is measuring."""
    command, sep, data = line.partition(b":")
    if not sep and command.endswith(b"\r"):
        command = command[:-1]
    return command, data


@dataclass
class LineOutcome:
    bucket: str  # "ok" | "cleartext" | "empty" | one of _CORRUPTED_BUCKETS
    sigil_first_byte: bool
    seq: "int | None" = None
    cycle_period: "int | None" = None  # [us]


def classify_line(line: bytes) -> LineOutcome:
    """Classify one demuxed wire LINE (no trailing `\\n`) into the bucket
    vocabulary described in this module's docstring. Never raises --
    every failure mode a corrupted byte stream can produce lands in a
    named bucket instead."""
    if not line:
        return LineOutcome(bucket="empty", sigil_first_byte=False)

    sigil = line[0:1] in _CONTROL_PLANE_SIGILS
    command_bytes, data = split_wire_line(line)

    try:
        command = command_bytes.decode("ascii")
    except UnicodeDecodeError:
        return LineOutcome(bucket="unrecognized_verb", sigil_first_byte=sigil)

    entry = wire_commands.VERB_BY_NAME.get(command)
    if entry is None:
        return LineOutcome(bucket="unrecognized_verb", sigil_first_byte=sigil)

    if not entry.binary:
        return LineOutcome(bucket="cleartext", sigil_first_byte=sigil)

    try:
        combined = cobs_decode(data, delimiter=0x0A)
    except WireFrameError:
        return LineOutcome(bucket="cobs_malformed", sigil_first_byte=sigil)

    if len(combined) < 2:
        return LineOutcome(bucket="cobs_malformed", sigil_first_byte=sigil)

    payload, crc_bytes = bytes(combined[:-2]), combined[-2:]
    received_crc = crc_bytes[0] | (crc_bytes[1] << 8)
    expected_crc = _crc_over_scope(command_bytes, payload)
    if expected_crc != received_crc:
        return LineOutcome(bucket="crc_mismatch", sigil_first_byte=sigil)

    try:
        envelope = envelope_pb2.ReplyEnvelope.FromString(payload)
    except Exception:
        return LineOutcome(bucket="crc_ok_protobuf_invalid", sigil_first_byte=sigil)
    if envelope.WhichOneof("body") is None:
        return LineOutcome(bucket="crc_ok_protobuf_invalid", sigil_first_byte=sigil)

    seq = None
    cycle_period = None
    if envelope.WhichOneof("body") == "tlm":
        seq = envelope.tlm.seq
        cycle_period = envelope.tlm.cycle_period
    return LineOutcome(bucket="ok", sigil_first_byte=sigil, seq=seq, cycle_period=cycle_period)


@dataclass
class CaptureStats:
    label: str
    port: str
    duration: float  # [s] requested
    actual_duration: float = 0.0  # [s]
    total_bytes: int = 0
    total_lines: int = 0
    buckets: "Counter[str]" = field(default_factory=Counter)
    sigil_biased: "Counter[str]" = field(default_factory=Counter)  # bucket -> count with sigil first byte
    window_series: "list[tuple[float, int, int]]" = field(default_factory=list)  # (windowStart[s], total, corrupted)
    connect_info: dict = field(default_factory=dict)


def find_episodes(window_series: "list[tuple[float, int, int]]", episode_window: float,
                   rate_threshold: float) -> "list[tuple[float, float]]":
    """Contiguous runs of windows whose corrupted/total ratio is at or
    above `rate_threshold` -- the "irregular multi-second episodes where
    100% of frames fail CRC" 123 observed. Returns a list of
    `(start_offset, length)` pairs, both in seconds."""
    episodes: "list[tuple[float, float]]" = []
    current_start: "float | None" = None
    current_len = 0.0
    for window_start, total, corrupted in window_series:
        rate = (corrupted / total) if total else 0.0
        if total > 0 and rate >= rate_threshold:
            if current_start is None:
                current_start = window_start
            current_len += episode_window
        else:
            if current_start is not None:
                episodes.append((current_start, current_len))
            current_start = None
            current_len = 0.0
    if current_start is not None:
        episodes.append((current_start, current_len))
    return episodes


def run_capture(port: str, label: str, duration: float, relay: bool,
                 episode_window: float = kEpisodeWindow) -> CaptureStats:
    """Connect (reusing `SerialConnection`'s proven handshake), hand off
    to a single-threaded raw capture loop (see module docstring
    "Methodology"), and classify every demuxed line for `duration`
    seconds. Never issues a motion command -- `SerialConnection.connect()`
    itself sends only `HELLO`/`PING` (and, for the relay path, the
    control-plane `?`/`!ECHO OFF`/`!MODE RAW250`/`!GO` sequence); nothing
    in this module ever constructs a `Move`/`Config`/`Stop` command. This
    is a read-only diagnostic: it never commands the drivetrain, and
    leaves the robot on the stand exactly as it found it."""
    conn = SerialConnection(port=port, mode="relay" if relay else None)
    try:
        info = conn.connect()
        if info.get("status") not in ("connected", "already_connected"):
            raise RuntimeError(f"connect() failed for {port!r}: {info}")

        # 125-005 (telemetry-emit-policy-rebuild-spec.md Part 7): this probe
        # never issues a motion command (see this function's own docstring),
        # so under the current kAuto default the robot -- parked the whole
        # capture -- would emit nothing to classify at all. `send_fast()`
        # writes directly over `_ser` (no reader-thread dependency), so this
        # is safe to call both before AND after `_stop_reader()` below.
        conn.send_fast("TLM:ON")

        # Hand off from SerialConnection's own background reader thread +
        # bounded queues to a single-threaded raw capture loop -- see
        # module docstring "Methodology" for why.
        conn._stop_reader()
        ser = conn._ser
        if ser is None:
            raise RuntimeError(f"connect() reported success but left no open port for {port!r}")

        stats = CaptureStats(label=label, port=port, duration=duration,
                              connect_info={k: v for k, v in info.items() if k != "lines"})
        demux = ByteStreamDemuxer()
        window_counts: "Counter[str]" = Counter()
        start = time.monotonic()
        window_start = start

        while True:
            now = time.monotonic()
            elapsed = now - start
            if elapsed >= duration:
                break
            try:
                n = ser.in_waiting or 1
                chunk = ser.read(n)
            except Exception as exc:
                print(f"[{label}] read error, ending capture early: {exc}", file=sys.stderr)
                break

            if chunk:
                stats.total_bytes += len(chunk)
                for line in demux.feed(chunk):
                    outcome = classify_line(line)
                    stats.total_lines += 1
                    stats.buckets[outcome.bucket] += 1
                    window_counts[outcome.bucket] += 1
                    if outcome.sigil_first_byte and outcome.bucket in _CORRUPTED_BUCKETS:
                        stats.sigil_biased[outcome.bucket] += 1

            if now - window_start >= episode_window:
                total = sum(window_counts.values())
                corrupted = sum(window_counts.get(b, 0) for b in _CORRUPTED_BUCKETS)
                stats.window_series.append((window_start - start, total, corrupted))
                window_counts = Counter()
                window_start = now

        if window_counts:
            total = sum(window_counts.values())
            corrupted = sum(window_counts.get(b, 0) for b in _CORRUPTED_BUCKETS)
            stats.window_series.append((window_start - start, total, corrupted))

        stats.actual_duration = time.monotonic() - start
        return stats
    finally:
        # Never leaves the port open, and (see docstring) never issued a
        # motion command in the first place, so there is nothing to
        # neutralize on an exception/Ctrl-C -- just release the port.
        # 125-005: undo this function's own send_fast("TLM:ON") -- best
        # effort, `_ser` may already be gone on a read-error early exit.
        try:
            conn.send_fast("TLM:OFF")
        except Exception:
            pass
        conn.disconnect()


def report(stats: CaptureStats, budget: Budget) -> bool:
    total = stats.total_lines
    corrupted = sum(stats.buckets.get(b, 0) for b in _CORRUPTED_BUCKETS)
    unparseable = stats.buckets.get("cobs_malformed", 0)
    corrupted_rate = (corrupted / total) if total else 0.0
    sigil_biased_total = sum(stats.sigil_biased.values())
    corrected_rate = ((corrupted - sigil_biased_total) / total) if total else 0.0

    episodes = find_episodes(stats.window_series, kEpisodeWindow, kEpisodeRateThreshold)
    longest_episode = max((length for _, length in episodes), default=0.0)

    byte_rate = stats.total_bytes / stats.actual_duration if stats.actual_duration else 0.0
    line_rate = total / stats.actual_duration if stats.actual_duration else 0.0

    print()
    print(f"=== {stats.label} ({stats.port}) budget={budget.label} ===")
    print(f"  connect              : {stats.connect_info}")
    print(f"  capture duration     : {stats.actual_duration:.1f} s (requested {stats.duration:.1f} s)")
    print(f"  bytes captured       : {stats.total_bytes} ({byte_rate:.1f} B/s)")
    print(f"  lines demuxed        : {total} ({line_rate:.2f} lines/s)")
    for bucket in ("ok", "cleartext", "empty", *_CORRUPTED_BUCKETS):
        count = stats.buckets.get(bucket, 0)
        pct = (count / total * 100.0) if total else 0.0
        print(f"    {bucket:<24}: {count:6d} ({pct:5.2f}%)")
    print(f"  corrupted (raw)      : {corrupted} ({corrupted_rate * 100:.2f}%) "
          f"[cobs_malformed + crc_mismatch + unrecognized_verb + crc_ok_protobuf_invalid]")
    print(f"  unparseable          : {unparseable} (cobs_malformed -- must be exactly 0)")
    print(f"  sigil-biased (124-010 undercounted-in-production): {sigil_biased_total} "
          f"-- production's own malformedCount_/malformed_frame_count would read "
          f"{corrected_rate * 100:.2f}% (this script's own, uncorrected number is "
          f"{corrupted_rate * 100:.2f}%; see module docstring)")
    print(f"  episodes (>={kEpisodeRateThreshold * 100:.0f}% corrupted, "
          f"{kEpisodeWindow:.0f}s windows): {len(episodes)}, longest {longest_episode:.1f} s")
    for offset, length in episodes:
        print(f"    t={offset:6.1f}s  length={length:.1f}s")

    unparseable_ok = unparseable <= budget.unparseable
    corrupted_ok = corrupted_rate <= budget.corrupted_rate
    episode_ok = longest_episode <= budget.episode_duration
    passed = unparseable_ok and corrupted_ok and episode_ok

    print(f"  BUDGET               : corrupted<={budget.corrupted_rate * 100:.0f}% "
          f"[{'PASS' if corrupted_ok else 'FAIL'}], "
          f"longest episode<={budget.episode_duration:.0f}s "
          f"[{'PASS' if episode_ok else 'FAIL'}], "
          f"unparseable<={budget.unparseable} [{'PASS' if unparseable_ok else 'FAIL'}]")
    print(f"  RESULT               : {'PASS' if passed else 'FAIL'}")
    return passed


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True)
    parser.add_argument("--label", default="capture")
    parser.add_argument("--relay", action="store_true",
                         help="Force the !ECHO OFF/!MODE RAW250/!GO relay handshake "
                              "before capturing (radio-relay path). Omit for direct USB "
                              "-- SerialConnection.connect() still auto-detects a relay "
                              "from its DEVICE:RADIOBRIDGE banner even without this flag; "
                              "pass it to force the handshake even if banner classify is "
                              "flaky.")
    parser.add_argument("--duration", type=float, default=120.0)  # [s]
    parser.add_argument("--episode-window", type=float, default=kEpisodeWindow)  # [s]
    parser.add_argument("--json-out", default=None)
    args = parser.parse_args()

    try:
        stats = run_capture(args.port, args.label, args.duration, args.relay,
                             args.episode_window)
    except Exception as exc:
        print(f"ERROR: capture failed for {args.port!r}: {exc}", file=sys.stderr)
        return 2

    budget = kRelayBudget if args.relay else kUsbBudget
    passed = report(stats, budget)

    if args.json_out:
        payload = asdict(stats)
        payload["buckets"] = dict(stats.buckets)
        payload["sigil_biased"] = dict(stats.sigil_biased)
        payload["budget"] = asdict(budget)
        with open(args.json_out, "w") as f:
            json.dump(payload, f, indent=2, default=str)
        print(f"\nwrote {args.json_out}")

    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
