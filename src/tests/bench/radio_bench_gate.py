#!/usr/bin/env python3
"""radio_bench_gate.py -- 124-013: THE SPRINT'S OWN ACCEPTANCE GATE.

Per the stakeholder's directive (verbatim, see the ticket): a standing
bench gate run over the **radio relay** (`!GO` data plane) -- NOT USB --
covering, in one session:

  1. Banner on connect: `DEVICE:` observed at boot on a fresh relay
     connect with no `HELLO` sent, and `connect()` completes WITHOUT
     entering the `_HELLO_CLASSIFY_TIMEOUT_S` fallback.
  2. `HELLO`/`PING`/`ID`/`VER` all answer over the relay.
  3. `move_wheels` starts the wheels; `stop` stops them -- over the relay.
  4. Encoder positions climb in telemetry while the move runs.
  5. Enqueue ack AND completion ack are both observed (the packed `acks`
     ring, ticket 008; confirmed generally fixed by ticket 011).
  6. A `wire_truth`-equivalent quality measurement (ticket 012's own
     script, imported here -- NOT reimplemented) through the relay,
     checked against ticket 012's stated relay loss budget.
  7. `kFaultCommsMalformed` stays clear THROUGHOUT the whole session
     (confirming ticket 010's relay-handshake fix holds under the full
     gate, not just its isolated repro) -- SUC-008.
  8. The 123-006 hardware repro (`move_wheels` whose own serialized bytes
     embed a literal 0x0A) goes 10/10 over the relay -- SUC-007.

Every check prints its own PASS/FAIL line (`Result.record()`, the same
idiom `move_protocol_bench.py`/`wire_truth.py` already use) and the script
exits nonzero if any check failed -- it scores itself; no human log
interpretation required.

Usage (the exact command lines to run, IN ORDER, on the stand)
----------------------------------------------------------------
`--port` is the RELAY's own serial port (the host's connection to the
radio-relay dongle, NOT the robot's direct USB port) -- confirm with
`mbdeploy list`'s ROLE column, never a stale example.

    # 1. Full acceptance gate (relay only). The default 120s wire-quality
    #    capture window matches wire_truth.py's own default -- real
    #    acceptance evidence, not a quick smoke; expect ~2.5-3 minutes
    #    total wall time.
    uv run python src/tests/bench/radio_bench_gate.py \\
        --port /dev/cu.usbmodemRELAY123

    # 2. Faster iteration while debugging THIS script itself (short
    #    wire-quality window -- NOT sufficient to actually clear the
    #    episode-duration budget check meaningfully, informational only
    #    at this duration):
    uv run python src/tests/bench/radio_bench_gate.py \\
        --port /dev/cu.usbmodemRELAY123 --wire-quality-duration 20

    # 3. Optional: also run an informational USB-direct wire-quality leg
    #    for comparison. Per the ticket, USB does NOT satisfy this gate
    #    on its own -- this leg is printed but never affects the exit
    #    code.
    uv run python src/tests/bench/radio_bench_gate.py \\
        --port /dev/cu.usbmodemRELAY123 --usb-port /dev/cu.usbmodemUSB456

Exit code: 0 if every check passed, 1 if any failed, 2 on a connection/
setup error before any scenario could run (e.g. no device on the given
port -- fails fast, never hangs: `serial.Serial.open()` raises
immediately for a nonexistent port).

Design notes for a future reader
---------------------------------
**Why `on_recv`, not `send()`, for HELLO/PING/ID/VER (Phase 2 below):**
124-005 turned `SerialConnection._handle_text_line()` into a no-op (see
that method's own docstring: "None of the four cleartext verbs currently
have a live reader-thread consumer... dropped silently"). `send()`
appends a `#<corr_id>` suffix and blocks on a reply queue that
`_handle_text_line()` never populates for `DEVICE`/`PONG`/`ID`/`VER` --
calling `conn.send("PING")` after `connect()` returns will now ALWAYS
time out (a live gap this ticket's bench run surfaced; worth a follow-up
issue against `cli.py`'s own `_push_calibration()`, which still calls
`conn.send("ID", ...)` and is therefore silently dead code post-124-005 --
out of THIS ticket's scope to fix). The one still-live hook for observing
a cleartext reply after the reader thread has started is `on_recv`
(`_handle_wire_line()` calls it for EVERY decoded line, binary or
cleartext, before the -- possibly no-op -- routing/drop step) -- this
script uses that.

**Why no watchdog widen/restore:** `.claude/rules/hardware-bench-testing.md`
and the stale, pre-124 `src/tests/CLAUDE.md` both describe widening a
`DEV WD` serial-silence watchdog at session start and restoring it in a
`finally` block. That command family does not exist on protocol v4/v5's
wire (`docs/protocol-v4.md` Sec 2.4 -- the only two-verb text rump left
is `HELLO`/`PING`; `SET`/`GET`/`DEV` are gone). There is no ambient
watchdog to widen: every `Move` on this wire carries its OWN bounded
`timeout` safety backstop instead (`.clasi/knowledge/` -- "MOVE is always
bounded"). What this script DOES keep from that rule, because it is
still the operative safety contract regardless of wire version: a `stop()`
call in a `finally` block on every connection this script opens, so the
robot is never left driving on an exception or Ctrl-C. The robot is on a
stand with its wheels off the ground for this entire session (see the
same rule file) -- safe to drive.

**Why `kFaultCommsMalformed` is checked with a SHARED, cross-phase
`SessionTelemetry`, not just once at connect:** the ticket bullet says
"stays clear THROUGHOUT (confirming ticket 010's fix holds under the full
gate, not just its isolated repro)" -- every scenario that watches
telemetry frames below feeds any `fault_malformed_frame` observation into
the SAME `SessionTelemetry` object; the final check aggregates across the
ENTIRE session, not just the dedicated fresh-connect phase.

**The "vacuous PASS" defect and its fix (found by the coordinator's own
dry-run against a nonexistent port, before this ticket closed):** a check
whose only logic is "we never observed the bad thing" is TRUE on a
session that observed NOTHING AT ALL -- a dead port, a failed connect, a
robot that boots, banners, then silently wedges all look identical to
"clean" if absence of evidence is scored as evidence of absence. This is
the exact failure shape sprint 123 shipped a wire bug through: a check
that could not fail was mistaken for a check that passed. Concretely, on
a fresh dry-run with NO device attached at all, the ORIGINAL version of
this script's SUC-008 aggregate printed `[PASS]` -- both connect attempts
had already FAILED immediately above it, `SessionTelemetry` (then a bare
`tracker: list[str]`) had accumulated zero frames and therefore zero
violations, and `len(tracker) == 0` read as "stayed clear." Every check in
this file was subsequently audited for the same shape and fixed where it
applied:
  - The SUC-008 phase-1 check (`scenario_comms_malformed_stays_clear_
    fresh()`) and the SUC-008 session-wide aggregate (`main()`) now both
    require a POSITIVE minimum frame count (`_MIN_FRESH_CONNECT_FRAMES`/
    `_MIN_SESSION_FRAMES_FOR_SUC008`) before "no fault seen" is allowed to
    read as PASS -- see `SessionTelemetry`'s own docstring.
  - The wire-quality budget check (`scenario_wire_quality_vs_budget()`)
    had the SAME shape one level down: `wire_truth.report()`'s own
    `corrupted_rate = 0/total if total else 0.0` scores a zero-line
    capture as comfortably within budget. A caller-side minimum-lines
    floor was added on top (`wire_truth.py` itself, ticket 012, is left
    untouched).
  - Every OTHER check in this file (HELLO/PING/ID/VER matching, both
    move_wheels acks, the active-flag/encoder-climb checks, all ten
    0x0A-repro attempts) was audited too and found to ALREADY require
    positive evidence by construction: each is `X is not None and X.ok`
    or `any(...)`/`bool(...)` over a possibly-empty list, which is FALSE
    -- not True -- on an empty/absent observation. No change was needed
    for those; they fail loudly on a hardware-absent or dead-link run,
    they do not pass vacuously.
  - `main()` also prints an explicit, loud banner if NEITHER phase ever
    reached "connected" at all, saying plainly that no criterion in the
    run can be considered scored -- rather than silently continuing to
    print a wall of (correctly-FAILing, but easy to misread in bulk)
    per-check lines with no framing.

Not reimplemented here: the wire-quality measurement itself. This module
imports `wire_truth` (a sibling file in this same directory -- Python
adds the invoked script's own directory to `sys.path[0]`, so a bare
`import wire_truth` resolves it with no package plumbing) and calls its
`run_capture()`/`report()`/`kRelayBudget` directly.
"""
from __future__ import annotations

import argparse
import struct
import sys
import time

from robot_radio.io import serial_conn
from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol, TLMFrame

ACK_TIMEOUT = 500  # [ms] wait_for_ack() bound for each command's enqueue ack

# Move.id values start well above any corr_id this session's SerialConnection
# will ever assign (a small monotonic counter starting near 1) -- same
# convention as move_protocol_bench.py/test_sim_wire_loopback.py -- so a
# completion ack (keyed by Move.id) is never confused with an enqueue ack
# (keyed by the envelope's own corr_id).
_NEXT_MOVE_ID = 9000


def _next_move_id() -> int:
    global _NEXT_MOVE_ID
    _NEXT_MOVE_ID += 1
    return _NEXT_MOVE_ID


class Result:
    """Same PASS/FAIL-per-check idiom as move_protocol_bench.py's own
    ``Result`` -- deliberately re-declared here rather than imported
    (bench scripts are standalone CLI tools, not a shared library; see
    src/tests/CLAUDE.md)."""

    def __init__(self) -> None:
        self.checks: list[tuple[str, bool, str]] = []

    def record(self, name: str, ok: bool, detail: str = "") -> None:
        self.checks.append((name, ok, detail))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))

    def ok(self) -> bool:
        passed = sum(1 for _, k, _ in self.checks if k)
        print(f"\n==== {passed}/{len(self.checks)} checks passed ====")
        return passed == len(self.checks)


class SessionTelemetry:
    """Shared, cross-phase bookkeeping fed by EVERY telemetry-observing
    helper below (`_drain()`/`_watch()`), across BOTH Phase 1 (fresh
    connect) and Phase 2 (main session) -- two things:

      - `frames_observed`: a running total of every `TLMFrame` actually
        decoded, any phase. This exists ONLY to stop a vacuous PASS: a
        check whose logic is "we never saw the bad thing" (e.g.
        kFaultCommsMalformed staying clear) is trivially true on a
        session that observed NOTHING -- a dead port, a failed connect, a
        wedged boot all look identical to "clean" if absence of evidence
        is scored as evidence of absence. `frames_observed` lets the
        final aggregate check require some minimum POSITIVE evidence
        (real frames genuinely decoded) before a "stayed clear" PASS is
        allowed at all.
      - `malformed_violations`: every frame, any phase, whose
        `fault_malformed_frame` flag (telemetry.proto flags bit 9,
        kFaultCommsMalformed) was set -- SUC-008's own "stays clear
        THROUGHOUT" wording, not just the isolated fresh-connect repro.
    """

    def __init__(self) -> None:
        self.frames_observed = 0
        self.malformed_violations: list[str] = []


def _drain(proto: NezhaProtocol, telemetry: "SessionTelemetry | None" = None,
           tag: str = "") -> list[TLMFrame]:
    frames = proto.read_pending_binary_tlm_frames()
    if telemetry is not None:
        for f in frames:
            telemetry.frames_observed += 1
            if f.fault_malformed_frame:
                telemetry.malformed_violations.append(
                    f"{tag}: seq={f.seq} t={f.t} fault_malformed_frame=True")
    return frames


def _watch(proto: NezhaProtocol, duration: float,  # [s]
           telemetry: "SessionTelemetry | None" = None, tag: str = "") -> list[TLMFrame]:
    """Drain telemetry for `duration` seconds, collecting every frame --
    see `SessionTelemetry`'s own docstring for what `telemetry` accumulates
    and why."""
    frames: list[TLMFrame] = []
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        frames.extend(_drain(proto, telemetry, tag))
        time.sleep(0.01)
    return frames


def _find_completion_ack(frames: list[TLMFrame], move_id: int):
    for f in frames:
        for entry in f.acks:
            if entry.corr_id == move_id:
                return entry
    return None


def _velocity_with_embedded_0x0a(lo: float = 90.0, hi: float = 250.0,  # [mm/s]
                                  step: float = 0.01, delimiter: int = 0x0A) -> float:
    """Computes a genuine, in-range wheel velocity whose IEEE-754 float32
    little-endian encoding contains a literal 0x0A byte -- the exact
    123-006 hazard shape (a MoveWheels envelope whose own serialized
    bytes embed a literal 0x0A), reproduced with a computed value rather
    than a hand-picked magic constant. [90, 250] mm/s stays comfortably
    above every known robot config's motor_deadband (tovez_nocal.json:
    15 mm/s) so the firmware actually drives, not just accepts-and-
    ignores. Deterministic (fixed search order). A local copy of
    `test_sim_wire_loopback.py`'s own helper of the same shape -- kept
    independent rather than imported, matching this test tier's
    established "each file owns its own tiny fixture" convention (the sim
    tier's own file header) and, more importantly, because `src/tests/sim`
    and `src/tests/bench` are two of the three domains this project's own
    CLAUDE.md says are NEVER combined."""
    v = lo
    while v <= hi:
        if delimiter in struct.pack("<f", v):
            return round(v, 2)
        v += step
    raise AssertionError(
        f"no velocity in [{lo}, {hi}] step {step} embeds a 0x{delimiter:02X} byte -- "
        "widen the search range")


# ---------------------------------------------------------------------------
# Phase 1 -- fresh relay connect: banner-on-boot + SUC-008 (zero application
# commands).
# ---------------------------------------------------------------------------

def scenario_banner_on_connect(port: str, result: Result) -> "SerialConnection | None":
    """Bullet 1: `DEVICE:` observed at boot on a fresh relay connect with
    no `HELLO` sent by US, and `connect()` completes WITHOUT entering the
    `_HELLO_CLASSIFY_TIMEOUT_S` fallback (the banner arrives on its own,
    not from `_banner_classify()` exhausting its retry budget with no
    banner ever seen).

    `_banner_classify()` unconditionally sends its OWN `HELLO` at t=0
    regardless (that is how it works -- see its own docstring); the
    distinction this check makes is TIMING: a banner already in flight at
    boot (DTR-assert reset) is captured within a small fraction of the
    budget, while the fallback path (no banner ever arrives) consumes the
    ENTIRE `_HELLO_CLASSIFY_TIMEOUT_S` window before giving up. This
    script wraps the bound method with a timing shim for the duration of
    ONE `connect()` call -- the same class of "reach into
    SerialConnection's own internals for a bench diagnostic" already
    established by `wire_truth.py` (`conn._stop_reader()`/`conn._ser`).

    Returns the (possibly still-open) `SerialConnection` so the caller can
    run the SUC-008 zero-commands check on the SAME fresh connection
    before disconnecting -- both checks need "nothing else has been sent
    yet" to mean the same thing.
    """
    conn = SerialConnection(port=port, mode="relay")
    banner_timing: dict = {}
    original_banner_classify = conn._banner_classify

    def _timed_banner_classify(timeout_s: float = serial_conn._HELLO_CLASSIFY_TIMEOUT_S):
        t0 = time.monotonic()
        role, banner_line = original_banner_classify(timeout_s)
        banner_timing["elapsed"] = time.monotonic() - t0
        banner_timing["banner_line"] = banner_line
        banner_timing["role"] = role
        return role, banner_line

    conn._banner_classify = _timed_banner_classify

    try:
        info = conn.connect()
    except Exception as exc:
        result.record("fresh relay connect() succeeded", False, f"raised {exc!r}")
        return None

    connected = info.get("status") == "connected"
    result.record("fresh relay connect() succeeded", connected, f"info={info}")
    if not connected:
        return conn

    announce = info.get("announcement")
    banner_line = banner_timing.get("banner_line", "")
    result.record("DEVICE: banner observed at boot (no application HELLO required)",
                  bool(banner_line) and announce is not None,
                  f"banner_line={banner_line!r} announcement={announce}")

    elapsed = banner_timing.get("elapsed", serial_conn._HELLO_CLASSIFY_TIMEOUT_S)
    budget = serial_conn._HELLO_CLASSIFY_TIMEOUT_S
    result.record("connect() did NOT enter the _HELLO_CLASSIFY_TIMEOUT_S fallback",
                  elapsed < budget * 0.5,
                  f"banner-classify elapsed={elapsed:.3f}s of {budget:.1f}s budget "
                  "(fallback = exhausting the full budget with no banner)")

    result.record("relay role classified correctly (DEVICE:RADIOBRIDGE)",
                  banner_timing.get("role") == "relay", f"role={banner_timing.get('role')!r}")
    return conn


_MIN_FRESH_CONNECT_FRAMES = 5  # positive-evidence floor -- see module docstring "vacuous PASS" note


def scenario_comms_malformed_stays_clear_fresh(conn: SerialConnection, result: Result,
                                                telemetry: "SessionTelemetry", duration: float = 3.0,  # [s]
                                                ) -> None:
    """SUC-008: `kFaultCommsMalformed` stays clear through a fresh relay
    connect with ZERO application commands. Must run on the SAME
    connection `scenario_banner_on_connect()` just opened, before
    anything else is sent -- this is what makes "zero application
    commands" true (the relay-handshake sequence `connect()` itself just
    ran -- `?`/`!ECHO OFF`/`!MODE RAW250`/`!GO` plus the classify `HELLO`
    -- is the exact leaked-control-plane-sigil hazard 124-010 fixed; it is
    not an application command).

    Requires POSITIVE evidence (at least `_MIN_FRESH_CONNECT_FRAMES`
    genuine telemetry frames) before "no fault seen" is allowed to PASS --
    a connect that succeeds but then goes silent (telemetry never
    actually flows -- a known "banner, then nothing" bench failure
    signature) must NOT read as a clean SUC-008 pass just because nothing
    bad was ever observed either."""
    proto = NezhaProtocol(conn)
    frames = _watch(proto, duration, telemetry, "fresh_connect_comms_malformed")
    saw_fault = any(f.fault_malformed_frame for f in frames)
    observed_enough = len(frames) >= _MIN_FRESH_CONNECT_FRAMES
    if not observed_enough:
        result.record("kFaultCommsMalformed clear on a fresh relay connect, zero application "
                      "commands sent (SUC-008)", False,
                      f"INCONCLUSIVE: only {len(frames)} telemetry frames observed over "
                      f"{duration:.1f}s (need >= {_MIN_FRESH_CONNECT_FRAMES}) -- connect "
                      "succeeded but telemetry never really flowed; a clean-looking absence "
                      "of faults here proves nothing")
        return
    result.record("kFaultCommsMalformed clear on a fresh relay connect, zero application "
                  "commands sent (SUC-008)", not saw_fault,
                  f"{len(frames)} telemetry frames observed over {duration:.1f}s; "
                  f"fault_malformed_frame seen={saw_fault}")


# ---------------------------------------------------------------------------
# Phase 2 -- main relay session.
# ---------------------------------------------------------------------------

def scenario_hello_ping_id_ver(conn: SerialConnection, result: Result) -> None:
    """Bullet 2: `HELLO`/`PING`/`ID`/`VER` all answer over the relay --
    `DEVICE:`/`PONG:`/`ID:`/`VER:` respectively (`comms.cpp`
    `dispatchCleartext()`). Uses `send_fast()` + the `on_recv` hook, NOT
    `send()` -- see this module's own docstring "Design notes" for why
    `send()` cannot observe these replies post-124-005."""
    captured: list[str] = []
    original_on_recv = conn.on_recv

    def _capture(line) -> None:
        if isinstance(line, str):
            captured.append(line)
        if original_on_recv:
            original_on_recv(line)

    conn.on_recv = _capture
    try:
        for verb, expect_prefix in (("HELLO", "DEVICE:"), ("PING", "PONG:"),
                                     ("ID", "ID:"), ("VER", "VER:")):
            captured.clear()
            conn.send_fast(verb)
            deadline = time.monotonic() + 1.0
            matched = None
            while time.monotonic() < deadline and matched is None:
                for line in captured:
                    if line.startswith(expect_prefix):
                        matched = line
                        break
                if matched is None:
                    time.sleep(0.02)
            result.record(f"{verb} -> {expect_prefix}... answers over the relay",
                          matched is not None, f"captured={captured!r}")
    finally:
        conn.on_recv = original_on_recv


def scenario_move_wheels_start_stop_and_climb(proto: NezhaProtocol, result: Result,
                                               telemetry: "SessionTelemetry") -> None:
    """Bullets 3+4: `move_wheels` starts the wheels, encoder positions
    climb in telemetry while it runs, and `stop` stops them -- one
    continuous story on the real drivetrain (wheels off the ground, see
    hardware-bench-testing.md -- safe to drive freely).

    Every check below already requires POSITIVE evidence to PASS (an
    empty/absent frame list makes `any(...)`/`bool(...)` False, i.e.
    FAIL, never a vacuous PASS) -- audited per the coordinator's review of
    this file; see module docstring "vacuous PASS" note."""
    _drain(proto, telemetry, "start_stop:drain")
    before_frames = _watch(proto, 0.15, telemetry, "start_stop:before")
    enc_before = None
    for f in reversed(before_frames):
        if f.enc is not None:
            enc_before = f.enc
            break

    move_id = _next_move_id()
    corr = proto.move_wheels(v_left=150.0, v_right=150.0, stop_time=3000.0, timeout=4000.0,
                              replace=True, move_id=move_id)
    ack = proto.wait_for_ack(corr, timeout=ACK_TIMEOUT)
    result.record("move_wheels enqueue ack ok (starts the wheels)",
                  ack is not None and ack.ok, f"ack={ack}")

    during_frames = _watch(proto, 1.3, telemetry, "start_stop:during")
    active_seen = any(f.active for f in during_frames)
    result.record("drivetrain active flag observed True while the move runs",
                  active_seen, f"{len(during_frames)} frames observed")

    enc_after = enc_before
    for f in reversed(during_frames):
        if f.enc is not None:
            enc_after = f.enc
            break

    climbed = False
    detail = f"before={enc_before} after={enc_after}"
    if enc_before is not None and enc_after is not None:
        d_left = enc_after[0] - enc_before[0]    # [mm]
        d_right = enc_after[1] - enc_before[1]   # [mm]
        # ~150 mm/s for ~1.3s -> ~195mm expected; 20mm is a generous floor
        # that still catches "the wheels never actually turned".
        climbed = d_left > 20.0 and d_right > 20.0
        detail = f"before={enc_before} after={enc_after} d_left={d_left:.1f}mm d_right={d_right:.1f}mm"
    result.record("encoder positions climbed in telemetry while the move ran", climbed, detail)

    corr_stop = proto.stop()
    ack_stop = proto.wait_for_ack(corr_stop, timeout=ACK_TIMEOUT)
    result.record("stop enqueue ack ok (stops the wheels)",
                  ack_stop is not None and ack_stop.ok, f"ack={ack_stop}")

    settle_frames = _watch(proto, 0.6, telemetry, "start_stop:after_stop")
    stopped = bool(settle_frames) and not settle_frames[-1].active
    result.record("drivetrain active flag clears after stop",
                  stopped,
                  f"last active={settle_frames[-1].active if settle_frames else None}")


def scenario_enqueue_and_completion_acks(proto: NezhaProtocol, result: Result,
                                          telemetry: "SessionTelemetry") -> None:
    """Bullet 5: enqueue ack AND completion ack are both observed, over
    the relay, for the SAME Move -- a short, self-completing MOVE (no
    interrupting stop()) so both ack outcomes are unambiguous, mirroring
    `test_sim_wire_loopback.py`'s own shape off-hardware. Both checks
    below already require a matched, `.ok` entry to PASS -- `None`
    (nothing observed) fails, never passes vacuously."""
    _drain(proto, telemetry, "acks:drain")
    move_id = _next_move_id()
    corr = proto.move_wheels(v_left=120.0, v_right=120.0, stop_time=500.0, timeout=1500.0,
                              replace=True, move_id=move_id)
    ack = proto.wait_for_ack(corr, timeout=ACK_TIMEOUT)
    result.record("enqueue ack observed (corr_id-keyed)", ack is not None and ack.ok, f"ack={ack}")

    frames = _watch(proto, 1.2, telemetry, "enqueue_and_completion_acks")
    completion = _find_completion_ack(frames, move_id)
    result.record("completion ack observed (Move.id-keyed)",
                  completion is not None and completion.ok, f"completion={completion}")
    proto.stop()


def scenario_embedded_0x0a_10_of_10(proto: NezhaProtocol, result: Result,
                                     telemetry: "SessionTelemetry") -> None:
    """SUC-007: the 123-006 repro shape -- a `move_wheels` envelope whose
    own serialized bytes embed a literal 0x0A -- goes 10/10 over the
    relay (005 confirmed this over USB; this confirms it over the actual
    acceptance transport). Each per-attempt check requires a matched,
    `.ok` ack AND completion entry -- an empty/no-connection session
    scores 0/10 (FAIL), never a vacuous 10/10."""
    v = _velocity_with_embedded_0x0a()
    result.record("computed velocity genuinely embeds a literal 0x0A byte (123-006 hazard shape)",
                  0x0A in struct.pack("<f", v), f"v={v}")

    passes = 0
    for i in range(1, 11):
        _drain(proto, telemetry, f"embedded_0x0a:{i}:drain")
        move_id = _next_move_id()
        corr = proto.move_wheels(v_left=v, v_right=v, stop_time=300.0, timeout=1200.0,
                                  replace=True, move_id=move_id)
        ack = proto.wait_for_ack(corr, timeout=ACK_TIMEOUT)
        frames = _watch(proto, 0.6, telemetry, f"embedded_0x0a:{i}")
        completion = _find_completion_ack(frames, move_id)
        ok = ack is not None and ack.ok and completion is not None and completion.ok
        if ok:
            passes += 1
        result.record(f"0x0A-embedding move_wheels repro {i}/10 (enqueue+completion ack ok)",
                      ok, f"v={v} ack={ack} completion={completion}")
    result.record("0x0A-embedding move_wheels repro went 10/10 over the relay (SUC-007)",
                  passes == 10, f"{passes}/10 passed")
    proto.stop()


# ---------------------------------------------------------------------------
# Phase 3 -- wire-quality vs budget (bullet 6), via ticket 012's own script.
# ---------------------------------------------------------------------------

def scenario_wire_quality_vs_budget(port: str, duration: float, result: Result,  # [s]
                                     json_out: "str | None" = None) -> None:
    """Bullet 6: a `wire_truth`-equivalent quality measurement through the
    relay, checked against ticket 012's stated relay loss budget. Imports
    and calls `wire_truth.run_capture()`/`report()` DIRECTLY (see module
    docstring) -- never reimplements the corruption classification or the
    budget constants.

    `wire_truth.report()`'s own budget arithmetic is a second instance of
    the same vacuous-PASS shape this module's `SessionTelemetry` exists to
    stop elsewhere: on a ZERO-line capture (dead port, failed connect,
    nothing ever demuxed), `corrupted_rate = 0/total if total else 0.0`
    computes 0.0 -- comfortably "within budget" -- and every sub-check
    reports PASS even though nothing was measured. This wrapper adds an
    explicit minimum-lines floor on TOP of `wire_truth`'s own report
    before trusting its verdict; `wire_truth.py` itself (ticket 012,
    already closed) is left untouched -- this is a caller-side guard, not
    a change to that script's own budget logic."""
    import wire_truth

    try:
        stats = wire_truth.run_capture(port, "radio-bench-gate-013", duration, True)
    except Exception as exc:
        result.record("relay wire-quality capture completed", False, f"raised {exc!r}")
        return

    passed = wire_truth.report(stats, wire_truth.kRelayBudget)

    min_lines = max(10, int(duration * 2))  # conservative floor -- see docstring above
    if stats.total_lines < min_lines:
        result.record("relay wire-quality within ticket 012's stated budget "
                      "(wire_truth.kRelayBudget)", False,
                      f"INCONCLUSIVE: only {stats.total_lines} lines demuxed over "
                      f"{stats.actual_duration:.1f}s (need >= {min_lines}) -- a budget PASS "
                      "against near-zero traffic proves nothing, see report above")
        return

    result.record("relay wire-quality within ticket 012's stated budget "
                  "(wire_truth.kRelayBudget)", passed, "see the capture report printed above")

    if json_out:
        import json
        from dataclasses import asdict
        payload = asdict(stats)
        payload["buckets"] = dict(stats.buckets)
        payload["sigil_biased"] = dict(stats.sigil_biased)
        payload["budget"] = asdict(wire_truth.kRelayBudget)
        with open(json_out, "w") as f:
            json.dump(payload, f, indent=2, default=str)
        print(f"\nwrote {json_out}")


def _run_usb_comparison(port: str, duration: float) -> None:  # [s]
    """Bullet 7's "USB may additionally be run for comparison" -- purely
    informational: prints its own wire_truth report but is NEVER passed
    to `Result.record()`, so it cannot affect this script's exit code."""
    import wire_truth

    stats = wire_truth.run_capture(port, "radio-bench-gate-013-usb-comparison", duration, False)
    wire_truth.report(stats, wire_truth.kUsbBudget)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def _args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", required=True,
                   help="the RADIO RELAY's own serial port (not the robot's direct USB port)")
    p.add_argument("--usb-port", default=None,
                   help="optional: also run an INFORMATIONAL USB-direct wire-quality leg for "
                        "comparison (never gates the exit code -- bullet 7)")
    p.add_argument("--wire-quality-duration", type=float, default=120.0,  # [s]
                   help="capture window for the relay (and, if given, USB) wire-quality leg; "
                        "matches wire_truth.py's own default")
    p.add_argument("--wire-quality-json-out", default=None,
                   help="optional path to write the relay capture's machine-readable summary")
    return p.parse_args()


# Cross-phase floor for the final SUC-008 aggregate below -- see
# `SessionTelemetry`'s own docstring. 20 frames is trivially easy over a
# genuinely live session (Phase 1's own 3s window alone nominally yields
# ~75 frames at the ~25Hz primary cadence) and unreachable with no
# hardware connected at all.
_MIN_SESSION_FRAMES_FOR_SUC008 = 20


def main() -> int:
    args = _args()
    result = Result()
    telemetry = SessionTelemetry()  # cross-phase bookkeeping -- see its own docstring

    print("=== Phase 1: fresh relay connect -- banner-on-boot + SUC-008 (zero commands) ===")
    fresh_conn = scenario_banner_on_connect(args.port, result)
    phase1_connected = fresh_conn is not None and fresh_conn.is_open
    if phase1_connected:
        scenario_comms_malformed_stays_clear_fresh(fresh_conn, result, telemetry)
    if fresh_conn is not None:
        try:
            fresh_conn.disconnect()
        except Exception:
            pass

    print("\n=== Phase 2: main relay session -- HELLO/PING/ID/VER, MOVE, acks, 0x0A repro ===")
    conn = SerialConnection(port=args.port, mode="relay")
    proto: "NezhaProtocol | None" = None
    phase2_connected = False
    try:
        info = conn.connect()
        phase2_connected = info.get("status") == "connected"
        if not phase2_connected:
            result.record("main relay connect() succeeded", False, f"info={info}")
        else:
            proto = NezhaProtocol(conn)
            scenario_hello_ping_id_ver(conn, result)
            scenario_move_wheels_start_stop_and_climb(proto, result, telemetry)
            scenario_enqueue_and_completion_acks(proto, result, telemetry)
            scenario_embedded_0x0a_10_of_10(proto, result, telemetry)
    except Exception as exc:
        result.record("main relay session completed without raising", False, f"raised {exc!r}")
    finally:
        if proto is not None:
            try:
                proto.stop()
            except Exception:
                pass
        try:
            conn.disconnect()
        except Exception:
            pass

    if not phase1_connected and not phase2_connected:
        print("\n*** NO HARDWARE CONNECTION WAS EVER ESTABLISHED THIS SESSION ***")
        print("*** every criterion below reflects a TOTAL ABSENCE of bench observations, "
              "not a specific wire defect -- NO criterion in this gate can be considered "
              "scored until a real relay connection succeeds. ***")

    # SUC-008, aggregated across the WHOLE session (both phases) -- requires
    # POSITIVE evidence (a real minimum of frames actually decoded) before a
    # "stayed clear" PASS is allowed; see SessionTelemetry's own docstring
    # for why this guard exists (the vacuous-PASS defect the coordinator's
    # dry-run surfaced: absence of a fault observation is not the same as
    # observing the absence of a fault).
    observed_enough = telemetry.frames_observed >= _MIN_SESSION_FRAMES_FOR_SUC008
    no_malformed = len(telemetry.malformed_violations) == 0
    if not observed_enough:
        detail = (f"INCONCLUSIVE: only {telemetry.frames_observed} telemetry frames observed "
                  f"across the ENTIRE session (need >= {_MIN_SESSION_FRAMES_FOR_SUC008}) -- "
                  "zero/near-zero observations must never score as a clean pass")
    elif telemetry.malformed_violations:
        detail = f"{telemetry.frames_observed} frames observed; violations={telemetry.malformed_violations}"
    else:
        detail = f"{telemetry.frames_observed} frames observed across the session, none malformed"
    result.record("kFaultCommsMalformed stayed clear across the ENTIRE gate session "
                  "(SUC-008, not just the isolated fresh-connect repro)",
                  observed_enough and no_malformed, detail)

    print("\n=== Phase 3: wire-quality vs budget (radio relay) ===")
    try:
        scenario_wire_quality_vs_budget(args.port, args.wire_quality_duration, result,
                                        args.wire_quality_json_out)
    except Exception as exc:
        result.record("relay wire-quality phase completed without raising", False,
                      f"raised {exc!r}")

    if args.usb_port:
        print("\n=== Informational only: USB wire-quality comparison "
              "(does NOT gate this script's exit code) ===")
        try:
            _run_usb_comparison(args.usb_port, args.wire_quality_duration)
        except Exception as exc:
            print(f"USB comparison capture failed (informational only, non-fatal): {exc!r}",
                  file=sys.stderr)

    passed = result.ok()
    print(f"\nGATE RESULT: {'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
