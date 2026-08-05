"""rogo command runner — argument-list execution, a stdin REPL/pipe mode,
optional telemetry-to-JSONL recording, and the shared verb engine the rogo
daemon (``robot_radio.io.server``) drives over its socket.

Rebuilt on the protocol-v5 surface (2026-08-05, out-of-process rogo revival;
closes the repl half of ``clasi/issues/later/
A-repl-motion-verbs-dead-and-mcp-calibration-push-noops.md``). The previous
generation of motion verbs called ``NezhaProtocol.twist()`` — deleted at the
116-001 MOVE cutover — so every motion command raised ``AttributeError``, and
its ``stop`` verb was documented as a panic stop when ``stop()`` had become
the PLANNED stop. Every verb here maps onto the v5 wire
(``docs/protocol-v5.md``): four cleartext verbs (``HELLO``/``PING``/``ID``/
``VER``), the binary command plane (``move``/``wheels``/``config``/``stop``/
``estop``), and the always-on binary ``Telemetry`` push.

Three ways in, one grammar:
  * argument list — ``rogo repl "drive 200" stop``
  * piped stdin   — ``cat run.rogo | rogo repl``  (one command per line)
  * interactive   — ``rogo repl``  (prompts on a tty)
plus the daemon: ``rogo serve`` holds one ``RogoSession`` open and feeds the
same ``dispatch()`` from TCP clients (``robot_radio.io.server``).

Telemetry recording (``--record FILE``) taps the SAME frame stream the command
loop drains: a command's ack rides inside a ``Telemetry`` frame's bounded ack
ring (``TLMFrame.acks``), so a second, independent reader would steal an
ack-bearing frame from the confirmer. Instead every frame is pumped exactly
once — recorded to the JSONL file AND scanned for the pending corr_id in the
same pass (``RogoSession.pump``). Single-threaded by construction; nothing is
stolen. The daemon's telemetry broadcast rides the same single pump via
``RogoSession.on_frame``.
"""
from __future__ import annotations

import collections
import dataclasses
import json
import math
import shlex
import sys
import threading
import time
from typing import Any, Callable, TextIO

from robot_radio.robot import NezhaProtocol
from robot_radio.robot.connection import make_robot as _make_robot
from robot_radio.robot.halt import halt_now
from robot_radio.robot.pb2 import envelope_pb2
from robot_radio.robot.protocol import (
    _CONFIG_GROUP_NAMES,
    _SET_KEY_TARGETS,
    AckEntry,
    TLMFrame,
)

# Open-loop convenience defaults for the derived drive/turn verbs.
DEFAULT_DRIVE_SPEED = 150.0   # [mm/s]
DEFAULT_TURN_SPEED = 90.0     # [deg/s]
ACK_TIMEOUT = 800             # [ms] enqueue-ack wait

# Move.timeout safety backstop, same policy (and values) as planner/tour.py's
# and testgui/transport.py's identically-named helpers — deliberately
# duplicated there, so duplicated here too rather than importing from either.
_MOVE_TIMEOUT_FACTOR = 3.0    # [multiple of expected duration]
_MOVE_MIN_TIMEOUT = 2000.0    # [ms]

# Fresh, never-reused Move.id allocation — same scheme as planner/tour.py's
# `_next_move_ids()` (see `_TOUR_MOVE_ID_BASE`'s comment there for the
# measured wedge a reused id causes: the stale completion ack matches on the
# first poll AND the duplicate wedges the firmware command path). The base
# sits far above tour's clock range (tour: 1<<20 + (t%1e6)*64 < 68M) so a
# repl and a tour sharing one process/robot session can never collide.
_MOVE_ID_BASE = 128 << 20
_MOVE_ID_MAX = (1 << 28) - 1  # ack ring packs `corr_id << 4 | err` into a uint32
_MOVE_ID_BLOCK = 64  # ids reserved per clock second -- bounds the stride
_move_id_cursor = [0]

# estop verification (playfield-testing.md "One estop() was never verified —
# repeat it"): after halt_now(), the active flag (flags bit 2) must be seen
# clear; if it stays set, re-issue. Measured 2026-08-03 on `vevov`: a single
# estop() failed 5 of 6 attempts against a latched brick; only repetition
# stopped the wheels.
_ESTOP_VERIFY_WINDOW = 700    # [ms] per attempt, watching for active to clear
_ESTOP_MAX_ROUNDS = 3


def _next_move_id() -> int:
    """One fresh, never-before-used ``Move.id`` — the key this Move's own
    COMPLETION ack echoes. The clock term keeps two separate host processes
    off each other's block; the cursor keeps successive allocations inside
    one process monotonic."""
    start = max(_move_id_cursor[0],
                _MOVE_ID_BASE + (int(time.time()) % 1_000_000) * _MOVE_ID_BLOCK)
    if start >= _MOVE_ID_MAX:
        start = _MOVE_ID_BASE
    _move_id_cursor[0] = start + 1
    return start


def _move_timeout(expected: float) -> float:  # [ms] both
    """Expected-duration-based ``Move.timeout`` safety backstop."""
    return max(_MOVE_MIN_TIMEOUT, expected * _MOVE_TIMEOUT_FACTOR)


class Recorder:
    """Append each telemetry frame to a JSON-lines file, one object per line.

    ``TLMFrame`` is a plain dataclass, so ``dataclasses.asdict`` gives a
    directly-serializable dict (nested ``AckEntry`` rows become dicts, tuples
    become lists). A host receive timestamp (``t_recv``, epoch seconds) is
    added so a recording is analyzable without relying on the robot clock.
    """

    def __init__(self, path: str) -> None:
        self.path = path
        self._fh: TextIO = open(path, "a", buffering=1)  # line-buffered
        self.count = 0

    def write(self, frame: TLMFrame) -> None:
        row = dataclasses.asdict(frame)
        row["t_recv"] = time.time()
        self._fh.write(json.dumps(row) + "\n")
        self.count += 1

    def close(self) -> None:
        try:
            self._fh.close()
        except Exception:
            pass


class RogoSession:
    """A persistent connection plus the single frame-pump the REPL, the
    recorder, and the daemon's telemetry broadcast all share.

    Output routing: verbs write through ``say()``/``err()`` (default
    stdout/stderr) so the daemon can capture one command's output and route
    it to the TCP client that issued it, without redirecting process-global
    streams.

    ``abort_event``: the daemon's priority-halt hook. ``wait()``/
    ``confirm()`` return early when it is set, so an ``estop`` from another
    client is never stuck behind an in-progress completion wait. Direct repl
    use never sets it.
    """

    def __init__(self, args: Any, record_path: str | None, verbose: bool) -> None:
        self.verbose = verbose
        self._robot, self.conn, self._meta = _make_robot(
            port=getattr(args, "port", None), mode=None, verbose=verbose, args=args)
        self.proto = NezhaProtocol(self.conn)
        self.recorder = Recorder(record_path) if record_path else None
        self._latest: TLMFrame | None = None
        self._ack_backlog: collections.deque = collections.deque(maxlen=64)
        self.out: TextIO = sys.stdout
        self.errout: TextIO = sys.stderr
        self.abort_event = threading.Event()
        self.on_frame: Callable[[TLMFrame], None] | None = None

    @classmethod
    def from_protocol(cls, proto: Any, conn: Any = None,
                      meta: dict | None = None) -> "RogoSession":
        """Build a session around an already-open protocol — used by tests
        (fake proto) and by any embedder that owns its own connection."""
        session = object.__new__(cls)
        session.verbose = False
        session._robot = None
        session.conn = conn if conn is not None else getattr(proto, "_conn", None)
        session._meta = meta or {}
        session.proto = proto
        session.recorder = None
        session._latest = None
        session._ack_backlog = collections.deque(maxlen=64)
        session.out = sys.stdout
        session.errout = sys.stderr
        session.abort_event = threading.Event()
        session.on_frame = None
        return session

    # -- output routing ------------------------------------------------------
    def say(self, text: str) -> None:
        print(text, file=self.out)

    def err(self, text: str) -> None:
        print(text, file=self.errout)

    # -- frame plumbing ------------------------------------------------------
    def pump(self) -> list[TLMFrame]:
        """Drain every pending telemetry frame ONCE: record it (if recording),
        hand it to ``on_frame`` (if the daemon subscribed), remember the
        freshest, and bank every ack-ring entry in ``_ack_backlog``. Returns
        the drained frames."""
        frames = self.proto.read_pending_binary_tlm_frames()
        for f in frames:
            self._latest = f
            self._ack_backlog.extend(f.acks)
            if self.recorder is not None:
                self.recorder.write(f)
            if self.on_frame is not None:
                self.on_frame(f)
        return frames

    def confirm(self, corr_id: int, timeout_ms: int = ACK_TIMEOUT):
        """Pump frames until an ack for ``corr_id`` has been seen (or timeout,
        or ``abort_event``). Returns the matching ``AckEntry`` or ``None``.

        Matches against ``_ack_backlog`` — every ack any prior ``pump()``
        drained — not only frames drained inside this call: an enqueue ack
        and its Move's completion ack can land in ONE drain batch (a short
        move, or a host that fell behind the push rate), and the completion
        entry must still be findable by the NEXT confirm() call. Safe because
        corr_ids/Move.ids are never reused within a session (see
        ``_next_move_id()``). Scans here rather than delegating to
        ``NezhaProtocol.wait_for_ack()`` — that method pumps frames itself,
        which would bypass ``self.recorder``/``on_frame`` and silently drop
        frames from a recording or a daemon broadcast."""
        deadline = time.monotonic() + timeout_ms / 1000.0
        while True:
            for ack in self._ack_backlog:
                if ack.corr_id == corr_id:
                    self._ack_backlog.remove(ack)
                    return ack
            if time.monotonic() >= deadline:
                return None
            for f in self.pump():
                for ack in f.acks:
                    if ack.corr_id == corr_id:
                        try:
                            self._ack_backlog.remove(ack)
                        except ValueError:
                            pass
                        return ack
            if self.abort_event.is_set():
                return None
            time.sleep(0.005)

    def wait(self, ms: float) -> None:
        """Pump (and thus record/broadcast) for ``ms`` milliseconds without
        commanding. Returns early on ``abort_event``."""
        deadline = time.monotonic() + ms / 1000.0
        while time.monotonic() < deadline:
            self.pump()
            if self.abort_event.is_set():
                return
            time.sleep(0.01)

    def latest(self, field: str, timeout_ms: int = 700) -> TLMFrame | None:
        """Return the freshest frame whose ``field`` is populated, pumping up
        to ``timeout_ms`` for one to arrive."""
        deadline = time.monotonic() + timeout_ms / 1000.0
        best = self._latest if (self._latest and getattr(self._latest, field) is not None) else None
        while best is None and time.monotonic() < deadline:
            for f in self.pump():
                if getattr(f, field) is not None:
                    best = f
            time.sleep(0.01)
        return best

    def close(self) -> None:
        try:
            halt_now(self.proto)
        except Exception:
            # halt_now already logged ROBOT MAY STILL BE MOVING; the
            # operator has been told -- which is the entire point. A
            # session teardown path must not raise past this.
            pass
        if self.recorder is not None:
            self.err(f"  recorded {self.recorder.count} telemetry frames -> {self.recorder.path}")
            self.recorder.close()
        try:
            if self.conn is not None:
                self.conn.disconnect()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Verb handlers — each takes (session, tokens) where tokens are the args after
# the verb, and returns None. They print a short human line via session.say();
# errors raise CommandError which the dispatcher catches so one bad line never
# aborts a batch.
# ---------------------------------------------------------------------------
class CommandError(Exception):
    pass


@dataclasses.dataclass
class DispatchResult:
    """What one ``dispatch()`` call did — the daemon serializes this to the
    requesting TCP client; the direct repl only looks at ``exit``."""
    exit: bool = False
    error: str | None = None


def _ack_str(session: RogoSession, corr_id: int) -> str:
    return _describe_ack(corr_id, session.confirm(corr_id))


def _describe_ack(corr_id: int, ack: "AckEntry | None") -> str:
    if ack is None:
        return f"corr_id={corr_id} NO ACK (timeout)"
    if ack.ok:
        return f"corr_id={corr_id} OK"
    try:
        name = envelope_pb2.ErrCode.Name(ack.err_code)
    except Exception:
        name = str(ack.err_code)
    return f"corr_id={corr_id} ERR {name}"


def _num(tok: str, name: str) -> float:
    try:
        return float(tok)
    except ValueError:
        raise CommandError(f"{name!r} must be a number, got {tok!r}")


def _kv(tokens: list[str]) -> dict[str, str]:
    out: dict[str, str] = {}
    for t in tokens:
        if "=" not in t:
            raise CommandError(f"expected key=value, got {t!r}")
        k, v = t.split("=", 1)
        out[k] = v
    return out


def _pop_nowait(tokens: list[str]) -> tuple[list[str], bool]:
    """Strip a trailing ``nowait`` keyword: (remaining_tokens, nowait)."""
    if tokens and tokens[-1] == "nowait":
        return tokens[:-1], True
    return tokens, False


def _wait_move_done(session: RogoSession, move_id: int, timeout_ms: float) -> str:
    """Wait for a Move's COMPLETION ack (ack-ring entry echoing ``Move.id``)
    and report how it ended. The v5 completion ack always carries err=0 —
    timeout-vs-stop-condition rides the frame's ``fault_move_timeout`` flag
    (flags bit 15), not the ack's err field."""
    ack = session.confirm(move_id, timeout_ms=int(timeout_ms))
    if session.abort_event.is_set():
        return "ABORTED (halt requested)"
    if ack is None:
        return f"move_id={move_id} NO COMPLETION (host wait expired)"
    frame = session._latest
    if frame is not None and frame.fault_move_timeout:
        return f"move_id={move_id} DONE (firmware timeout backstop fired)"
    return f"move_id={move_id} DONE"


# -- motion -----------------------------------------------------------------

def verb_twist(session: RogoSession, tokens: list[str]) -> None:
    """twist <v_x> <omega> <ms> — bounded body-twist Move, TIME stop."""
    if len(tokens) != 3:
        raise CommandError("usage: twist <v_x [mm/s]> <omega [rad/s]> <duration [ms]>")
    v_x, omega, dur = (_num(tokens[0], "v_x"), _num(tokens[1], "omega"), _num(tokens[2], "duration"))
    if dur <= 0:
        raise CommandError("duration must be > 0")
    cid = session.proto.move_twist(v_x, 0.0, omega, stop_time=dur,
                                   timeout=_move_timeout(dur))
    session.say(f"  twist v_x={v_x:g} omega={omega:g} dur={dur:g}  {_ack_str(session, cid)}")


def verb_wheels(session: RogoSession, tokens: list[str]) -> None:
    """wheels <left> <right> <ms> — dumb teleop wheel-pair hold (WHEELS arm,
    bypasses the planner). Time-bounded by construction."""
    if len(tokens) != 3:
        raise CommandError("usage: wheels <left [mm/s]> <right [mm/s]> <duration [ms]>")
    left, right, dur = (_num(tokens[0], "left"), _num(tokens[1], "right"),
                        _num(tokens[2], "duration"))
    if dur <= 0:
        raise CommandError("duration must be > 0")
    cid = session.proto.wheels(left, right, dur)
    session.say(f"  wheels {left:g} {right:g} dur={dur:g}  {_ack_str(session, cid)}")


def verb_drive(session: RogoSession, tokens: list[str]) -> None:
    """drive <mm> [speed] [nowait] — straight Move with a DISTANCE stop
    condition (closed on encoder odometry firmware-side, v5 Move stop oneof —
    no longer the open-loop timed twist the pre-v5 repl sent). Waits for the
    completion ack unless ``nowait``."""
    tokens, nowait = _pop_nowait(tokens)
    if not tokens:
        raise CommandError("usage: drive <mm> [speed mm/s] [nowait]")
    dist = _num(tokens[0], "mm")
    speed = _num(tokens[1], "speed") if len(tokens) > 1 else DEFAULT_DRIVE_SPEED
    if speed <= 0:
        raise CommandError("speed must be > 0")
    if dist == 0:
        raise CommandError("distance must be nonzero")
    expected = abs(dist) / speed * 1000.0  # [ms]
    move_id = _next_move_id()
    v = math.copysign(speed, dist)
    cid = session.proto.move_wheels(v, v, stop_distance=abs(dist),
                                    timeout=_move_timeout(expected), move_id=move_id)
    session.say(f"  drive {dist:g}mm @ {speed:g}mm/s (~{expected:.0f}ms)  {_ack_str(session, cid)}")
    if not nowait:
        session.say(f"  {_wait_move_done(session, move_id, _move_timeout(expected) + 1000)}")


def verb_turn(session: RogoSession, tokens: list[str]) -> None:
    """turn <deg> [speed deg/s] [nowait] — in-place Move with an ANGLE stop
    condition (|heading change| on encoder odometry), + = CCW in the world
    frame. Positive commanded omega DECREASES world yaw (measured 2026-07-29,
    .claude/rules/playfield-testing.md "Heading convention"), so a CCW turn
    commands NEGATIVE omega."""
    tokens, nowait = _pop_nowait(tokens)
    if not tokens:
        raise CommandError("usage: turn <deg> [speed deg/s] [nowait]")
    deg = _num(tokens[0], "deg")
    speed = _num(tokens[1], "speed") if len(tokens) > 1 else DEFAULT_TURN_SPEED
    if speed <= 0:
        raise CommandError("speed must be > 0")
    if deg == 0:
        raise CommandError("angle must be nonzero")
    expected = abs(deg) / speed * 1000.0  # [ms]
    omega = -math.copysign(speed, deg) * math.pi / 180.0  # [rad/s] see docstring
    move_id = _next_move_id()
    cid = session.proto.move_twist(0.0, 0.0, omega,
                                   stop_angle=math.radians(abs(deg)),
                                   timeout=_move_timeout(expected), move_id=move_id)
    session.say(f"  turn {deg:g}deg @ {speed:g}deg/s (~{expected:.0f}ms)  {_ack_str(session, cid)}")
    if not nowait:
        session.say(f"  {_wait_move_done(session, move_id, _move_timeout(expected) + 1000)}")


def verb_stop(session: RogoSession, tokens: list[str]) -> None:
    """stop — PLANNED stop: enters the planner queue and waits behind the
    in-flight Move (measured 2026-07-29: sent mid-leg it acted 5.9s later,
    after the full 39.8cm leg). For "halt NOW" use estop."""
    cid = session.proto.stop()
    session.say(f"  stop (planned -- queues behind the active move)  {_ack_str(session, cid)}")


def verb_estop(session: RogoSession, tokens: list[str]) -> None:
    """estop — halt everything NOW, then VERIFY the halt took: watch for the
    active flag (flags bit 2) to clear and re-issue if it does not. A single
    unverified estop() write can be lost with the wheels still turning
    (measured 2026-08-03: 5 of 6 single attempts failed against the latched
    Nezha brick — playfield-testing.md)."""
    for attempt in range(1, _ESTOP_MAX_ROUNDS + 1):
        halt_now(session.proto, log=session.err)
        deadline = time.monotonic() + _ESTOP_VERIFY_WINDOW / 1000.0
        stopped = None
        while time.monotonic() < deadline:
            for f in session.pump():
                if f.active is not None:
                    stopped = not f.active
            if stopped:
                break
            time.sleep(0.01)
        if stopped:
            session.say(f"  estop VERIFIED (active clear, attempt {attempt})")
            return
        if stopped is None:
            session.say(
                f"  estop sent (attempt {attempt}) -- no telemetry to verify "
                "against; CONFIRM THE ROBOT IS STOPPED")
            return
        session.err(f"  estop attempt {attempt}: active flag still set, re-issuing")
    session.err(f"  estop NOT VERIFIED after {_ESTOP_MAX_ROUNDS} attempts -- "
                "ROBOT MAY STILL BE MOVING")


# -- cleartext liveness/identity (v5 restored HELLO/PING/ID/VER) ------------

def _cleartext(session: RogoSession, verb: str, reply_prefix: str,
               timeout_ms: int = 700) -> None:
    """Send one cleartext verb and print the first reply line containing
    ``reply_prefix`` (the v5 cleartext plane is line-oriented, no corr_id).

    Captures the reply via the connection's ``on_recv`` hook, NOT
    ``read_pending_lines()`` — the reader thread's ``_handle_text_line()``
    never routes ``DEVICE:``/``PONG:``/``ID:``/``VER:`` lines into the
    drainable queues (see ``radio_bench_gate.py``'s "Design notes" for the
    post-124-005 rationale; this is that gate's own capture pattern).
    Verified against gopiv on the bench 2026-08-05: the queue-drain version
    of this helper timed out on all four verbs; this version answers.
    Falls back to ``read_pending_lines()`` when the connection has no
    ``on_recv`` attribute (e.g. a test fake)."""
    conn = session.conn
    captured: list[str] = []
    hookable = hasattr(conn, "on_recv")
    if hookable:
        original_on_recv = conn.on_recv

        def _capture(line) -> None:
            if isinstance(line, str):
                captured.append(line)
            if original_on_recv:
                original_on_recv(line)

        conn.on_recv = _capture
    try:
        session.proto.send_fast(verb)
        deadline = time.monotonic() + timeout_ms / 1000.0
        while time.monotonic() < deadline:
            if not hookable:
                captured.extend(session.proto.read_pending_lines())
            for line in captured:
                if reply_prefix in line:
                    session.say(f"  {line.strip()}")
                    return
            session.pump()  # keep recording/broadcast alive while we wait
            time.sleep(0.01)
        session.say(f"  {verb}: no {reply_prefix} reply within {timeout_ms}ms")
    finally:
        if hookable:
            conn.on_recv = original_on_recv


def verb_ping(session, tokens): _cleartext(session, "PING", "PONG")
def verb_id(session, tokens): _cleartext(session, "ID", "ID:")
def verb_ver(session, tokens): _cleartext(session, "VER", "VER:")
def verb_hello(session, tokens): _cleartext(session, "HELLO", "DEVICE:", timeout_ms=1500)


# -- config -----------------------------------------------------------------

def verb_config(session: RogoSession, tokens: list[str]) -> None:
    """config key=val ... — one set_config_field() round trip per key
    (flat wire keys: see protocol.py's ``_SET_KEY_TARGETS`` for the
    authoritative list)."""
    kv = _kv(tokens)
    if not kv:
        raise CommandError("usage: config <key>=<value> [<key>=<value> ...]")
    bad = [k for k in kv if k not in _SET_KEY_TARGETS]
    if bad:
        raise CommandError(
            f"unknown config key(s): {bad!r} (known: {sorted(_SET_KEY_TARGETS)})")

    for key, raw_value in kv.items():
        target, field_name = _SET_KEY_TARGETS[key]
        ack = session.proto.set_config_field(target, field_name, float(raw_value))
        if ack is None:
            session.say(f"  config {key}={raw_value}  NO ACK (timeout or rejected)")
        elif ack.ok:
            session.say(f"  config {key}={raw_value}  OK")
        else:
            try:
                name = envelope_pb2.ErrCode.Name(ack.err_code)
            except Exception:
                name = str(ack.err_code)
            session.say(f"  config {key}={raw_value}  ERR {name}")


_GROUP_BY_NAME = {name.lower(): target
                  for target, name in _CONFIG_GROUP_NAMES.items()}


def verb_get(session: RogoSession, tokens: list[str]) -> None:
    """get <group> — read one config group back from the robot (GetConfig/
    ConfigSnapshot wire read-back, 132-011) and dump its fields plus
    provenance (LIVE/BAKED/PERSISTED)."""
    if len(tokens) != 1 or tokens[0].lower() not in _GROUP_BY_NAME:
        raise CommandError(f"usage: get <group>  (groups: {sorted(_GROUP_BY_NAME)})")
    target = _GROUP_BY_NAME[tokens[0].lower()]
    snapshot = session.proto.get_config_snapshot(target)
    if snapshot is None:
        session.say(f"  get {tokens[0]}: no reply (timeout or rejected)")
        return
    values = snapshot.values
    row = values.model_dump() if hasattr(values, "model_dump") else dict(values)
    session.say(f"  {tokens[0]} [{snapshot.source_name}]:")
    for field_name in sorted(row):
        session.say(f"    {field_name} = {row[field_name]}")


# -- telemetry reads --------------------------------------------------------

def _read_and_print(session: RogoSession, field: str, label: str) -> None:
    frame = session.latest(field)
    if frame is None:
        session.say(f"  {label}: (no frame with {field}= arrived)")
        return
    session.say(f"  {label}: {getattr(frame, field)}")


def verb_enc(session, tokens): _read_and_print(session, "enc", "enc [mm] (L,R)")
def verb_pose(session, tokens): _read_and_print(session, "pose", "pose [mm,mm,cdeg]")
def verb_otos(session, tokens): _read_and_print(session, "otos", "otos [mm,mm,cdeg]")
def verb_vel(session, tokens): _read_and_print(session, "vel", "vel [mm/s]")
def verb_twistfb(session, tokens): _read_and_print(session, "twist", "twist fb (v,omega_mrad)")
def verb_line(session, tokens): _read_and_print(session, "line", "line (g1..g4)")
def verb_color(session, tokens): _read_and_print(session, "color", "color (r,g,b,c)")


def verb_tlm(session: RogoSession, tokens: list[str]) -> None:
    """tlm — dump the freshest full telemetry frame as JSON.
    tlm on|off|now — switch the firmware telemetry mode (TLM:ON/OFF/NOW)."""
    if tokens:
        mode = tokens[0].lower()
        if mode == "on":
            session.proto.tlmOn()
        elif mode == "off":
            session.proto.tlmOff()
        elif mode == "now":
            session.proto.tlmNow()
        else:
            raise CommandError("usage: tlm [on|off|now]")
        session.say(f"  tlm {mode} sent")
        return
    session.pump()
    if session._latest is None:
        session.wait(200)
    if session._latest is None:
        session.say("  tlm: (no frame arrived)")
        return
    session.say("  " + json.dumps(dataclasses.asdict(session._latest)))


# -- misc -------------------------------------------------------------------

def verb_sleep(session: RogoSession, tokens: list[str]) -> None:
    if not tokens:
        raise CommandError("usage: sleep <ms>")
    session.wait(_num(tokens[0], "ms"))


def verb_record(session: RogoSession, tokens: list[str]) -> None:
    """record <file.jsonl> | record off — toggle telemetry recording mid-session."""
    if not tokens or tokens[0] == "off":
        if session.recorder is not None:
            session.say(f"  recording stopped ({session.recorder.count} frames -> {session.recorder.path})")
            session.recorder.close()
            session.recorder = None
        else:
            session.say("  (not recording)")
        return
    if session.recorder is not None:
        session.recorder.close()
    session.recorder = Recorder(tokens[0])
    session.say(f"  recording telemetry -> {tokens[0]}")


def verb_raw(session: RogoSession, tokens: list[str]) -> None:
    """raw <move|wheels|stop|estop|config> [field=val ...] — build a
    CommandEnvelope arm directly.
      raw move v_x= v_y= omega= | v_left= v_right=, one of time=/distance=/
               angle=, timeout= (required), replace=0|1, id=
      raw wheels v_left= v_right= duration= [id=]
      raw stop / raw estop: no fields
      raw config: same flat keys as the config verb."""
    if not tokens:
        raise CommandError("usage: raw <move|wheels|stop|estop|config> [field=value ...]")
    arm, rest = tokens[0], tokens[1:]
    if arm == "stop":
        # The user explicitly built a `raw stop` envelope by name --
        # deliberate planned stop by request, same rationale as verb_stop.
        cid = session.proto.stop()
        session.say(f"  raw stop  {_ack_str(session, cid)}")
        return
    if arm == "estop":
        cid = session.proto.estop()
        session.say(f"  raw estop  {_ack_str(session, cid)}")
        return
    if arm == "move":
        kv = _kv(rest)
        known = {"v_x", "v_y", "omega", "v_left", "v_right",
                 "time", "distance", "angle", "timeout", "replace", "id"}
        unknown = set(kv) - known
        if unknown:
            raise CommandError(f"raw move: unknown field(s) {sorted(unknown)}")
        if "timeout" not in kv:
            raise CommandError("raw move: timeout= is required (safety backstop)")
        try:
            cid = session.proto.move(
                v_x=float(kv.get("v_x", 0.0)), v_y=float(kv.get("v_y", 0.0)),
                omega=float(kv.get("omega", 0.0)),
                v_left=float(kv["v_left"]) if "v_left" in kv else None,
                v_right=float(kv["v_right"]) if "v_right" in kv else None,
                stop_time=float(kv["time"]) if "time" in kv else None,
                stop_distance=float(kv["distance"]) if "distance" in kv else None,
                stop_angle=float(kv["angle"]) if "angle" in kv else None,
                timeout=float(kv["timeout"]),
                replace=kv.get("replace", "1") not in ("0", "false"),
                id=int(kv["id"]) if "id" in kv else None)
        except ValueError as exc:
            raise CommandError(f"raw move: {exc}")
        session.say(f"  raw move {' '.join(rest)}  {_ack_str(session, cid)}")
        return
    if arm == "wheels":
        kv = _kv(rest)
        unknown = set(kv) - {"v_left", "v_right", "duration", "id"}
        if unknown:
            raise CommandError(f"raw wheels: unknown field(s) {sorted(unknown)}")
        try:
            cid = session.proto.wheels(
                float(kv.get("v_left", 0.0)), float(kv.get("v_right", 0.0)),
                float(kv.get("duration", 0.0)),
                move_id=int(kv.get("id", 0)))
        except ValueError as exc:
            raise CommandError(f"raw wheels: {exc}")
        session.say(f"  raw wheels {' '.join(rest)}  {_ack_str(session, cid)}")
        return
    if arm == "config":
        verb_config(session, rest)
        return
    raise CommandError(f"raw: unknown arm {arm!r} (move|wheels|stop|estop|config)")


_VERBS = {
    "twist": verb_twist, "wheels": verb_wheels, "drive": verb_drive,
    "turn": verb_turn, "stop": verb_stop, "estop": verb_estop, "halt": verb_estop,
    "ping": verb_ping, "id": verb_id, "ver": verb_ver, "hello": verb_hello,
    "config": verb_config, "get": verb_get, "raw": verb_raw,
    "enc": verb_enc, "pose": verb_pose, "otos": verb_otos, "vel": verb_vel,
    "twistfb": verb_twistfb, "line": verb_line, "color": verb_color, "tlm": verb_tlm,
    "sleep": verb_sleep, "wait": verb_sleep, "record": verb_record,
}

# Verbs the daemon treats as "halt now": jump the command queue and abort any
# in-progress wait. Kept here so the repl and the server can never disagree
# about which verbs are the panic path.
HALT_VERBS = frozenset({"estop", "halt"})

_HELP = """\
rogo commands (protocol v5: bounded Moves + always-on telemetry + cleartext liveness):
  drive <mm> [speed] [nowait]  straight Move, DISTANCE stop  [mm] [mm/s]
  turn <deg> [speed] [nowait]  in-place Move, ANGLE stop, +=CCW  [deg] [deg/s]
  twist <v_x> <omega> <ms>     bounded body twist, TIME stop  [mm/s] [rad/s] [ms]
  wheels <l> <r> <ms>          teleop wheel pair (bypasses planner)  [mm/s] [ms]
  stop                         PLANNED stop -- queues behind the active move
  estop | halt                 halt NOW + verify the active flag cleared
  ping | id | ver | hello      cleartext liveness/identity round trip
  config <k>=<v> ...           push config fields (SET vocabulary)
  get <group>                  read a config group back (drive, motors, otos, ...)
  raw <arm> [f=v ...]          build an envelope arm directly (move|wheels|stop|estop|config)
  enc | pose | otos | vel      read + print the latest telemetry field
  twistfb | line | color       read + print the latest telemetry field
  tlm [on|off|now]             dump freshest frame as JSON / switch TLM mode
  sleep <ms> | wait <ms>       idle (keeps recording telemetry)
  record <file> | record off   toggle telemetry->JSONL recording
  help                         this list;   # ... comments and blank lines ignored
  quit | exit                  leave the REPL
Full reference (daemon socket protocol, client recipes): rogo --agent"""


def dispatch(session: RogoSession, line: str) -> DispatchResult:
    """Execute one command line. ``DispatchResult.exit`` requests REPL exit;
    ``DispatchResult.error`` carries a handler error (already printed to
    ``session.errout``) so the daemon can flag the reply."""
    line = line.strip()
    if not line or line.startswith("#"):
        return DispatchResult()
    if line in ("quit", "exit"):
        return DispatchResult(exit=True)
    if line in ("help", "?"):
        session.say(_HELP)
        return DispatchResult()
    try:
        tokens = shlex.split(line)
    except ValueError as exc:
        session.err(f"  parse error: {exc}")
        return DispatchResult(error=f"parse error: {exc}")
    verb, rest = tokens[0], tokens[1:]
    handler = _VERBS.get(verb)
    if handler is None:
        session.err(f"  unknown command {verb!r} (try 'help')")
        return DispatchResult(error=f"unknown command {verb!r}")
    try:
        handler(session, rest)
    except CommandError as exc:
        session.err(f"  {exc}")
        return DispatchResult(error=str(exc))
    except ConnectionError as exc:
        session.err(f"  connection error: {exc}")
        raise
    return DispatchResult()


def run(args: Any, verbose: bool) -> int:
    """Entry point for the ``repl`` subcommand. Runs positional commands if
    given (argument-list mode), else reads stdin (pipe or interactive).
    With ``--connect``, runs through a rogo daemon instead of opening the
    serial port (see ``robot_radio.io.client``)."""
    if getattr(args, "connect", None) is not None:
        from robot_radio.io.client import run_connected
        return run_connected(args, verbose)

    record_path = getattr(args, "record", None)
    commands = getattr(args, "commands", None) or []

    try:
        session = RogoSession(args, record_path, verbose)
    except ConnectionError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    mode = session._meta.get("mode") if isinstance(session._meta, dict) else None
    print(f"connected (mode={mode})", file=sys.stderr)
    if record_path:
        print(f"recording telemetry -> {record_path}", file=sys.stderr)

    try:
        if commands:
            # Join the argv tail into one command line (so a single command
            # works unquoted, like the old rogo: `rogo run drive 200`),
            # and split on ';' to allow a quick multi-command one-liner
            # (`rogo run "drive 200; stop"`).
            for cmd in " ".join(commands).split(";"):
                if dispatch(session, cmd).exit:
                    break
        elif sys.stdin.isatty():
            print("rogo REPL — 'help' for commands, 'quit' to exit.", file=sys.stderr)
            while True:
                try:
                    line = input("rogo> ")
                except EOFError:
                    break
                if dispatch(session, line).exit:
                    break
        else:
            for line in sys.stdin:
                if dispatch(session, line).exit:
                    break
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
    finally:
        session.close()
    return 0
