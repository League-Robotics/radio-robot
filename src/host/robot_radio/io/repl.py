"""rogo command runner — argument-list execution, a stdin REPL/pipe mode, and
optional telemetry-to-JSONL recording (out-of-process addition, 2026-07-15).

The P4 single-loop firmware exposes exactly three ``CommandEnvelope`` arms —
``twist`` / ``stop`` / ``config`` (``protos/envelope.proto``) — with no text
command parser at all. Every verb here maps onto one of those three arms plus
telemetry reads (encoders/pose/line/color come from the always-on ``Telemetry``
push, not a request/reply).

Three ways in, one grammar:
  * argument list — ``rogo repl "twist 150 0 1000" stop``
  * piped stdin   — ``cat run.rogo | rogo repl``  (one command per line)
  * interactive   — ``rogo repl``  (prompts on a tty)

Telemetry recording (``--record FILE``) taps the SAME frame stream the command
loop drains: a command's ack rides inside a ``Telemetry`` frame's single ack
slot (``TLMFrame.ack`` -- 115-003's frame-v2 rewrite replaced the pre-115
depth-3 ack ring with this one slot), so a second, independent reader would
steal an ack-bearing frame from the confirmer. Instead every frame is pumped
exactly once — recorded to the JSONL file AND scanned for the pending corr_id
in the same pass (``RogoSession.pump``). Single-threaded by construction;
nothing is stolen.
"""
from __future__ import annotations

import dataclasses
import json
import math
import shlex
import sys
import time
from typing import Any, TextIO

from robot_radio.robot import NezhaProtocol
from robot_radio.robot.connection import make_robot as _make_robot
from robot_radio.robot.pb2 import envelope_pb2
from robot_radio.robot.protocol import TLMFrame

# Open-loop convenience defaults for the derived drive/turn verbs. Both build a
# timed ``twist`` and let the firmware's deadman stop at ``duration`` — there is
# no closed-loop distance/heading verb on the P4 wire, so these are open-loop
# and calibration-sensitive (documented at the call sites).
DEFAULT_DRIVE_SPEED = 150.0   # [mm/s]
DEFAULT_TURN_SPEED = 90.0     # [deg/s]
ACK_TIMEOUT = 800             # [ms]


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
    """A persistent connection plus the single frame-pump both the REPL and the
    recorder share."""

    def __init__(self, args: Any, record_path: str | None, verbose: bool) -> None:
        self.verbose = verbose
        self._robot, self.conn, self._meta = _make_robot(
            port=getattr(args, "port", None), mode=None, verbose=verbose, args=args)
        self.proto = NezhaProtocol(self.conn)
        self.recorder = Recorder(record_path) if record_path else None
        self._latest: TLMFrame | None = None

    # -- frame plumbing ------------------------------------------------------
    def pump(self) -> list[TLMFrame]:
        """Drain every pending telemetry frame ONCE: record it (if recording)
        and remember the freshest. Returns the drained frames so callers can
        also scan their single ack slot (``TLMFrame.ack``)."""
        frames = self.proto.read_pending_binary_tlm_frames()
        for f in frames:
            self._latest = f
            if self.recorder is not None:
                self.recorder.write(f)
        return frames

    def confirm(self, corr_id: int, timeout_ms: int = ACK_TIMEOUT):
        """Pump frames until one carries an ack for ``corr_id`` (or timeout).
        Returns the matching ``AckEntry`` or ``None``."""
        deadline = time.monotonic() + timeout_ms / 1000.0
        while time.monotonic() < deadline:
            for f in self.pump():
                if f.ack is not None and f.ack.corr_id == corr_id:
                    return f.ack
            time.sleep(0.005)
        return None

    def wait(self, ms: float) -> None:
        """Pump (and thus record) for ``ms`` milliseconds without commanding."""
        deadline = time.monotonic() + ms / 1000.0
        while time.monotonic() < deadline:
            self.pump()
            time.sleep(0.01)

    def latest(self, field: str, timeout_ms: int = 700) -> TLMFrame | None:
        """Return the freshest frame whose ``field`` is populated, pumping up to
        ``timeout_ms`` for one to arrive."""
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
            self.proto.stop()
        except Exception:
            pass
        if self.recorder is not None:
            print(f"  recorded {self.recorder.count} telemetry frames -> {self.recorder.path}",
                  file=sys.stderr)
            self.recorder.close()
        try:
            self.conn.disconnect()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Verb handlers — each takes (session, tokens) where tokens are the args after
# the verb, and returns None. They print a short human line; errors raise
# CommandError which the dispatcher catches so one bad line never aborts a batch.
# ---------------------------------------------------------------------------
class CommandError(Exception):
    pass


def _ack_str(session: RogoSession, corr_id: int) -> str:
    ack = session.confirm(corr_id)
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


def verb_twist(session: RogoSession, tokens: list[str]) -> None:
    if len(tokens) != 3:
        raise CommandError("usage: twist <v_x [mm/s]> <omega [rad/s]> <duration [ms]>")
    v_x, omega, dur = (_num(tokens[0], "v_x"), _num(tokens[1], "omega"), _num(tokens[2], "duration"))
    cid = session.proto.twist(v_x=v_x, omega=omega, duration=dur)
    print(f"  twist v_x={v_x:g} omega={omega:g} dur={dur:g}  {_ack_str(session, cid)}")


def verb_stop(session: RogoSession, tokens: list[str]) -> None:
    cid = session.proto.stop()
    print(f"  stop  {_ack_str(session, cid)}")


def verb_drive(session: RogoSession, tokens: list[str]) -> None:
    """drive <mm> [speed mm/s] — open-loop timed forward/back twist.

    No closed-loop distance verb exists on the P4 wire; this arms a twist for
    distance/speed seconds and lets the firmware deadman stop it. Distance is
    approximate and calibration-sensitive."""
    if not tokens:
        raise CommandError("usage: drive <mm> [speed mm/s]")
    dist = _num(tokens[0], "mm")
    speed = _num(tokens[1], "speed") if len(tokens) > 1 else DEFAULT_DRIVE_SPEED
    if speed <= 0:
        raise CommandError("speed must be > 0")
    dur = abs(dist) / speed * 1000.0
    v_x = math.copysign(speed, dist)
    cid = session.proto.twist(v_x=v_x, omega=0.0, duration=dur + 50)
    print(f"  drive {dist:g}mm @ {speed:g}mm/s (~{dur:.0f}ms, open-loop)  {_ack_str(session, cid)}")
    session.wait(dur)
    session.proto.stop()
    session.wait(150)  # let it settle / record the stop


def verb_turn(session: RogoSession, tokens: list[str]) -> None:
    """turn <deg> [speed deg/s] — open-loop timed spin (+=CCW). Approximate."""
    if not tokens:
        raise CommandError("usage: turn <deg> [speed deg/s]")
    deg = _num(tokens[0], "deg")
    speed = _num(tokens[1], "speed") if len(tokens) > 1 else DEFAULT_TURN_SPEED
    if speed <= 0:
        raise CommandError("speed must be > 0")
    dur = abs(deg) / speed * 1000.0
    omega = math.copysign(speed, deg) * math.pi / 180.0  # [rad/s]
    cid = session.proto.twist(v_x=0.0, omega=omega, duration=dur + 50)
    print(f"  turn {deg:g}deg @ {speed:g}deg/s (~{dur:.0f}ms, open-loop)  {_ack_str(session, cid)}")
    session.wait(dur)
    session.proto.stop()
    session.wait(150)


def verb_config(session: RogoSession, tokens: list[str]) -> None:
    """config key=val ... — one ConfigDelta (flat wire keys: tw, pid.kp, ml/mr,
    sTimeout, ...). NOTE: current firmware acks ERR_UNIMPLEMENTED for config —
    the envelope is built and sent, but the delta is not applied yet."""
    kv = _kv(tokens)
    if not kv:
        raise CommandError("usage: config <key>=<value> [<key>=<value> ...]")
    try:
        cid = session.proto.config(**kv)
    except ValueError as exc:
        raise CommandError(str(exc))
    print(f"  config {' '.join(tokens)}  {_ack_str(session, cid)}")


def _read_and_print(session: RogoSession, field: str, label: str) -> None:
    frame = session.latest(field)
    if frame is None:
        print(f"  {label}: (no frame with {field}= arrived)")
        return
    print(f"  {label}: {getattr(frame, field)}")


def verb_enc(session, tokens): _read_and_print(session, "enc", "enc [mm] (L,R)")
def verb_pose(session, tokens): _read_and_print(session, "pose", "pose [mm,mm,cdeg]")
def verb_otos(session, tokens): _read_and_print(session, "otos", "otos [mm,mm,cdeg]")
def verb_vel(session, tokens): _read_and_print(session, "vel", "vel [mm/s]")
def verb_twistfb(session, tokens): _read_and_print(session, "twist", "twist fb (v,omega_mrad)")
def verb_line(session, tokens): _read_and_print(session, "line", "line (g1..g4)")
def verb_color(session, tokens): _read_and_print(session, "color", "color (r,g,b,c)")


def verb_tlm(session: RogoSession, tokens: list[str]) -> None:
    """Dump the freshest full telemetry frame as JSON."""
    session.pump()
    if session._latest is None:
        session.wait(200)
    if session._latest is None:
        print("  tlm: (no frame arrived)")
        return
    row = dataclasses.asdict(session._latest)
    print("  " + json.dumps(row))


def verb_sleep(session: RogoSession, tokens: list[str]) -> None:
    if not tokens:
        raise CommandError("usage: sleep <ms>")
    session.wait(_num(tokens[0], "ms"))


def verb_record(session: RogoSession, tokens: list[str]) -> None:
    """record <file.jsonl> | record off — toggle telemetry recording mid-session."""
    if not tokens or tokens[0] == "off":
        if session.recorder is not None:
            print(f"  recording stopped ({session.recorder.count} frames -> {session.recorder.path})")
            session.recorder.close()
            session.recorder = None
        else:
            print("  (not recording)")
        return
    if session.recorder is not None:
        session.recorder.close()
    session.recorder = Recorder(tokens[0])
    print(f"  recording telemetry -> {tokens[0]}")


def verb_raw(session: RogoSession, tokens: list[str]) -> None:
    """raw <twist|stop|config> [field=val ...] — build a CommandEnvelope arm
    directly. twist fields: v_x, omega, duration. config: same flat keys as the
    config verb. stop: no fields."""
    if not tokens:
        raise CommandError("usage: raw <twist|stop|config> [field=value ...]")
    arm = tokens[0]
    rest = tokens[1:]
    if arm == "stop":
        cid = session.proto.stop()
        print(f"  raw stop  {_ack_str(session, cid)}")
        return
    if arm == "twist":
        kv = _kv(rest)
        unknown = set(kv) - {"v_x", "omega", "duration"}
        if unknown:
            raise CommandError(f"raw twist: unknown field(s) {sorted(unknown)}")
        env = envelope_pb2.CommandEnvelope(twist=envelope_pb2.Twist(
            v_x=float(kv.get("v_x", 0.0)),
            omega=float(kv.get("omega", 0.0)),
            duration=float(kv.get("duration", 0.0))))
        cid = session.conn.send_envelope_fast(env)
        print(f"  raw twist {' '.join(rest)}  {_ack_str(session, cid)}")
        return
    if arm == "config":
        verb_config(session, rest)
        return
    raise CommandError(f"raw: unknown arm {arm!r} (twist|stop|config)")


_VERBS = {
    "twist": verb_twist, "stop": verb_stop, "drive": verb_drive, "turn": verb_turn,
    "config": verb_config, "raw": verb_raw,
    "enc": verb_enc, "pose": verb_pose, "otos": verb_otos, "vel": verb_vel,
    "twistfb": verb_twistfb, "line": verb_line, "color": verb_color, "tlm": verb_tlm,
    "sleep": verb_sleep, "wait": verb_sleep, "record": verb_record,
}

_HELP = """\
rogo commands (all map to the P4 binary envelope: twist/stop/config + telemetry):
  twist <v_x> <omega> <dur>   body twist  [mm/s] [rad/s] [ms]
  stop                        panic-stop the drivetrain
  drive <mm> [speed]          open-loop timed forward/back (approx.)
  turn <deg> [speed]          open-loop timed spin, +=CCW (approx.)
  config <k>=<v> ...          one ConfigDelta (firmware acks UNIMPLEMENTED today)
  raw <twist|stop|config> ..  build an envelope arm directly (field=value)
  enc | pose | otos | vel     read + print the latest telemetry field
  twistfb | line | color      read + print the latest telemetry field
  tlm                         dump the freshest full frame as JSON
  sleep <ms> | wait <ms>      idle (keeps recording telemetry)
  record <file> | record off  toggle telemetry->JSONL recording
  help                        this list;   # ... comments and blank lines ignored
  quit | exit                 leave the REPL"""


def dispatch(session: RogoSession, line: str) -> bool:
    """Execute one command line. Returns False to request REPL exit."""
    line = line.strip()
    if not line or line.startswith("#"):
        return True
    if line in ("quit", "exit"):
        return False
    if line in ("help", "?"):
        print(_HELP)
        return True
    try:
        tokens = shlex.split(line)
    except ValueError as exc:
        print(f"  parse error: {exc}", file=sys.stderr)
        return True
    verb, rest = tokens[0], tokens[1:]
    handler = _VERBS.get(verb)
    if handler is None:
        print(f"  unknown command {verb!r} (try 'help')", file=sys.stderr)
        return True
    try:
        handler(session, rest)
    except CommandError as exc:
        print(f"  {exc}", file=sys.stderr)
    except ConnectionError as exc:
        print(f"  connection error: {exc}", file=sys.stderr)
        raise
    return True


def run(args: Any, verbose: bool) -> int:
    """Entry point for the ``repl`` subcommand. Runs positional commands if
    given (argument-list mode), else reads stdin (pipe or interactive)."""
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
            # works unquoted, like the old rogo: `rogo run twist 150 0 1000`),
            # and split on ';' to allow a quick multi-command one-liner
            # (`rogo run "twist 150 0 500; stop"`).
            for cmd in " ".join(commands).split(";"):
                dispatch(session, cmd)
        elif sys.stdin.isatty():
            print("rogo REPL — 'help' for commands, 'quit' to exit.", file=sys.stderr)
            while True:
                try:
                    line = input("rogo> ")
                except EOFError:
                    break
                if not dispatch(session, line):
                    break
        else:
            for line in sys.stdin:
                if not dispatch(session, line):
                    break
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
    finally:
        session.close()
    return 0
