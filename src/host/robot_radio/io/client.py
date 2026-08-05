"""robot_radio.io.client — RogoClient, the library side of the rogo daemon.

Any local program talks to a running ``rogo serve`` daemon through this class
instead of opening the serial port itself — the daemon holds the one serial
connection open (opening/closing it per-invocation resets the robot on macOS;
see ``robot_radio.io.server``'s module docstring), and any number of clients
share it:

    from robot_radio.io.client import RogoClient

    with RogoClient() as robot:                  # ROGO_ADDR or 127.0.0.1:7646
        robot.cmd("drive 200")                    # blocks until the Move completes
        robot.cmd("turn 90 nowait")               # returns after the enqueue ack
        print(robot.cmd("enc")["output"])         # ['  enc [mm] (L,R): (412, 408)']
        robot.estop()                             # panic path -- jumps the queue

    with RogoClient() as robot:                   # telemetry stream
        robot.subscribe_tlm(decimate=5)           # every 5th frame
        for frame in robot.frames(duration=2.0):
            print(frame["enc"])

Single-threaded by design: ``cmd()`` reads replies inline until its own
result arrives, handing any telemetry pushes that arrive in between to
``on_tlm`` (or a bounded internal deque). Not thread-safe — one RogoClient
per thread; the daemon happily accepts many connections.
"""
from __future__ import annotations

import collections
import json
import os
import socket
import time
from typing import Any, Callable, Iterator

from robot_radio.io.server import parse_addr

ADDR_ENV = "ROGO_ADDR"
DEFAULT_TIMEOUT = 15.0  # [s] per-command reply wait (a waited Move can be slow)
_TLM_BACKLOG = 256      # frames kept when no on_tlm callback is installed


class RogoDaemonError(ConnectionError):
    """The daemon is unreachable, shut down mid-conversation, or replied
    with something unintelligible."""


class RogoTimeout(RogoDaemonError):
    """A bounded wait expired with no qualifying reply. The connection is
    still healthy — ``RogoClient`` buffers partial lines itself, so a
    timeout never corrupts the stream."""


class RogoClient:
    """One TCP connection to a ``rogo serve`` daemon.

    ``addr`` is ``HOST:PORT`` / ``:PORT`` / ``PORT``; ``None`` falls back to
    ``$ROGO_ADDR``, then ``127.0.0.1:7646``. ``on_tlm``, if given, receives
    each subscribed telemetry frame (a dict) as it arrives; without it,
    frames are kept on ``self.tlm_frames`` (bounded deque, oldest dropped).
    """

    def __init__(self, addr: str | None = None, *,
                 timeout: float = DEFAULT_TIMEOUT,  # [s]
                 on_tlm: Callable[[dict], None] | None = None) -> None:
        host, port = parse_addr(addr if addr is not None else os.environ.get(ADDR_ENV))
        self.addr = (host, port)
        self.timeout = timeout
        self.on_tlm = on_tlm
        self.tlm_frames: collections.deque = collections.deque(maxlen=_TLM_BACKLOG)
        self._req_counter = 0
        try:
            self._sock = socket.create_connection(self.addr, timeout=timeout)
        except OSError as exc:
            raise RogoDaemonError(
                f"no rogo daemon at {host}:{port} ({exc}) -- start one with "
                "'rogo serve'") from exc
        self._buf = b""  # partial-line receive buffer (timeout-safe)
        self.hello = self._await_event(timeout, lambda e: e.get("type") == "hello")

    # -- context manager -----------------------------------------------------

    def __enter__(self) -> "RogoClient":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass

    # -- request/reply -------------------------------------------------------

    def cmd(self, line: str, *, timeout: float | None = None) -> dict:  # [s]
        """Send one repl command line; block until ITS result arrives (routing
        any interleaved telemetry pushes on the way). Returns the result
        object: ``{"ok": bool, "error": str|None, "output": [str, ...], ...}``.
        Raises ``RogoDaemonError`` if the daemon goes away or the wait
        expires."""
        self._req_counter += 1
        req_id = f"py-{self._req_counter}"
        payload = json.dumps({"id": req_id, "cmd": line}) + "\n"
        try:
            self._sock.sendall(payload.encode())
        except OSError as exc:
            raise RogoDaemonError(f"send failed: {exc}") from exc
        try:
            return self._await_event(
                timeout if timeout is not None else self.timeout,
                lambda e: e.get("type") == "result" and e.get("id") == req_id)
        except RogoTimeout:
            raise RogoDaemonError(f"no result for {line!r} within the timeout")

    def estop(self) -> dict:
        """Halt the robot NOW. The daemon puts halt verbs at the front of its
        queue and aborts any in-progress wait, so this is safe to call while
        another client's move is running."""
        return self.cmd("estop", timeout=max(self.timeout, 5.0))

    def status(self) -> dict:
        """Daemon status: serial port, mode, uptime, client/queue counts."""
        return self.cmd("status").get("status", {})

    # -- telemetry -----------------------------------------------------------

    def subscribe_tlm(self, decimate: int = 1) -> dict:
        """Ask the daemon to push telemetry: every ``decimate``-th frame."""
        return self.cmd(f"sub tlm {decimate}")

    def unsubscribe_tlm(self) -> dict:
        return self.cmd("unsub tlm")

    def frames(self, duration: float | None = None) -> Iterator[dict]:  # [s]
        """Yield subscribed telemetry frames as they arrive (already-queued
        backlog first), for ``duration`` seconds (forever if ``None``)."""
        deadline = None if duration is None else time.monotonic() + duration
        while True:
            while self.tlm_frames:
                yield self.tlm_frames.popleft()
            remaining = 3600.0 if deadline is None else deadline - time.monotonic()
            if remaining <= 0:
                return
            try:
                self._read_one_event(remaining)  # tlm events route to the deque
            except RogoTimeout:
                if deadline is not None:
                    return
                continue

    # -- internals -----------------------------------------------------------

    def _read_line(self, timeout: float) -> str:  # [s]
        """One newline-terminated reply line, buffering partials — a timeout
        leaves any partial line in ``self._buf`` and the stream intact
        (``socket.makefile`` readers corrupt themselves on timeout, which is
        why this class does its own line assembly)."""
        deadline = time.monotonic() + timeout
        while b"\n" not in self._buf:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RogoTimeout("daemon reply wait expired")
            self._sock.settimeout(remaining)
            try:
                chunk = self._sock.recv(65536)
            except socket.timeout as exc:
                raise RogoTimeout("daemon reply wait expired") from exc
            except OSError as exc:
                raise RogoDaemonError(f"connection lost: {exc}") from exc
            if not chunk:
                raise RogoDaemonError("daemon closed the connection")
            self._buf += chunk
        line, _, self._buf = self._buf.partition(b"\n")
        return line.decode("utf-8", errors="replace")

    def _read_one_event(self, timeout: float) -> dict | None:  # [s]
        """Read + parse one event line. Telemetry pushes are routed (callback
        or deque) and returned as well; ``shutdown`` raises."""
        line = self._read_line(timeout)
        try:
            event = json.loads(line)
        except json.JSONDecodeError as exc:
            raise RogoDaemonError(f"unparseable daemon reply: {line!r}") from exc
        kind = event.get("type")
        if kind == "tlm":
            frame = event.get("frame", {})
            if self.on_tlm is not None:
                self.on_tlm(frame)
            else:
                self.tlm_frames.append(frame)
        elif kind == "shutdown":
            raise RogoDaemonError("daemon shut down")
        return event

    def _await_event(self, timeout: float,  # [s]
                     predicate: Callable[[dict], bool]) -> dict:
        """Read events (routing telemetry) until ``predicate`` matches one."""
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RogoTimeout("daemon reply wait expired")
            event = self._read_one_event(remaining)
            if event is not None and predicate(event):
                return event


# ---------------------------------------------------------------------------
# `rogo repl --connect` — the interactive/pipe repl run through the daemon
# ---------------------------------------------------------------------------

def run_connected(args: Any, verbose: bool) -> int:
    """Entry point for ``rogo repl --connect [ADDR]`` (called from
    ``repl.run()``): same three input modes as the direct repl, but every
    line goes to the daemon instead of a locally-opened serial port. Ctrl-C
    sends an ``estop`` through the daemon before exiting — an interactive
    user's interrupt means "stop the robot", same posture as the direct
    repl's own Ctrl-C cleanup."""
    import sys

    def _print_tlm(frame: dict) -> None:
        print(f"  tlm {json.dumps(frame)}")

    try:
        client = RogoClient(getattr(args, "connect", None) or None,
                            on_tlm=_print_tlm)
    except RogoDaemonError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    hello = client.hello
    print(f"connected to rogo daemon at {client.addr[0]}:{client.addr[1]} "
          f"(serial={hello.get('serial_port')}, mode={hello.get('mode')})",
          file=sys.stderr)

    def _one(line: str) -> bool:
        """Send one line; print its output. Returns False to exit."""
        line = line.strip()
        if not line or line.startswith("#"):
            return True
        if line in ("quit", "exit"):
            return False
        result = client.cmd(line)
        for out_line in result.get("output", []):
            print(out_line)
        if result.get("status") is not None:
            print(f"  {json.dumps(result['status'])}")
        if not result.get("ok") and result.get("error"):
            print(f"  error: {result['error']}", file=sys.stderr)
        return True

    commands = getattr(args, "commands", None) or []
    try:
        if commands:
            for cmd in " ".join(commands).split(";"):
                if not _one(cmd):
                    break
        elif sys.stdin.isatty():
            print("rogo REPL (daemon) — 'help' for commands, 'quit' to exit.",
                  file=sys.stderr)
            while True:
                try:
                    line = input("rogo> ")
                except EOFError:
                    break
                if not _one(line):
                    break
        else:
            for line in sys.stdin:
                if not _one(line):
                    break
    except KeyboardInterrupt:
        print("\ninterrupted -- sending estop through the daemon", file=sys.stderr)
        try:
            result = client.estop()
            for out_line in result.get("output", []):
                print(out_line, file=sys.stderr)
        except RogoDaemonError as exc:
            print(f"ESTOP FAILED ({exc}) -- ROBOT MAY STILL BE MOVING",
                  file=sys.stderr)
    except RogoDaemonError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        client.close()
        return 1
    client.close()
    return 0
