"""robot_radio.io.server — the rogo daemon: one held-open serial connection,
many TCP clients.

``rogo serve`` opens the robot's serial connection ONCE and keeps it open for
the daemon's whole lifetime, exposing the repl's verb grammar to any number of
local programs over TCP (default ``127.0.0.1:7646`` — "ROGO" on a phone
keypad). This is the fix for the one-shot-CLI failure mode documented in
``clasi/issues/later/A-port-close-resets-the-robot-live-config-still-wiped.md``:
on macOS, CLOSING the serial port drops DTR (HUPCL) and resets the MCU, so
every ``rogo <cmd>`` invocation that opens/closes the port reboots the robot
and wipes live-pushed config. A daemon that never closes the port never
resets the robot between commands.

Wire protocol (client side — see ``robot_radio.io.client.RogoClient``)
----------------------------------------------------------------------
Requests are newline-delimited. Two request shapes, same grammar:

  * plain text — exactly a repl command line (``drive 200``, ``estop``,
    ``config pid.kp=0.4``); the server assigns a request id.
  * JSON — ``{"id": <any>, "cmd": "<repl line>"}`` — the reply echoes ``id``
    so a pipelining client can correlate.

Every reply is one JSON line:

  * ``{"type": "hello", ...}``   — once, on connect (serial port, mode).
  * ``{"type": "result", "id": ..., "ok": bool, "error": str|null,
       "output": [<human line>, ...]}`` — one per request.
  * ``{"type": "tlm", "frame": {...}}`` — telemetry pushes, only after
    ``sub tlm [decimate]`` (``unsub tlm`` stops them).

Server-local verbs (handled here, never reaching the robot):
``sub tlm [N]`` / ``unsub tlm`` — telemetry subscription (every Nth frame);
``status`` — daemon status; ``quit``/``exit`` — drop THIS client (the serial
port stays open, other clients unaffected); ``shutdown`` — halt the robot and
stop the daemon.

Threading model — one wire owner, no exceptions
-----------------------------------------------
All serial I/O happens on the single EXECUTOR thread, which alternates
between draining the command queue and pumping telemetry (the pump feeds
recording and the tlm broadcast via ``RogoSession.on_frame``). Per-client
reader threads only parse lines and enqueue work; they never touch the wire.

The one cross-thread signal is the panic path: a reader that receives an
``estop``/``halt`` line sets ``session.abort_event`` (which makes any
in-progress completion wait return early) and enqueues the halt at the FRONT
of the queue — so another client's long ``drive`` can never delay a halt by
more than one loop iteration. This mirrors
``.claude/rules/playfield-testing.md``'s halting rules; the halt itself runs
the repl's ``estop`` verb, which verifies the active flag actually cleared
and re-issues if not.
"""
from __future__ import annotations

import collections
import dataclasses
import io
import json
import shlex
import socket
import sys
import threading
import time
from typing import Any

from robot_radio.io.repl import HALT_VERBS, RogoSession, dispatch
from robot_radio.robot.protocol import TLMFrame

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 7646  # "ROGO" on a phone keypad
_IDLE_PUMP_INTERVAL = 0.02  # [s] executor telemetry-pump cadence when idle


def parse_addr(addr: str | None) -> tuple[str, int]:
    """Parse ``HOST:PORT`` / ``:PORT`` / ``PORT`` / ``HOST`` (empty/None ->
    the default listen address)."""
    if not addr:
        return (DEFAULT_HOST, DEFAULT_PORT)
    if ":" in addr:
        host, _, port_s = addr.rpartition(":")
        return (host or DEFAULT_HOST, int(port_s))
    if addr.isdigit():
        return (DEFAULT_HOST, int(addr))
    return (addr, DEFAULT_PORT)


def frame_to_json(frame: TLMFrame) -> dict:
    """One telemetry frame as a JSON-ready dict — the same shape
    ``repl.Recorder`` writes, plus the host receive timestamp."""
    row = dataclasses.asdict(frame)
    row["t_recv"] = time.time()
    return row


class _Client:
    """One connected TCP client: socket, send lock, telemetry subscription."""

    _next_id = [1]

    def __init__(self, sock: socket.socket, addr: tuple) -> None:
        self.sock = sock
        self.addr = addr
        self.id = _Client._next_id[0]
        _Client._next_id[0] += 1
        self.alive = True
        self._send_lock = threading.Lock()
        self._req_counter = 0
        self.tlm_decimate = 0   # 0 = not subscribed; N = every Nth frame
        self._tlm_countdown = 0

    def next_req_id(self) -> str:
        self._req_counter += 1
        return f"c{self.id}-{self._req_counter}"

    def send_json(self, obj: dict) -> bool:
        """Serialize + send one reply line. Returns False (and marks the
        client dead) on any socket error — the caller reaps."""
        data = (json.dumps(obj) + "\n").encode()
        try:
            with self._send_lock:
                self.sock.sendall(data)
            return True
        except OSError:
            self.alive = False
            return False

    def close(self) -> None:
        self.alive = False
        try:
            self.sock.close()
        except OSError:
            pass


class RogoServer:
    """The daemon: accept loop + per-client readers + the single executor
    that owns the serial wire. Construct with an open ``RogoSession``
    (the CLI builds a real one; tests inject ``RogoSession.from_protocol()``
    around a fake)."""

    def __init__(self, session: RogoSession,
                 host: str = DEFAULT_HOST, port: int = DEFAULT_PORT) -> None:
        self.session = session
        self._listen_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listen_sock.bind((host, port))
        self._listen_sock.listen(8)
        self.host, self.port = self._listen_sock.getsockname()[:2]
        self._clients: list[_Client] = []
        self._clients_lock = threading.Lock()
        # Work items are (client, req_id, line). One deque per priority
        # class; the executor always drains halts first.
        self._halt_queue: collections.deque = collections.deque()
        self._cmd_queue: collections.deque = collections.deque()
        self._work_available = threading.Event()
        self._stop = threading.Event()
        self._started = time.monotonic()
        self._threads: list[threading.Thread] = []
        # The broadcast hook: EVERY session pump (idle loop, or inside a
        # command's ack/completion wait) hands each frame here exactly once.
        session.on_frame = self._broadcast_frame

    # -- lifecycle -----------------------------------------------------------

    def start(self) -> None:
        """Start the accept and executor threads (non-blocking)."""
        for name, target in (("rogo-accept", self._accept_loop),
                             ("rogo-exec", self._executor_loop)):
            t = threading.Thread(target=target, name=name, daemon=True)
            t.start()
            self._threads.append(t)

    def serve_forever(self) -> None:
        """``start()`` + block until ``shutdown()`` (or KeyboardInterrupt)."""
        self.start()
        try:
            while not self._stop.wait(0.2):
                pass
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        """Stop the daemon: halt the robot (via ``session.close()``, which
        runs ``halt_now`` before disconnecting), drop every client, close the
        listen socket. Idempotent."""
        if self._stop.is_set():
            return
        self._stop.set()
        self.session.abort_event.set()  # unstick any in-progress wait
        self._work_available.set()
        try:
            self._listen_sock.close()
        except OSError:
            pass
        with self._clients_lock:
            clients = list(self._clients)
            self._clients.clear()
        for c in clients:
            c.send_json({"type": "shutdown"})
            c.close()
        for t in self._threads:
            if t is not threading.current_thread():
                t.join(timeout=2.0)
        self.session.close()

    # -- accept + per-client reader ------------------------------------------

    def _accept_loop(self) -> None:
        while not self._stop.is_set():
            try:
                sock, addr = self._listen_sock.accept()
            except OSError:
                return  # listen socket closed -- shutting down
            client = _Client(sock, addr)
            with self._clients_lock:
                self._clients.append(client)
            t = threading.Thread(target=self._client_loop, args=(client,),
                                 name=f"rogo-client-{client.id}", daemon=True)
            t.start()

    def _client_loop(self, client: _Client) -> None:
        meta = self.session._meta if isinstance(self.session._meta, dict) else {}
        client.send_json({
            "type": "hello", "server": "rogo", "client_id": client.id,
            "serial_port": meta.get("port"), "mode": meta.get("mode"),
        })
        try:
            reader = client.sock.makefile("r", encoding="utf-8", errors="replace")
            for raw in reader:
                line = raw.strip()
                if not line:
                    continue
                req_id, cmd = self._parse_request(client, line)
                if not self._handle_request(client, req_id, cmd):
                    break
        except OSError:
            pass
        finally:
            client.close()
            with self._clients_lock:
                if client in self._clients:
                    self._clients.remove(client)

    @staticmethod
    def _parse_request(client: _Client, line: str) -> tuple[Any, str]:
        """(req_id, command line) from either request shape."""
        if line.startswith("{"):
            try:
                obj = json.loads(line)
                req_id = obj.get("id")
                if req_id is None:
                    req_id = client.next_req_id()
                return req_id, str(obj.get("cmd", ""))
            except (json.JSONDecodeError, AttributeError):
                return client.next_req_id(), line  # not JSON after all
        return client.next_req_id(), line

    def _handle_request(self, client: _Client, req_id: Any, cmd: str) -> bool:
        """Route one request. Returns False to drop the client connection."""
        cmd = cmd.strip()
        if not cmd:
            client.send_json({"type": "result", "id": req_id, "ok": True,
                              "error": None, "output": []})
            return True
        try:
            tokens = shlex.split(cmd)
        except ValueError as exc:
            client.send_json({"type": "result", "id": req_id, "ok": False,
                              "error": f"parse error: {exc}", "output": []})
            return True
        verb = tokens[0]

        if verb in ("quit", "exit"):
            client.send_json({"type": "result", "id": req_id, "ok": True,
                              "error": None, "output": ["bye"]})
            return False
        if verb == "sub":
            return self._handle_sub(client, req_id, tokens)
        if verb == "unsub":
            client.tlm_decimate = 0
            client.send_json({"type": "result", "id": req_id, "ok": True,
                              "error": None, "output": ["tlm unsubscribed"]})
            return True
        if verb == "status":
            client.send_json({"type": "result", "id": req_id, "ok": True,
                              "error": None, "output": [],
                              "status": self._status()})
            return True
        if verb == "shutdown":
            client.send_json({"type": "result", "id": req_id, "ok": True,
                              "error": None, "output": ["shutting down"]})
            threading.Thread(target=self.shutdown, name="rogo-shutdown",
                             daemon=True).start()
            return False

        if verb in HALT_VERBS:
            # Panic path: abort whatever wait the executor is in, and put
            # the halt at the FRONT of the work queue.
            self.session.abort_event.set()
            self._halt_queue.append((client, req_id, cmd))
        else:
            self._cmd_queue.append((client, req_id, cmd))
        self._work_available.set()
        return True

    def _handle_sub(self, client: _Client, req_id: Any, tokens: list[str]) -> bool:
        if len(tokens) < 2 or tokens[1] != "tlm":
            client.send_json({"type": "result", "id": req_id, "ok": False,
                              "error": "usage: sub tlm [decimate]", "output": []})
            return True
        decimate = 1
        if len(tokens) > 2:
            try:
                decimate = max(1, int(tokens[2]))
            except ValueError:
                client.send_json({"type": "result", "id": req_id, "ok": False,
                                  "error": f"decimate must be an integer, got {tokens[2]!r}",
                                  "output": []})
                return True
        client.tlm_decimate = decimate
        client._tlm_countdown = 0
        client.send_json({"type": "result", "id": req_id, "ok": True, "error": None,
                          "output": [f"tlm subscribed (every {decimate} frame(s))"]})
        return True

    def _status(self) -> dict:
        meta = self.session._meta if isinstance(self.session._meta, dict) else {}
        with self._clients_lock:
            n_clients = len(self._clients)
        return {
            "serial_port": meta.get("port"),
            "mode": meta.get("mode"),
            "uptime": round(time.monotonic() - self._started, 1),  # [s]
            "clients": n_clients,
            "queued": len(self._cmd_queue) + len(self._halt_queue),
        }

    # -- executor (the one serial-wire owner) --------------------------------

    def _executor_loop(self) -> None:
        while not self._stop.is_set():
            item = self._take_work()
            if item is None:
                try:
                    self.session.pump()  # feeds _broadcast_frame via on_frame
                except ConnectionError:
                    self._stop.set()
                    return
                self._work_available.wait(_IDLE_PUMP_INTERVAL)
                continue
            client, req_id, cmd = item
            self._execute(client, req_id, cmd)

    def _take_work(self):
        if self._halt_queue:
            return self._halt_queue.popleft()
        if self._cmd_queue:
            return self._cmd_queue.popleft()
        self._work_available.clear()
        # Re-check after clearing -- a reader may have raced the clear.
        if self._halt_queue:
            return self._halt_queue.popleft()
        if self._cmd_queue:
            return self._cmd_queue.popleft()
        return None

    def _execute(self, client: _Client, req_id: Any, cmd: str) -> None:
        # A queued halt owns the abort flag; clear it so the halt's own
        # verification pumps aren't self-aborted. (Any non-halt command that
        # was waiting when the flag went up has already returned early.)
        first = cmd.split(None, 1)[0] if cmd.split() else ""
        if first in HALT_VERBS:
            self.session.abort_event.clear()
        buf = io.StringIO()
        self.session.out = buf
        self.session.errout = buf
        error: str | None = None
        try:
            result = dispatch(self.session, cmd)
            error = result.error
        except ConnectionError as exc:
            error = f"connection error: {exc}"
            self._stop.set()
        except Exception as exc:  # a verb bug must not kill the daemon
            error = f"internal error: {exc!r}"
        finally:
            self.session.out = sys.stdout
            self.session.errout = sys.stderr
        client.send_json({
            "type": "result", "id": req_id, "ok": error is None,
            "error": error, "output": buf.getvalue().splitlines(),
        })

    # -- telemetry broadcast (called from session.pump, executor thread) -----

    def _broadcast_frame(self, frame: TLMFrame) -> None:
        with self._clients_lock:
            subscribers = [c for c in self._clients if c.alive and c.tlm_decimate > 0]
        if not subscribers:
            return
        row = None
        dead = []
        for c in subscribers:
            c._tlm_countdown -= 1
            if c._tlm_countdown > 0:
                continue
            c._tlm_countdown = c.tlm_decimate
            if row is None:
                row = frame_to_json(frame)
            if not c.send_json({"type": "tlm", "frame": row}):
                dead.append(c)
        if dead:
            with self._clients_lock:
                for c in dead:
                    if c in self._clients:
                        self._clients.remove(c)
