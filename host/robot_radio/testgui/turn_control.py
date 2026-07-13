"""robot_radio.testgui.turn_control — a tiny line-JSON TCP control socket.

Lets an external process (an agent, a script) drive the TestGUI turn graphs
while a human watches: fire in-place pivots, send arbitrary command lines
(e.g. live ``SET`` config), clear the traces, and pull the recorded series
back for analysis. Needed because the robot is reached over the radio relay,
whose serial port has a SINGLE owner (the running GUI) — so the only way in
is through the GUI itself.

Wire protocol: one JSON object per line (UTF-8, ``\\n`` terminated). Requests:

    {"cmd": "ping"}                 -> {"ok": true, "pong": true}
    {"cmd": "turn", "deg": 90}      -> {"ok": true, "reply": "<line>"}  (SEG pivot)
    {"cmd": "send", "line": "..."}  -> {"ok": true, "reply": "<line>"}  (any command)
    {"cmd": "clear"}                -> {"ok": true}
    {"cmd": "get"}                  -> {"ok": true, "series": {name: [[t,v],...]}}

All GUI-touching work is marshalled onto the Qt main thread via a QObject
bridge (blocking-queued for the ones that return a value); the socket thread
never touches Qt or the transport directly.
"""
from __future__ import annotations

import json
import socket
import threading
from typing import Callable

from PySide6.QtCore import QObject, Signal, Slot

DEFAULT_PORT = 8127


class _Bridge(QObject):
    """Lives on the Qt main thread; its slots run there when signals fire."""

    _line = Signal(object)      # {"line": str, "event": Event, "result": [reply]}
    _clear = Signal()
    _snapshot = Signal(object)  # {"event": Event, "result": dict}

    def __init__(self, send_line: Callable[[str], str], clear_fn: Callable[[], None],
                 get_series: Callable[[], dict]) -> None:
        super().__init__()
        self._send_line = send_line
        self._clear_fn = clear_fn
        self._get_series = get_series
        self._line.connect(self._do_line)
        self._clear.connect(self._do_clear)
        self._snapshot.connect(self._do_snapshot)

    @Slot(object)
    def _do_line(self, req: dict) -> None:
        try:
            req["result"][0] = self._send_line(req["line"])
        except Exception as exc:  # noqa: BLE001 — never let a bad line kill the GUI
            req["result"][0] = f"__error__: {exc}"
        finally:
            req["event"].set()

    @Slot()
    def _do_clear(self) -> None:
        try:
            self._clear_fn()
        except Exception:  # noqa: BLE001
            pass

    @Slot(object)
    def _do_snapshot(self, req: dict) -> None:
        try:
            series = self._get_series()
            # Copy on the main thread (where series is mutated) so the socket
            # thread never iterates a dict/list mid-append.
            req["result"].update({k: [[float(t), float(v)] for (t, v) in pts]
                                  for k, pts in series.items()})
        except Exception as exc:  # noqa: BLE001
            req["result"]["__error__"] = str(exc)
        finally:
            req["event"].set()


class TurnControlServer:
    """Background TCP server marshalling requests onto the Qt main thread."""

    def __init__(self, send_line: Callable[[str], str], clear_fn: Callable[[], None],
                 get_series: Callable[[], dict], port: int = DEFAULT_PORT,
                 host: str = "127.0.0.1") -> None:
        self._bridge = _Bridge(send_line, clear_fn, get_series)  # created on main thread
        self._port = port
        self._host = host
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    def start(self) -> int | None:
        try:
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((self._host, self._port))
            srv.listen(4)
            srv.settimeout(0.5)
        except OSError:
            return None
        self._sock = srv
        self._thread = threading.Thread(target=self._serve, name="turn-control", daemon=True)
        self._thread.start()
        return self._port

    def stop(self) -> None:
        self._stop.set()
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:  # noqa: BLE001
                pass

    # --- server internals -------------------------------------------------
    def _send_line_sync(self, line: str) -> str:
        box = {"line": line, "event": threading.Event(), "result": [""]}
        self._bridge._line.emit(box)
        if not box["event"].wait(timeout=5.0):
            return "__error__: timeout"
        return box["result"][0]

    def _serve(self) -> None:
        while not self._stop.is_set():
            try:
                conn, _addr = self._sock.accept()  # type: ignore[union-attr]
            except socket.timeout:
                continue
            except OSError:
                break
            with conn:
                self._handle(conn)

    def _handle(self, conn: socket.socket) -> None:
        conn.settimeout(60.0)
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = conn.recv(4096)
            except (socket.timeout, OSError):
                break
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                reply = self._dispatch(line.decode("utf-8", "replace").strip())
                try:
                    conn.sendall((json.dumps(reply) + "\n").encode("utf-8"))
                except OSError:
                    return

    def _dispatch(self, line: str) -> dict:
        if not line:
            return {"ok": False, "error": "empty"}
        try:
            req = json.loads(line)
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": f"bad json: {exc}"}
        cmd = req.get("cmd")
        if cmd == "ping":
            return {"ok": True, "pong": True}
        if cmd == "turn":
            try:
                cdeg = int(round(float(req["deg"]) * 100))
            except Exception:  # noqa: BLE001
                return {"ok": False, "error": "turn needs numeric 'deg'"}
            return {"ok": True, "reply": self._send_line_sync(f"SEG 0 {cdeg}")}
        if cmd == "send":
            wire = str(req.get("line", "")).strip()
            if not wire:
                return {"ok": False, "error": "send needs 'line'"}
            return {"ok": True, "reply": self._send_line_sync(wire)}
        if cmd == "clear":
            self._bridge._clear.emit()
            return {"ok": True}
        if cmd == "get":
            box: dict = {"event": threading.Event(), "result": {}}
            self._bridge._snapshot.emit(box)
            if not box["event"].wait(timeout=3.0):
                return {"ok": False, "error": "snapshot timeout"}
            return {"ok": True, "series": box["result"]}
        return {"ok": False, "error": f"unknown cmd: {cmd!r}"}
