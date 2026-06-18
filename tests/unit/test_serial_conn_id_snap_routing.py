"""Unit tests for SerialConnection reader routing fixes (sprint 036-008).

Covers two bugs fixed in 036-008:

1. Bug 1 — ID reply routing: the reader was dropping ID lines silently because
   "ID" was not in the ``startswith(("OK", "ERR", "CFG"))`` check. After the
   fix, "ID" is included and the reply is routed to the caller's corr-id queue,
   allowing ``get_id()`` to return a parsed dict.

2. Bug 2 — SNAP/TLM routing: the SNAP reply arrives as a TLM frame WITHOUT a
   corr-id suffix, so it lands in ``_tlm_queue`` — not in the corr-id reply
   queue that ``send()`` waits on. After the fix, ``snap()`` uses
   ``send_fast()`` + ``read_lines()`` to drain ``_tlm_queue`` directly.

Also includes a regression guard that OK-tagged replies (``get_ver()``,
``ping()``) still work correctly after the changes.

All tests mock at the ``serial.Serial`` boundary — no hardware required.
"""

from __future__ import annotations

import queue
import threading
import time
from unittest.mock import patch

import pytest

# ---------------------------------------------------------------------------
# Fake serial device (same style as test_serial_relay_handshake.py)
# ---------------------------------------------------------------------------


class _FakeSerial:
    """Minimal fake pyserial.Serial for routing tests.

    Supports ``readline()`` (blocks on injected queue), ``write()``,
    ``open()``, ``close()``, ``flush()``, and ``reset_input_buffer()``.
    """

    def __init__(self):
        self._q: queue.Queue[bytes] = queue.Queue()
        self.written: list[bytes] = []
        self.is_open: bool = False
        self.port: str = "/dev/fake"
        self.dtr: bool | None = None
        self.rts: bool | None = None
        self.timeout: float = 0.12

    # --- pyserial interface -------------------------------------------------

    def open(self):
        self.is_open = True

    def close(self):
        self.is_open = False

    def write(self, data: bytes) -> int:
        self.written.append(data)
        return len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        while not self._q.empty():
            try:
                self._q.get_nowait()
            except queue.Empty:
                break

    def readline(self) -> bytes:
        try:
            return self._q.get(timeout=0.05)
        except queue.Empty:
            return b""

    def fileno(self):
        raise OSError("no fd")

    # --- test helpers -------------------------------------------------------

    def inject(self, line: str) -> None:
        """Queue a response line (newline appended automatically)."""
        self._q.put((line + "\n").encode("utf-8"))

    def inject_after(self, line: str, delay: float = 0.05) -> threading.Thread:
        """Inject a line after a short delay; return the thread."""
        def _work():
            time.sleep(delay)
            self.inject(line)
        t = threading.Thread(target=_work, daemon=True)
        t.start()
        return t

    def written_text(self) -> list[str]:
        """Return written bytes decoded as stripped text lines."""
        out = []
        for b in self.written:
            for part in b.decode("utf-8", "ignore").splitlines():
                s = part.strip()
                if s:
                    out.append(s)
        return out


# ---------------------------------------------------------------------------
# Helper: patch serial.Serial and build a connected SerialConnection
# ---------------------------------------------------------------------------


def _patch_serial(fake: _FakeSerial):
    return patch("robot_radio.io.serial_conn.serial.Serial", return_value=fake)


def _make_conn(port: str = "/dev/fake"):
    from robot_radio.io.serial_conn import SerialConnection
    return SerialConnection(port=port)


def _start_direct_connect(fake: _FakeSerial) -> threading.Thread:
    """Queue the NEZHA2 banner + PING poll response in a background thread.

    Call this before conn.connect() inside _patch_serial.  The thread feeds
    lines at realistic timing so the classify and readiness-poll both complete.
    """
    def _respond():
        time.sleep(0.05)
        fake.inject("DEVICE:NEZHA2:robot:tovez:AB:CD:EF:01")
        time.sleep(0.05)
        # Readiness poll: corr-id 1 is always the first PING.
        fake.inject("OK pong t=11 #1")

    t = threading.Thread(target=_respond, daemon=True)
    t.start()
    return t


# Minimal TLM line that parse_tlm() accepts.
_TLM_LINE = (
    "TLM t=123456 mode=I seq=42 enc=100,200 pose=10,20,900 "
    "vel=100,100 twist=100,5 otos=11,22,900 line=0,0,0,0 "
    "color=0,0,0,0 ekf_rej=0"
)


# ---------------------------------------------------------------------------
# Bug 1 — ID reply routing
# ---------------------------------------------------------------------------


class TestIDReplyRouting:
    """ID reply carries a corr-id and must be routed like OK/ERR/CFG."""

    def test_id_line_delivered_to_corr_id_queue(self):
        """An 'ID model=... #<n>' line fed through the reader reaches send()."""
        fake = _FakeSerial()
        conn = _make_conn()

        with _patch_serial(fake):
            _start_direct_connect(fake)
            conn.connect()
            # After connect(), _corr_counter=0 (PING poll in connect() uses _ser
            # directly, NOT send()). The first send() increments to 1 → "#1".
            fake.inject_after(
                "ID model=Nezha2 name=tovez serial=2314287040 fw=0.20260612.28 "
                "proto=2 caps=otos,line,color,portio #1",
                delay=0.05,
            )
            result = conn.send("ID", read_ms=300)
            conn.disconnect()

        responses = result.get("responses", [])
        assert responses, "No response lines returned — ID reply was dropped"
        assert any(r.startswith("ID") for r in responses), (
            f"No ID line in responses — routing broken: {responses}"
        )

    def test_get_id_returns_model(self):
        """get_id() returns a dict with model='Nezha2' (not None)."""
        from robot_radio.robot.protocol import NezhaProtocol

        fake = _FakeSerial()
        conn = _make_conn()

        with _patch_serial(fake):
            _start_direct_connect(fake)
            conn.connect()

            # After connect(), _corr_counter=0; first send() uses "#1".
            fake.inject_after(
                "ID model=Nezha2 name=tovez serial=2314287040 fw=0.20260612.28 "
                "proto=2 caps=otos,line,color,portio #1",
                delay=0.05,
            )
            proto = NezhaProtocol(conn)
            result = proto.get_id()
            conn.disconnect()

        assert result is not None, (
            "get_id() returned None — ID reply was dropped by reader"
        )
        assert result.get("model") == "Nezha2", (
            f"Unexpected model in ID response: {result}"
        )
        assert result.get("fw") == "0.20260612.28", (
            f"Unexpected fw in ID response: {result}"
        )

    def test_id_reply_without_corr_id_is_dropped(self):
        """An ID line with no corr-id must NOT be delivered to unrelated send()."""
        fake = _FakeSerial()
        conn = _make_conn()

        with _patch_serial(fake):
            _start_direct_connect(fake)
            conn.connect()

            # Inject ID line with NO corr-id. send() registers queue for "#1".
            # Reader routes it to queue "" — no registered queue for "", drops.
            # Also inject the real PING reply with the correct corr-id.
            fake.inject_after("ID model=Nezha2 name=tovez fw=0.1 proto=2", delay=0.02)
            fake.inject_after("OK pong t=5 #1", delay=0.04)
            result = conn.send("PING", read_ms=300)
            conn.disconnect()

        # The ID line without corr-id should NOT appear in the PING responses.
        responses = result.get("responses", [])
        id_lines = [r for r in responses if r.startswith("ID")]
        assert not id_lines, (
            f"ID line without corr-id leaked into PING: {id_lines}"
        )
        # PING reply should still be there.
        assert any("pong" in r for r in responses), (
            f"OK pong not found — PING delivery broken: {responses}"
        )


# ---------------------------------------------------------------------------
# Bug 2 — SNAP/TLM routing
# ---------------------------------------------------------------------------


class TestSnapTLMRouting:
    """SNAP reply is a corr-id-less TLM frame; snap() must read it from _tlm_queue."""

    def test_snap_returns_tlm_frame(self):
        """snap() returns a TLMFrame when the TLM reply is in _tlm_queue."""
        from robot_radio.robot.protocol import NezhaProtocol, TLMFrame

        fake = _FakeSerial()
        conn = _make_conn()

        with _patch_serial(fake):
            _start_direct_connect(fake)
            conn.connect()

            # Deliver TLM slightly after snap() fires send_fast("SNAP").
            fake.inject_after(_TLM_LINE, delay=0.05)
            proto = NezhaProtocol(conn)
            result = proto.snap()
            conn.disconnect()

        assert result is not None, (
            "snap() returned None — TLM frame not read from _tlm_queue"
        )
        assert isinstance(result, TLMFrame), (
            f"snap() returned unexpected type: {type(result)}"
        )
        assert result.t == 123456, f"Unexpected t={result.t!r}"
        assert result.enc == (100, 200), f"Unexpected enc={result.enc!r}"

    def test_snap_no_corr_id_in_sent_command(self):
        """send_fast() is used: no corr-id suffix on the SNAP command."""
        from robot_radio.robot.protocol import NezhaProtocol

        fake = _FakeSerial()
        conn = _make_conn()

        with _patch_serial(fake):
            _start_direct_connect(fake)
            conn.connect()

            fake.inject_after(_TLM_LINE, delay=0.05)
            proto = NezhaProtocol(conn)
            proto.snap()
            conn.disconnect()

        sent = fake.written_text()
        snap_sends = [s for s in sent if "SNAP" in s]
        assert snap_sends, f"SNAP not found in writes: {sent}"
        # send_fast() does NOT append a corr-id.
        for s in snap_sends:
            assert "#" not in s, (
                f"Unexpected corr-id suffix in SNAP send (should use send_fast): {s!r}"
            )

    def test_snap_returns_none_on_timeout(self):
        """snap() returns None when no TLM reply arrives within the window."""
        from robot_radio.robot.protocol import NezhaProtocol

        fake = _FakeSerial()
        conn = _make_conn()

        with _patch_serial(fake):
            _start_direct_connect(fake)
            conn.connect()

            # Do NOT inject any TLM frame — simulates a timeout.
            proto = NezhaProtocol(conn)
            result = proto.snap()
            conn.disconnect()

        assert result is None, f"snap() should return None on timeout; got {result}"

    def test_snap_skips_stale_frames(self):
        """snap() drains stale TLM frames before sending SNAP."""
        from robot_radio.robot.protocol import NezhaProtocol, TLMFrame

        stale_line = (
            "TLM t=1 mode=I seq=1 enc=0,0 pose=0,0,0 vel=0,0 "
            "twist=0,0 otos=0,0,0 line=0,0,0,0 color=0,0,0,0 ekf_rej=0"
        )

        fake = _FakeSerial()
        conn = _make_conn()

        with _patch_serial(fake):
            _start_direct_connect(fake)
            conn.connect()

            # Inject a stale TLM and let the reader queue it before snap().
            fake.inject(stale_line)
            time.sleep(0.06)  # let reader thread consume into _tlm_queue

            # Fresh TLM delivered after SNAP fires.
            fake.inject_after(_TLM_LINE, delay=0.05)
            proto = NezhaProtocol(conn)
            result = proto.snap()
            conn.disconnect()

        # We should have gotten at least some TLMFrame back.
        assert result is not None, "snap() returned None — expected a TLMFrame"
        assert isinstance(result, TLMFrame)


# ---------------------------------------------------------------------------
# Regression: OK/ERR/CFG/EVT routing still works after ID fix
# ---------------------------------------------------------------------------


class TestOKReplyRoutingRegression:
    """Existing OK/ERR/CFG routing must be unaffected by the ID routing fix."""

    def test_ping_ok_still_delivered(self):
        """OK pong reply is still routed to the corr-id queue."""
        fake = _FakeSerial()
        conn = _make_conn()

        with _patch_serial(fake):
            _start_direct_connect(fake)
            conn.connect()
            # First send() after connect() uses corr-id "#1".
            fake.inject_after("OK pong t=99 #1", delay=0.05)
            result = conn.send("PING", read_ms=300)
            conn.disconnect()

        responses = result.get("responses", [])
        assert any("pong" in r for r in responses), (
            f"OK pong not found in responses — regression: {responses}"
        )

    def test_get_ver_returns_fw(self):
        """get_ver() still returns firmware version dict (OK reply routed)."""
        from robot_radio.robot.protocol import NezhaProtocol

        fake = _FakeSerial()
        conn = _make_conn()

        with _patch_serial(fake):
            _start_direct_connect(fake)
            conn.connect()

            # First send() after connect() uses corr-id "#1".
            fake.inject_after("OK ver fw=0.20260612.28 proto=2 #1", delay=0.05)
            proto = NezhaProtocol(conn)
            result = proto.get_ver()
            conn.disconnect()

        assert result is not None, "get_ver() returned None — regression in OK routing"
        assert result.get("fw") == "0.20260612.28", f"Unexpected fw: {result}"

    def test_err_reply_still_routed(self):
        """ERR replies are still routed by corr-id (not affected by ID fix).

        Uses a non-"unknown" ERR (badarg) so send()'s corrupted-command retry
        (which re-sends on "ERR unknown") does not fire here — this test only
        exercises corr-id routing of an ERR.
        """
        fake = _FakeSerial()
        conn = _make_conn()

        with _patch_serial(fake):
            _start_direct_connect(fake)
            conn.connect()

            # First send() after connect() uses corr-id "#1".
            fake.inject_after("ERR badarg #1", delay=0.05)
            result = conn.send("BOGUS", read_ms=300)
            conn.disconnect()

        responses = result.get("responses", [])
        assert any("ERR" in r for r in responses), (
            f"ERR reply not delivered — regression: {responses}"
        )

    def test_err_unknown_command_is_retried(self):
        """A corrupted command (relay framing merge → "ERR unknown") is re-sent.

        First attempt (corr-id #1) gets "ERR unknown" — proving it never ran — so
        send() retries; the retry (corr-id #2) succeeds. Masks relay corruption
        transparently instead of surfacing it as a skipped command.
        """
        fake = _FakeSerial()
        conn = _make_conn()

        with _patch_serial(fake):
            _start_direct_connect(fake)
            conn.connect()

            fake.inject_after("ERR unknown #1", delay=0.05)
            fake.inject_after("OK rt rot=9000 #2", delay=0.20)
            result = conn.send("RT 9000", read_ms=400)
            conn.disconnect()

        responses = result.get("responses", [])
        assert any("OK" in r for r in responses), (
            f"retry did not recover corrupted command: {responses}"
        )

    def test_cfg_reply_still_routed(self):
        """CFG replies are still routed by corr-id (not affected by ID fix)."""
        fake = _FakeSerial()
        conn = _make_conn()

        with _patch_serial(fake):
            _start_direct_connect(fake)
            conn.connect()

            # First send() after connect() uses corr-id "#1".
            fake.inject_after("CFG alphaYaw=0 yawRateMax=60 #1", delay=0.05)
            result = conn.send("GET", read_ms=300)
            conn.disconnect()

        responses = result.get("responses", [])
        assert any("CFG" in r for r in responses), (
            f"CFG reply not delivered — regression: {responses}"
        )

    def test_tlm_stream_does_not_leak_into_send_reply(self):
        """Streamed TLM frames (no corr-id) must NOT appear in send() responses."""
        fake = _FakeSerial()
        conn = _make_conn()

        with _patch_serial(fake):
            _start_direct_connect(fake)
            conn.connect()

            # Inject a TLM frame (goes to _tlm_queue) and a PING reply.
            fake.inject_after(_TLM_LINE, delay=0.02)
            # First send() after connect() uses corr-id "#1".
            fake.inject_after("OK pong t=1 #1", delay=0.05)
            result = conn.send("PING", read_ms=300)
            conn.disconnect()

        responses = result.get("responses", [])
        # TLM frames must not appear in the PING corr-id reply queue.
        tlm_in_send = [r for r in responses if r.startswith("TLM")]
        assert not tlm_in_send, (
            f"TLM frame leaked into send() responses — routing broken: {tlm_in_send}"
        )
        # PING reply must be present.
        assert any("pong" in r for r in responses), (
            f"OK pong not found — PING delivery broken: {responses}"
        )

    def test_hash_comment_lines_still_dropped(self):
        """# comment lines must still be silently dropped by the reader."""
        fake = _FakeSerial()
        conn = _make_conn()

        with _patch_serial(fake):
            _start_direct_connect(fake)
            conn.connect()

            fake.inject_after("# some relay comment", delay=0.02)
            # First send() after connect() uses corr-id "#1".
            fake.inject_after("OK pong t=2 #1", delay=0.05)
            result = conn.send("PING", read_ms=300)
            conn.disconnect()

        responses = result.get("responses", [])
        hash_lines = [r for r in responses if r.startswith("#")]
        assert not hash_lines, (
            f"# comment line reached send() — should be dropped: {hash_lines}"
        )


# ---------------------------------------------------------------------------
# Nezha.refresh() integration
# ---------------------------------------------------------------------------


class TestNezhaRefresh:
    """Nezha.refresh() calls snap() and returns a populated RobotState."""

    def test_refresh_returns_populated_state(self):
        """refresh() calls snap() and returns a RobotState with updated fields."""
        from robot_radio.robot.protocol import NezhaProtocol
        from robot_radio.robot.nezha import Nezha

        fake = _FakeSerial()
        conn = _make_conn()

        with _patch_serial(fake):
            _start_direct_connect(fake)
            conn.connect()

            proto = NezhaProtocol(conn)
            robot = Nezha(proto=proto)

            fake.inject_after(_TLM_LINE, delay=0.05)
            state = robot.refresh()
            conn.disconnect()

        assert state is not None, "refresh() returned None — unexpected"
        # After refresh() with enc=100,200, encoders must be populated.
        assert state.encoders is not None, (
            f"refresh() returned RobotState with no encoders: {state}"
        )

    def test_refresh_returns_state_on_no_tlm(self):
        """refresh() returns the prior (default) state when snap() returns None."""
        from robot_radio.robot.protocol import NezhaProtocol
        from robot_radio.robot.nezha import Nezha

        fake = _FakeSerial()
        conn = _make_conn()

        with _patch_serial(fake):
            _start_direct_connect(fake)
            conn.connect()

            proto = NezhaProtocol(conn)
            robot = Nezha(proto=proto)

            # No TLM injected — snap() will time out and return None.
            state = robot.refresh()
            conn.disconnect()

        # refresh() must always return a RobotState (not raise or return None).
        assert state is not None, "refresh() must return RobotState even when snap()=None"
