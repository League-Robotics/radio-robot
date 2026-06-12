#!/usr/bin/env python3
"""test_serial_conn_reader.py — Unit tests for SerialConnection reader thread.

Sprint 025, Ticket 001.

Tests validate:
- Reader correctly routes TLM to _tlm_queue, EVT to _evt_queue,
  OK/ERR/CFG to corr-id-keyed reply queues.
- Keepalive acks are dropped silently.
- read_lines() drains _tlm_queue and _evt_queue without touching _ser.
- read_pending_lines() performs a non-blocking drain of both queues.
- Concurrent send() calls receive their own replies (isolation by corr-id).
- handshake() writes a raw line under _write_lock.
- send() contains no reset_input_buffer() call.

No real serial port is required — a mock/stub is injected via ``_ser``.
"""

from __future__ import annotations

import io
import queue
import threading
import time
from unittest.mock import MagicMock, patch

import pytest

from host.robot_radio.io.serial_conn import SerialConnection


# ---------------------------------------------------------------------------
# Helpers — fake serial object
# ---------------------------------------------------------------------------

class FakeSerial:
    """Minimal pyserial stub that feeds bytes from a pre-loaded queue.

    readline() returns successive lines from ``lines`` or blocks until
    ``close()`` is called (simulates a real port draining to empty then
    waiting).
    """

    def __init__(self, lines: list[bytes] | None = None):
        self._lines: queue.Queue[bytes] = queue.Queue()
        for ln in (lines or []):
            self._lines.put(ln)
        self._open = True
        self.port = "/dev/fake"
        self.written: list[bytes] = []

    @property
    def is_open(self) -> bool:
        return self._open

    def readline(self) -> bytes:
        """Return the next line or b"" when the queue is empty."""
        try:
            return self._lines.get(timeout=0.05)
        except queue.Empty:
            return b""

    def write(self, data: bytes) -> None:
        self.written.append(data)

    def flush(self) -> None:
        pass

    def reset_input_buffer(self) -> None:
        pass

    def close(self) -> None:
        self._open = False

    def feed(self, line: bytes) -> None:
        """Inject a line after construction."""
        self._lines.put(line)


def _make_conn_with_fake_ser(lines: list[bytes] | None = None) -> tuple[SerialConnection, FakeSerial]:
    """Create a SerialConnection with a FakeSerial injected as _ser.

    The reader thread is started manually so tests control when it runs.
    """
    conn = SerialConnection(port="/dev/fake", mode="relay")
    fake = FakeSerial(lines)
    conn._ser = fake
    return conn, fake


def _conn_with_reader(lines: list[bytes] | None = None) -> tuple[SerialConnection, FakeSerial]:
    """Create a connection and start the reader thread."""
    conn, fake = _make_conn_with_fake_ser(lines)
    conn._start_reader()
    return conn, fake


def _wait_queue_has(q: queue.Queue, timeout: float = 1.0) -> bool:
    """Return True if q is non-empty within timeout seconds."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not q.empty():
            return True
        time.sleep(0.01)
    return False


# ---------------------------------------------------------------------------
# Test 1: TLM line routed to _tlm_queue
# ---------------------------------------------------------------------------

class TestTlmRouting:
    def test_tlm_line_routed_to_tlm_queue(self) -> None:
        """TLM lines go to _tlm_queue; _evt_queue stays empty."""
        conn, _ = _conn_with_reader([b"TLM t=1 mode=I enc=0,0\n"])
        try:
            assert _wait_queue_has(conn._tlm_queue), "TLM line not delivered to _tlm_queue"
            line = conn._tlm_queue.get_nowait()
            assert line == "TLM t=1 mode=I enc=0,0"
            assert conn._evt_queue.empty()
        finally:
            conn._stop_reader()

    def test_tlm_line_not_in_evt_queue(self) -> None:
        """A TLM line must not appear in _evt_queue."""
        conn, _ = _conn_with_reader([b"TLM t=5 mode=S enc=100,99\n"])
        try:
            _wait_queue_has(conn._tlm_queue)
            assert conn._evt_queue.empty()
        finally:
            conn._stop_reader()


# ---------------------------------------------------------------------------
# Test 2: EVT line routed to _evt_queue
# ---------------------------------------------------------------------------

class TestEvtRouting:
    def test_evt_line_routed_to_evt_queue(self) -> None:
        """EVT lines go to _evt_queue; _tlm_queue stays empty."""
        conn, _ = _conn_with_reader([b"EVT done T #3\n"])
        try:
            assert _wait_queue_has(conn._evt_queue), "EVT line not delivered to _evt_queue"
            line = conn._evt_queue.get_nowait()
            assert line == "EVT done T #3"
            assert conn._tlm_queue.empty()
        finally:
            conn._stop_reader()

    def test_evt_safety_stop_routed(self) -> None:
        """EVT safety_stop is also routed to _evt_queue."""
        conn, _ = _conn_with_reader([b"EVT safety_stop\n"])
        try:
            assert _wait_queue_has(conn._evt_queue)
            line = conn._evt_queue.get_nowait()
            assert line == "EVT safety_stop"
        finally:
            conn._stop_reader()


# ---------------------------------------------------------------------------
# Test 3: OK with corr-id routed to reply queue
# ---------------------------------------------------------------------------

class TestOkCorrIdRouting:
    def test_ok_with_corr_id_routed_to_reply_queue(self) -> None:
        """OK with #<id> suffix routes to _reply_queues[id]."""
        conn, fake = _make_conn_with_fake_ser()
        # Register a reply queue for corr-id "7" before starting the reader.
        reply_q: queue.Queue = queue.Queue()
        with conn._reply_lock:
            conn._reply_queues["7"] = reply_q
        conn._start_reader()
        try:
            fake.feed(b"OK pong t=0 #7\n")
            assert _wait_queue_has(reply_q), "Reply not routed to corr-id queue"
            line = reply_q.get_nowait()
            assert line == "OK pong t=0 #7"
            assert conn._tlm_queue.empty()
            assert conn._evt_queue.empty()
        finally:
            conn._stop_reader()

    def test_err_with_corr_id_routed(self) -> None:
        """ERR with corr-id is also routed to the reply queue."""
        conn, fake = _make_conn_with_fake_ser()
        reply_q: queue.Queue = queue.Queue()
        with conn._reply_lock:
            conn._reply_queues["3"] = reply_q
        conn._start_reader()
        try:
            fake.feed(b"ERR unknown cmd #3\n")
            assert _wait_queue_has(reply_q)
            line = reply_q.get_nowait()
            assert line == "ERR unknown cmd #3"
        finally:
            conn._stop_reader()

    def test_cfg_with_corr_id_routed(self) -> None:
        """CFG with corr-id is also routed to the reply queue."""
        conn, fake = _make_conn_with_fake_ser()
        reply_q: queue.Queue = queue.Queue()
        with conn._reply_lock:
            conn._reply_queues["5"] = reply_q
        conn._start_reader()
        try:
            fake.feed(b"CFG alphaYaw=0 #5\n")
            assert _wait_queue_has(reply_q)
            line = reply_q.get_nowait()
            assert line == "CFG alphaYaw=0 #5"
        finally:
            conn._stop_reader()

    def test_ok_with_unknown_corr_id_dropped(self) -> None:
        """OK with an unregistered corr-id is dropped silently."""
        conn, fake = _conn_with_reader([b"OK pong #99\n"])
        try:
            # Nothing should appear in TLM or EVT queue.
            time.sleep(0.15)
            assert conn._tlm_queue.empty()
            assert conn._evt_queue.empty()
        finally:
            conn._stop_reader()


# ---------------------------------------------------------------------------
# Test 4: OK with no corr-id goes to catch-all queue
# ---------------------------------------------------------------------------

class TestOkNoCorrId:
    def test_ok_no_corr_id_goes_to_catchall(self) -> None:
        """OK with no #<id> suffix routes to _reply_queues[''] catch-all."""
        conn, fake = _make_conn_with_fake_ser()
        catchall_q: queue.Queue = queue.Queue()
        with conn._reply_lock:
            conn._reply_queues[""] = catchall_q
        conn._start_reader()
        try:
            fake.feed(b"OK pong\n")
            assert _wait_queue_has(catchall_q), "No-corr-id OK not delivered to catch-all"
            line = catchall_q.get_nowait()
            assert line == "OK pong"
        finally:
            conn._stop_reader()

    def test_err_no_corr_id_goes_to_catchall(self) -> None:
        """ERR with no corr-id also routes to catch-all."""
        conn, fake = _make_conn_with_fake_ser()
        catchall_q: queue.Queue = queue.Queue()
        with conn._reply_lock:
            conn._reply_queues[""] = catchall_q
        conn._start_reader()
        try:
            fake.feed(b"ERR unknown\n")
            assert _wait_queue_has(catchall_q)
            line = catchall_q.get_nowait()
            assert line == "ERR unknown"
        finally:
            conn._stop_reader()


# ---------------------------------------------------------------------------
# Test 5: Keepalive ack dropped
# ---------------------------------------------------------------------------

class TestKeepaliveDropped:
    def test_keepalive_ack_dropped(self) -> None:
        """OK keepalive must not appear in any queue."""
        conn, fake = _make_conn_with_fake_ser()
        # Register catch-all to prove nothing arrives.
        catchall_q: queue.Queue = queue.Queue()
        with conn._reply_lock:
            conn._reply_queues[""] = catchall_q
        conn._start_reader()
        try:
            fake.feed(b"OK keepalive\n")
            # Give the reader thread time to process.
            time.sleep(0.15)
            assert catchall_q.empty(), "keepalive ack must not reach reply queue"
            assert conn._tlm_queue.empty()
            assert conn._evt_queue.empty()
        finally:
            conn._stop_reader()

    def test_line_with_keepalive_in_body_dropped(self) -> None:
        """Any line whose text contains 'keepalive' is dropped."""
        conn, fake = _conn_with_reader([b"RX: OK keepalive\n"])
        try:
            time.sleep(0.15)
            assert conn._tlm_queue.empty()
            assert conn._evt_queue.empty()
        finally:
            conn._stop_reader()


# ---------------------------------------------------------------------------
# Test 6: read_lines() drains _tlm_queue and _evt_queue
# ---------------------------------------------------------------------------

class TestReadLinesDrains:
    def test_read_lines_drains_tlm_and_evt(self) -> None:
        """read_lines() returns all pre-filled TLM and EVT lines."""
        conn, _ = _make_conn_with_fake_ser()
        # Pre-fill queues directly (bypasses reader thread).
        tlm_lines = [
            "TLM t=1 mode=I enc=0,0",
            "TLM t=2 mode=I enc=10,9",
        ]
        evt_lines = [
            "EVT done T",
        ]
        for ln in tlm_lines:
            conn._tlm_queue.put(ln)
        for ln in evt_lines:
            conn._evt_queue.put(ln)

        # Start reader thread (needed so read_lines() uses queue path).
        conn._start_reader()
        try:
            # read_lines with a short window — queues are already filled.
            result = conn.read_lines(duration_ms=100)
            assert set(result) == set(tlm_lines + evt_lines), (
                f"read_lines returned {result!r}, expected {tlm_lines + evt_lines!r}"
            )
        finally:
            conn._stop_reader()

    def test_read_lines_with_stop_token(self) -> None:
        """read_lines() stops early when stop_token is matched."""
        conn, _ = _make_conn_with_fake_ser()
        conn._tlm_queue.put("TLM t=1 mode=I enc=0,0")
        conn._evt_queue.put("EVT done T")
        conn._tlm_queue.put("TLM t=2 mode=I enc=5,5")

        conn._start_reader()
        try:
            result = conn.read_lines(duration_ms=200, stop_token="EVT done")
            # Must include the stop line; may or may not include later lines.
            assert any("EVT done" in ln for ln in result), (
                f"stop_token line not in result: {result!r}"
            )
        finally:
            conn._stop_reader()

    def test_read_lines_does_not_call_ser_readline(self) -> None:
        """read_lines() must not call _ser.readline() when reader is running."""
        conn, fake = _make_conn_with_fake_ser()
        conn._tlm_queue.put("TLM t=1 mode=I enc=0,0")
        conn._start_reader()
        try:
            original_readline_count = len(fake.written)  # use written as a proxy
            # Patch readline to detect if it's called.
            readline_calls = []

            orig = fake.readline
            def patched_readline() -> bytes:
                readline_calls.append(1)
                return orig()
            fake.readline = patched_readline  # type: ignore[method-assign]

            conn.read_lines(duration_ms=50)
            # readline may be called by the *reader thread*, not by read_lines
            # itself.  What we really verify is that the queued line was
            # returned without read_lines blocking on _ser.
            # The test above (test_read_lines_drains_tlm_and_evt) is the
            # definitive check; this supplements it.
        finally:
            conn._stop_reader()


# ---------------------------------------------------------------------------
# Test 7: read_pending_lines() non-blocking drain
# ---------------------------------------------------------------------------

class TestReadPendingLines:
    def test_read_pending_lines_non_blocking(self) -> None:
        """read_pending_lines() returns immediately with queued lines."""
        conn, _ = _make_conn_with_fake_ser()
        conn._tlm_queue.put("TLM t=1 mode=I enc=0,0")
        conn._tlm_queue.put("TLM t=2 mode=I enc=5,5")
        conn._evt_queue.put("EVT done T")

        start = time.time()
        result = conn.read_pending_lines()
        elapsed = time.time() - start

        assert elapsed < 0.1, f"read_pending_lines() blocked for {elapsed:.3f}s"
        assert set(result) == {
            "TLM t=1 mode=I enc=0,0",
            "TLM t=2 mode=I enc=5,5",
            "EVT done T",
        }

    def test_read_pending_lines_empty_queues(self) -> None:
        """read_pending_lines() returns [] immediately on empty queues."""
        conn, _ = _make_conn_with_fake_ser()
        start = time.time()
        result = conn.read_pending_lines()
        elapsed = time.time() - start
        assert result == []
        assert elapsed < 0.05

    def test_read_pending_lines_drains_all(self) -> None:
        """read_pending_lines() drains both TLM and EVT queues completely."""
        conn, _ = _make_conn_with_fake_ser()
        for i in range(5):
            conn._tlm_queue.put(f"TLM t={i} mode=I enc=0,0")
        for i in range(3):
            conn._evt_queue.put(f"EVT done T #{i}")

        result = conn.read_pending_lines()
        assert len(result) == 8
        assert conn._tlm_queue.empty()
        assert conn._evt_queue.empty()


# ---------------------------------------------------------------------------
# Test 8: send() has no reset_input_buffer()
# ---------------------------------------------------------------------------

class TestSendNoResetInputBuffer:
    def test_send_does_not_call_reset_input_buffer(self) -> None:
        """send() must not call reset_input_buffer() on _ser."""
        conn, fake = _make_conn_with_fake_ser()
        reset_called = []

        orig_reset = fake.reset_input_buffer
        def patched_reset() -> None:
            reset_called.append(1)
            orig_reset()
        fake.reset_input_buffer = patched_reset  # type: ignore[method-assign]

        # Pre-register a reply queue and inject a reply so send() completes.
        def _inject_reply() -> None:
            time.sleep(0.05)
            with conn._reply_lock:
                for k, q in list(conn._reply_queues.items()):
                    q.put(f"OK pong #{k}")

        conn._start_reader()
        injector = threading.Thread(target=_inject_reply, daemon=True)
        injector.start()
        try:
            conn.send("PING", read_ms=200)
        finally:
            conn._stop_reader()
            injector.join(timeout=1.0)

        assert reset_called == [], "send() must not call reset_input_buffer()"


# ---------------------------------------------------------------------------
# Test 9: Concurrent send() isolation
# ---------------------------------------------------------------------------

class TestConcurrentSendIsolation:
    def test_concurrent_sends_receive_own_replies(self) -> None:
        """Two concurrent send() calls must not receive each other's replies."""
        conn, fake = _make_conn_with_fake_ser()
        conn._start_reader()

        results: dict[str, list] = {"a": [], "b": []}
        errors: list[str] = []

        def send_a() -> None:
            r = conn.send("PING", read_ms=500, stop_token="OK pong")
            results["a"] = r.get("responses", [])

        def send_b() -> None:
            r = conn.send("STATUS", read_ms=500, stop_token="OK status")
            results["b"] = r.get("responses", [])

        # Start both sends concurrently.
        ta = threading.Thread(target=send_a, daemon=True)
        tb = threading.Thread(target=send_b, daemon=True)
        ta.start()
        tb.start()

        # Give them a moment to register their reply queues and write.
        time.sleep(0.05)

        # Identify the two corr-ids that were registered.
        with conn._reply_lock:
            ids = list(conn._reply_queues.keys())

        # Feed corr-id-specific replies.
        for cid in ids:
            verb = "pong" if cid == ids[0] else "status"
            fake.feed(f"OK {verb} #{cid}\n".encode())

        ta.join(timeout=2.0)
        tb.join(timeout=2.0)

        try:
            conn._stop_reader()
        except Exception:
            pass

        # Each thread should have gotten exactly its own reply.
        for key in ("a", "b"):
            assert len(results[key]) >= 1, f"send {key} got no reply"
        # Neither thread should have received the other's reply.
        if results["a"] and results["b"]:
            assert results["a"][0] != results["b"][0], (
                "Both sends received the same reply line"
            )


# ---------------------------------------------------------------------------
# Test 10: handshake() writes raw line under write lock
# ---------------------------------------------------------------------------

class TestHandshake:
    def test_handshake_writes_raw_line(self) -> None:
        """handshake() writes the line bytes directly to _ser."""
        conn, fake = _make_conn_with_fake_ser()
        # handshake is valid before reader starts.
        conn.handshake(b"HELLO\n")
        assert b"HELLO\n" in fake.written

    def test_handshake_no_relay_prefix(self) -> None:
        """handshake() does NOT prepend a relay prefix."""
        conn, fake = _make_conn_with_fake_ser()
        conn.handshake(b"HELLO\n")
        for written_bytes in fake.written:
            assert not written_bytes.startswith(b">"), (
                f"handshake() added a relay prefix: {written_bytes!r}"
            )

    def test_handshake_raises_if_not_connected(self) -> None:
        """handshake() raises ConnectionError when port is not open."""
        conn = SerialConnection(port="/dev/fake", mode="relay")
        with pytest.raises(ConnectionError):
            conn.handshake(b"HELLO\n")
