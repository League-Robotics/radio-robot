"""src/tests/unit/test_serial_conn_debug_channel.py -- 129-003 (DBG debug
message channel, bench/Sim only; issue 05-dbg-debug-message-channel-for-
bench-and-sim.md).

``SerialConnection`` routes a cleartext ``DBG:<message>`` reply line
straight to an ``on_debug`` callback (``_handle_text_line()``), never
through ``_text_queue`` -- DBG is an unsolicited, unbounded diagnostic
stream, not a request/reply verb a blocked ``send_cleartext()`` caller is
waiting on.

The ticket's own acceptance criterion (and the reason this file exists):
*"A malformed or oversized DBG line does not disturb telemetry frame
delivery -- regression test for the `_log` NameError that killed a reader
thread mid-session in the abandoned prior attempt; the host handler must be
exception-proof by construction (wrap the whole `on_debug` dispatch, never
let it propagate into the reader loop)."* The abandoned session's own
``on_debug`` handler referenced an undefined ``_log`` name and raised a
``NameError`` INSIDE the reader thread's own call stack -- since nothing
between ``_handle_text_line()`` and ``_reader_loop()`` caught it, the whole
thread died, silently ending TELEMETRY delivery too (not just DBG).
``_handle_text_line()``'s DBG branch now wraps the ``on_debug`` call in its
own ``try/except Exception: pass`` -- this file proves that wrapper holds
even when the installed callback raises exactly that class of bug, AND that
a subsequent binary TLM frame in the SAME reader-thread pass is still
decoded and queued normally.

Mirrors ``test_serial_conn_binary_plane.py``'s own ``_new_conn()``
no-real-I/O construction pattern and ``FakeSerial``/``text_line``/
``binary_frame`` fixtures (``_wire_test_helpers.py``).

Collected under ``src/tests/unit/`` -- ``pyproject.toml``'s ``testpaths``
includes ``tests/unit``, so ``uv run python -m pytest`` collects it by
default.
"""

from __future__ import annotations

import pytest

from robot_radio.io import wire_commands
from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.pb2 import envelope_pb2

from _wire_test_helpers import FakeSerial, binary_frame, text_line

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _new_conn(on_debug=None) -> SerialConnection:
    """A SerialConnection with no real I/O performed (mirrors
    test_serial_conn_binary_plane.py's own _new_conn())."""
    return SerialConnection(on_debug=on_debug)


def _raising_debug_handler(_line: str) -> None:
    """Reproduces the historical defect verbatim: a NameError from a
    reference to an undefined name inside the installed on_debug callback
    (the abandoned session's own bug, not a synthetic stand-in for "any
    exception") -- `_log` is never defined anywhere in this module."""
    _log.warning("debug: %s", _line)  # noqa: F821 -- deliberately undefined


# ---------------------------------------------------------------------------
# 1. DBG registry membership (sanity -- the differential/inventory checks
#    already live in test_command_registry.py; this just confirms the host
#    codec's own view agrees, since the tests below depend on it).
# ---------------------------------------------------------------------------


def test_dbg_is_a_registered_cleartext_verb():
    assert "DBG" in wire_commands.VERB_BY_NAME
    assert "DBG" in wire_commands.CLEARTEXT_VERBS
    assert "DBG" not in wire_commands.BINARY_VERBS


# ---------------------------------------------------------------------------
# 2. DBG routes to on_debug, never _text_queue
# ---------------------------------------------------------------------------


def test_dbg_line_routes_to_on_debug_callback():
    received: list[str] = []
    conn = _new_conn(on_debug=received.append)

    conn._ser = FakeSerial(text_line("DBG:v=1234"))
    conn._reader_loop()

    assert received == ["DBG:v=1234"]


def test_dbg_line_never_enters_text_queue():
    """DBG is an unsolicited diagnostic stream, not a send_cleartext()
    request/reply -- it must not compete for _text_queue's bounded 64-slot
    capacity with HELLO/PING/ID/VER/STATUS/HELP."""
    conn = _new_conn(on_debug=lambda _line: None)

    conn._ser = FakeSerial(text_line("DBG:hello"))
    conn._reader_loop()

    assert conn._text_queue.empty()


def test_dbg_line_with_no_on_debug_installed_is_dropped_silently():
    conn = _new_conn()  # on_debug defaults to None

    conn._ser = FakeSerial(text_line("DBG:no listener"))
    conn._reader_loop()  # must not raise

    assert conn._text_queue.empty()


# ---------------------------------------------------------------------------
# 3. The regression: a raising on_debug handler must not kill the reader
#    thread, and telemetry delivered in the SAME pass must still arrive.
# ---------------------------------------------------------------------------


def test_raising_on_debug_handler_does_not_propagate_out_of_reader_loop():
    conn = _new_conn(on_debug=_raising_debug_handler)

    conn._ser = FakeSerial(text_line("DBG:this triggers the buggy handler"))
    conn._reader_loop()  # must NOT raise -- this is the regression itself


def test_raising_on_debug_handler_does_not_disturb_telemetry_in_the_same_pass():
    """The ticket's own acceptance wording: "does not disturb telemetry
    frame delivery". A DBG line with a raising handler, immediately
    followed (same reader-thread pass) by a genuine binary TLM push frame,
    must still leave that TLM frame correctly decoded and queued -- proving
    the DBG-handler bug did not take the reader thread down mid-stream."""
    conn = _new_conn(on_debug=_raising_debug_handler)

    tlm = envelope_pb2.ReplyEnvelope(corr_id=0)
    tlm.tlm.now = 4242
    tlm.tlm.seq = 7

    stream = b"".join([
        text_line("DBG:boom"),
        binary_frame(tlm, b"TLM"),
    ])
    conn._ser = FakeSerial(stream)
    conn._reader_loop()  # must not raise despite the DBG handler's bug

    reply = conn._binary_tlm_queue.get_nowait()
    assert reply.WhichOneof("body") == "tlm"
    assert reply.tlm.now == 4242
    assert reply.tlm.seq == 7


def test_raising_on_debug_handler_does_not_disturb_telemetry_delivered_before_it():
    """Same property, reverse order: telemetry that arrived BEFORE the
    DBG-triggered exception must already be safely queued regardless of
    what happens afterward (it is, since _reader_loop processes lines
    sequentially and the queue push is a distinct, already-completed step
    -- this test pins that ordering assumption explicitly)."""
    conn = _new_conn(on_debug=_raising_debug_handler)

    tlm = envelope_pb2.ReplyEnvelope(corr_id=0)
    tlm.tlm.now = 99
    tlm.tlm.seq = 1

    stream = b"".join([
        binary_frame(tlm, b"TLM"),
        text_line("DBG:boom"),
    ])
    conn._ser = FakeSerial(stream)
    conn._reader_loop()  # must not raise

    reply = conn._binary_tlm_queue.get_nowait()
    assert reply.tlm.now == 99


# ---------------------------------------------------------------------------
# 4. Oversized / malformed DBG content
# ---------------------------------------------------------------------------


def test_oversized_dbg_line_does_not_raise_and_still_reaches_on_debug():
    """A DBG line far larger than anything Core::debugf()'s own
    kDebugMsgMaxBytes (200) bound would ever produce -- proves the HOST side
    places no assumption on DBG line length either (a crafted/corrupted
    line, not just a well-behaved firmware one)."""
    received: list[str] = []
    conn = _new_conn(on_debug=received.append)

    huge_message = "x" * 50_000
    conn._ser = FakeSerial(text_line(f"DBG:{huge_message}"))
    conn._reader_loop()  # must not raise

    assert len(received) == 1
    assert received[0] == f"DBG:{huge_message}"


def test_oversized_dbg_line_with_raising_handler_does_not_disturb_telemetry():
    """The two hazards combined: an oversized line AND a raising handler,
    with a genuine TLM frame right after -- the exact compound shape the
    ticket's acceptance criterion names ("malformed OR oversized")."""
    conn = _new_conn(on_debug=_raising_debug_handler)

    tlm = envelope_pb2.ReplyEnvelope(corr_id=0)
    tlm.tlm.now = 555
    tlm.tlm.seq = 2

    stream = b"".join([
        text_line("DBG:" + ("y" * 50_000)),
        binary_frame(tlm, b"TLM"),
    ])
    conn._ser = FakeSerial(stream)
    conn._reader_loop()  # must not raise

    reply = conn._binary_tlm_queue.get_nowait()
    assert reply.tlm.now == 555


def test_dbg_line_with_non_utf8_bytes_does_not_raise():
    """A DBG line containing bytes that are not valid UTF-8 -- decoded with
    errors="ignore" by _handle_wire_line() before this method ever sees it
    (that class's own doc comment), so this must not raise either."""
    received: list[str] = []
    conn = _new_conn(on_debug=received.append)

    garbled = b"DBG:bad-bytes-" + bytes([0xFF, 0xFE, 0x80]) + b"-end\n"
    conn._ser = FakeSerial(garbled)
    conn._reader_loop()  # must not raise

    assert len(received) == 1
    assert received[0].startswith("DBG:bad-bytes-")


if __name__ == "__main__":
    import sys

    sys.exit(pytest.main([__file__, "-v"]))
