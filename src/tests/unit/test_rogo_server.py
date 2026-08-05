"""src/tests/unit/test_rogo_server.py — the rogo daemon + client library
(2026-08-05, out-of-process rogo revival).

End-to-end over real TCP on an ephemeral loopback port, with the serial side
faked (``_fake_rogo_proto.FakeRogoProto`` behind a real ``RogoSession``):

  1. one client round-trips a command and gets its JSON result;
  2. plain-text and JSON request shapes both work, ids echoed;
  3. multiple concurrent clients share the one held-open session;
  4. ``sub tlm`` streams pumped telemetry frames to the subscriber only;
  5. an ``estop`` from a second client jumps the queue and aborts a wait
     already in progress on the executor (the daemon's panic path);
  6. ``shutdown`` halts the robot (estop reaches the fake) and drops clients.

The daemon exists to hold the serial port open across client sessions —
closing the port resets the robot on macOS (HUPCL,
``A-port-close-resets-the-robot-live-config-still-wiped.md``) — so test 6
also pins that the session is closed exactly once, at daemon shutdown,
never per-client.
"""
from __future__ import annotations

import json
import socket
import time

import pytest

from robot_radio.io.client import RogoClient, RogoDaemonError
from robot_radio.io.repl import RogoSession
from robot_radio.io.server import RogoServer, parse_addr

from _fake_rogo_proto import FakeRogoProto


class FakeConn:
    """The connection object RogoSession.close() disconnects."""

    def __init__(self) -> None:
        self.disconnects = 0

    def disconnect(self) -> None:
        self.disconnects += 1


@pytest.fixture()
def server():
    proto = FakeRogoProto()
    conn = FakeConn()
    session = RogoSession.from_protocol(proto, conn=conn,
                                        meta={"port": "/dev/fake", "mode": "direct"})
    srv = RogoServer(session, host="127.0.0.1", port=0)  # ephemeral port
    srv.start()
    yield srv, proto, conn
    srv.shutdown()


def addr_of(srv: RogoServer) -> str:
    return f"{srv.host}:{srv.port}"


def test_parse_addr_shapes():
    assert parse_addr(None) == ("127.0.0.1", 7646)
    assert parse_addr("") == ("127.0.0.1", 7646)
    assert parse_addr(":7700") == ("127.0.0.1", 7700)
    assert parse_addr("7700") == ("127.0.0.1", 7700)
    assert parse_addr("10.0.0.5:7700") == ("10.0.0.5", 7700)


def test_client_round_trip_and_hello(server):
    srv, proto, _ = server
    with RogoClient(addr_of(srv)) as client:
        assert client.hello["serial_port"] == "/dev/fake"
        result = client.cmd("ping", timeout=3.0)
        assert result["ok"] is True
        assert any("PONG" in line for line in result["output"])
    assert ("send_fast", "PING") in proto.calls


def test_plain_text_request_shape_over_raw_socket(server):
    srv, _, _ = server
    sock = socket.create_connection((srv.host, srv.port), timeout=3.0)
    reader = sock.makefile("r")
    assert json.loads(reader.readline())["type"] == "hello"
    sock.sendall(b"stop\n")
    reply = json.loads(reader.readline())
    assert reply["type"] == "result" and reply["ok"] is True
    assert any("planned" in line for line in reply["output"])
    sock.close()


def test_json_request_id_is_echoed(server):
    srv, _, _ = server
    sock = socket.create_connection((srv.host, srv.port), timeout=3.0)
    reader = sock.makefile("r")
    reader.readline()  # hello
    sock.sendall(json.dumps({"id": "xyz-1", "cmd": "status"}).encode() + b"\n")
    reply = json.loads(reader.readline())
    assert reply["id"] == "xyz-1"
    assert reply["status"]["serial_port"] == "/dev/fake"
    assert reply["status"]["clients"] == 1
    sock.close()


def test_multiple_clients_share_one_session(server):
    srv, proto, _ = server
    with RogoClient(addr_of(srv)) as a, RogoClient(addr_of(srv)) as b:
        ra = a.cmd("ping", timeout=3.0)
        rb = b.cmd("ver", timeout=3.0)
        assert ra["ok"] and rb["ok"]
    sent = [c[1] for c in proto.calls if c[0] == "send_fast"]
    assert "PING" in sent and "VER" in sent


def test_tlm_subscription_streams_only_to_subscriber(server):
    srv, proto, _ = server
    with RogoClient(addr_of(srv)) as sub_client, RogoClient(addr_of(srv)) as other:
        assert sub_client.subscribe_tlm(decimate=1)["ok"]
        proto.push_telemetry(enc=(11, 22), active=False)
        frames = list(sub_client.frames(duration=2.0))
        assert any(tuple(f.get("enc") or ()) == (11, 22) for f in frames)
        # The unsubscribed client saw no telemetry events.
        assert len(other.tlm_frames) == 0
        # And unsubscribing stops the stream.
        assert sub_client.unsubscribe_tlm()["ok"]
        proto.push_telemetry(enc=(33, 44), active=False)
        assert list(sub_client.frames(duration=0.3)) == []


def test_estop_from_second_client_jumps_queue_and_aborts_wait(server):
    """The panic path: client A parks the executor in a long `sleep`; client
    B's estop must abort that wait, run first, and reach the wire fast."""
    srv, proto, _ = server
    a = RogoClient(addr_of(srv))
    b = RogoClient(addr_of(srv))
    try:
        # Fire the sleep without waiting for its result.
        a._sock.sendall(json.dumps({"id": "slow", "cmd": "sleep 1500"}).encode() + b"\n")
        time.sleep(0.15)  # let the executor enter the sleep's pump-wait loop
        t0 = time.monotonic()
        result = b.estop()
        elapsed = time.monotonic() - t0
        assert result["ok"] is True
        assert proto.estop_count >= 1
        assert elapsed < 1.0, f"estop took {elapsed:.2f}s -- did not jump the queue"
        # A's aborted sleep still answers (early), well before its 1.5s.
        got_slow = a._await_event(
            1.0, lambda e: e.get("type") == "result" and e.get("id") == "slow")
        assert got_slow["ok"] is True
    finally:
        a.close()
        b.close()


def test_quit_drops_client_but_daemon_and_session_stay_up(server):
    srv, proto, conn = server
    sock = socket.create_connection((srv.host, srv.port), timeout=3.0)
    reader = sock.makefile("r")
    reader.readline()  # hello
    sock.sendall(b"quit\n")
    assert json.loads(reader.readline())["output"] == ["bye"]
    assert reader.readline() == ""  # server closed this connection...
    sock.close()
    assert conn.disconnects == 0  # ...but the serial session stays open
    with RogoClient(addr_of(srv)) as again:  # and new clients still connect
        assert again.cmd("status", timeout=3.0)["ok"]


def test_shutdown_halts_robot_and_closes_session_exactly_once(server):
    srv, proto, conn = server
    with RogoClient(addr_of(srv)) as client:
        assert client.cmd("ping", timeout=3.0)["ok"]
    srv.shutdown()
    assert proto.estop_count >= 1  # halt_now ran before the port closed
    assert conn.disconnects == 1
    srv.shutdown()  # idempotent
    assert conn.disconnects == 1
    with pytest.raises(RogoDaemonError):
        RogoClient(addr_of(srv))
