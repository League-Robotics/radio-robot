"""tests/unit/test_bridge_pty_e2e.py -- 097-004 (M5 rogo Translator Proxy).

End-to-end proof of the PTY transport itself: a REAL ``os.openpty()``
(via ``ProtocolBridge.start()``) fronting a ``_FakeConn`` double standing
in for the real robot connection, with a REAL ``serial.Serial`` client
(pyserial, exactly what every legacy consumer already uses) opening the
published PTY slave path and exchanging text-v2 lines -- the acceptance
criterion's own "TEST CLIENT opens serial.Serial(<pty-slave-path>) ...
NOT nc -U" requirement.

Proves, against the bridge's OWN ``SerialConnection`` writes (asserted on
``fake.envelope_calls``), that the proxy speaks ONLY binary underneath
while the client sees plain text-v2 -- and exercises the two background
threads together for real (pty-reader + tlm-pump), unlike
``test_bridge_routing.py``'s single-threaded, un-``start()``-ed bridge.

No real hardware: ``_FakeConn`` stands in for the robot end exactly as it
does in ``test_bridge_routing.py``, just made thread-safe (``queue.Queue``)
since the pump thread and the test's own assertions now run concurrently.
"""

from __future__ import annotations

import queue
import time

import pytest
import serial

from robot_radio.io.proxy import ProtocolBridge
from robot_radio.robot.pb2 import config_pb2, envelope_pb2, telemetry_pb2


class _FakeConn:
    """Thread-safe stand-in for the real robot ``SerialConnection`` --
    records every envelope the bridge sends it; canned replies are
    consumed FIFO by client-triggered verbs. ``stream`` (StreamControl)
    envelopes are auto-ACKed rather than drawn from the queue: the
    tlm-pump thread issues these on its own schedule (arming/disarming the
    EVT-watch period), and making the test predict its exact call count/
    timing would be flaky by construction -- auto-ack keeps that
    background bookkeeping invisible to the scripted reply queue, the same
    way the real robot always acks a well-formed STREAM request."""

    def __init__(self):
        self.envelope_calls: list[envelope_pb2.CommandEnvelope] = []
        self._reply_queue: "queue.Queue" = queue.Queue()
        self._tlm_queue: "queue.Queue" = queue.Queue()

    def queue_reply(self, reply: "envelope_pb2.ReplyEnvelope | None") -> None:
        self._reply_queue.put(reply)

    def push_tlm(self, reply: "envelope_pb2.ReplyEnvelope") -> None:
        self._tlm_queue.put(reply)

    def send_envelope(self, envelope: envelope_pb2.CommandEnvelope,
                      read_timeout: int = 500) -> dict:
        self.envelope_calls.append(envelope)
        if envelope.WhichOneof("cmd") == "stream":
            return {"sent": envelope, "mode": "direct",
                    "reply": envelope_pb2.ReplyEnvelope(ok=envelope_pb2.Ack())}
        try:
            reply = self._reply_queue.get_nowait()
        except queue.Empty:
            reply = None
        return {"sent": envelope, "mode": "direct", "reply": reply}

    def send_fast(self, message: str) -> None:
        pass

    def drain_binary_tlm(self) -> list:
        return []

    def read_binary_tlm(self, duration: int) -> list:
        frames = []
        deadline = time.monotonic() + duration / 1000.0
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            try:
                frames.append(self._tlm_queue.get(timeout=remaining))
            except queue.Empty:
                break
        return frames

    @property
    def is_open(self) -> bool:
        return True


@pytest.fixture
def bridge_and_client(tmp_path):
    fake = _FakeConn()
    fake.queue_reply(envelope_pb2.ReplyEnvelope(
        id=envelope_pb2.DeviceId(model="NEZHA2", name="GUTOV", serial=2121102,
                                 fw_version="v0.20260710.1", proto_version=3)))
    bridge = ProtocolBridge(fake, link=str(tmp_path / "robot-pty"), watch_period=20)
    bridge.start()
    client = serial.Serial(bridge.slave_path, baudrate=115200, timeout=2.0)
    try:
        yield bridge, fake, client
    finally:
        client.close()
        bridge.stop()


def _readline(client: "serial.Serial") -> str:
    raw = client.readline()
    assert raw, "no line received from the proxy within the client's read timeout"
    return raw.decode("utf-8", "ignore").strip()


# ---------------------------------------------------------------------------
# Symlink + single-slave-path plumbing
# ---------------------------------------------------------------------------


def test_start_publishes_symlink_to_the_pty_slave(tmp_path):
    fake = _FakeConn()
    fake.queue_reply(envelope_pb2.ReplyEnvelope(id=envelope_pb2.DeviceId()))
    link = tmp_path / "robot-pty"
    bridge = ProtocolBridge(fake, link=str(link))
    try:
        slave_path = bridge.start()
        assert link.is_symlink()
        assert str(link.resolve()) == slave_path
    finally:
        bridge.stop()
    assert not link.exists()  # cleaned up on stop()


# ---------------------------------------------------------------------------
# S / PING / HELLO / D -- text in, binary underneath, text back
# ---------------------------------------------------------------------------


def test_s_drive_over_the_pty(bridge_and_client):
    bridge, fake, client = bridge_and_client
    fake.queue_reply(envelope_pb2.ReplyEnvelope(ok=envelope_pb2.Ack(q=1)))
    client.write(b"S 200 200 #1\n")
    line = _readline(client)
    assert line == "OK drive l=200 r=200 #1"
    drive_calls = [c for c in fake.envelope_calls if c.WhichOneof("cmd") == "drive"]
    assert len(drive_calls) == 1
    assert list(w.speed for w in drive_calls[0].drive.wheels.w) == [200.0, 200.0]


def test_ping_over_the_pty(bridge_and_client):
    bridge, fake, client = bridge_and_client
    fake.queue_reply(envelope_pb2.ReplyEnvelope(ok=envelope_pb2.Ack(t=42)))
    client.write(b"PING\n")
    assert _readline(client) == "OK pong t=42"
    assert any(c.WhichOneof("cmd") == "ping" for c in fake.envelope_calls)


def test_hello_over_the_pty_answers_locally_from_cached_device_id(bridge_and_client):
    bridge, fake, client = bridge_and_client
    calls_before = len(fake.envelope_calls)
    client.write(b"HELLO\n")
    assert _readline(client) == "DEVICE:NEZHA2:robot:GUTOV:2121102"
    # No new wire traffic -- HELLO answers from the cache fetched at start().
    assert len(fake.envelope_calls) == calls_before


def test_d_distance_and_evt_done_over_the_pty(bridge_and_client):
    bridge, fake, client = bridge_and_client
    fake.queue_reply(envelope_pb2.ReplyEnvelope(ok=envelope_pb2.Ack(q=1, rem=0.0)))
    client.write(b"D 200 200 300 #2\n")
    assert _readline(client) == "OK drive l=200 r=200 mm=300 #2"

    # Synthesize the drive: active goes True, then False -- EVT done fires.
    fake.push_tlm(envelope_pb2.ReplyEnvelope(
        tlm=telemetry_pb2.Telemetry(now=1, mode=3, seq=1, active=True)))
    time.sleep(0.15)
    fake.push_tlm(envelope_pb2.ReplyEnvelope(
        tlm=telemetry_pb2.Telemetry(now=2, mode=0, seq=2, active=False)))

    deadline = time.monotonic() + 3.0
    line = None
    while time.monotonic() < deadline:
        raw = client.readline()
        if raw:
            line = raw.decode("utf-8", "ignore").strip()
            break
    assert line == "EVT done D #2 reason=idle"


# ---------------------------------------------------------------------------
# GET -- one CFG line out of a multi-target binary fan-out
# ---------------------------------------------------------------------------


def test_get_over_the_pty_produces_one_cfg_line(bridge_and_client):
    bridge, fake, client = bridge_and_client
    fake.queue_reply(envelope_pb2.ReplyEnvelope(cfg=envelope_pb2.ConfigSnapshot(
        target=config_pb2.CONFIG_DRIVETRAIN,
        drivetrain=config_pb2.DrivetrainConfigPatch(
            trackwidth=128.0, rotational_slip=0.92, ekf_q_xy=0.01,
            ekf_q_theta=0.02, ekf_r_otos_xy=0.05, ekf_r_otos_theta=0.03))))
    fake.queue_reply(envelope_pb2.ReplyEnvelope(cfg=envelope_pb2.ConfigSnapshot(
        target=config_pb2.CONFIG_MOTOR_LEFT,
        motor=config_pb2.MotorConfigPatch(
            travel_calib=0.487, kp=1.0, ki=0.1, kff=0.5, i_max=2.0, kaw=0.02))))
    fake.queue_reply(envelope_pb2.ReplyEnvelope(cfg=envelope_pb2.ConfigSnapshot(
        target=config_pb2.CONFIG_MOTOR_RIGHT,
        motor=config_pb2.MotorConfigPatch(travel_calib=0.481))))
    fake.queue_reply(envelope_pb2.ReplyEnvelope(cfg=envelope_pb2.ConfigSnapshot(
        target=config_pb2.CONFIG_PLANNER,
        planner=config_pb2.PlannerConfigPatch(min_speed=50.0))))
    fake.queue_reply(envelope_pb2.ReplyEnvelope(cfg=envelope_pb2.ConfigSnapshot(
        target=config_pb2.CONFIG_WATCHDOG, watchdog=500)))

    client.write(b"GET #7\n")
    line = _readline(client)

    assert line.startswith("CFG ")
    assert line.endswith("#7")
    assert "tw=128" in line
    assert "ml=0.487" in line
    assert "mr=0.481" in line
    assert "rotSlip=0.920" in line
    assert "minSpeed=50" in line
    assert "sTimeout=500" in line
    # Exactly one line -- no second reply queued/pending behind it.
    client.timeout = 0.2
    assert client.readline() == b""


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
