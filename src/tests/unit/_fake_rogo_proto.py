"""Shared fake NezhaProtocol for the rogo repl/daemon unit tests.

Duck-types exactly the ``NezhaProtocol`` surface the rebuilt repl verbs
(``robot_radio.io.repl``) and the rogo daemon (``robot_radio.io.server``)
touch. Every command records itself on ``calls`` and, when ``auto_ack`` is
on, pushes the telemetry frames a real firmware would: an enqueue ack for
the returned corr_id, and (for Moves carrying a nonzero ``move_id``) a
completion ack echoing ``Move.id`` on a later frame — the v5 two-ack shape
(docs/protocol-v5.md section 7).

Frames flow through the same ``read_pending_binary_tlm_frames()`` drain the
real protocol exposes, so ``RogoSession.pump()``/``confirm()`` run their real
code paths. ``collections.deque`` append/popleft are atomic under the GIL,
so the daemon tests may push frames from the test thread while the
executor thread drains.
"""
from __future__ import annotations

import collections

from robot_radio.robot.protocol import AckEntry, TLMFrame


class FakeRogoProto:
    def __init__(self) -> None:
        self.calls: list[tuple] = []
        self.estop_count = 0
        self.auto_ack = True
        # When True, every estop's ack frame reports the drivetrain STILL
        # active -- exercises the repl estop verb's verify-and-reissue loop.
        self.estop_leaves_active = False
        self._corr = 100
        self._frames: collections.deque = collections.deque()
        self._lines: collections.deque = collections.deque()
        self._conn = None

    # -- test-side helpers ---------------------------------------------------

    def push_frame(self, frame: TLMFrame) -> None:
        self._frames.append(frame)

    def push_telemetry(self, **fields) -> None:
        self._frames.append(TLMFrame(**fields))

    @staticmethod
    def ack_frame(corr_id: int, ok: bool = True, err_code: int = 0,
                  active: "bool | None" = None) -> TLMFrame:
        return TLMFrame(acks=[AckEntry(corr_id=corr_id, ok=ok, err_code=err_code)],
                        active=active)

    def _next_corr(self) -> int:
        self._corr += 1
        return self._corr

    def _enqueue_acks(self, corr_id: int, move_id: int = 0) -> None:
        if not self.auto_ack:
            return
        self.push_frame(self.ack_frame(corr_id))
        if move_id:
            self.push_frame(self.ack_frame(move_id, active=False))

    # -- NezhaProtocol surface: motion ---------------------------------------

    def move_twist(self, v_x, v_y, omega, *, stop_time=None, stop_distance=None,
                   stop_angle=None, timeout, replace=True, move_id=0) -> int:
        self.calls.append(("move_twist", {
            "v_x": v_x, "v_y": v_y, "omega": omega, "stop_time": stop_time,
            "stop_distance": stop_distance, "stop_angle": stop_angle,
            "timeout": timeout, "replace": replace, "move_id": move_id}))
        cid = self._next_corr()
        self._enqueue_acks(cid, move_id)
        return cid

    def move_wheels(self, v_left, v_right, *, stop_time=None, stop_distance=None,
                    stop_angle=None, timeout, replace=True, move_id=0) -> int:
        self.calls.append(("move_wheels", {
            "v_left": v_left, "v_right": v_right, "stop_time": stop_time,
            "stop_distance": stop_distance, "stop_angle": stop_angle,
            "timeout": timeout, "replace": replace, "move_id": move_id}))
        cid = self._next_corr()
        self._enqueue_acks(cid, move_id)
        return cid

    def move(self, *, v_x=0.0, v_y=0.0, omega=0.0, v_left=None, v_right=None,
             stop_time=None, stop_distance=None, stop_angle=None,
             timeout, replace=True, id=None) -> int:
        move_id = id if id is not None else 0
        if v_left is not None or v_right is not None:
            if v_left is None or v_right is None:
                raise ValueError("move(): v_left and v_right must both be given")
            return self.move_wheels(v_left, v_right, stop_time=stop_time,
                                    stop_distance=stop_distance, stop_angle=stop_angle,
                                    timeout=timeout, replace=replace, move_id=move_id)
        return self.move_twist(v_x, v_y, omega, stop_time=stop_time,
                               stop_distance=stop_distance, stop_angle=stop_angle,
                               timeout=timeout, replace=replace, move_id=move_id)

    def wheels(self, v_left, v_right, duration, *, move_id=0) -> int:
        if duration <= 0:
            raise ValueError("wheels(): duration must be > 0")
        self.calls.append(("wheels", {"v_left": v_left, "v_right": v_right,
                                      "duration": duration, "move_id": move_id}))
        cid = self._next_corr()
        self._enqueue_acks(cid, move_id)
        return cid

    def stop(self, *, move_id=0) -> int:
        self.calls.append(("stop", {"move_id": move_id}))
        cid = self._next_corr()
        self._enqueue_acks(cid)
        return cid

    def estop(self) -> int:
        self.calls.append(("estop", {}))
        self.estop_count += 1
        cid = self._next_corr()
        if self.auto_ack:
            self.push_frame(self.ack_frame(cid, active=self.estop_leaves_active))
        return cid

    # -- NezhaProtocol surface: cleartext, config, telemetry -----------------

    def send_fast(self, cmd: str) -> None:
        self.calls.append(("send_fast", cmd))
        replies = {
            "PING": "PONG:t=1234",
            "ID": "ID:differential:tovez:0.1",
            "VER": "VER:0.1",
            "HELLO": "DEVICE:NEZHA2:tovez:microbit:9906",
        }
        if cmd in replies:
            self._lines.append(replies[cmd])

    def read_pending_lines(self) -> list[str]:
        out = list(self._lines)
        self._lines.clear()
        return out

    def set_config_field(self, target, field_name, value, **kwargs):
        self.calls.append(("set_config_field", target, field_name, value))
        return AckEntry(corr_id=self._next_corr(), ok=True, err_code=0)

    def read_pending_binary_tlm_frames(self) -> list[TLMFrame]:
        out = []
        while self._frames:
            out.append(self._frames.popleft())
        return out

    def tlmOn(self) -> None:
        self.calls.append(("send_fast", "TLM:ON"))

    def tlmOff(self) -> None:
        self.calls.append(("send_fast", "TLM:OFF"))

    def tlmNow(self) -> None:
        self.calls.append(("send_fast", "TLM:NOW"))
