"""Deliver a Move reliably over a lossy radio: send, confirm, resend.

MEASURED 2026-08-14: the relay loses ~5 % of Moves (2/40) while the same
commands over USB lose none (0/40), and the relay-to-relay link loses 2-3 % of
packets whenever they are spaced closer than 20 ms. A Move that is lost is lost
SILENTLY -- the host's enqueue ack is generated locally, so the caller sees a
perfectly normal acceptance for a command the robot never received.

Nothing downstream can compensate for that, because from the host a lost Move
and a running Move look identical until ACTIVE never appears. The fix belongs
here: treat delivery as unconfirmed until TELEMETRY shows the robot moving, and
resend until it does.

    send_move_confirmed(proto, observer, v_x=..., stop_distance=...)

`observer` is whatever protocol object can see the robot's telemetry -- the
same connection for USB, or a second link for a radio test rig.
"""
import time


ACTIVE_BIT = 1 << 2


def confirmed_move(proto, observer, mk_move, *, attempts: int = 6,
                   confirm: float = 1.2, spacing: float = 0.025):
    """`mk_move()` issues one Move. Resend it until the robot actually moves."""
    for attempt in range(1, attempts + 1):
        observer.read_pending_binary_tlm_frames()
        mk_move()
        deadline = time.time() + confirm
        while time.time() < deadline:
            for f in observer.read_pending_binary_tlm_frames():
                if f.flags is not None and (f.flags & ACTIVE_BIT):
                    return True, attempt
            time.sleep(0.02)
        time.sleep(spacing)
    return False, attempts
