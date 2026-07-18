"""src/tests/sim/tune_velocity_pid.py -- tune the inner velocity PID to a clean
square-wave step (no overshoot on the rise, no reversal on the stop).

Drives a raw twist step in the zero-error sim, sets candidate gains via a
MotorConfigPatch, and scores the per-wheel step response: overshoot above the
commanded speed, and reverse spin after the deadman stops the motors. The
commanded speed is read live from the firmware (Path B ctypes hook, frame.cmd_vel).
"""
from __future__ import annotations

import base64

from robot_radio.io.sim_loop import SimLoop
from robot_radio.testgui.transport import _sim_lib_path

TRACK = 128.0
TARGET = 150.0  # [mm/s] commanded step


def _config_line(**gains) -> str:
    from robot_radio.robot.pb2 import config_pb2, envelope_pb2
    delta = envelope_pb2.ConfigDelta(motor=config_pb2.MotorConfigPatch(**gains))
    env = envelope_pb2.CommandEnvelope(corr_id=7, config=delta)
    return "*B" + base64.b64encode(env.SerializeToString()).decode("ascii")


def step_response(kp, ki, kff, i_max, kaw):
    loop = SimLoop(track_width=TRACK, lib_path=_sim_lib_path())
    loop.connect(start_tick_thread=False)
    try:
        loop.set_otos_raw_scale_err(0.0, 0.0)
        loop.set_enc_scale_err(1, 0.0); loop.set_enc_scale_err(2, 0.0)
        loop.set_enc_tick_quant(1, 0.0); loop.set_enc_tick_quant(2, 0.0)
        loop.set_enc_slip(1, 0.0, 0.0); loop.set_enc_slip(2, 0.0, 0.0)
        loop.inject_command(_config_line(kp=kp, ki=ki, kff=kff, i_max=i_max, kaw=kaw))
        loop.set_true_pose(0.0, 0.0, 0.0)
        loop.step(1); loop._drain_tlm_into_queue(); loop.read_pending_binary_tlm_frames()  # noqa: SLF001

        rows = []  # (phase, actL, actR)
        # drive phase: hold the twist for ~2.4s (deadman re-fed each cycle)
        for _ in range(48):
            loop.twist(TARGET, 0.0, 300.0)
            loop.step(1); loop._drain_tlm_into_queue()  # noqa: SLF001
            fr = loop.read_pending_binary_tlm_frames()
            if fr and fr[-1].vel:
                rows.append(("drive", fr[-1].vel[0], fr[-1].vel[1]))
        # stop phase: motors off, watch for reverse
        for _ in range(25):
            loop.stop()
            loop.step(1); loop._drain_tlm_into_queue()  # noqa: SLF001
            fr = loop.read_pending_binary_tlm_frames()
            if fr and fr[-1].vel:
                rows.append(("stop", fr[-1].vel[0], fr[-1].vel[1]))
        return rows
    finally:
        loop.disconnect()


def score(rows):
    drive = [(l, r) for ph, l, r in rows if ph == "drive"]
    stop = [(l, r) for ph, l, r in rows if ph == "stop"]
    # steady state = last third of the drive phase
    tail = drive[len(drive) * 2 // 3:] or drive[-3:]
    hold = sum((l + r) / 2 for l, r in tail) / max(1, len(tail))
    peak = max((max(l, r) for l, r in drive), default=0.0)
    overshoot = max(0.0, peak - TARGET)
    # worst reverse spin during the stop (motors commanded to 0)
    reverse = -min((min(l, r) for l, r in stop), default=0.0)
    reverse = max(0.0, reverse)
    return {"hold": hold, "overshoot": overshoot, "reverse": reverse, "peak": peak}


def main():
    # (label, kp, ki, kff, i_max, kaw)
    candidates = [
        ("baseline  ", 0.3, 0.5, 0.15, 20.0, 3.0),
        ("kff+       ", 0.3, 0.5, 0.20, 20.0, 3.0),
        ("kff++ ki-  ", 0.3, 0.2, 0.20, 20.0, 3.0),
        ("kff++ ki0  ", 0.3, 0.0, 0.20, 20.0, 3.0),
        ("kff.2 kp.2 ", 0.2, 0.1, 0.20, 20.0, 3.0),
        ("kff.2 kp.15", 0.15, 0.1, 0.20, 20.0, 3.0),
        ("ff-heavy   ", 0.1, 0.05, 0.20, 10.0, 3.0),
    ]
    print(f"{'gains':12} | hold  peak  overshoot  reverse")
    for label, kp, ki, kff, imax, kaw in candidates:
        s = score(step_response(kp, ki, kff, imax, kaw))
        print(f"{label} | {s['hold']:5.0f} {s['peak']:5.0f}   {s['overshoot']:6.1f}   {s['reverse']:6.1f}")


if __name__ == "__main__":
    main()
