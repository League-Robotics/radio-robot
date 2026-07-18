"""src/tests/sim/scoreboard_700.py -- the single scoreboard for "drive 700, all
sources read 700, clean square wave".

Drives straight forward (open-loop twist) until the plant's TRUE x passes 700mm,
then reports every distance source (truth / encoder-deadreckon / OTOS / fused)
and the wheel-speed shape (does it reach the commanded speed; front jitter; end
reversal). Zero simulated error everywhere. Loop target: all four ~700, wheel
reaches command, no jitter, no reversal.
"""
from __future__ import annotations

import base64

from robot_radio.io.sim_loop import SimLoop
from robot_radio.testgui.transport import _sim_lib_path

TARGET_MM = 700.0
SPEED = 150.0
ML_MR = 0.704861  # [mm/deg] geometry-derived wheel-travel calib the GUI pushes on connect


def push_calibration(loop):
    """Reproduce the GUI's connect-time calibration push (SET ml/mr -> the
    per-wheel wheel-travel calib on MotorConfigPatch), so the scoreboard sees
    exactly what TestGUI sees, not a bare no-push baseline."""
    from robot_radio.robot.pb2 import config_pb2, envelope_pb2

    def send(patch):
        env = envelope_pb2.CommandEnvelope(
            corr_id=1, config=envelope_pb2.ConfigDelta(motor=patch))
        loop.inject_command("*B" + base64.b64encode(env.SerializeToString()).decode("ascii"))

    send(config_pb2.MotorConfigPatch(side=config_pb2.LEFT, travel_calib=ML_MR))
    send(config_pb2.MotorConfigPatch(side=config_pb2.RIGHT, travel_calib=ML_MR))
    loop.step(1); loop._drain_tlm_into_queue(); loop.read_pending_binary_tlm_frames()  # noqa: SLF001


def run():
    loop = SimLoop(track_width=128.0, lib_path=_sim_lib_path())
    loop.connect(start_tick_thread=False)
    loop.set_otos_raw_scale_err(0.0, 0.0)
    for p in (1, 2):
        loop.set_enc_scale_err(p, 0.0)
        loop.set_enc_tick_quant(p, 0.0)
        loop.set_enc_slip(p, 0.0, 0.0)
    push_calibration(loop)   # match TestGUI's connect-time ml/mr push
    loop.set_true_pose(0.0, 0.0, 0.0)
    loop.step(1); loop._drain_tlm_into_queue(); loop.read_pending_binary_tlm_frames()  # noqa: SLF001

    vels = []          # reported R-wheel speed each cycle (shape)
    at_target = None   # frame snapshot the cycle true x first passes TARGET
    frames = 0
    for _ in range(400):
        loop.twist(SPEED, 0.0, 300.0)
        loop.step(1); loop._drain_tlm_into_queue()  # noqa: SLF001
        fr = loop.read_pending_binary_tlm_frames()
        tp = loop.get_true_pose()
        if fr:
            f = fr[-1]
            frames += 1
            if f.vel:
                vels.append(f.vel[1])
            if at_target is None and tp["x"] >= TARGET_MM:
                enc = (f.enc[0] + f.enc[1]) / 2.0 if f.enc else None  # per-wheel encoder mm
                at_target = {
                    "true": tp["x"],
                    "fused": f.pose[0] if f.pose else None,       # frame.pose = fused
                    "encoder": enc,                                # per-wheel encoder distance
                    "otos": f.otos[0] if f.otos else None,
                }
        if at_target is not None:
            break
    # stop and watch for reversal
    stop_vels = []
    for _ in range(20):
        loop.stop()
        loop.step(1); loop._drain_tlm_into_queue()  # noqa: SLF001
        fr = loop.read_pending_binary_tlm_frames()
        if fr and fr[-1].vel:
            stop_vels.append(fr[-1].vel[1])
    loop.disconnect()

    print("\n==================  DRIVE 700 mm  ==================")
    if at_target is None:
        print("  never reached 700mm true -- motion failed")
        return
    t = at_target
    def line(name, val):
        if val is None:
            print(f"  {name:9}: (absent)")
        else:
            err = val - t["true"]
            print(f"  {name:9}: {val:6.0f} mm   ({err:+.0f} vs truth, {val/t['true']*100:4.0f}%)")
    line("truth", t["true"])
    line("encoder", t["encoder"])
    line("otos", t["otos"])
    line("fused", t["fused"])

    cruise = vels[len(vels) // 3: len(vels) * 2 // 3] or vels
    hold = sum(cruise) / max(1, len(cruise))
    peak = max(vels) if vels else 0
    reverse = -min(stop_vels) if stop_vels else 0
    print("\n----------------  WHEEL SPEED  ----------------")
    print(f"  commanded  : {SPEED:.0f} mm/s")
    print(f"  hold       : {hold:.0f} mm/s   ({hold - SPEED:+.0f} vs command)")
    print(f"  front peak : {peak:.0f} mm/s   (jitter above hold = {max(0, peak - hold):.0f})")
    print(f"  end reverse: {max(0, reverse):.0f} mm/s")
    print(f"  rise cycles: {[int(v) for v in vels[:8]]}")
    print(f"  stop cycles: {[int(v) for v in stop_vels[:8]]}")


if __name__ == "__main__":
    run()
