import math, sys
from pathlib import Path
REPO = Path("/sessions/nice-eager-faraday/mnt/radio-robot-elite")
sys.path.insert(0, str(REPO / "src" / "host"))
from robot_radio.config.robot_config import load_robot_config
from robot_radio.io.sim_loop import SimLoop
from robot_radio.testgui.traces import EncoderDeadReckoner

loop = SimLoop(track_width=128.0, lib_path=Path("/tmp/simbuild/libfirmware_host.so"))
loop.connect(start_tick_thread=False)
loop.configure_from_robot(load_robot_config(REPO / "data/robots/tovez_nocal.json"))
loop.set_otos_raw_scale_err(0.0, 0.0)
for p in (1, 2):
    loop.set_enc_scale_err(p, 0.0); loop.set_enc_tick_quant(p, 0.0); loop.set_enc_slip(p, 0.0, 0.0)
loop.step(5); loop.drain_pending_tlm()
loop.move(omega=2.0, stop_angle=math.radians(360), timeout=15000.0, replace=True, id=9)
rk = EncoderDeadReckoner(128.0)
rk_gated = EncoderDeadReckoner(128.0)   # only fed while frame.active (GUI behavior)
last = None; done = None
first_active_enc = None
for i in range(400):
    loop.step(1)
    for f in loop.drain_pending_tlm():
        last = f
        if f.enc is not None:
            rk.update(*f.enc)
            if f.active:
                if first_active_enc is None:
                    first_active_enc = f.enc
                rk_gated.update(*f.enc)
        if f.ack is not None and f.ack.corr_id == 9:
            done = i
    if done is not None and i >= done + 25:
        break
px, py, ph = last.pose
print(f"firmware pose theta = {ph/100:+.1f} deg   enc final = {last.enc}")
print(f"reckoner (all frames)    theta = {math.degrees(rk._theta):+.1f} deg")
print(f"reckoner (active-gated)  theta = {math.degrees(rk_gated._theta):+.1f} deg")
print(f"first ACTIVE frame enc = {first_active_enc}")
loop.disconnect()
