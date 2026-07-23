"""Repro: straight 700mm leg -- truth vs firmware pose vs host-visible encoder pairs."""
import math, sys
from pathlib import Path
REPO = Path("/sessions/nice-eager-faraday/mnt/radio-robot-elite")
sys.path.insert(0, str(REPO / "src" / "host"))
from robot_radio.config.robot_config import load_robot_config
from robot_radio.io.sim_loop import SimLoop

loop = SimLoop(track_width=128.0, lib_path=Path("/tmp/simbuild/libfirmware_host.so"))
loop.connect(start_tick_thread=False)
loop.configure_from_robot(load_robot_config(REPO / "data/robots/tovez_nocal.json"))
loop.set_otos_raw_scale_err(0.0, 0.0)
for p in (1, 2):
    loop.set_enc_scale_err(p, 0.0); loop.set_enc_tick_quant(p, 0.0); loop.set_enc_slip(p, 0.0, 0.0)
loop.step(5); loop.drain_pending_tlm()
t0 = loop.get_true_pose()
loop.move(v_x=150.0, stop_distance=700.0, timeout=15000.0, replace=True, id=7)
prev = None
print(" t[s] | truth_th[deg] truth_y[mm] | dL-dR per frame [mm] (host view)")
done = None
for i in range(300):
    loop.step(1)
    frames = loop.drain_pending_tlm()
    tp = loop.get_true_pose()
    for f in frames:
        if f.ack is not None and f.ack.corr_id == 7:
            done = i
    row = ""
    if frames and frames[-1].enc_left is not None:
        el, er = frames[-1].enc_left.position, frames[-1].enc_right.position
        if prev is not None:
            row = f"dL={el-prev[0]:+6.2f} dR={er-prev[1]:+6.2f}  (dL-dR={el-prev[0]-(er-prev[1]):+5.2f})"
        prev = (el, er)
    if i % 2 == 0 or (done is not None and i <= done + 6):
        print(f"{i*0.04:5.2f} | {math.degrees(tp['h']-t0['h']):+7.3f} {tp['y']-t0['y']:+8.2f} | {row}")
    if done is not None and i >= done + 15:
        break
tp = loop.get_true_pose()
frames = loop.drain_pending_tlm()
print(f"\nFINAL: truth x={tp['x']-t0['x']:+.1f} y={tp['y']-t0['y']:+.1f} th={math.degrees(tp['h']-t0['h']):+.2f}deg")
loop.disconnect()
