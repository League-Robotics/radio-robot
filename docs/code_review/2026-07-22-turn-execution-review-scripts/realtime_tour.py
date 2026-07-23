"""Real-time-threaded mini-tour (GUI-like conditions) vs deterministic."""
import math, sys, time
from pathlib import Path
from types import SimpleNamespace
import os
REPO = Path(os.environ.get("RRE_REPO", Path(__file__).resolve().parent.parent))  # run from repo, or set RRE_REPO
sys.path.insert(0, str(REPO / "src" / "host"))
from robot_radio.config.robot_config import load_robot_config
from robot_radio.io.sim_config import SimConfigConn
from robot_radio.io.sim_loop import SimLoop
from robot_radio.robot.protocol import NezhaProtocol
from robot_radio.planner.heading import HeadingCorrector
from robot_radio.planner.model import PlannerParams
from robot_radio.planner.tour import parse_tour, run_tour

LIB = next((p for p in (REPO / "src/sim/build/libfirmware_host.dylib",
                        REPO / "src/sim/build/libfirmware_host.so",
                        Path("/tmp/simbuild/libfirmware_host.so")) if p.exists()), None)
CFG = REPO / "data" / "robots" / "tovez_nocal.json"
PUSH = dict(stop_lead_ms=45.0, a_max=800.0, a_decel=800.0, alpha_max=7.0,
            alpha_decel=7.0, j_max=5000.0, yaw_jerk_max=100.0)
MINI = ["D 200 200 345", "RT 9000", "D 200 200 240", "RT 9000", "D 200 200 345"]

loop = SimLoop(track_width=128.0, lib_path=LIB)
loop.connect(start_tick_thread=True)
loop.set_speed_factor(1)
loop.configure_from_robot(load_robot_config(CFG))
proto = NezhaProtocol(SimConfigConn(loop))
corr = proto.estimator_config(**PUSH)
deadline = time.monotonic() + 3.0
ok = False
while time.monotonic() < deadline and not ok:
    for f in loop.read_pending_binary_tlm_frames():
        if f.ack is not None and f.ack.corr_id == corr:
            ok = True
    time.sleep(0.02)
print("push acked:", ok)
loop.set_otos_raw_scale_err(0.0, 0.0)
for p in (1, 2):
    loop.set_enc_scale_err(p, 0.0); loop.set_enc_tick_quant(p, 0.0); loop.set_enc_slip(p, 0.0, 0.0)

legs = parse_tour(MINI)
params = PlannerParams()
heading = HeadingCorrector(params, robot_config=SimpleNamespace(
    geometry=SimpleNamespace(otos_untrusted=True)))
poses = [loop.get_true_pose()]
def on_leg(index, total, leg, res):
    pose = loop.get_true_pose()
    if leg.kind == "turn":
        d = math.degrees(pose["h"] - poses[-1]["h"])
        while d > 180: d -= 360
        while d <= -180: d += 360
        print(f"  REALTIME turn leg {index+1}: achieved={d:+8.2f} error={d-leg.value:+7.2f} deg")
    poses.append(pose)
res = run_tour(loop, params, heading, legs, v_max=150.0, on_leg=on_leg)
print("completed:", res.stopped_at is None)
loop.disconnect()
