"""ccw_square_50.py — drive a 50 cm CCW square (forward 50, turn left 90, ×4) and
record FOUR odometry tracks, drawn persistently on the AprilCam view:

  - camera (ground truth, tag 100 -> robot centre)
  - OTOS   (raw otos=, robot-zeroed -> world)
  - encoder(wheel odometry, integrated host-side, robot-zeroed -> world)
  - fused  (firmware pose=, robot-zeroed -> world)

The legs are forward-as-faced (camera-tracked, geofence safety stop so the robot
never leaves the playfield); the +90 turns are camera-tracked CCW. The overlay is
cleared ONCE at start, then the four tracks are published with a long TTL so they
REMAIN on screen after the program ends. A PNG is also saved.

Run:  uv run python tests/bench/ccw_square_50.py
"""
import sys, time, math
sys.path.insert(0, "/Volumes/Proj/proj/RobotProjects/radio-robot-elite/host")
sys.path.insert(0, "/Volumes/Proj/proj/RobotProjects/radio-robot-elite/tests/bench")
from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol, parse_tlm
from world_goto_chart import (open_daemon, read_cam_pose, playfield_from_daemon,
                              geofence_from_playfield, in_fence)

LEG = 50.0; SPEED = 100; MARGIN = 8.0; PERSIST = 100000.0; TRACK = 128.0; TAG = 100
COL = {"camera": [60, 220, 90], "otos": [0, 190, 255],
       "enc": [255, 150, 0], "fused": [255, 69, 160]}
def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi

# --- daemon: pick the live camera that sees tag 100 ---
dc, cam = open_daemon(1); cam = cam[0] if isinstance(cam, (tuple, list)) else cam
chosen = None
for c in (dc.list_cameras() or [cam]):
    a = getattr(dc.get_tags(c), "frame_id", None); time.sleep(0.4)
    b = getattr(dc.get_tags(c), "frame_id", None)
    if a is not None and b != a and any(t.id == TAG and t.world_xy is not None
                                        for t in dc.get_tags(c).tags):
        chosen = c; break
if chosen is None: sys.exit("daemon not live / tag100 not seen")
cam = chosen
dc.publish_overlay(cam, [], ttl=2.0)              # clear overlay ONCE, at start
pf = playfield_from_daemon(dc); fence = geofence_from_playfield(pf, MARGIN)

def rcam():
    for _ in range(20):
        c = read_cam_pose(dc, cam, TAG, 0.5)
        if c: return c
    return None

conn = SerialConnection("/dev/cu.usbmodem2121402"); conn.connect()
p = NezhaProtocol(conn)
for _ in range(12):
    if p.ping(): break
    time.sleep(0.5)

c0 = rcam()
if c0 is None: sys.exit("no camera fix")
cx0, cy0, h0 = c0; ch0, sh0 = math.cos(h0), math.sin(h0)
print(f"start ({cx0:+.1f},{cy0:+.1f}) facing {math.degrees(h0):+.0f}°")

p.send("OZ", 400); p.send("ZERO enc pose", 400); time.sleep(0.5)
conn.send_fast("STREAM 50"); time.sleep(0.2)

camW = []; otosW = []; encW = []; fusW = []
o0 = [None]; po0 = [None]; pe = [None]; eh = [0.0]; exy = [0.0, 0.0]

def tw(rx, ry):  # robot-zeroed body coords (cm) -> world via start pose
    return (cx0 + rx * ch0 - ry * sh0, cy0 + rx * sh0 + ry * ch0)

def pump():
    for ln in conn.read_lines(duration=40):
        f = parse_tlm(ln)
        if not f or f.pose is None: continue
        if f.otos:
            if o0[0] is None: o0[0] = f.otos
            otosW.append(tw((f.otos[0] - o0[0][0]) / 10.0, (f.otos[1] - o0[0][1]) / 10.0))
        if f.pose:
            if po0[0] is None: po0[0] = f.pose
            fusW.append(tw((f.pose[0] - po0[0][0]) / 10.0, (f.pose[1] - po0[0][1]) / 10.0))
        if f.enc:
            if pe[0] is None:                      # (re)baseline after a D reset
                pe[0] = f.enc
            else:
                dL = f.enc[0] - pe[0][0]; dR = f.enc[1] - pe[0][1]
                if abs(dL) > 5000 or abs(dR) > 5000:  # encoder reset jump -> rebaseline
                    pe[0] = f.enc
                else:
                    dC = (dL + dR) / 2; dT = (dR - dL) / TRACK
                    eh[0] += dT
                    exy[0] += dC * math.cos(eh[0]); exy[1] += dC * math.sin(eh[0])
                    pe[0] = f.enc
            encW.append(tw(exy[0] / 10.0, exy[1] / 10.0))

def flat(L):
    o = []
    for x, y in L: o += [x, y]
    return o

def publish(ttl=PERSIST):
    e = [{"type": "polyline", "params": flat(L), "color": COL[k], "thickness": 3}
         for L, k in ((camW, "camera"), (otosW, "otos"), (encW, "enc"), (fusW, "fused"))
         if len(L) >= 2]
    try: dc.publish_overlay(cam, e, ttl=ttl)
    except Exception: pass

def drive_leg(label):
    c = rcam(); bx, by, bh = c; bch, bsh = math.cos(bh), math.sin(bh)
    D = LEG
    while D > 10 and not in_fence(bx + D * bch, by + D * bsh, fence): D -= 5
    if D < 10:
        print(f"  {label}: no room forward (would leave playfield); stopping square")
        return False
    if D < LEG: print(f"  {label}: forward {D:.0f}cm (clipped for fence)")
    else:       print(f"  {label}: forward {D:.0f}cm")
    p.send("STOP", 150); time.sleep(0.4)          # clean state after a VW turn
    p.distance(SPEED, SPEED, int(D * 10)); pe[0] = None   # D resets encoders -> rebaseline
    t0 = time.monotonic(); lp = 0
    while time.monotonic() - t0 < 7:
        pump(); cc = read_cam_pose(dc, cam, TAG, 0.2)
        if cc:
            camW.append((cc[0], cc[1]))
            if not in_fence(cc[0], cc[1], fence): print(f"  {label}: fence stop"); break
            if math.hypot(cc[0] - bx, cc[1] - by) >= D: break
        if time.monotonic() - lp > 0.3: publish(); lp = time.monotonic()
    for _ in range(4): p.stop(); time.sleep(0.05)
    return True

def turn_ccw_90(label):
    c = rcam(); target = wrap(c[2] + math.pi / 2); cur = c[2]
    print(f"  {label}: +90 CCW")
    t0 = time.monotonic()
    while time.monotonic() - t0 < 9:
        err = wrap(target - cur)
        if abs(math.degrees(err)) < 5: break
        conn.send_fast(f"VW 0 {int(max(150, min(600, abs(math.degrees(err)) * 10))) * (1 if err > 0 else -1)}")
        pump(); cc = read_cam_pose(dc, cam, TAG, 0.12)
        if cc: cur = cc[2]; camW.append((cc[0], cc[1]))
        time.sleep(0.02)
    for _ in range(6): conn.send_fast("VW 0 0"); time.sleep(0.03)
    p.stop(); time.sleep(0.3)

# --- the square: forward 50, turn left, ×4 ---
try:
    for i in range(4):
        if not drive_leg(f"side{i+1}"): break
        pump()
        turn_ccw_90(f"turn{i+1}")
        pump()
finally:
    for _ in range(5): p.stop(); time.sleep(0.05)
    conn.send_fast("STREAM 0")
publish()   # final, persistent
print("tracks left on view (ttl %.0fs)" % PERSIST)

# --- report closure error (how far the fused/camera end is from the start) ---
def closure(name, L):
    if len(L) < 2: print(f"  {name}: <2 pts"); return
    e = math.hypot(L[-1][0] - L[0][0], L[-1][1] - L[0][1])
    print(f"  {name:>7}: end-to-start gap = {e:.1f} cm")
print("square closure (smaller = better; a perfect square returns to start):")
for name, L in (("camera", camW), ("OTOS", otosW), ("encoder", encW), ("fused", fusW)):
    closure(name, L)

# --- save a PNG too ---
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
fig, ax = plt.subplots(figsize=(8, 8)); ax.set_facecolor("#101418")
for L, c, l in ((camW, "#3cdc5a", "camera (truth)"), (otosW, "#00becf", "OTOS"),
                (encW, "#ff9600", "encoder"), (fusW, "#ff45a0", "fused")):
    if L: ax.plot([q[0] for q in L], [q[1] for q in L], "-o", ms=2, lw=1.6, color=c, label=l)
ax.set_aspect("equal"); ax.grid(alpha=0.2)
ax.legend(facecolor="#202428", labelcolor="white")
ax.set_title("CCW 50cm square — camera / OTOS / encoder / fused", color="white")
ax.tick_params(colors="#aaa")
fig.savefig("tests/bench/out/ccw_square_50.png", dpi=110, facecolor="#101418")
print("saved tests/bench/out/ccw_square_50.png")
conn.disconnect()
