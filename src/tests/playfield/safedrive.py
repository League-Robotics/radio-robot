"""safedrive -- never issue a move that leaves the table.

Every failure tonight was the same: a move was ISSUED without first checking
where it would go. The fence only ever reacted, and reacting is too late once
the tag leaves the field (the fence goes blind exactly when it is needed).

So this module refuses at ISSUE time:

  1. Read the camera. No fix -> no move, ever.
  2. PREDICT the whole path. A GO_TO drives a tangent arc from the current pose
     to the target, so the arc is computable in closed form:
         sweep = wrap(2*(bearing - yaw0));  R = chord / (2 sin(sweep/2))
     Sample it end to end (plus the robot's own footprint radius).
  3. If ANY sampled point is outside the table minus a margin, the move is
     refused and returns a reason. Nothing is sent.
  4. Only then issue, and still guard during: tag loss or a boundary breach
     halts.

Table: 134.3 x 89.3 cm, A1-centred, so +-67.15 / +-44.65 cm.
"""
import math, time

FIELD_X, FIELD_Y = 67.15, 44.65    # [cm] physical half-extents
BODY_R = 9.0                       # [cm] robot footprint radius, centre to corner
STOP_DIST = 5.0                    # [cm] travel between a halt decision and rest
MARGIN = 3.0                       # [cm] extra keep-out
TAG = 100
GRACE = 0.6                        # [s] tag-loss tolerance while moving
TURN_FIRST = 0.8726646             # [rad] tovez.json navigator.turn_first_angle


def wrap(d):
    return math.remainder(d, 2 * math.pi)


def raw(dc, cam):
    tf = dc.get_tags(cam)
    return next((t for t in tf.tags if t.id == TAG and t.world_xy), None)


def fix(dc, cam, n=7):
    """Median-of-n camera pose, or None. NEVER drive without this."""
    xs, ys, ws = [], [], []
    for _ in range(n):
        t = raw(dc, cam)
        if t:
            xs.append(t.world_xy[0]); ys.append(t.world_xy[1]); ws.append(t.yaw)
        time.sleep(0.06)
    if not xs:
        return None
    med = lambda v: sorted(v)[len(v) // 2]
    w0 = ws[0]
    return med(xs), med(ys), w0 + med([wrap(w - w0) for w in ws])


def keepout():
    """Where a PLANNED path may go (tag-centre coords)."""
    return FIELD_X - BODY_R - MARGIN, FIELD_Y - BODY_R - MARGIN


def halt_line():
    """Where the guard must ALREADY be halting (tag-centre coords).

    Tag 100 reports the centre of rotation, so the body extends BODY_R beyond
    it, and the robot still travels ~STOP_DIST after the halt decision. Halting
    at the planner keep-out was the bug: it allowed a tag position whose BODY
    was already off the table.
    """
    return (FIELD_X - BODY_R - STOP_DIST, FIELD_Y - BODY_R - STOP_DIST)


def arcPoints(p0, fwd, left, n=60):
    """World points [cm] along the tangent arc a GO_TO(fwd,left) will drive."""
    x0, y0, yaw = p0
    # target in world
    tx = x0 + (fwd * math.cos(yaw) - left * math.sin(yaw)) / 10.0
    ty = y0 + (fwd * math.sin(yaw) + left * math.cos(yaw)) / 10.0
    dx, dy = tx - x0, ty - y0
    chord = math.hypot(dx, dy)
    if chord < 1e-6:
        return [(x0, y0)], (tx, ty)
    bearing = math.atan2(dy, dx)
    sweep = wrap(2.0 * wrap(bearing - yaw))
    pts = []
    if abs(sweep) < 1e-4:                       # straight
        for i in range(n + 1):
            pts.append((x0 + dx * i / n, y0 + dy * i / n))
        return pts, (tx, ty)
    R = chord / (2.0 * math.sin(sweep / 2.0))   # [cm], signed
    cx = x0 + R * math.cos(yaw + math.pi / 2)
    cy = y0 + R * math.sin(yaw + math.pi / 2)
    a0 = math.atan2(y0 - cy, x0 - cx)
    for i in range(n + 1):
        a = a0 + sweep * i / n
        pts.append((cx + abs(R) * math.cos(a), cy + abs(R) * math.sin(a)))
    return pts, (tx, ty)


def plannedPath(p0, fwd, left, n=60):
    """The path Motion::Navigator will ACTUALLY drive.

    Above navigator.turn_first_angle it pivots in place first and then runs
    essentially straight at the target; only below that does it sweep a tangent
    arc. Modelling every move as a tangent arc made large-bearing targets look
    like huge off-table loops and got them refused for no reason.
    """
    x0, y0, yaw = p0
    tx = x0 + (fwd * math.cos(yaw) - left * math.sin(yaw)) / 10.0
    ty = y0 + (fwd * math.sin(yaw) + left * math.cos(yaw)) / 10.0
    bearing = math.atan2(ty - y0, tx - x0)
    if abs(wrap(bearing - yaw)) > TURN_FIRST:
        pts = [(x0 + (tx - x0) * i / n, y0 + (ty - y0) * i / n) for i in range(n + 1)]
        return pts, (tx, ty), "pivot-then-straight"
    pts, tgt = arcPoints(p0, fwd, left, n)
    return pts, tgt, "tangent arc"


def checkPath(pts):
    """None if every point is safely on the table, else a reason string."""
    kx, ky = keepout()
    worst, wp = 0.0, None
    for (x, y) in pts:
        over = max(abs(x) - kx, abs(y) - ky)
        if over > worst:
            worst, wp = over, (x, y)
    if worst > 0:
        return (f"path leaves the table by {worst:.1f} cm at "
                f"({wp[0]:+.1f},{wp[1]:+.1f}); keep-out is +-{kx:.1f}/+-{ky:.1f}")
    return None


def guard(proto, dc, cam, seconds, track=None, odo=None):
    """Wait for rest. Halts on tag loss or boundary breach. Silent on the link."""
    t0 = time.time(); lastSeen = time.time(); hist = []
    kx, ky = halt_line()
    while time.time() - t0 < seconds:
        for f in proto.read_pending_binary_tlm_frames():
            if odo is not None and f.pose:
                odo.append(f.pose)
        t = raw(dc, cam)
        if t is None:
            if time.time() - lastSeen > GRACE:
                proto.estop(); time.sleep(0.3); proto.estop()
                return False, "tag lost"
        else:
            lastSeen = time.time()
            x, y = t.world_xy
            if track is not None:
                track.append((x, y, t.yaw))
            if abs(x) > kx or abs(y) > ky:
                proto.estop(); time.sleep(0.3); proto.estop()
                return False, f"boundary ({x:+.1f},{y:+.1f})"
            hist.append((time.time(), x, y))
            hist = [h for h in hist if time.time() - h[0] <= 1.3]
            if time.time() - t0 > 2.2 and len(hist) >= 5:
                dx = max(h[1] for h in hist) - min(h[1] for h in hist)
                dy = max(h[2] for h in hist) - min(h[2] for h in hist)
                if math.hypot(dx, dy) < 0.4:
                    break
        time.sleep(0.06)
    time.sleep(0.9)
    return True, "ok"


def goto(proto, dc, cam, fwd, left, speed=140.0, track=None, odo=None):
    """Camera-checked GO_TO. Refuses rather than driving off the table."""
    p0 = fix(dc, cam)
    if p0 is None:
        return None, "no camera fix -- refusing to move"
    pts, target, how = plannedPath(p0, fwd, left)
    bad = checkPath(pts)
    if bad:
        return None, f"REFUSED ({how}): " + bad
    dist = math.hypot(fwd, left)
    proto.read_pending_binary_tlm_frames()
    proto.go_to(fwd, left, frame=1, arrive=10.0,
                timeout=int(dist / speed * 1000) + 20000)
    ok, why = guard(proto, dc, cam, dist / speed + 16.0, track=track, odo=odo)
    return (fix(dc, cam), why) if ok else (fix(dc, cam), "HALTED: " + why)


def twist(proto, dc, cam, v_x, omega, stop_distance, speed=None, track=None, odo=None):
    """Camera-checked move_twist. Same refusal rule."""
    p0 = fix(dc, cam)
    if p0 is None:
        return None, "no camera fix -- refusing to move"
    v = abs(v_x) if v_x else 1.0
    dur = stop_distance / v
    if abs(omega) < 1e-6:
        s = math.copysign(stop_distance, v_x)
        fwd, left = s, 0.0
    else:
        th = omega * dur
        R = v_x / omega
        fwd = R * math.sin(th)
        left = R * (1 - math.cos(th))
    pts, _ = arcPoints(p0, fwd, left)
    bad = checkPath(pts)
    if bad:
        return None, "REFUSED: " + bad
    proto.read_pending_binary_tlm_frames()
    proto.move_twist(v_x, 0.0, omega, stop_distance=stop_distance,
                     timeout=int(dur * 1000) + 15000)
    ok, why = guard(proto, dc, cam, dur + 12.0, track=track, odo=odo)
    return (fix(dc, cam), why) if ok else (fix(dc, cam), "HALTED: " + why)


def turn(proto, dc, cam, deltaYaw, omega=0.9, track=None):
    """In-place turn -- cannot translate, so only tag-loss guarding is needed."""
    p0 = fix(dc, cam)
    if p0 is None:
        return None, "no camera fix -- refusing to move"
    if abs(deltaYaw) < math.radians(3):
        return p0, "ok"
    w = -omega if deltaYaw > 0 else omega      # +omega DECREASES camera yaw
    proto.move_twist(0.0, 0.0, w, stop_angle=abs(deltaYaw), timeout=15000)
    ok, why = guard(proto, dc, cam, abs(deltaYaw) / omega + 6.0, track=track)
    return (fix(dc, cam), why if ok else "HALTED: " + why)


def toCentre(proto, dc, cam, want=12.0, speed=120.0):
    """Walk to mid-field in checked hops, verifying progress across HOPS.

    Progress is only meaningful across a translation: an in-place turn cannot
    change the radius, so comparing before/after a turn always looks like a
    stall (that bug aborted the first attempt at this).
    """
    prevHopR = None
    for _ in range(12):
        p = fix(dc, cam)
        if p is None:
            return False, "no camera fix"
        r = math.hypot(p[0], p[1])
        if r <= want:
            return True, f"centred at ({p[0]:+.1f},{p[1]:+.1f})"
        err = wrap(math.atan2(-p[1], -p[0]) - p[2])
        if abs(err) > math.radians(6):
            _, why = turn(proto, dc, cam, err)
            if why.startswith("HALTED"):
                return False, why
            continue                      # turn does not count as a hop
        if prevHopR is not None and r > prevHopR - 0.5:
            return False, (f"not converging across hops: {prevHopR:.1f} -> {r:.1f} cm "
                           f"at ({p[0]:+.1f},{p[1]:+.1f})")
        prevHopR = r
        hop = min(200.0, max(60.0, (r - want) * 10.0))
        _, why = twist(proto, dc, cam, speed, 0.0, hop)
        if why.startswith(("REFUSED", "HALTED")):
            return False, why
    p = fix(dc, cam)
    return (True, f"stopped at ({p[0]:+.1f},{p[1]:+.1f})") if p else (False, "lost")
