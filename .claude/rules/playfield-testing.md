# Playfield Testing

The playfield is the camera-covered table. Driving off it is a **failure**, not
a synonym for driving (never call it "the floor" —
`.clasi/knowledge/playfield-not-floor.md`). This is the one regime that can fail
badly, so every run is camera-supervised. Bench/stand facts live in
[hardware-bench-testing.md](hardware-bench-testing.md) — the two regimes are
never combined.

## Room lights — turn them on yourself

Same network relay as the bench room: a **Shelly Plus 1 at `192.168.1.122`**,
no auth. **They turn off on their own.** When tags vanish, suspect the lights
FIRST — a dark field looks exactly like a broken camera or a lost robot.

```bash
curl -s "http://192.168.1.122/rpc/Switch.GetStatus?id=0"   # read: "output" is the lights
curl -s "http://192.168.1.122/rpc/Switch.Set?id=0&on=true"  # ON
curl -s "http://192.168.1.122/rpc/Switch.Set?id=0&on=false" # OFF
```

Prefer leaving them **on**; only turn them off when explicitly asked. Check
`output: true` BEFORE arming a run — 2026-07-29 a tour aborted with "tag 100 not
seen", which sent us hunting the camera when the lights had simply gone out.
`was_on` in the Set reply is not fully trustworthy (there may be a detached
physical input); confirm with `GetStatus`.

## Camera bring-up

`open_camera(pattern="arducam")` → `create_playfield(camera_id)` →
`start_detection(playfield_id)`, then poll `get_tags(playfield_id)`. The daemon
keeps no state across restarts, so do this every session. Field is
**134.3 × 89.3 cm**, A1-centred (tag 1 = origin), so limits are
**±67.15 / ±44.65 cm**.

A mobile tag is registered via `register_mobile_tag` as the robot's **centre of
rotation** — `world_xy` already reports the centre, not the raw tag. **Do not
hardcode the tag id** — it has already changed once (tovez: 100 → 52,
2026-08-14, see `tovez.json`'s own `_tag_note`) and a stale id here is exactly
what sent a session hunting a "lost" robot the camera could see fine. Read the
live value from the active robot config (`geometry`/`vision.robot_tag_id`,
whatever the loader exposes as `cfg.robot_tag_id`) or `list_mobile_tags()`,
never from memory or an old doc. Tag 1 is fixed: if tag 1 is missing, the
problem is the room, not the robot.

## Heading convention (measured, never assume)

Camera yaw: **0 = East, +90 = North**, radians, CCW-positive (ROS REP-103).
**Positive commanded omega INCREASES camera yaw** — `omega > 0` is CCW, so a
square tour with `omega > 0` at its corners runs COUNTER-CLOCKWISE in world
frame.

CHANGED 2026-08-13, and the old text said the exact opposite — if you find a
script or note claiming "+omega DECREASES camera yaw", it predates this and is
wrong. Everything now speaks one convention: camera, OTOS, encoder odometry
and commanded omega are all REP-103 CCW-positive.

The old behaviour was not a convention choice, it was a bug: on `tovez` the
firmware's "left" wheel was physically the RIGHT wheel (it is wired port 1 =
right), and since `omega = (vR - vL) / b` that negated every wheel-derived
heading while leaving forward motion correct — so nothing surfaced it, and it
got patched FOUR times downstream instead (`navigator.yaw_sign`, the
Navigator's own `pose.heading = -state.otos.heading`, `planner.cpp`'s
`applyOtosHeading(-...)`, and the sim's `kOtosHardwareMountSign`). All four are
gone; the truth now lives once, in the robot JSON's
`motors.left_port`/`right_port`.

Verified on tovez against camera truth: forward 150 mm commanded → +149 mm
along heading; `omega = +1.0` with a 90° angle stop → +92.6° CCW; from due
north, +90° → faces West, −90° → faces East.

## Halting: `estop()`, never `stop()`

`NezhaProtocol.stop()` is a **planned** stop — a planner queue entry that waits
behind the in-flight move. Commanded mid-leg it does nothing until that leg
finishes. `estop()` is the panic stop (clears Drive targets AND the planner
queue in one cycle).

MEASURED 2026-07-29 on a 400 mm leg with the halt sent 0.5 s in:

| verb | travel | active drops |
|---|---|---|
| `stop()` | 39.8 cm (full leg) | 5.9 s |
| `estop()` | 2.9 cm | 0.10 s |

Any geofence, Ctrl-C handler, or "halt now" path **must** call `estop()`. A halt
that raises must not be swallowed — a silent failure is indistinguishable from
a halt that worked, which is how a fence spent a day detecting correctly and
stopping nothing.

### One `estop()` was never verified — repeat it (2026-08-03)

Read the 2.9 cm / 0.10 s row above for what it is: **one measurement, of a
halt that happened to land.** Nobody had checked what a halt path calling
`estop()` exactly ONCE does when the write is lost — and the answer, measured
on `vevov` 2026-08-03 across 16/16 reproductions, is that it does nothing:

- a stop issued **once** by a host that then went quiet produced **936 mm of
  continued travel with no decay**, still going when the capture ended;
- `estop()` **failed 5 of 6 attempts**. Only repetition stopped the wheels.

The Nezha brick physically latches its last commanded speed and does not reset
on an nRF52 reset, so a lost zero write is permanent, not a glitch that clears
itself. Sprint 133 ticket 001 fixed both halves of the firmware gap (a
derived-idle safety arbitration step in `Core::RobotLoop`, and arming the stop
re-assertion window on the commanded nonzero→zero transition instead of on an
encoder reading). That fix is verified in sim and by construction; **hardware
re-verification on `tovez` is sprint 133 ticket 004 and has not happened yet.**

Until it has, a halt path that calls `estop()` once and returns is unverified.
Call it, confirm the robot actually stopped (telemetry `flags` bit 2 dropping,
encoders holding), and call it again if it did not.

## Untethered = the RADIOBRIDGE port

On battery the robot is reached through the relay dongle, NOT its own USB port.
Take the port from `mbdeploy list`'s ROLE column (`RADIOBRIDGE`).
`SerialConnection` detects the role and does the `!GO` handshake itself.

## Camera pose at EVERY segment boundary — not just start and end

**Mandatory** (stakeholder, 2026-07-29). Any multi-segment playfield run
captures a camera fix at every segment boundary, at REST (after the settle
dwell, so there is no velocity lag), median-of-7 samples. A 4-leg/4-turn square
tour yields NINE poses: start, then after each leg and each turn.

Start-and-end only cannot attribute error. From boundary poses you get directly:

| quantity | from |
|---|---|
| per-leg length | distance between the two rest poses spanning that leg |
| per-turn angle | yaw delta across that turn |
| per-leg cross-track | perpendicular deviation from the commanded heading |
| closure | first pose vs last |

2026-07-29: a run reported closure 226 mm / heading 322.8° from encoder odometry
while the camera said 71.7 mm / 368.9°. With only two fixes there was no way to
split the residual between leg-length and rotation error — and no way to catch
that the encoder yardstick itself was wrong. Per-boundary fixes make the
encoder-vs-camera disagreement visible on the FIRST segment instead of at the
end of the run.

Log them, and report per-segment truth alongside the odometry estimate.

## Never drive blind

Close the loop on the camera, cap each positioning leg (~25 cm) and re-fix
between hops, and check the fence INSIDE the move at ~10 Hz — not between
segments, where it can only narrate a crash that already happened.

## Pre-flight path check — compute before you command, every time

**2026-08-19 incident.** After a real edge-out and a subsequent geofence-caught
near-miss, the recovery pattern became "recenter the robot, then just launch
the tour and see if the geofence catches it." It did — but only because the
geofence is a backstop, and treating it as the *primary* check is the bug this
section exists to kill. Concretely: robot measured at (15.9, 15.8) cm, heading
+131.2°. A single 500 mm forward leg at that heading projects to
`(15.9 + 50·cos131.2°, 15.8 + 50·sin131.2°) ≈ (−17.0, 53.4)` cm — past the
field's 44.65 cm physical half-height before the leg even finishes. Two lines
of arithmetic would have shown this. None were done; the move was sent and the
geofence caught it mid-leg instead.

**The rule:** before sending ANY commanded motion on the playfield — a full
script like `square_tour.py`, a hand-rolled `WHEELS` sequence, anything —
compute the full projected path from a **measured** start pose (camera fix,
not an assumption) through every planned leg and turn, and confirm every
waypoint clears the geofence margin. If it doesn't, don't send the command:
reposition, reorient, or shorten the leg first. A command is not "probably
fine, the geofence will catch it" — it is verified safe before it is sent, and
the geofence is what catches the *unexpected* drift on top of that, not the
*expected* geometry.

Minimal check, straight-line legs + in-place turns (covers `square_tour.py`'s
own shape — adapt the loop for other paths):

```python
python3 -c "
import math
HALF_W, HALF_H, MARGIN = 67.15, 44.65, 12.0
x, y, heading = 15.9, 15.8, math.radians(131.2)   # measured start (cm, deg)
LEG, TURN = 50.0, math.radians(90.0)              # cm, rad -- match the planned path
for i in range(4):
    x += LEG * math.cos(heading)
    y += LEG * math.sin(heading)
    ok = abs(x) < HALF_W - MARGIN and abs(y) < HALF_H - MARGIN
    print(f'leg {i+1}: ({x:+.1f}, {y:+.1f}) cm  {\"OK\" if ok else \"!! OUT OF BOUNDS\"}')
    heading += TURN
"
```

Run this (with the actual measured pose) and read every line before the first
command goes out. If any leg prints "OUT OF BOUNDS," stop — do not launch the
run and hope the geofence intervenes.
