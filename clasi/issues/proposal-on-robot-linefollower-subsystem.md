---
status: pending
---

# On-robot LineFollower subsystem

## Description

Line-following currently runs as a host-side control loop
(`src/tests/bench/line_follow.py`): it reads binary telemetry, computes a PD
steering correction from the 4-channel line sensor, and sends a new twist
over serial/radio every `CMD_PERIOD = 0.09s` (~11 Hz). "Crossing" detection
(all 4 channels lit) and "lost line" recovery are both host-side heuristics
today — the firmware and wire protocol have no concept of either.

Proposal: move the control loop itself onto the robot as a `Motion::`
subsystem that ticks inside the main firmware loop, at loop cadence, and
talks to `Motion::Planner` directly — eliminating the host round trip from
the control loop's critical path. The host keeps a simple on/off switch and
watches telemetry; it stops making the moment-to-moment steering decisions.
When the subsystem hits a crossing or loses lock on the line, it stops and
reports why, rather than deciding the course itself.

## Cause

The host loop's command rate (~11 Hz) is throttled well below the ~31 Hz the
firmware loop itself runs at, and every command still pays a full host round
trip (sensor sample → radio/USB uplink → Python → radio/USB downlink →
firmware) before it reaches the wheels. The script's own comments already
flag this as a real, unquantified risk: the D-term note that "true samples
arrive faster than CMD_PERIOD (~35ms vs 90ms)" and the warning that
telemetry "can gap for SECONDS (relay/radio dropout)"
(`src/tests/bench/line_follow.py:588,591`). Tight line-following needs a
control loop tighter than a host round trip can reliably deliver.

## Proposed fix

**Shape**: not a new pattern in the codebase — `Motion::Navigator`
(`src/motion/navigator/navigator.h`, sprint 135) is architecturally almost
exactly this: a subsystem that ticks once per cycle when it owns Drive, reads
a sensor-derived field off `Types::RobotState` (OTOS pose, for Navigator),
runs a control law to get a wheel command, issues it into `Motion::Planner`
via `planner_.move(next, replace=true)` each cycle, has explicit
start/cancel/active() lifecycle, and aborts with a fault reason when its
sensor goes stale. `LineFollower` is a sibling: same shape, different sensor
(`state.perception.line` instead of OTOS pose) and different control law (PD
on line-centroid error instead of arc-to-world-target). It belongs in
`src/motion` for the same reason Navigator does — it depends only on
`Types::RobotState` and `Motion::Planner`'s public surface, never on
`App::*`/`Devices::*`/`Config::*` (`src/motion/DESIGN.md` §3,
`planner.h:119-127`).

**`App::RobotLoop::cycle()` integration** — three places, all precedented by
how Navigator itself was added in sprint 135:

1. Tick dispatch (`robot_loop.cpp:820-827`) gains a third arm, LineFollower
   first (most specific/urgent owner):
   ```
   if (lineFollower_.active()) lineFollower_.tick(state_)
   else if (navigator_.active()) navigator_.tick(state_)
   else { planner_.tick(state_); planner_.update(state_); }
   ```
   Exactly one of the three may call `planner_.tick()`/`update()` a cycle.
2. `zeroUnownedMotion()` (`robot_loop.cpp:503-527`) needs
   `lineFollower_.active()` added to its ownership check. Not optional: this
   exact function has a documented, measured bug from forgetting to add a
   new owner — omitting `navigator_.active()` produced a 15x/2.2s velocity
   sawtooth on the playfield (`robot_loop.cpp:504-524`). A fourth owner
   risks the identical bug if this line is missed.
3. `haltOnStall()` (`robot_loop.cpp:547-564`) already does the "everywhere
   halt" (`drive_.estop(); planner_.estop(); navigator_.cancel();`) — add
   `lineFollower_.cancel()` there too, so a stall while line-following
   halts cleanly instead of leaving the subsystem believing it still owns
   motion.

**Command surface**: protocol v5's binary command plane already grew past
its original 3-arm design once, precedented exactly this way — `GO_TO` was
added as a new `cmd_kind` to give Navigator a wire entry point (sprint 135
ticket 004). A `LINE_FOLLOW` command kind follows the same precedent:
`enable=true` calls `lineFollower_.start()`; `enable=false` or a
`STOP`/`WHEELS`/`ESTOP` arriving mid-run calls `lineFollower_.cancel()` —
the same "new owner takes over, calls cancel() on the losing owners first"
pattern every existing handler uses (e.g. `handleWheels()`,
`robot_loop.cpp:362-370`). Gains (KP/KD, cruise speed, error→speed shaping
curve) are not wire literals — per `configuration-discipline.md`, they come
from the robot's config file and are pushed live via `CONFIG`, the same way
`Planner::applyShaperLimits()` is configured today. The command itself stays
a bare on/off.

**Control loop**:
- *Cadence*: the line sensor currently samples on alternating cycles (line
  on odd cycles, color on even — `telemetry.h:63-72`), ~15.5 Hz at
  `kCycle = 32ms` (~31 Hz loop). While `LineFollower` is active, suspend the
  alternation and sample line every cycle — color isn't needed mid-line-
  follow. Gate the existing odd/even alternation in `robot_loop.cpp`'s pace
  block (`:620-629`) on `lineFollower_.active()`; revert to normal
  alternation the instant it goes inactive.
- *Control law*: port the host's existing, already-tuned PD directly —
  `line_error()`'s coverage-weighted 4-channel centroid, `KP=0.030 KD=0.004`
  (`line_follow.py:337-353`). The host's D-term is a finite difference over
  the throttled command period because that's the only clock it has
  (`line_follow.py:588`); firmware ticks at a fixed, precise `kCycle`, so
  the D-term gets a true, low-noise `dt` for free.
- *Output*: a `Move` (wheels/twist kind, `replace=true`) into
  `Planner::move()` each cycle — identical mechanism to what Navigator
  already does, just a different source for `v_x`/`omega`.

**Stop conditions**: stop (don't drive through) at a crossing and report
why; stop on losing lock / an unreadable read and report why. This is a
deliberate simplification versus the host script's current behavior (which
holds omega and drives straight through a crossing, and attempts a
reverse-then-search recovery on lost line) — the firmware subsystem's job is
"stop and tell you why," not "decide the course." No on-board recovery
search, no automatic drive-through, for v1.

Reporting reuses the stall detector's exact pattern
(`RobotLoop::haltOnStall()`, `drive.cpp`'s `updateStall()`): a
subsystem-owned latch, surfaced as dedicated `flags` bits, cleared only when
a new motion command arrives (the same job `clearStallLatch()` does for
stall). Bits 26-31 are free (every bit 0-25 is taken; bit 11 is deliberately
poisoned/never-reuse, `telemetry.h:14-56`):

| bit | meaning |
|---|---|
| 26 | `kFlagLineFollowActive` — subsystem currently owns motion |
| 27 | `kFlagEventLineFollowAtCross` — stopped at a crossing (event, not fault) |
| 28 | `kFlagFaultLineFollowLost` — stopped: lost lock / unreadable read |

Rich telemetry falls out for free, not as new wire format: the loop already
emits the full `RobotState`-derived frame every cycle at ~31 fps (`line=`,
`pose=`, `twist=`, `flags`) regardless of what owns motion. With line data
fresh every cycle while engaged, turning `LineFollower` on already rides the
existing firehose at native rate.

**Out of scope for v1**:
- Path-anticipation (`course_profile.json`'s pre-measured "kink zone"
  speed/gain shaping) — track-specific knowledge, not a sensing problem.
  Ship the fixed-gain reactive loop first, revisit once proven on hardware.
- On-board decision-making at a crossing (turn left/right/straight) — the
  contract is stop-and-report; sequencing stays a host/human call.

**Sizing**: comparable in scope to sprint 135's Navigator + GO_TO addition —
one new `Motion::` subsystem (few hundred lines + a `*_test.cpp`, following
`navigator.cpp`/`navigator_test.cpp`), one new wire command kind (envelope
proto + host `NezhaProtocol` method + `docs/protocol-v5.md` update), three
new telemetry flag bits + host-side decode, and the `RobotLoop` integration
above. Reads as sprint-sized, not ticket-sized.

## Verification

Per `hardware-bench-testing.md`'s standing verification gate, anything
touching sensing/motor-control/command protocol must be bench-verified on
the stand before it's called done, then run on the playfield per
`playfield-testing.md` (camera-verified, per-segment poses) against the
current host loop's own measured course performance as the baseline to
beat.

## Related

- `src/motion/navigator/navigator.h` / `navigator.cpp` — the direct
  architectural precedent (sprint 135), including the `GO_TO` wire-command
  addition and the `zeroUnownedMotion()` third-owner sawtooth bug.
- `src/tests/bench/line_follow.py` — the host-side implementation this
  proposal supersedes; source of the validated PD gains and centroid/
  crossing heuristics to port.
- `src/motion/DESIGN.md`, `docs/design/design.md` §2/§5 — the firm/motion
  layer split and dependency rules this subsystem must follow.
- `.claude/rules/configuration-discipline.md` — gains/config must come from
  the robot's config file, not wire literals.
- `.claude/rules/hardware-bench-testing.md`,
  `.claude/rules/playfield-testing.md` — the verification gates this work
  is subject to.
