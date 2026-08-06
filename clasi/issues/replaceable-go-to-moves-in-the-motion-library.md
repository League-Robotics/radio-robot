---
status: pending
---

# Replaceable go-to moves in the motion library

## Description

Stakeholder directive (2026-08-05): support proper path planning — pure pursuit
and other path followers — by making go-to moves first-class in the motion
library. A goto targets an (x, y) point, robot-frame or world-frame. A new goto
with replace semantics drops the active move AND the entire planner queue and
re-plans from the CURRENT velocity: every goto still plans to decelerate and
land at rest at its target, but if a replacement arrives before decel begins,
the robot never slows. Velocity continuity across replacements is the core
requirement. Stopping should be rare — only when geometry forces it (target
behind, wheel reversal, arrival).

Two usage modes, both required:

- **EXTERNAL** — a host-side planner (pure pursuit tracing a path) streams
  updated goto targets down the wire.
- **INTERNAL** — the motion library itself closes the loop on the OTOS,
  re-checking each cycle whether the current trajectory reaches the target and
  re-planning internally.

### Current state (verified 2026-08-05)

- The firmware has **no point-target concept**. `Motion::Planner`
  (`src/motion/planner/`) executes only scalar-bounded Moves: Kind ∈ {Time,
  Distance, Angle, Stop} × {Twist, Wheels}. No x/y target on the wire
  (`src/protos/envelope.proto:189-206`) or in `Motion::Move`.
  `Types::Mode::GoTo = 4` is declared but has zero readers/writers.
- **`replace=true` already preserves velocity.** `Planner::move()`
  (planner.cpp:243-282) flushes queue + active but leaves
  `cmdLeft_/cmdRight_/cmdLeftPrevious_/profileVelocity_` intact; the
  replacement activates the same tick with no zero-command gap. `planWheels()`
  seeds each wheel from its own previous command (planner.cpp:1751-1757 — the
  fix recorded in
  `clasi/issues/done/replace-rescales-carried-profile-velocity-by-new-shape.md`).
  Each new move plans to land at rest (`activeBoundary_ = 0`) — exactly the
  requested semantics: always plan the decel, never execute it if a
  replacement arrives first.
- **The goto loop already exists twice on the host**:
  `src/host/robot_radio/pathplan/` (sprint 127: `solveArcToPoint`,
  `gotoWorld`/`gotoRobot`/`followPath` with pure-pursuit lookahead,
  `ReplaceThreshold` throttle, `MoveIdAllocator`, ack-verified retry) and
  `src/tests/bench/goto_otos.py` (2026-08-05: OTOS-navigated, replan @ 0.55 s,
  TURN_FIRST=50° pivot policy, fine-approach terminal commit; worked on
  hardware).
- **The case for firmware-internal, honestly stated** — robustness and
  precision, not necessity (the host loop demonstrably works):
  - Terminal granularity: one replan's travel drops from ~80 mm (host cadence)
    to ~7.5 mm (20 Hz internal at 150 mm/s); settling tolerance shrinks
    accordingly.
  - The inner loop sheds the ack-verify/retry/dedup machinery entirely, which
    has caused real failures (the one-move-sent-in-264-solve-iterations
    playfield incident; retry-stretched cycles overshooting a waypoint,
    `src/host/robot_radio/pathplan/solver.py:385-389`).
  - Independence from link quality, host liveness, and telemetry-silence
    gating (`TlmMode::kAuto` stops frames when idle).
  - NOT bought: actuation lag — 150 ms dead time + 230 ms plant tau are plant
    properties, identical for both loops.
- **Caution — a widely-cited number is folklore.** The "~20% inbound command
  loss" in `speed_map.py`/`square_tour.py`/`planner_square_tour.py` traces to
  a single unsourced 2026-07-27 docstring (no measurement artifact; written
  45 min after the 12-deep command ring landed, so possibly describing
  pre-ring firmware); one copy blames the DAPLink USB bridge, another the
  radio relay. `wire_truth.py` measures the OTHER direction (telemetry), and
  its relay budget is explicitly an unmeasured proposal. The firmware's own
  dropped-command counter (`commandsDroppedCount`, telemetry flags bit 18) has
  never been checked by any bench script. The cloverleaf 19/71 hardware
  failure (`clasi/issues/later/path-following-hardware-gaps.md`) is explicitly
  UNDIAGNOSED — candidates include the script's own accounting and `Move.id`
  dedup collisions, not established link loss.

### Open points for the stakeholder

1. **Frame convention**: the directive described robot-frame targets with "y in
   the direction of heading"; the codebase convention everywhere is
   **+x forward, +y left** (`solver.py`, `goto_otos.py`). This issue adopts the
   codebase convention — flag if a change is actually wanted.
2. An old directive
   (`clasi/issues/later/otos-sampled-only-at-rest-not-integrated-during-motion.md`,
   2026-07-27) says don't use OTOS during motion. Shipped practice since (OTOS
   heading in the planner, `goto_otos.py`) and this directive itself supersede
   it — mark that issue superseded when this lands.

## Proposed fix

### Architecture: `Motion::Navigator` layered above the Planner

A new class in `src/motion/navigator/`, fed by `RobotLoop::cycle()` just before
`planner_.tick()`. It owns one goto target; each cycle it reads `RobotState`
(OTOS pose), runs a C++ port of `solveArcToPoint`, and when the solution
changes materially issues an internal
`planner_.move(distanceTwistArc, replace=true)` — the same proven mechanism the
host loop uses, at 20 Hz with zero wire loss.

Both usage modes fall out of this one mechanism: INTERNAL mode *is* the
Navigator's per-tick re-solve; EXTERNAL mode is just a new target arriving over
the wire (a target update, not a new mechanism). Pure pursuit itself (path →
lookahead point) stays host-side and streams targets; a lost/late target update
is benign — the robot keeps converging on the previous point and lands at rest
there.

**Rejected: GoTo as a new Move::Kind inside the Planner** (variable curvature
within one Move). The planner's core proof assumes a fixed left:right ratio
captured once at activation (planner.cpp:1012-1017, 1689-1699 — the
discrete-exact-landing argument); per-tick re-solved curvature makes the
ratio-handoff hazard the steady state, guts
`boundaryLambda()`/`shapesCompatible()`, and forces OTOS x/y fusion into
`PoseTracker` (estimator-v2, deliberately deferred). The trajectory-manager
design of record
(`clasi/issues/later/build-a-position-driven-online-trajectory-manager-after-wheel-positions-are-available.md`)
explicitly defers "variable-curvature geometry within one Move". A 20 Hz
replace stream approximates continuous curvature to well under actuation lag.

### Key mechanics

- **Replace cadence**: re-solve every tick, **replace only on material change**
  (port `ReplaceThreshold`: ω/arc-length deltas + mandatory refresh at half-arc
  consumed). Replace-every-tick would permanently disarm the stall backstop
  (`activateNext()` resets `stallTicks`, planner.cpp:980-993) and fight the
  planner's own arrival detection. With a stable solution the in-flight Move
  lands at rest at the target by itself — arrival needs no special terminal
  mode in the common case; the fine-approach commit from `goto_otos.py`
  survives only as a tight-tolerance fallback.
- **Maintain-velocity / feasibility policy**:
  - Slowing into a tight arc is **automatic** — `shapeLimits()` projects ω/α
    ceilings onto the arc's shape and the ratio lock brakes the dominant wheel
    while preserving curvature (shape.cpp:132-156, profile.cpp:66-72). The
    Navigator adds only an approach-speed taper near the target (tracking
    accuracy, not feasibility).
  - **Target behind** (|bearing| > ~90°: no tangent arc exists) or bearing >
    TURN_FIRST (~50°): pivot first. If moving, enqueue `plannedStop()` + queued
    pivot (the planner sequences brake→rest→pivot cleanly) rather than
    replacing at speed, which ratio-lock hard-brakes the reversing wheel.
    Below TURN_FIRST, never pivot — steer the error out with curvature
    (measured limit-cycle otherwise).
  - Stops are genuinely required only for: target-behind, wheel-direction
    reversal, arrival. Everything else maintains velocity by construction.
- **World pose**: Navigator reads `state.otos.{x,y,heading}` directly
  (world-seeded via the existing `SEED` verb; OTOS is tape-calibrated to
  0.2%). No PoseTracker fusion (that's estimator-v2). On OTOS staleness: skip
  re-solve, let the bounded in-flight Move land. On real disconnect: propagate
  by `state.pose` deltas for a bounded window, then abort the goto with a
  fault-flagged ack. Goto acceptance gated on `otos.connected`.
- **Robot-frame targets resolve to world once at acceptance** (matches
  `gotoRobot()` and `goto_otos.py`: "so the target does not chase the robot as
  it turns"). Convention: +x forward, +y left.
- **One completion ack per goto**, keyed on `GoTo.id`, fired by the Navigator
  on arrival/abort/timeout. Internal segment Moves carry id=0 and their
  TickResults route through the Navigator — NOT to `publishMoveResult()`,
  which would push a spurious `ack(0)` per internal replacement
  (robot_loop.cpp:449-451 acks unconditionally).
- **Ownership**: a goto cancels Drive teleop (`drive_.takeover()`); a wire
  MOVE/WHEELS/ESTOP cancels the Navigator. Same one-owner rule as today.

### Wire protocol (protocol v5 addition)

New top-level `CommandEnvelope` arm — not a new Move variant (different
dispatch target; avoids ctypes-mirror churn on `Motion::Move`). `goto` is a
C++ keyword → arm `go_to = 26`, verb `GO_TO` (precedent:
`get_config`/`GET_CONFIG`).

```proto
message GoTo {
  float  x = 1;        // [mm]
  float  y = 2;        // [mm]
  uint32 frame = 3;    // 0 = WORLD (OTOS/SEED frame), 1 = ROBOT (resolved once at acceptance)
  float  speed = 4;    // [mm/s] cruise; 0 = config default
  float  arrive = 5;   // [mm] arrival tolerance; 0 = config default
  float  timeout = 6;  // [ms] REQUIRED whole-goto backstop
  uint32 id = 8;       // echoed in the ONE completion ack
}
```

A goto is **always** a replacement (that's its meaning — a target update); no
`replace` flag. Terminal heading omitted at v1 (proto3-additive later). A
future additive field worth pre-planning: optional path-curvature feed-forward
at the target — the fix `path-following-hardware-gaps.md` identifies for tight
corners. Dedup via the existing 16-slot accepted-id ring; host keeps
`MoveIdAllocator`.

### Implementation stages (≈7 tickets, one sprint)

0. **Measure inbound command loss on current firmware** (small, bench): stream
   N id-distinct commands at a known rate over USB and over the relay; count
   enqueue acks AND read `commandsDroppedCount`/flags bit 18 (never before
   consulted) to split link loss from firmware ring overflow. Replaces the
   unsourced "~20%" comments with a measured number; sizes the EXTERNAL-mode
   retry policy honestly.
1. **Arc-solver core** — `src/motion/navigator/arc_solver.{h,cpp}`: pure C++
   port of `solveArcToPoint` + omega slew clamp + behind-guard,
   `NavigatorLimits` struct. Standalone ctest mirroring solver.py's test
   surface.
2. **Navigator state machine** — `navigator.{h,cpp}`: Idle → (Pivot | Cruise)
   → FineApproach → Done/Aborted; material-change throttle; stop-then-pivot;
   timeouts; OTOS dropout policy; TickResult consumption. ctest against a real
   `Planner` with scripted RobotState — velocity continuity across
   replacements asserted numerically.
3. **Wire + routing** — `GoTo` proto arm + codegen, `RobotLoop::handleGoto()`,
   TickResult routing, estop/MOVE/WHEELS cancellation, `NavigatorLimits` into
   robot JSON + bake path (configuration-discipline: runtime and bake read the
   same file).
4. **Sim system tests** — goto W/R end-to-end over the wire codec in
   `src/tests/sim/system/`, streamed-target EXTERNAL-mode scenario.
5. **Bench/playfield (HITL)** — port `goto_otos.py` to emit `GO_TO`; A/B
   against the host loop: camera-truth arrival error, per-boundary minimum
   wheel speed as the "never slows" metric; tune `NavigatorLimits` on tovez.
6. **Host shrink** — `pathplan.gotoWorld/gotoRobot` become thin `GO_TO`
   senders (keep ack-verified retry — link loss, whatever ticket 0 measures it
   to be, is a link property); `followPath` keeps `pursuitTarget()` host-side,
   streams `GO_TO` targets.

Build-list gotcha budgeted in ticket 1: new motion .cpp files touch
`src/motion/planner/CMakeLists.txt` (or a sibling lib),
`src/sim/CMakeLists.txt` `MOTION_SOURCES`, and the 22 pytest `_APP_SOURCES`
lists under `src/tests/sim/`.

### Known landmines (receipts)

- Spurious `ack(0)` per internal segment if TickResult isn't routed through
  the Navigator (robot_loop.cpp:449-451; telemetry.cpp:127-139 doesn't filter
  id 0).
- Aligning phase (~2 s trim) would fire on internal pivots — Navigator
  replaces the pivot with the outbound arc before it completes
  (planner.cpp:766-770).
- Angle-threshold rotation calibration is skipped when OTOS present
  (robot_loop.cpp:210-221) — Navigator pivots bypass `handleMove()` entirely,
  correct only because goto is gated on `otos.connected`.
- Omega sign: commanded ω is opposite world-CCW (`YAW_SIGN = -1`, reconciled
  by the OTOS-heading negation at planner.cpp:499-513). One constant at the
  Navigator's command boundary, commented to flip together with the deferred
  kinematics fix.

## Verification

- `planner_tests`-style ctests: arc solver parity with solver.py;
  Navigator+Planner velocity continuity (no zero-command tick, per-wheel step
  bounds across N replacements); target-behind; overshoot recapture; dropout
  abort.
- Sim: end-to-end GO_TO over the codec, streamed-target run.
- Stand (tovez, by UID): GO_TO smoke — enqueue ack, encoders climbing, single
  completion ack.
- Playfield: camera-supervised A/B vs host `gotoWorld` — arrival error at
  camera truth, minimum wheel speed across replacements, waypoint stream never
  landing at rest until the final target. Geofence via `estop()` inside the
  loop at ~10 Hz.

## Related

- `src/host/robot_radio/pathplan/solver.py`, `pathplan/planner.py` — the
  host-side implementation being ported.
- `src/tests/bench/goto_otos.py` — the benched goto policy (TURN_FIRST,
  fine-approach, resolve-once, YAW_SIGN).
- `clasi/issues/later/path-following-hardware-gaps.md` — curvature
  feed-forward + the undiagnosed cloverleaf hardware run.
- `clasi/issues/later/build-a-position-driven-online-trajectory-manager-after-wheel-positions-are-available.md`
  — trajectory-manager design of record; defers variable-curvature-per-Move.
- `clasi/issues/later/estimator-v2-otos-fusion-sim-first.md` — why the
  Navigator reads OTOS directly instead of fusing into PoseTracker.
- `clasi/issues/later/otos-sampled-only-at-rest-not-integrated-during-motion.md`
  — superseded by this directive when it lands.
- `clasi/issues/done/replace-rescales-carried-profile-velocity-by-new-shape.md`
  — the fixed defect that makes velocity-preserving replace trustworthy.
- `docs/design/motion-planner-sketch.md`, `src/motion/DESIGN.md`,
  `docs/protocol-v5.md`.
