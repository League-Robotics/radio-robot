---
status: in-progress
sprint: '127'
tickets:
- 127-001
- 127-003
- 127-004
- 127-005
- 127-006
- 127-007
- 127-008
---

# Host-side path planner — `goto` and path following

## Description

Sprint 126 turned the OTOS into a calibrated instrument (units settled as mm,
centre-frame and lever-arm confirmed against camera truth, `otos_linear_scale`
corrected 1.067 → 1.0275 with the residual collapsing +3.91% → +0.37%,
`otos_angular_scale` confirmed at 0.987) but deliberately fused it into nothing.
This issue is how the calibrated sensor gets used.

Build an **outer position loop on the host**: a path planner that owns the
world-frame pose, computes the residual to a goal, and drives the firmware by
continuously replacing the in-flight move. It exposes `gotoWorld(x, y, theta)` and
`gotoRobot(x, y, theta)`, and path following is a small increment on top.

The firmware is not modified. No wire changes, no protobuf regeneration.

## Cause

The alternative — fusing OTOS into `Motion::Planner` — was considered and
rejected. An outer host loop is better on four counts:

- **The firmware stays a relative-move executor.** Its stop conditions are already
  measured from each move's own activation baseline, so they never needed a world
  frame. No fusion inside a control loop, no second heading source fighting
  `rotational_slip` (0.9117, camera-fitted), no wrap discipline inside a P-loop.
- **It defuses `geometry.otos_untrusted`.** Under fusion, OTOS must be good over a
  whole tour, and mount compliance is invisible. Here it only has to be good
  between camera re-anchors, and it is checked against the camera at every one.
  Mount compliance becomes a measured per-segment error term rather than a trust
  problem — which keeps sprint 128's remount gate intact instead of pre-empting it.
- **Path following falls out for free.** The goto solver aims at a target point;
  path following is the same solver with a different target-picker (the lookahead
  point on a curve). Segment length *is* the pure-pursuit lookahead — one knob.
- **It deletes work.** Cross-track correction becomes geometry ("you are not on
  the path, aim at the carrot") instead of a new firmware control loop.

Three findings from investigation constrain the design.

### 1. `replace=True` works, but profile continuity across it has two untested edges

`replace` is verified 3× on hardware
([`src/tests/bench/move_protocol_bench.py:316`](../../src/tests/bench/move_protocol_bench.py)),
in sim, and in unit tests; `NezhaProtocol` already defaults it to `True`.

**Edge A — axis change.** `src/motion/planner/planner.cpp:723-729` zeroes
`profileVelocity_`/`profileAccel_` on an axis change, justified by "we landed at
~0 there." That is an invariant normal chaining guarantees — `shapesCompatible()`
refuses to hand off at speed across an axis change — and that `replace` violates:
preempt at speed and the profiler is told the robot is stopped, so the next step
accel-limits up from zero while the PID integrator (which `replace` does not
reset, unlike `estop()`) carries its old state. Nothing tests it.

The axis is set by the **stop condition**, not by wheel signs: `axisOf()`
(`planner.cpp:835-843`) returns `Angular` only for `Kind::Angle`. A
distance-stopped twist is `Axis::Linear` however tight the arc — even tight enough
to reverse the inner wheel. So the design constraint is just "always use a
distance stop," and the operative rule is **never change axis while moving**; at
rest the zeroing is truthful, so stop → turn-in-place → resume is safe. Two
non-obvious ways to trip it: a planned `Stop` is `Axis::None`, and any
`move_wheels` is `Axis::Wheels`.

**Edge B — large curvature change within the Linear axis.** `profileVelocity_` is
held in axis units (body mm/s) and converted per-move at `planner.cpp:1075` via
`profileVelocity_ / active_.axisPerLambda`. A straight gives
`axisPerLambda = 1.0`; an arc with `unitLeft = −0.5, unitRight = 1.0` gives `0.25`.
Carrying 150 mm/s across that replace tells the profiler the dominant wheel is at
600 mm/s when it is at 150. Body speed genuinely is continuous through a curvature
change, so carrying it is the right invariant; the flaw is that per-wheel accel
limiting derives `previous` from the new shape rather than from measured wheel
velocity, so the differential adjustment is not correctly rate-limited. Negligible
for the per-cycle corrections a path planner emits, real for a curvature step.
Reasoned from code, not measured.

Also: `handleMove()` short-circuits on a duplicate `move.id` **before** honoring
`replace` (`src/firm/app/robot_loop.cpp:216-225`), so the path planner must issue
monotonically increasing ids or its replacements silently vanish while still
acking OK.

### 2. A host-side path planner needs no `set-pose` command

There is no pose-set command today (envelope field 7 is reserved-forever), and one
is not needed here. The host maintains a single SE(2) transform
`T_world_from_odom`, re-estimated at each camera fix, so
`world_pose = T ∘ firmware_pose`. No wire change, no host-side integration, no
drift bookkeeping — the firmware already integrates; the host only re-anchors.

A pose-set command becomes necessary only when the path planner moves *into*
firmware. Trap to record for that day: `Motion::Planner` keeps its **own**
`PoseTracker` (`src/motion/planner/planner.h:262`) separate from `Odometry`, and a
pose command must reset both or it lies to the stop conditions.

### 3. The same host code runs against sim and hardware unchanged

`SimConfigConn(SimLoop)` is a duck-typed `SerialConnection`, so `NezhaProtocol`
works verbatim against the sim
([`src/host/robot_radio/io/sim_config.py:43`](../../src/host/robot_radio/io/sim_config.py)),
and [`src/tests/bench/square_tour.py:341-390`](../../src/tests/bench/square_tour.py)
already proves the pattern — one tour body, two backends, with only "advance time"
and "ground truth" differing.

There is direct precedent for a host-resident planner driving real hardware at
rate: `HilSession`
([`src/motion/planner/bench/hil_drive.py:83`](../../src/motion/planner/bench/hil_drive.py))
runs at 20 Hz with `move_wheels(replace=True)`. It records the constant this work
needs — `actuationDelay = 150 ms` (transport + firmware PID lag) — and the
hard-won warning that closing the *wheel-velocity* loop host-side is unstable.
This design is an outer *position* loop over the firmware's velocity loop, which
is the stable topology, but the carrot distance must comfortably exceed the lag
distance (~30 mm at 150 mm/s; use ≥100 mm).

## Proposed fix

New package `src/host/robot_radio/pathplan/`. (`planner/` is taken by trajectory
profiles; `path/` is taken by pure curve geometry.) Three test tiers, matching the
repo's existing ones: pure unit → sim → bench.

**T1 — Characterize `replace` preemption.** Sim scenario + bench scenario:
Linear→Linear at speed, same curvature (the normal case; confirm profile
continuity); Linear→Linear at speed with a **large curvature step** (Edge B —
measure the per-wheel command discontinuity and use it to size the solver's
curvature slew limit); Linear→Angular at speed (Edge A); an axis change **from
rest** (confirm benign, since that is the sanctioned path for a terminal in-place
turn); and high-rate replacement (~20 Hz for several seconds — confirm the wheels
run smoothly when replacements arrive faster than the ~1-tick activation latency,
and that nothing pathological happens to the queue). Also verify the duplicate-id
no-op above.

**T2 — Promote camera fix + geofence into the package.** There are currently four
camera-fix implementations and two geofences, and `Geofence` lives in a bench
script imported by `sys.path` hack
(`src/tests/bench/otos_calibration_bench.py:59-65`). Move `Geofence` /
`GeofenceViolation` / `checkPlayfieldLights` / `captureFix` / `captureFixWithRetry`
into `robot_radio/field/`, re-point the two bench scripts, delete the hack. Move,
do not rewrite. One fix in transit: `captureFix`'s per-axis median medians raw yaw
with no angular unwrap (`square_tour.py:289-293`) and is wrap-unsafe near ±π — use
the circular mean `read_camera_pose` already implements
(`src/host/robot_radio/testkit/camera.py:38`).

**T3 — `WorldPose`, the host-side frame tracker.** Maintains `T_world_from_odom`
and re-anchors it from camera fixes. Consumes `TLMFrame.pose` (firmware encoder
pose) and `TLMFrame.otos_reading`, both of which carry `age`. Adds a **host
receive timestamp** to `TLMFrame` — there is none today, and no drain path records
one.

Deliberate by-product: tracking a transform for the encoder pose *and* one for the
OTOS pose yields the encoder-vs-OTOS divergence over time, free, in every run.
That is mid-motion OTOS characterization the project has never had, and it is what
sets the camera re-anchor cadence by data rather than assumption. Design for
occasional re-anchors — stakeholder reports OTOS holding over minutes and multiple
metres to within a few percent (2026-07-30, from demos, not yet measured here).

Reuse `Pose` / `heading_error` from `src/host/robot_radio/nav/pose.py` — plain data
types, explicitly carved out of the dormant-code freeze by `robot_radio/DESIGN.md`.
For time alignment use frame-age extrapolation only (`t - age`, the pattern
`hil_drive.py:117-143` uses); `ClockSync` exists but has no live caller and is
blocked on a `serial_conn.py` corr-id bug — do not take its activation on here.

**T4 — The goto solver (pure functions, no I/O).**
`solveArcToPoint(currentPose, targetPoint, limits) -> (v_x, omega, arcLength)`.
Given a pose and a target there is exactly one circular arc through the target
tangent to the current heading; compute its curvature and length and emit a
Distance-stopped twist move. Fully unit-testable without a robot.

The API takes a full pose `(x, y, theta)` so other drivetrains can use it; the
differential-drive solver **ignores the final `theta`**, documented at the
signature. Honoring it means a terminal turn from rest — a separate action and a
follow-up, not v1.

Two constraints the solver enforces so the planner never sees a T1 hazard:
a **curvature slew limit** (the emitted turn radius ramps, never steps), and a
**target-behind guard** (arc-to-point degenerates toward infinite curvature when
the target is behind; detect it and stop-then-turn, which is safe because the axis
change then happens from rest).

**T5 — The path planner loop.** `gotoWorld` / `gotoRobot` as one implementation
with two entry points; the robot-frame variant composes through
`T_world_from_odom`. Loop: read telemetry → update `WorldPose` → solve → issue
`move_twist(..., stop_distance=arcLength, replace=True)`. Two required properties:

- **Throttled replacement** — re-solve every frame, but only send when the
  solution has moved materially (curvature or remaining distance past a
  threshold). Cuts link traffic and command churn.
- **Explicit termination** — a tolerance and a give-up rule, not a loop that keeps
  trying to null the error. Below the drivetrain deadband, commands get zeroed and
  the robot stalls (the failure already fixed once with the `copysign(deadband)`
  boost). Minimum reliable move distance and turn angle are **unmeasured** and set
  the hard floor on achievable accuracy; T7 measures them.

Geofence is checked **inside** the time-advance primitive at ~10 Hz, never between
segments — the existing `drainFrames(proto, seconds, geofence)` idiom. Every halt
path calls `estop()`, never `stop()`. Backend split copied from `square_tour.py`'s
`_Backend`, so the same planner runs against `SimLoop` and hardware.

**T6 — Sim gate.** In `src/tests/sim/`: goto convergence against the real firmware
loop compiled into the sim dylib, using `SimLoop.get_true_pose()` as ground truth.
Assert convergence to tolerance, bounded overshoot, no stall, and sane behavior
under injected OTOS drift and encoder slip (`set_otos_drift`, `set_enc_slip`
already exist on `SimLoop`).

**T7 — Bench + playfield gate.** On the stand first (T1), then on the playfield
with camera truth. First-class measured outputs: goto convergence vs iteration
count; **minimum reliable move distance and turn angle**; encoder-vs-OTOS
divergence rate during motion (from T3), which sets the camera re-anchor cadence.
Chart saved as PNG.

**T8 — Path following.** Replace the target-picker: instead of "the goal," pick
the point on a `SampledPath` a lookahead distance ahead. Everything else is
unchanged. `src/host/robot_radio/path/` already holds arcs, Catmull-Rom, Bezier,
and a visibility-graph `plan_path` — all pure, all dormant, all reusable. Lookahead
floor ~100-150 mm per the lag analysis above.

### Files

| Path | Change |
|---|---|
| `src/host/robot_radio/pathplan/` | **new** — solver, world-pose tracker, planner loop |
| `src/host/robot_radio/field/` | receives `Geofence` + camera-fix helpers |
| `src/host/robot_radio/robot/protocol.py` | host receive timestamp on `TLMFrame` |
| `src/tests/bench/square_tour.py` | `Geofence` moves out; re-point |
| `src/tests/bench/otos_calibration_bench.py` | drop the `sys.path` hack |
| `src/tests/sim/system/` | replace-preemption + goto convergence gates |
| `src/tests/bench/` | goto bench gate |
| Reused as-is | `nav/pose.py`, `path/`, `io/sim_config.py`, `testkit/camera.py` |

## Verification

1. `uv run python -m pytest` — unit tier (solver math, world-pose transform,
   throttling logic) and sim tier.
2. `just build-sim && uv run python -m pytest src/tests/sim` — goto convergence
   through the real firmware loop.
3. On the stand: the T1 replace-preemption bench scenario, all five cases.
4. On the playfield, lights confirmed via `Switch.GetStatus` first — goto to a
   series of camera-verified world targets, then a followed path. Per-boundary
   camera fixes logged. `estop()` in a `finally` on every path. PNG chart produced.
5. Confirm `estimator.weight_heading_otos` / `weight_omega_otos` are still `0.0`
   and `geometry.otos_untrusted` is untouched — this work must not cross sprint
   128's remount gate.

## Related

Deliberately out of scope:

- **`set-pose` wire command** — needed only when the path planner moves into
  firmware. Cheapest as a `ConfigDelta` arm (no new verb, no registry change). See
  the `PoseTracker`/`Odometry` double-reset trap under Cause §2.
- **Firmware-side path planner** — doing this host-side first is the point.
- **OTOS fusion into `Motion::Planner`** — `headingOtosWeight` stays 0.0. This
  architecture removes the need; sprint 128's remount gate stays intact.
- **`ClockSync` activation** — blocked on a separate `serial_conn.py` corr-id bug.

Related issues:

- [`otos-sampled-only-at-rest-not-integrated-during-motion.md`](later/otos-sampled-only-at-rest-not-integrated-during-motion.md)
  — the stakeholder directive to sample OTOS only at rest. Complementary: a
  firmware-side at-rest re-anchor would give this path planner a good per-segment
  starting pose even with no camera available.
- [`estimator-v2-otos-fusion-sim-first.md`](later/estimator-v2-otos-fusion-sim-first.md)
  — the fusion approach this issue supersedes for the world-positioning use case.
- [`bench-accuracy-campaign-s3.md`](later/bench-accuracy-campaign-s3.md) — sprint
  128, gated on the physical OTOS remount.
