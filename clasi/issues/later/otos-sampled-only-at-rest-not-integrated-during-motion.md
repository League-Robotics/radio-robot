---
status: pending
filed: 2026-07-27
filed_by: team-lead (stakeholder directive)
tickets: []
superseded_by:
  - '135-002'
  - '135-003'
  - '135-006'
  - '135-007'
---

> **SUPERSEDED (2026-08-06, sprint 135 ticket 007).** Sprint 135's
> `Motion::Navigator` (ticket 003, `src/motion/navigator/navigator.cpp`)
> reads `state.otos.{x,y,heading}` on EVERY internal tick (50 ms) while a
> `GO_TO` is active — including mid-motion, the exact regime this issue's
> stakeholder directive said to stop integrating OTOS in ("we are not
> going to try to integrate the optical flow odometer while we are
> moving"). This is a deliberate, documented design decision (sprint.md's
> own Architecture Overview, "Navigation policy" responsibility; Open
> Questions item 6 of this very issue — "Does anything still need OTOS
> mid-motion?" — is answered YES, definitively, by this sprint), not an
> oversight: closed-loop point-target navigation cannot steer a live arc
> toward a target without a fresh pose every cycle while moving, so
> "sample only at rest" and "drive a continuously re-solving GO_TO" are
> mutually exclusive, and this sprint chose the latter.
>
> Hardware-verified, not just designed: ticket 006's bench pass on `tovez`
> (direct serial) ran a forward `GO_TO`, a target-behind `GO_TO`
> (stop-then-pivot-then-arc), and a mid-goto replacement, all completing
> cleanly with correct encoder motion and clean ack-ring behavior — the
> pivot phase's own stop condition explicitly reads OTOS heading
> mid-motion (ticket 006 Completion Notes, "Target-behind GO_TO" section:
> "per `Motion::Navigator`'s design, the pivot's own stop condition reads
> OTOS heading"). The stand's own OTOS-frozen limitation (wheels off the
> ground never translate/rotate the chassis) meant the pivot-to-arc PHASE
> TRANSITION specifically was not observed that session, but the
> mid-motion OTOS READ path itself was exercised and behaved correctly
> throughout every scenario run.
>
> This issue's own concerns (bus/loop-timing cost, measurement quality,
> "corrections matter at leg boundaries") were about `Motion::Planner`'s
> pre-existing continuous OTOS fusion into `Motion::StateEstimator`
> (`src/motion/state_estimator.h`) — that fusion path is UNCHANGED by
> sprint 135 and this issue's analysis of it is not wrong, merely
> superseded in scope: `Motion::Navigator` reads `state.otos` directly for
> its own arc-solving (bypassing `StateEstimator` entirely, per
> sprint.md's own Out of Scope: "no change to how `Motion::Planner`'s
> internal pose estimate is computed"), a second, independent OTOS
> consumer this issue never anticipated. Kept here for its per-module
> history of the ORIGINAL fusion-path concern; do not resurrect the
> "stop integrating while moving" directive against `Motion::Navigator`
> without a new decision that reconciles it with closed-loop goto
> navigation.

# Sample the OTOS only at rest — stop integrating optical flow during motion

## Stakeholder directive (2026-07-27)

> Whenever there is a stop, you are going to grab an update from the optical
> flow odometer. We are not going to try to integrate the optical flow
> odometer while we are moving. When the motors are stopped, you will take
> the time to grab a measure from the optical flow odometer and update the
> state estimation as to where we are located.

Two halves, and both matter:

1. **Stop integrating OTOS during motion.** While the wheels are turning,
   pose comes from wheel odometry alone.
2. **At rest, take a deliberate measurement** — and *take the time to do it
   properly*, since nothing is time-critical when the robot is stopped — then
   use it to correct the state estimate's position.

## Current behavior

`RobotLoop::cycle()` ticks the OTOS **every cycle**, inside the pace block
(`src/firm/app/robot_loop.cpp` ~line 760):

```cpp
otos_.tick(nowUs);
const bool otosPresent = otos_.present() && otos_.poseFresh();
state_.otos.connected = otos_.connected();
// ... state_.otos.{x,y,heading,...}, sampleTime
```

`Motion::StateEstimator` then fuses `input.otos` whenever it is fresh
(`src/motion/state_estimator.h`) — including mid-motion. So today the optical
flow is a continuous fusion input, which is exactly what this issue removes.

## Why

- **Bus and loop-timing cost.** An OTOS I2C read on the motion path is
  expensive, and the project has been bitten by this before — the per-pass
  OTOS tick had to be folded into the flip-flop schedule precisely because it
  wrecked motion timing. The brick already holds only one pending encoder
  read, and the L-settle / clear / R-settle windows are the tight part of the
  cycle. Moving the OTOS read off the motion path buys that budget back.
- **Measurement quality.** A deliberate at-rest read can afford settle time
  and averaging in a way a mid-motion snatch cannot. Optical flow during
  motion is also the regime most prone to trouble.
- **It matches how the pose is actually used.** Corrections matter at leg
  boundaries — where the next move is planned from — not continuously.

## Open questions (decide during planning, do not assume)

1. **What counts as "stopped"?** `kFlagActive` (flags bit 2) dropping is the
   obvious trigger, but there is a difference between *commanded* stop and
   *settled* stop. Wheels coast. A read taken while the chassis is still
   rocking is worse than no read. There is a known ~1.2 s post-STOP settle
   figure in the bench notes — confirm what is actually needed.
2. **How long do we dwell?** "Take the time" needs a number, and it trades
   directly against tour throughput: every leg boundary pays it. Is this one
   read, an average of N, or read-until-stable?
3. **Does the correction snap or blend?** Jumping the estimate discontinuously
   at every stop will show up in traces and could confuse anything
   differentiating pose. The estimator carries ZOH bases — decide whether the
   correction rewrites the basis or feeds a bounded update.
4. **Heading too, or position only?** The directive says "where we are
   located". OTOS heading and wheel-derived heading disagree in known ways
   (see the turn over/under-rotation and rotational-slip history). Sampling
   position but not heading is a legitimate option and should be an explicit
   choice, not an accident.
5. **What happens when the OTOS is absent or unhealthy?** Today
   `otos_.present()`/`poseFresh()`/`connected()` gate fusion. With reads only
   at rest, a dead sensor is discovered far less often — make sure the health
   signal does not silently go stale, and that the fault bit still means
   something.
6. **Does anything still need OTOS mid-motion?** Confirm no current consumer
   (heading hold, land-at-zero completion, the telemetry `otos` fields the
   host plots) depends on a fresh mid-motion OTOS reading. The telemetry
   fields will now hold their last at-rest value while moving — that is a
   visible behavior change for host tooling and should be intentional.

## Scope / touch points

- `src/firm/app/robot_loop.cpp` — move `otos_.tick()` off the per-cycle pace
  block onto a stop-triggered path; publish `state_.otos` accordingly.
- `src/motion/state_estimator.{h,cpp}` — OTOS becomes a discrete correction
  at rest rather than a continuous fusion input.
- `src/firm/devices/otos.{h,cpp}` — if an at-rest read wants settle/averaging
  behavior distinct from the current single tick.
- Telemetry: `otos` fields and the OTOS-present/health flags now update only
  at rest. Host consumers (`src/host/robot_radio/`, TestGUI traces) follow.
- Sim: `SimPlant`/`FakeOtos` must reproduce the at-rest-only cadence, or the
  sim will silently diverge from hardware — which is exactly how the last
  three wire/loop defects escaped to the bench.

## Acceptance sketch

- Zero OTOS I2C traffic while a Move is active — measurable, not asserted.
- A stop produces exactly one OTOS-derived correction, visible in telemetry.
- Loop-timing headroom during motion measurably improves against the current
  baseline (`cycleBusy`/`cyclePeriod` already ride the wire).
- Bench-verified on the stand, over the real link — and note that the whole
  radio-relay path is still unverified, so state which transport the
  acceptance ran on.

Related: [`otos-fake-seam-should-be-one-interface-two-implementations.md`](otos-fake-seam-should-be-one-interface-two-implementations.md),
[`fakeotos-belongs-in-devices-not-app-invariant-tension.md`](fakeotos-belongs-in-devices-not-app-invariant-tension.md).
