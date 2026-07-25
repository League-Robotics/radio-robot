---
status: done
---

# Telemetry frame should BE the robot state — one object, filled and sent

## The idea (stakeholder, 2026-07-25)

> Make the telemetry frame — the data that you send for telemetry — exactly
> equal to the robot state. We don't have to fill up two objects like this.
> Just make it the robot state. Fill that one up with data and then just send it.

Today the robot-loop hot path fills **two** parallel objects each cycle:

- `App::Telemetry::Frame frame_` — the wire/telemetry payload, staged
  field-by-field in `robot_loop.cpp` (`frame_.encLeft`, `frame_.pose`,
  `frame_.otos`, `frame_.twist`, `frame_.line`, `frame_.color`, cycle timing,
  flags) and handed to `tlm_.setFrame(frame_)` for emit.
- `Motion::StateEstimator::Input estimatorInput` — a second struct, populated
  by copying the *same* primary-source values out again
  (`estimatorInput.encLeftPosition = frame_.encLeft.position`, `poseX =
  frame_.pose.x`, `twistVX = frame_.twist.v_x`, …) and passed to the estimator.

That is two structs carrying the same robot state, filled from the same
sources, one copied out of the other. The stakeholder wants a single robot-state
object: fill it once from the primary sources, feed the estimator from it, and
send *it* as the telemetry frame — no second parallel struct, no field-by-field
re-copy.

## Why this matters

- Removes a whole class of drift/staleness bugs where the two objects disagree
  (fill order, generation, which one got the fresh reading).
- One assembly point, one source of truth for "what the robot currently is."
- Simpler hot path — no `Frame`→`Input` marshalling boilerplate.

## Scope / touch points

- [src/firm/app/robot_loop.cpp](../../src/firm/app/robot_loop.cpp) — the dual
  `frame_` / `estimatorInput` staging (~lines 184–260, 649–770).
- `App::Telemetry::Frame` (telemetry.h) and
  `Motion::StateEstimator::Input` (`src/motion/state_estimator.h`) — the two
  structs to unify into one robot-state type.
- The base↔motion boundary: whatever the unified type is, `StateEstimator`
  consumes it and `Telemetry` serializes it. Watch the firmware-base vs.
  motion-library split (`Motion::WheelSink` is the only sanctioned boundary
  interface) — a shared robot-state struct crossing that boundary is a design
  decision, not a free refactor.

## Tension to resolve during design

Prior stakeholder guidance (2026-07-25, `robot_loop.cpp` correction; captured in
memory `telemetry-frame-single-assembly-from-sources`) was: assemble the
outbound frame in ONE block right before emit, from primary sources, and **feed
the estimator from the sources, never by reading OUT of `frame_`**. This issue
(unify into one object) and that guidance both point at "one assembly from
primary sources" — but reconcile the direction: the unified robot-state object
must itself be filled from primary sources, and the estimator reads that
object (which now *is* the sources' snapshot), not a stale copy. Confirm the
intended data-flow with the stakeholder before implementing.

Related in-flight work: sprint 123 ticket
[007](../sprints/123-firmware-base-hardening-cobs-crc-binary-framing-telemetry-migration/tickets/007-robot-loop-single-point-telemetry-frame-assembly-from-primary-sources-feed-stateestimator-from-sources-not-frame.md)
(single-point telemetry frame assembly / feed StateEstimator from sources) — this
issue may fold into or extend that ticket rather than stand alone.
