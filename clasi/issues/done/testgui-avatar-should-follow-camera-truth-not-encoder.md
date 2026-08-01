---
status: done
---

# TestGUI avatar should follow the Camera/Truth line, not the encoder line

## Stakeholder statement (2026-07-31)

> The avatar should be attached to the "Camera/Truth" line, not to the
> encoder line, if the truth is really the truth.

Observed on the Playfield tab with all four traces enabled (Camera, Fused,
OTOS, Encoder): the traces diverge visibly across a tour, and the avatar
tracks the encoder-derived path — the least trustworthy estimate on screen —
while the camera, which is the one independent ground truth available, is
drawn but not used to place the robot.

## Where it is

`CanvasController._update_marker()` (`src/host/robot_radio/testgui/canvas.py`,
~line 839) picks the avatar's position with:

```python
pts = self._trace_model.encoder or self._trace_model.fused
```

Camera is not in that preference order at all. The heading is the same story:
`__main__.py`'s `on_frame_ready` passes `TraceModel.encoder_yaw` into the
parameter still named `fused_yaw_rad`, so both the position AND the rotation
of the avatar come from encoder dead reckoning.

There IS a camera path — `CanvasController.set_avatar_pose()`, fed by the
live-view worker from tag 100 — and once it has been called it LOCKS the
avatar to the camera pose. So the encoder preference above governs whenever
that worker is not feeding (and is what the operator is seeing).

## Why it is that way (and why the reason has expired)

`_update_marker()`'s own docstring records the decision: sprint 097 preferred
the encoder trace over `fused` because fused was "pinned at the anchor until
sprint 098 wires `PoseEstimator::tick()` — a fused-only avatar would never
move today." That was a stopgap for a fused estimate that did not yet move.
It was never an argument for preferring encoder over CAMERA, and the
condition it was written around no longer describes the system.

## What "truth" should mean here

Camera is the only estimate on that canvas not derived from the robot's own
sensors, and the project already treats it as the yardstick — see
`.claude/rules/playfield-testing.md` ("Camera pose at EVERY segment boundary",
and the 2026-07-29 run where encoder odometry reported 226 mm closure while
the camera said 71.7 mm). An avatar drawn from encoder dead reckoning shows
the operator a robot that is not where the robot is, and it drifts worst
exactly when a run is going wrong — which is when the operator is watching.

## Suggested direction (not a decision)

- Preference order for BOTH position and heading: camera -> fused -> encoder,
  with the last-good camera fix held (and visibly aged) when tag 100 drops out
  rather than silently falling back to a diverging source.
- Make the fallback visible in the UI — the operator should be able to tell at
  a glance which source is currently placing the avatar, since "camera lost"
  and "camera fine" must not look identical (the same failure mode
  `.clasi/knowledge/tag-loss-blinds-the-geofence.md` records).
- Rename the `fused_yaw_rad` parameter once it stops being the fused yaw; it
  has already been carrying the encoder yaw for several sprints, which is how
  this stayed invisible.

## Related

- `clasi/issues/unify-sim-and-robot-composition-roots.md` — same theme: a
  sim/UI stopgap that outlived its reason and quietly became the behaviour.
