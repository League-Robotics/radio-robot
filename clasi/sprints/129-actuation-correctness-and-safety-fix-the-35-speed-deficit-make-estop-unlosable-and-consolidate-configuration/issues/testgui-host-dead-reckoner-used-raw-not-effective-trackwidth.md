---
status: in-progress
priority: high
sprint: '129'
tickets:
- 129-005
---

# The GUI's encoder trace integrated with the RAW track width while the firmware used the EFFECTIVE one

## Symptom

On the bench 2026-07-31 the Fused and Encoder traces on the Playfield tab were
**710 mm and 15 deg apart** after a session of turning, having agreed to 0.1 deg
on a straight leg earlier the same session:

```
pose     x +537  y +180  th +180.5    (firmware)
encpose  x -173  y -175  th +195.6    (host dead-reckoned)
```

This *looks* like a broken estimator or a bad sensor, which is what makes it
expensive — it sent us hunting the OTOS and the fusion weights first.

## Cause

`__main__.py`'s `_on_robot_changed` fed the host `EncoderDeadReckoner`
`cfg.trackwidth` — the **raw** caliper value (tovez: 128.0 mm). The firmware
integrates its own pose with the **effective** width,
`trackwidth / rotational_slip` = 140.4 mm (`App::effectiveTrackWidth()`).

Two integrators, different geometry:

- predicted ratio 140.4 / 128.0 = **1.097**
- observed heading ratio 195.6 / 180.5 = **1.084**

within 1.2%. The error is proportional to **accumulated rotation**, which is why
it is invisible on a straight leg and enormous after a session of turns.

## Fix

Use `_effective_track_width(cfg)` (already in `testgui/transport.py`, written
for the sim's own copy of this bug) for the host dead-reckoner.

**Second consumer, same bug:** `TurnGraphPanel`'s `recorder` keeps its *own*
`EncoderDeadReckoner`, constructed with a hard-coded `trackwidth = 128.0` that
nothing ever updated — so the Heading and Distance strip charts carried the same
rotation-proportional error, for every robot, always. It must be set from the
same source at the same time.

## Follow-up

`_DEFAULT_TRACKWIDTH = 128.0` and `TurnTraceRecorder(trackwidth=128.0)` are
per-file literal defaults for a value that is a per-robot config property. A
default that silently produces a plausible-but-wrong number is worse than a
missing one; consider making the geometry required at construction so an
unconfigured recorder cannot draw at all.

Related: [[testgui-trace-sources-naming-and-visibility]],
[[remove-testgui-geometry-and-actuation-overrides]]
