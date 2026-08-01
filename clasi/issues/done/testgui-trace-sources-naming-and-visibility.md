---
status: done
priority: medium
---

# Playfield traces: name each trace for its source, and show a row only when that source reports

Stakeholder direction, 2026-07-31.

## 1. "Fused" is a misnomer — rename to "Pose"

The trace labelled **Fused** is fed from `frame.pose`, which
`src/protos/telemetry.proto:327` documents as *"encoder-odometry integrated
pose, always present"*. Nothing is fused into it:

- `weight_heading_otos` / `weight_omega_otos` are **0.0** in every robot JSON
  (the deliberate encoder-only-v1 decision).
- Position is never blended at any weight — `state_estimator.cpp` does
  `body_.x = input.pose.x` straight from odometry. Only heading/omega are
  eligible to blend at all.

So the field is pure encoder odometry wearing a label that claims a sensor
fusion. That label is how a **frozen OTOS gets mistaken for a fusion
disagreement** — which is exactly what happened on the bench.

Rename the user-facing label to **Pose**. Note the internal trace key is still
`"fused"` (~130 references across the GUI and its tests); the key rename is a
separate mechanical sweep.

## 2. Each trace names its source

| Row | Source |
|---|---|
| Camera / Truth | an actual overhead camera fix |
| Encoder | host-integrated from raw `enc` counts |
| OTOS | the OTOS sensor's own reported pose |
| Pose | telemetry's `pose` — the robot's own belief |

**Encoder stays host-integrated and independent of `pose`.** An earlier attempt
in this session pointed it at `frame.pose`, which collapsed it into a duplicate
of Pose. Keeping the two independent is the point: the gap between them is the
diagnostic (geometry mismatch, estimator divergence, a wheel slipping while
counts keep incrementing). That only carries information once the host uses the
same effective geometry the firmware does — see
[[testgui-host-dead-reckoner-used-raw-not-effective-trackwidth]].

## 3. A row exists only once its source has produced a point

One rule covers every case:

- **Camera** — no overhead camera on the bench or in Sim, so no truth exists
  there; hide the row rather than offering a checkbox for a line that can never
  be drawn.
- **OTOS** — hidden until the sensor reports. A permanently empty OTOS row and a
  live-but-frozen OTOS look identical in a legend, and only one of them is a
  sensor fault.
- **Encoder** — hidden until raw `enc` counts arrive.

A row that appears when its data does carries information; a row that is always
present carries none.

## Acceptance

- Sim and bench sessions show no Camera row.
- Disconnecting the OTOS removes the OTOS row rather than leaving a stale line.
- Label reads "Pose"; the trace it draws is byte-for-byte `frame.pose`.
