---
status: pending
---

# Rig profile: persist OTOS distrust instead of a per-session SET ritual

From the 2026-07-14 overnight rig report + code review Part 0 finding **B2**
(docs/code_review/2026-07-13-devices-drive-review.md).

## Problem

The bench rig's OTOS is servo-mounted and mechanically decoupled from the wheels, so
its pose is *structurally* invalid there — a per-robot property, not a runtime tuning
decision. On current firmware its fused pose poisons the drivetrain (2026-07-14: segments
were admitted/ACKed but never executed until `SET ekfROtosTheta=1e9 ekfROtosXy=1e9`
forced encoder-driven pose). That SET is runtime-only and resets on every reboot; the
stakeholder must remember to re-issue it each session, and forgetting it silently
reproduces the "robot ignores segments" failure.

## Fix

Give the rig's robot profile (data/robots/*.json → boot config) a persistent
"OTOS untrusted / encoder-only pose" switch so the distrust applies from boot with no
manual step. Under the single-loop rebuild this concern shifts (the robot stops fusing —
host fuses; the robot reports raw encoder odometry + raw OTOS in telemetry), but the
per-robot "this OTOS does not track the wheels" fact still belongs in the robot profile
either way (host-side fusion must know to ignore it on the rig too).

Verify: reboot the rig, drive a segment (or twist) with NO manual SET — motion executes
and reported pose tracks encoders.
