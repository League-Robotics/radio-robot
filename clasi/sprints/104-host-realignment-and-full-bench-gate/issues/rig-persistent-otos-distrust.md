---
status: in-progress
sprint: '104'
tickets:
- 104-005
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

## Resolution status (104-005, 2026-07-15)

**Root failure mode is structurally gone, not merely worked around.** The original
symptom ("segments admitted/ACKed but never executed" until the manual `SET
ekfROtosTheta=1e9 ekfROtosXy=1e9` ritual) was caused by an on-robot EKF gating drive
on a poisoned fused pose. Under the single-loop firmware (sprint 103) the robot no
longer fuses pose on-robot at all — there is no EKF left to poison, so there is
nothing left for the OTOS-untrusted fact to gate on-robot. 103-010's own bench
session first-hand-confirmed this: the rig drove cleanly with no manual SET at all.
104-005 re-verified this holds on this sprint's tree too: rebooted the robot
(`pyocd commander ... reset`, no reflash — same firmware as 103/104), then
`tests/bench/twist_drive.py --port /dev/cu.usbmodem2121102 --v-x 150 --duration 1200`
with zero manual SET issued anywhere in the session — 6/6 checks passed, encoders
climbed `(0,0) -> (137,132)` in the commanded direction, ack ring confirmed both
`twist()` and `stop()`.

**What remains (forward-looking, sprint 106's scope, NOT this ticket's):** the
per-robot "this OTOS does not track the wheels on its current mount" fact is now
persisted in the robot profile — `geometry.otos_untrusted` (bool, default `false`)
in `host/robot_radio/config/robot_config.py`'s `RobotConfig.GeometryConfig`, set
`true` in both `data/robots/tovez.json` (the rig's actively-loaded profile since
093's active-pointer switch, and the profile `match_robot_by_id()` resolves to for
`device_announcement_name=tovez` by its own exact-robot-name-match preference) and
`data/robots/tovez_nocal.json` (same physical hardware, calibration-stripped
variant). It has no consumer yet — the single-loop firmware doesn't fuse, so
nothing reads it today — it exists so sprint 106's host-side fusion has an
authoritative, version-controlled source to ignore OTOS pose on this rig from day
one, instead of reinventing a per-session manual step. This issue stays OPEN until
106 adds that consumer; `completes_issue: false` on ticket 104-005.
