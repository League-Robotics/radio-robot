---
status: pending
---

# aprilcam detection freezes silently and returns stale poses — the geofence trusts them

## Description

Observed twice on 2026-07-30 during sprint 127 playfield work.

**The detection loop can stop advancing while still answering queries.** Two
consecutive `get_tags()` calls returned **frame 505** with byte-identical pixel
coordinates and yaw — after the robot had demonstrably rotated 183 degrees (the
turn command completed, and a later query confirmed yaw had gone from −162.3 to
+21.0 degrees). No error, no timeout, no staleness flag. Just the same frame.

Separately, the **daemon died outright** twice: once with the control socket
removed entirely (`connect failed: No such file or directory`), taking the
geofence's gRPC channel down mid-run as an `_InactiveRpcError` rather than a
`GeofenceViolation`.

## Why this matters

`Geofence.check()` polls `dc.get_tags()` at ~10 Hz and is the only thing standing
between a misbehaving robot and the edge of the table. It fails closed on tag
*loss* — but a frozen detection loop is not tag loss. It is a confident,
plausible, **wrong** answer, and the fence will happily confirm a robot is parked
safely while it drives off the table.

This is not hypothetical: in the same session a `followPath` bug drove the robot
920 mm across the field, and the fence stopping it was the only thing that
prevented a fall. Had detection been frozen at that moment, it would not have.

## Proposed fix

Two layers, and the second is ours regardless of what aprilcam does.

**1. aprilcam** — a detection loop that has stopped should say so. Either surface
the frame's age/timestamp so callers can judge it, or return an explicit error
once the loop stalls, rather than serving the last good frame indefinitely.

**2. `robot_radio.field.Geofence`** — do not trust the daemon to be honest.
`get_tags()` already returns a monotonically increasing `frame` counter; treat a
**non-advancing frame counter** as equivalent to tag loss and fail closed on it,
using the existing `lost_grace` window. Also catch the gRPC transport errors
(`_InactiveRpcError`) and convert them into a `GeofenceViolation` so a dead daemon
halts the robot through the normal path instead of an unhandled exception that
happens to unwind through someone's `finally`.

## Verification

- Kill the aprilcam daemon mid-run: the geofence must estop and raise
  `GeofenceViolation`, not an unhandled gRPC error.
- Freeze detection (stop the loop while leaving the daemon up): the geofence must
  detect the stalled frame counter within `lost_grace` and halt.

## Related

- `.claude/rules/playfield-testing.md` — "Fails CLOSED on losing the tag: if the
  robot cannot be seen, it is not known to be safe." A stale frame is a robot
  that cannot be seen, dressed as one that can.
- `src/host/robot_radio/field/geofence.py` — `check()` and its `lost_grace`
  handling.
