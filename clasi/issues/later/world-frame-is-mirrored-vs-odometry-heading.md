# World-frame GO_TO is unusable: odometry integrates heading MIRRORED

**Status:** open, blocks camera-free world-frame navigation
**Measured:** 2026-08-13, tovez on the playfield, camera as ground truth only

## The finding

The robot integrates heading with the OPPOSITE handedness to the world frame
that `SEED` and `GO_TO frame=0` are expressed in. Directly measured, seeding
the robot's pose straight from camera truth and then commanding one in-place
turn:

```
TURN    camera +52.3 deg   |   robot -50.1 deg     -> opposite sign
```

Magnitudes agree (52.3 vs 50.1, within the known ~2 deg terminal quantum);
only the SIGN disagrees.

## Why it matters

A real field has no overhead camera. The intended camera-free workflow is:
seed a known start pose once, then drive to WORLD coordinates. That path is
currently broken, and it fails in a way that looks like poor accuracy rather
than a frame bug:

| command | miss (camera truth) |
|---|---|
| ROBOT-frame `GO_TO` (frame=1), 5 legs 359-828 mm | **mean 17.6 mm, max 37.7 mm** |
| WORLD-frame `GO_TO` (frame=0) after `SEED`, 4 legs | **mean 469 mm, max 677 mm** |

Robot-frame works because it never touches the world frame -- the target is
resolved once, at acceptance, against the robot's current pose, so handedness
never enters. World-frame cannot be made to work by choosing a seed
convention, because the two choices each break a different half:

* seed MIRRORED (`x, -y, -yaw`): positions track truth (validated: a 200 mm
  leg agreed to **4 mm**) but the frame is left-handed, so every steering
  decision the Navigator makes is inverted;
* seed DIRECT (`x, y, yaw`): rotations track, positions do not.

## What is NOT the cause

* Not distance odometry: 300 mm legs measure 298-299 mm reported vs 299-300 mm
  camera -- **1-2 mm** error.
* Not gyro drift: standstill drift measures **-0.0098 deg/s**, worth ~2 mm of
  lateral error over a full-length 1208 mm run.
* Not straight-line tracking: 10 consecutive 300 mm legs held **0.75 deg mean**
  heading change, 0/10 excursions over 5 deg.
* Not the arrival tolerance (that was a separate, already-fixed bug -- see
  `2b789650`, 100 mm -> 10 mm).

## Related, and probably the same root

`data/robots/tovez.json`'s `navigator.yaw_sign = -1.0` is documented as
"commanded omega is opposite to world CCW (measured)". That is this same
handedness mismatch, patched at the omega level for the Navigator only. The
odometry integration and the SEED/world frame were never reconciled, so the
patch does not extend to pose.

## Suggested fix, in order of preference

1. Make `Motion::Odometry`'s heading integration and the SEED/world frame agree
   on handedness at the source, and then delete `yaw_sign` as the workaround it
   is. One convention, stated once.
2. Failing that, define the world frame explicitly (which way is +yaw) in
   `docs/protocol-v5.md` and make `SEED` document/convert, so a caller can seed
   correctly instead of discovering this by driving into a wall.

Do NOT "fix" this by flipping a sign at a call site -- there are at least three
places that already compensate independently (`yaw_sign`,
`planner.cpp`'s `applyOtosHeading(-state.otos.heading, ...)`, and the sim's
`kOtosHardwareMountSign`), which is how it stayed hidden this long.

## Reproduce

Seed from a known pose, command one in-place turn, compare the reported pose
heading delta against truth. Scratch script: `seeded_tour.py` (frame check +
turn check) from the 2026-08-13 session.
