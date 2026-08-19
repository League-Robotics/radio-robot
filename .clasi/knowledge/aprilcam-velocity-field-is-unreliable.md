# aprilcam `get_tags` velocity/speed fields are unreliable — verify motion via encoders, not `vel_world`/`speed_world`

**2026-08-19.** The `mcp__aprilcam__get_tags` tool's `vel_world`/`speed_world`/
`heading_rad` fields do not reliably reflect real motion, and should not be
trusted as evidence a robot is moving (or stopped).

## What was observed

- AprilTag **1** — glued to a fixed field corner, physically incapable of
  motion — reported `speed_world: 0.4403088390827179` and the exact same
  `vel_world: [0.3124207556247711, 0.3102662265300751]` on **every single
  `get_tags` call all session**, regardless of camera frame or elapsed time.
  A stationary, immovable tag cannot have a real nonzero velocity; this
  value is either stale, cached, or a fixed artifact of some other
  computation entirely.
- The mobile robot tag (52) showed the identical pattern: `speed_world:
  1.1394932270050049` and the identical `vel_world` vector across two
  separate `get_tags` calls taken ~1s apart, while the tag's own
  `world_xy` moved by under 0.1cm between those same two calls (i.e. it was
  genuinely stationary) and the robot's own encoder positions (read
  directly from telemetry) were bit-for-bit unchanged across the same
  window.

This produced a false safety alarm: a `speed_world` of 1.14 m/s looked like
the robot was moving fast near a wall, when it had not moved at all —
confirmed by (a) two consecutive camera position reads agreeing to within
noise, and (b) encoder tick counts unchanged since the last confirmed-halted
telemetry check.

## What to do instead

**Never treat `vel_world`/`speed_world`/`heading_rad` (the ones paired with
those velocity fields) from `get_tags` as evidence of real motion.** To
check whether a robot is actually moving:

1. Read the robot's own telemetry (`enc_left`/`enc_right` `.velocity`, or
   compare `.position` across two reads a beat apart) — this is ground
   truth and was correct throughout this incident.
2. If camera-only confirmation is needed, take two `get_tags`/`capture_frame`
   reads a second or so apart and diff `world_xy` directly — don't read the
   velocity field, compute your own delta.

`orientation_yaw` (the tag's own per-frame corner-derived heading) was
separately found reliable and stable across reads in the same session — it
is only the *velocity-derived* fields that are suspect, not tag detection
in general. See also `.claude/rules/playfield-testing.md`'s heading
convention section, which already establishes `orientation_yaw` (with a
registered mobile-tag `yaw_deg` offset) as the trustworthy heading source —
this note extends the same distrust to the velocity fields specifically.
