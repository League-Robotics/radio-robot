---
status: pending
---

# EncoderDeadReckoner has no encoder-counter-reset guard — a mid-session firmware reboot corrupts the avatar/encoder trace

## Description

`host/robot_radio/testgui/traces.py`'s `EncoderDeadReckoner.update()` (097
WIP — the host-side dead-reckoning fallback that keeps the `encoder` trace
and canvas avatar moving on the binary plane) diffs the cumulative
`enc_left`/`enc_right` readings against `_prev_enc` with **no plausibility
guard** on the delta.

## Failure scenario

DAPLink's CDC serial port lives on the interface chip (KL27) and **survives
an nRF52 application-chip reset**. So if the robot reboots mid-session
(reset button, watchdog, crash), the TestGUI session stays connected and
telemetry resumes with `enc ≈ (0, 0)` while `_prev_enc` still holds large
cumulative values. The next `update()` integrates one giant negative delta
as backward motion — and, since the two wheels' totals differ, a large
bogus heading change — corrupting the encoder trace and avatar until the
operator manually clears traces.

## Prior art

The pre-rebuild `TraceModel` had exactly this guard, added after the same
class of bug (see memory note `encoder-odometry-drive-reset`): *"detect the
reset (both enc counts collapse far below baseline) and rebaseline WITHOUT
integrating, preserving accumulated heading/xy."*

## Suggested fix

In `EncoderDeadReckoner.update()`: when both wheel deltas are
large-and-negative (counter collapsed toward zero from a substantial
baseline), treat the reading as a counter reset — cache it as the new
`_prev_enc` baseline and return the pose unchanged instead of integrating.

This also future-proofs sprint 098's `ZERO enc` binary arm against any
reset path that doesn't go through `TraceModel.clear()` — the
`_set_origin()` path already resets the reckoner via
`_reset_baselines()`; **un-commanded** resets are the exposure.
