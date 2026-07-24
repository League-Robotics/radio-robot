---
status: pending
filed: 2026-07-23
filed_by: programmer agent (sprint 120 ticket 001 hardware verification session)
related:
- bench-single-ack-slot-observability-collapses-at-40ms.md
- tlm-rate-15-19hz-vs-50hz-nominal-serial.md
sprint: '125'
---

# Bench move commands intermittently never reach the firmware (dropped/corrupted CommandEnvelope, not an ack-observability issue)

## Observed

Robot UID `9906360200052820a8fdb5e413abb276000000006e052820` ("tovez"),
port `/dev/cu.usbmodem2121102`, 2026-07-23 bench session (sprint 120
ticket 001, ack-ring hardware verification).

Running `src/tests/bench/move_protocol_bench.py` repeatedly (both against
the NEW 120 ack-ring firmware/host code, and via a side-by-side A/B check
against the UNMODIFIED pre-120 firmware+host code at commit `047555a5`)
shows an intermittent failure mode **distinct** from the ack-ring issue
120-001 fixes: some `move_twist()`/`move_wheels()`/`stop()`/`config()`
calls get **no ack at all** (`wait_for_ack()` times out completely after
500ms) **and** the commanded motion never happens at all — encoder
position is bit-for-bit identical before and after the call (e.g.
`before_theta_cdeg=-1229 after_theta_cdeg=-1229 dtheta=0.000rad`,
`d_left=0 d_right=0`).

This is qualitatively different from "ack lost, move executed" (the
single-slot bug 120-001 fixes) — here the ENVELOPE itself appears to
never reach/decode on the firmware, matching `docs/protocol-v4.md` sec
7.4's "a malformed/undecodable frame gets no reply at all" behavior.

## Root-cause isolation performed

Ran `move_protocol_bench.py` five times against the CURRENT (post-120,
ack-ring) build in one session (occasional fresh `mbdeploy` reflashes in
between): **38/43, 34/43, 33/43, 30/43, 35/43**.

Then checked out commit `047555a5` (the last commit before any 120-001
work started) into a separate `git worktree`, built ONLY that firmware,
reflashed the SAME robot, and ran that SAME pre-120
`move_protocol_bench.py` (single ack slot, no ring) against it: **38/43**
— the SAME failure pattern (ack=None + zero encoder movement, landing on
`scenario_angle_stop`/`scenario_wheels_variant_signs` specifically that
run).

This proves the dropped-envelope symptom is **pre-existing** and **not**
caused by the 120 ack-ring change — the pre-120 code doesn't touch
`CommandEnvelope` decode at all, and shows the identical symptom at a
similar rate.

## The ack ring itself is proven solid on hardware

- A dedicated rapid-fire N=5 back-to-back `move_twist()` enqueue test
  (`src/tests/bench/ack_ring_rapid_fire_bench.py`) passed all 5/5
  ack-observability checks on 3 separate runs (15/15 total acks
  observed).
- `twist_drive.py`'s previously-always-missed `stop()` ack landed cleanly
  in 2 of 3 runs (the 1 miss showed the SAME zero-movement
  dropped-envelope signature described above, not a ring-depth issue).

So the ack ring correctly solves the problem it was built for. This is a
**separate**, pre-existing bench-link reliability gap: some fraction of
outbound `CommandEnvelope` writes over the direct USB serial link are
silently lost or corrupted before ever reaching
`RobotLoop::processMessage()`, upstream of the ack ring or `Telemetry`
entirely.

## Possibly related (not confirmed)

`tlm-rate-15-19hz-vs-50hz-nominal-serial.md` (host reads ~15 frames/s
against a 25Hz emit rate) — if that is a genuine serial-throughput/
backpressure issue on this same link, it could plausibly also explain
occasional dropped inbound envelopes, though that issue is about the
INBOUND telemetry read rate, not outbound command loss — the connection
is speculative, not established.

## Suggested next steps

1. Confirm with `on_send`/`on_recv` verbose logging (`SerialConnection`'s
   own callback hooks) whether the envelope bytes are actually written to
   the OS serial port on a failing call (rule out a host-side write bug)
   vs. genuinely never arriving/decoding on the firmware side.
2. Check `Comms::malformedCount()`/`kFlagFaultCommsMalformed` on frames
   following a suspected drop — if `malformedCount()` is NOT
   incrementing, the bytes never arrived at all (a link/transport
   problem, not a decode problem).
3. Try a different USB cable/port for the same robot to rule out a
   marginal physical connection.
4. Check whether this correlates with recent motor activity/EMI (motor
   back-EMF coupling into the USB link is a common failure mode for
   exactly this symptom shape).
5. **Forward note for sprint 120 ticket 002 (bench tour bring-up):**
   the 120-001 ack ring makes a dropped enqueue OBSERVABLE (a missing ack
   within the expected window), where before it was silently
   indistinguishable from a merely-unobserved ack. Ticket 002's own bench
   tour runner does not need to wait on a deep serial-RX fix here — it
   can retry-on-missing-ack (re-send the same leg's `Move` if
   `wait_for_ack()` times out with no completion or enqueue ack observed)
   to get a tour to close reliably over this lossy link in the meantime.

Not part of sprint 120 ticket 001's scope (which is specifically the ack
ring, proven fixed by the evidence above) — filed as its own issue.
