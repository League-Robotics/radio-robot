---
status: pending
filed: 2026-07-23
filed_by: team-lead (stakeholder session — encpose 349.1 vs pose 359.7 on a 360deg turn)
related: []
tickets: []
---

# encpose active-gate freezes the host dead-reckoner before motion ends (~10deg short per 360)

## Description

After a single managed 360° turn (v0.20260723.1 + ticket-005 fix), telemetry
shows `pose θ +359.7` / `otos θ −0.1` (both correct) but `encpose θ +349.1` —
10.6° short — with `enc L −401 R +401` (correct totals on screen).

Reproduced headlessly to the decimal: replaying the same run's frames through
`EncoderDeadReckoner(128)` gives **+359.4°** when fed every frame, and
**+349.1°** when fed only frames with `active=True` — which is exactly what the
GUI does: `TraceModel.feed()` returns immediately on `frame.active is False`
(the OOP "motion-state gate" added to stop idle trace growth), so the
dead-reckoner never ingests the tail of the motion — the taper end, the final
cycle between the last active frame and rest, and the plant coast — roughly
±11 mm of wheel travel here. Real motion, counted by firmware odometry and
OTOS/truth, invisible to encpose.

Consequences: encpose (telemetry row AND the orange encoder trace) runs
systematically short by the post-completion tail of EVERY move, accumulating
across a tour — this is display-side only; no control consumes it, but it
makes the encoder trace look like a sensor disagreement that doesn't exist.

## Fix

Gate only the *trace-point appending* on `active`, never the integrator: feed
`EncoderDeadReckoner.update()` (and `last_encpose`) on every frame carrying
`enc`, then apply the active/epsilon gates to `_append_if_moved()` as today.
Alternative with the same effect: keep integrating until both wheel velocities
read zero. Either way the idle-growth problem the gate was added for stays
solved (the reckoner is O(1) state; only the polylines grow).

## Repro

`docs/code_review/2026-07-22-turn-execution-review-scripts/` — `encpose_check.py`
(prints all-frames vs active-gated reckoner against firmware pose for one 360°).
