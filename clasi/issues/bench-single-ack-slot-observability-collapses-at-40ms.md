---
status: pending
filed: 2026-07-23
filed_by: team-lead (phase-B bench session, v0.20260723.2 on the stand)
related:
- tlm-rate-15-19hz-vs-50hz-nominal-serial.md
---

# Single telemetry ack slot loses transient acks at the 40ms cycle (bench)

## Observed (real hardware, v0.20260723.2, robot on the stand)

`move_protocol_bench.py`: **31/43 checks pass**. Every FAIL is a missed
transient ack (`ack=None`); every functional behavior PASSES — moves execute,
ALL completion acks fire (`move_id` observed in every scenario), timeout-fault
(flags bit 15), empty-queue drain, preempt/replace-flush, STOP-flush-pending
all correct. `twist_drive.py`: 5/6 (only the `stop()` ack missed).

The misses cluster on (a) the FIRST command in a scenario and (b) rapid-fire
sends (ERR_FULL's 3rd/4th/5th enqueue, all missed). A team-lead turn-accuracy
capture lost 3/6 enqueue acks and each miss cascaded into a garbage heading
reading — i.e. this actively corrupts bench measurement, not just the gate.

## Cause (three things compounding)

1. 119-005 moved the enqueue/CONFIG/STOP ack to ride the SAME cycle as
   dispatch (it now rides the frame emitted at the top of the pace block).
2. The 40ms cycle (118) halved the ack-slot emission rate vs the old ~20ms
   nominal.
3. The telemetry frame carries a SINGLE ack slot (`ack_corr`/`ack_err`, one
   per frame). A second command's ack overwrites the first before the host
   polls, and the host only reads ~15 frames/s (see related TLM-rate issue),
   so a transient single-slot ack has ~15 chances/s to be caught and rapid
   sends collide.

The completion acks survive because they're polled over the whole Move
duration; only the instantaneous enqueue/STOP/CONFIG acks are lost.

## Fix directions (pick in the sprint)

- **Small ack FIFO in the frame** (preferred): carry the last N (e.g. 4) acks
  per frame instead of one slot, so a host reading at 15Hz still drains every
  ack. Append-only wire change per docs/protocol-v4.md; bump the frame's ack
  section, document it.
- OR a host-side guarantee that every frame is read (couples to the TLM-rate
  issue — the host is dropping >40% of frames).
- Either way, add a bench-gate assertion that N rapid enqueues all surface
  their acks.

## Acceptance

- `move_protocol_bench.py` 43/43 on hardware.
- A rapid-fire N-enqueue test surfaces all N acks.
- Bench measurement harnesses (turn/accuracy) no longer lose acks.
