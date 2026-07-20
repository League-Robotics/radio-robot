---
status: in-progress
sprint: '114'
tickets:
- 114-005
- 114-006
---

# Deadband compensation: a small command must produce real motion, so terminal corrections can finish the move

## The defect (diagnosed 2026-07-20, reproduced on the stakeholder's own trace)

The motor write shaping has an output dead zone — `outputDeadband = 0.03` duty,
≈ **15 mm/s** of wheel speed (`sim_harness.h`'s own comment says "the PD stalls
below that"). Any command under that threshold is *zeroed*: the wheel gets
nothing.

Observed consequence on a real run: after the main motion, the robot sits a few
degrees short of the goal. The heading feedback commands a small correction —
about **11 mm/s** — to close the last bit. 11 is inside the 15 mm/s dead zone, so
the wheel cannot move, so the error never shrinks, so the command never changes.
It holds flat at ~11 mm/s **for ~8 seconds** with the wheels at zero, until an
arrive-timeout gives up. The stakeholder's wheel-speed plot shows exactly this:
a flat commanded ±11 with a dead-zero actual.

## Why it surfaced now

Sprint 112-004 deleted the minimum-speed floor and instead relied on
`heading_kp = 6` so that `kp × tolerance ≥ deadband` — i.e. any correction for an
above-tolerance error was automatically big enough to clear the dead zone.
Lowering `heading_kp` to 2.5 for the model-reference feedback broke that
invariant, putting the residual correction back inside the dead zone. The dead
zone was always there; the gain change re-exposed it.

## The fix (not a gain patch)

**Compensate the dead zone at its source**: the motor write should *boost* a
small nonzero command up to where the wheel actually turns, instead of zeroing
it. Then even an 11 mm/s correction produces the smallest real motion, the wheel
creeps the last degree onto the goal, and the move completes — **at any gain**,
so the model-reference keeps its clean, low, stable gain. Do not re-raise
`heading_kp` to paper over it, and do not reinstate a blanket minimum-speed
floor that also fires when no correction is wanted.

Mind the failure mode on the other side: because the motor cannot produce
velocities between zero and its minimum, naive compensation can hunt around the
target. The terminal behaviour must settle, not oscillate.

## Also required: re-validate the traces against the real config

Sprint 113 made the sim read `data/robots/*.json`. The motion traces were tuned
while the sim used a hardcoded `velGains.kp = 0.003`, but the config says
`vel_kp = 0.002`, so a configured sim now runs different motor dynamics than the
traces were validated against. Re-check the wheel-speed traces against the real
configured values before declaring the motion good.

## Acceptance (the stakeholder's own shape spec)

- No multi-second hold of an ineffective command; the terminal correction
  actually moves the wheel and the move completes promptly once it arrives.
- Straight and turn both land on the goal.
- The wheel-speed trace is a clean trapezoid: ramps up smoothly, holds max,
  ramps back to zero at the endpoint.
- No oscillations. No bumps at the end.
- A straight never goes below zero. On a turn, one wheel goes entirely below
  zero (the mirror) — that is expected and correct.
- Verified in the sim against the configured values, then on the bench.
