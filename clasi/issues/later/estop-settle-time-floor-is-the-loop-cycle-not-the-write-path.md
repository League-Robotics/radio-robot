---
status: pending
priority: medium
---

# ESTOP settle time floors at ~0.19s — the loop cycle, not the write path

Measured during sprint 129 ticket 001 (2026-08-01, `tovez` on the stand,
new `src/tests/bench/estop_unlosable_bench.py`, 10 consecutive trials at
150 mm/s):

- **Wheels stop and stay stopped: 10/10, zero relapses.** The runaway
  defect (write-on-change permanently suppressing a stop) is fixed.
- **Settle time 0.176–0.203 s, mean ~0.192 s** — consistently ~30–50 ms
  over ticket 001's stated 0.15 s bound. Tight unimodal band, not the old
  bug's bimodal "sometimes never stops" signature.

## Where the residual comes from

Three contributions, none in `nezha_motor.{h,cpp}`/`drive.{h,cpp}` (ticket
001's scope):

1. `App::RobotLoop`'s 40 ms cycle — up to ~2 cycles of command-routing and
   blackboard-pickup latency before the stop reaches the motor write.
2. Genuine coast-down: the Nezha stop is COAST, not brake.
3. The bench measurement's own ~40 ms telemetry-frame quantization.

## What to decide

- Is 0.15 s the right bound at all, given a 40 ms scheduler? Either
  re-derive it from the loop period (e.g. "within 3 cycles") or shorten
  the path.
- Shortening options if the bound must hold: route ESTOP outside the
  normal cycle (immediate write on receipt), and/or use an active brake
  instead of coast where the motor controller supports it.
- Measurement: subtract or reduce telemetry quantization so the number
  reflects the robot, not the sampling.

## Acceptance

- A stated, justified settle-time bound derived from the loop schedule.
- Measured settle time meets it, re-measured with the quantization
  accounted for.
