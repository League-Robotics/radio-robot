---
status: done
filed: 2026-07-22
filed_by: team-lead (turn-execution review R4/D4, claims verified against code)
related:
- restore-the-interleaved-request-settle-tick-loop-schedule.md
- tlm-rate-15-19hz-vs-50hz-nominal-serial.md
sprint: '118'
tickets:
- 118-003
---

# Sim control period must equal the firmware period (one robot, not two)

## Description

`SimHarness::kCycleDtUs = 50000` (`sim_harness.h:472`) vs firmware
`kCycle = 20` ms (`robot_loop.cpp:27`): every sim-tuned millisecond constant
and every "N cycles of latency" result is measured on a plant with 2.5× the
shipped control period. The sim is deterministic about a DIFFERENT robot.

Verified 2026-07-22: the 50 ms was chosen to dodge `NezhaMotor`'s duty
write-rate throttle — `kMinWriteIntervalUs = 40000` (25 Hz max,
`nezha_motor.cpp:622`, enforced `:625`) — per the derivation comment at
`sim_harness.h:468-471`. At a 20 ms cycle roughly every other duty write
would be silently dropped. Note this throttle bites HARDWARE too: the 20 ms
loop's duty writes are already being dropped every other cycle on the robot.

## Resolution (rides the interleave restore)

`restore-the-interleaved-request-settle-tick-loop-schedule.md` sets
`kCycle = 40` ms (stakeholder-confirmed). Then:

1. Set `kCycleDtUs = 40000` so sim period == firmware period == actuation
   period (the 40 ms throttle no longer forces a mismatch). The invariant to
   enforce (static_assert / test) is **sim step == kCycle**, not any
   particular number.
2. **Throttle margin hazard:** at cycle == throttle exactly, hardware timing
   jitter (a 39.x ms cycle) makes the `<` comparison at `nezha_motor.cpp:625`
   drop that write. Give the throttle margin (e.g. `kMinWriteIntervalUs =
   kCycle·1000 − 5000` or a cycle-aware guard) so an on-schedule write never
   loses to jitter. Sim (exact virtual steps) won't catch this — reason it
   out in code review and verify on the bench (fault/skip counter or encoder
   smoothness).
3. Fix the independent hardcoded copies of the 50 ms assumption (verified
   list): `app_robot_loop_harness.cpp:870` (own kCycleDtUs=50000) and `:1335`
   (`plant.tick(0.05f)`); `turn_prediction_capture.py:89` (`_CYCLE_S=0.05`);
   `test_tour_closure_gate.py:245` (`clock.now_s += 0.05`);
   `sim_loop.py:149` (`_CYCLE_DURATION_S = 0.050`). Prefer deriving from one
   exported constant (ctypes export or generated) so this class of drift
   can't recur.
4. Re-baseline cadence-sensitive gates (closure gate, button acceptance,
   estimator tracking) at 40 ms; `kPrimaryPeriod 20→40` comes with the
   interleave issue. The `tlm-rate-15-19hz-vs-50hz-nominal-serial.md` issue's
   nominal becomes 25 Hz — re-measure on the bench and update or resolve it.

## Acceptance

- Sim step and firmware kCycle come from/assert against the same constant.
- Full sim suite + closure gates green at 40 ms with bands re-baselined
  (deterministic per-leg band unchanged or tightened, never widened without
  stakeholder sign-off).
- No surviving hardcoded 0.05/50 ms cycle assumption (grep gate).
- Bench (phase B): measured TLM period ≈40 ms; no duty-write drops while
  driving (throttle margin verified).
