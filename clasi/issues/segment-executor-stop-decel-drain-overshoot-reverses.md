---
status: pending
---

# SegmentExecutor STOP-decel drain overshoots into reverse

## Context

Surfaced during sprint 097 close, immediately after the never-solved-Ruckig-channel
UB fix landed (`fix(097): SegmentExecutor samples never-solved Ruckig trajectory`).
The UB used to corrupt `test_move_streaming_chains_at_speed` before it ever reached
this code path, masking this second, independent bug. With the UB fixed, the test's
**second** assertion now fails deterministically:

`tests/sim/unit/test_bare_loop_move_and_tlm.py::test_move_streaming_chains_at_speed`
— "Draining the stream must still end in the graceful decel (settled, no reverse)":
`(vel_l+vel_r)/2` dips below the test's −8 mm/s tolerance during the drain.

## Evidence (from the 097 debugging)

- The **commanded** setpoint (`TLM cmd_vel`, the executor's own plan, not the
  PID-lagged measurement) itself dips to **−16.85 mm/s** during the BLEND-phase
  drain of a streamed micro-MOVE chain — i.e. the executor deliberately commands a
  brief reverse at motion end.
- A plain non-streamed `TRANSLATE`-phase move at a comparable peak (~397 mm/s,
  natural exhaustion, no explicit STOP) also dips — to −7.18 mm/s measured /
  −10.25 mm/s commanded (right at the −8 threshold). So the overshoot is NOT
  stream-specific; it is worse for BLEND than TRANSLATE by an unquantified factor.
- A comparable passing test (`test_stop_over_wire_mid_move...`) uses an explicit
  `STOP` at higher speed (~800–1000 mm/s) with a tighter −5.0 threshold and stays
  clean — so the trigger is specifically the **dead-time-projected natural-exhaustion
  early-fire** stop path, not explicit STOP.

## Suspected root cause

The STOP_DISTANCE dead-time-projected early-fire re-arms the stop decel
(`armTranslateStopDecel`/`armPivotStopDecel` → a `solveToVelocity(0, …)` re-solved
mid-decel) seeded with the residual NEGATIVE acceleration from the in-flight plan.
A jerk-limited re-solve from a negative-acceleration initial state can overshoot the
zero-velocity target into reverse before settling. It scales with peak speed.

## Scope / risk

Fixing this touches stop-arming logic shared by every motion phase
(TRANSLATE / BLEND / PRE_PIVOT / TERMINAL_PIVOT) — a real regression surface. Needs
its own investigation (how a mid-decel re-arm is seeded / clamped), sim coverage,
and a HARDWARE bench gate (`.claude/rules/hardware-bench-testing.md`) — on the floor
it manifests as a small brief backward creep at the end of a decelerating move; on
the stand it is harmless. It is PRE-EXISTING (present before sprint 097; 097 only
unmasked it) and orthogonal to the protocol-v3 program.

## Files

- `source/motion/segment_executor.cpp` — `armTranslateStopDecel`/`armPivotStopDecel`,
  the dead-time-projected fire in `tick()`.
- `source/motion/jerk_trajectory.{h,cpp}` — `solveToVelocity` seeding.
- Reproducer: `tests/sim/unit/test_bare_loop_move_and_tlm.py::test_move_streaming_chains_at_speed`
  (drain-no-reverse assertion). If deferred, that assertion is split into its own
  `xfail(strict=True)` test referencing this issue so the chaining regression check
  (which guards the UB fix) stays a live, passing test.
