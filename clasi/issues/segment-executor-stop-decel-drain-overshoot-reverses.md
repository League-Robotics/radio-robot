---
status: resolved
---

# SegmentExecutor STOP-decel drain overshoots into reverse

## RESOLUTION (2026-07-11) — root cause was NOT the executor

Both manifestations (the drain reverse-dip below AND the pivot over-rotation
in the addendum) are **fixed**, and the fix exonerates
`Motion::SegmentExecutor`: per-pass instrumentation proved the executor's
emitted omega integral was EXACTLY the commanded angle (90.00°/180.00°) in
every failing scenario — the plan was never wrong. The real defects:

1. **TLM `cmd=` mislabel** (`source/telemetry/tlm_frame.cpp`): cmd= read
   `bb.drivetrain.vel()` (the MEASURED array per `Drivetrain::state()`'s own
   contract) instead of `cmd()`. Telemetry therefore showed command ==
   measured, hiding the tracking error and mis-directing this issue's whole
   original analysis toward the executor. Fixed to read `cmd()`.
2. **Sim plant velocity-gain miscalibration** (`tests/_infra/sim/sim_api.cpp`
   `defaultMotorConfigSet()`, duplicated in
   `tests/sim/unit/drivetrain_harness.cpp`): hand-typed `kff = 0.0038`
   against a plant whose exact feed-forward is `1/kNominalMaxSpeed = 0.0025`
   (`vel = duty * 400`) → every sim wheel ran ~1.25× its setpoint. That is
   the entire ~+20°/pivot over-rotation; translate legs were immune because
   STOP_DISTANCE truncates on measured encoders. Fixed:
   `kff = 1/kNominalMaxSpeed`.
3. **Phantom sim dead-time model** (`source/motion/segment_executor.cpp`
   HOST_BUILD `kOutputHops`): with tracking now exact, the modeled 40 ms lag
   described lag the sim plant doesn't have — the replan expectation read the
   plant as ahead (phantom-divergence retargets, −8.5°/90° pivot) and the
   dead-time-projected stop fired mid-decel, replacing the plan's S-tail with
   a steeper `solveToVelocity(0)` ramp (the DRAIN REVERSE-DIP this issue was
   filed for). Retuned 2.0 → 0.0 (sim only; the real brick's 4.0 untouched).

Post-fix: pivots land 90→88.6°, 180→179.9°, 360→359.9°; TOUR_1 closes to
≤15 mm/≤5°; the drain no-reverse and both tour-closure tests pass un-xfailed;
full `tests/sim` green (615 passed). The former xfail(strict) markers are
removed (`test_move_streaming_drain_no_reverse`,
`test_isolated_rotation_leg_reveals_independent_residual`,
`test_tour1_closes_the_loop`).

**Still open on hardware (tracked separately, needs a bench session):** the
REAL robot's turn accuracy with its own gains — the same command-vs-measured
signature is now finally observable over the wire because cmd= is honest;
the bench gate for the cmd= firmware change itself is part of this fix's
verification.

---

Original filing below (analysis superseded by the resolution above).

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

## Sharper root cause — the pivot manifestation (found during sim-tour closure work, 2026-07-11)

The same mechanism has a **much larger, tour-breaking manifestation on PIVOT**
(rotation), independent of the drain/BLEND drain above. It is the reason the
recorded TOUR_1 does NOT close in sim even with a physically-consistent plant.

Symptom (measured against `sim_get_true_pose_*` ground truth, plant trackwidth
128 == firmware kinematics 128, all sim calibrations at unity):

- An isolated `RT 9000` (commanded 90° in-place pivot) settles at **~110°** —
  a **~22% over-rotation** — not near 90°. Six of these compound around TOUR_1
  into a final pose error of **~199 mm / +119°** from the origin. Translation
  legs close well; rotation is the whole error budget.

Why it is NOT a trackwidth/kinematics bug (that was the first hypothesis, and
was separately fixed — plant `kDefaultTrackwidth` 150→128 to match firmware,
commit c4c883ae): with kinematics and plant now identical, a correct controller
would still stop at 90°. It stops at 110° because of the **`maybeReplanPivot`
divergence-replan cascade** feeding the terminal stop:

1. During the pivot, `maybeReplanPivot()` re-solves the Ruckig profile whenever
   measured heading diverges from the plan. Near completion these re-solves
   leave the rotational channel carrying **too much angular velocity too close
   to the target** — the profile keeps planning to "arrive at speed then stop"
   but the arrival keeps slipping.
2. `armPivotStopDecel()` then fires an **unconstrained `solveToVelocity(0)`**
   ramp from that high near-target velocity. With no distance bound it coasts
   well past the target heading before reaching zero → the 110° overshoot.

The naive fix — making the terminal stop **distance-bounded** (solve to hit the
target heading exactly) — was implemented and **reverted**: Ruckig, handed a
target it has already blown past / cannot reach forward within jerk limits,
**reverses** to hit it, which regresses the no-reverse-creep acceptance tests
(`test_move_straight/pure_in_place_turn_executes_and_settles_no_reverse_creep`,
−28…−39 mm/s reverse) and `test_pivot_completes_promptly_single_peaked`. The
reverse creep is the SAME defect as the drain manifestation above — a
target-bounded re-solve from a bad initial state overshoots into reverse.

The real fix is upstream of the terminal stop: **`maybeReplanPivot` must taper
angular velocity toward zero as the pivot approaches completion** (decel-
feasibility-aware replan — never plan to be carrying velocity that the terminal
stop cannot bleed off within the remaining angle at jerk limits), so the
terminal `solveToVelocity(0)` starts from a low, stoppable velocity and neither
overshoots forward nor needs to reverse. This is a delicate closed-loop control
retune touching the shared replan+stop cascade.

## Scope / risk

Fixing this touches stop-arming AND replan logic shared by every motion phase
(TRANSLATE / BLEND / PRE_PIVOT / TERMINAL_PIVOT) — a real regression surface (it
already burned one target-bounded-stop attempt into no-reverse-creep failures).
Needs its own investigation (taper the `maybeReplanPivot` cascade so terminal
velocity is always stoppable within remaining distance/angle), full
`tests/sim` reverification, and a **HARDWARE bench gate**
(`.claude/rules/hardware-bench-testing.md`) — the pivot overshoot is a real
22%-over-rotation on the floor, not a stand-harmless creep, so it must be seen
on the stand before it is called fixed. PRE-EXISTING (present before sprint 097;
097 and the sim-tour closure work only surfaced/quantified it) and orthogonal to
the protocol-v3 program. Blocks the two `xfail(strict)` markers in
`tests/sim/unit/test_tour_closure.py`
(`test_isolated_rotation_leg…`, `test_tour1_closes_the_loop`).

## Files

- `source/motion/segment_executor.cpp` — `armTranslateStopDecel`/`armPivotStopDecel`,
  the dead-time-projected fire in `tick()`.
- `source/motion/jerk_trajectory.{h,cpp}` — `solveToVelocity` seeding.
- Reproducer: `tests/sim/unit/test_bare_loop_move_and_tlm.py::test_move_streaming_chains_at_speed`
  (drain-no-reverse assertion). If deferred, that assertion is split into its own
  `xfail(strict=True)` test referencing this issue so the chaining regression check
  (which guards the UB fix) stays a live, passing test.
