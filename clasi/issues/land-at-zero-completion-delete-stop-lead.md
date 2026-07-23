---
status: pending
filed: 2026-07-22
filed_by: team-lead (turn-execution review R2b/D3, claims verified against code)
related:
- stop-decision-must-see-this-cycles-odometry.md
- simple-velocity-control-acceleration-limited-shaper.md
- turn-error-characterization-postcompensation-tests-need-rewrite-after-lead-deletion.md
sprint: '119'
---

# Land at zero: complete on (remaining≈0 AND ω_cmd≈0); delete stop_lead_ms

## Description

Turn completion today is an open-loop time-lead guess: the stop fires when a
predicted heading (`stateEstimator_.bodyAt(nowMs + stopLead_)`,
`move_queue.cpp:232-233`) crosses the threshold, where `stopLead_` is a
hand-tuned scalar (currently 45 ms in all three robot JSONs) whose own
`_estimator_note` records three shipped retunes in one day (90→60→45), each
forced by an unrelated pipeline stage changing. It silently encodes ω=2 rad/s,
the 50 ms sim cycle, and the current shaper curve; hardware still shows a
4-8° residual with no better value in the bracket — because no single value
exists.

The decel taper already commands ω = √(2·α_decel·(remaining − jerkMargin))
(`velocity_shaper.cpp:101-109`), i.e. the robot is DESIGNED to arrive at the
goal at ~zero speed. Let it finish: declare completion when
`remaining ≤ ε AND |ω_cmd| ≤ ε_ω`, keep the StopCondition threshold (and
timeout) as the always-armed backstop, and DELETE `stop_lead_ms` + the
anticipation block rather than re-deriving it. There is then no tail to
predict.

## Design constraints (verified against code 2026-07-22)

1. **The predicate lives in `MoveQueue::tick()`**, not StopCondition:
   `Motion::StopCondition` is pure and dependency-free with no access to
   shaper state (`stop_condition.h:5-16`). No new StopCondition Kind. The
   threshold/timeout outcome remains the backstop path, always evaluated.
2. **Shaping-off keeps threshold semantics.** With all-zero ShaperLimits,
   `shapeAndStage()` early-returns (`move_queue.cpp:143`) — no taper exists
   and ω_cmd never bleeds, so a land-at-zero gate would never fire. The
   backstop is the completion path in that regime (exactly today's behavior).
3. **Scope: TWIST moves with Angle (ω axis) and Distance (v_x axis) stops
   only.** TIME stops have no spatial remaining (`move_queue.cpp:149-160`);
   WHEELS moves never taper the stop axis (`:111-137`). Both keep pure
   threshold/timeout semantics.
4. **ε_ω must clear the deadband floor.** Sub-deadband nonzero targets are
   boosted to ~15 mm/s per wheel (`nezha_motor.cpp:559-566`; ≈0.23 rad/s ≈
   13°/s equivalent on the ω axis at 128 mm trackwidth), so commanded ω never
   settles below that while nonzero. Set ε_ω just above the deadband-
   equivalent floor; on completion `Drive::stop()` stages exact zero, which
   bypasses the boost and engages the rest gate. Residual coast from the
   floor is bounded by τ·ω_floor ≈ 0.13 s · 13°/s ≈ 1.7° worst-case — budget
   it in the acceptance band.
5. **Delete list** once the gate is in: `stopLead_` member + ctor param
   (`move_queue.{h,cpp}`), the anticipation block (`move_queue.cpp:230-240`),
   `stop_lead_ms` from the estimator config patch arm / pydantic model /
   `estimator_kwargs` push / `data/robots/*.json` (+ their `_estimator_note`
   archaeology blocks), `gen_boot_config.py` bake. The StateEstimator's
   `bodyAt()` then has no firmware production consumer — QUARANTINE the
   estimator (keep module + update() + tests; it is the planned consumer for
   fake-OTOS/fusion bench work), do NOT delete it.
6. **Sequencing:** implement AFTER the loop reorder
   (`stop-decision-must-see-this-cycles-odometry.md`) so `remaining` is
   computed from this-cycle odometry.
7. `test_turn_error_characterization.py` disposition (existing issue) folds
   into this change — the lead-compensation premise it characterizes is gone.

## Acceptance

- Sim closure gate: TOUR_1 per-leg ≤ current shipped bands (deterministic
  ≤2.5°, GUI-path ≤5°) with `stop_lead_ms` DELETED — same or better than the
  tuned-lead baseline, or the change does not ship.
- Isolated 90° twist turn lands within ±2° sim deterministic.
- Distance stops (v_x axis) land within current bands.
- TIME/WHEELS moves byte-identical behavior (regression tests).
- No `stop_lead` string survives anywhere in src/ or data/ (grep gate).
