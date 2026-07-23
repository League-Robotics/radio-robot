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

## Addendum (2026-07-23, sprint 118 ticket 002 — fresh sweep data, NO retune applied)

`stop-decision-must-see-this-cycles-odometry.md` landed (`moveQueue_.tick()`
relocated into the pace block, after `odom_.integrate()`/
`stateEstimator_.update()` — see that issue/ticket for the mechanism). This
removes the odometry staleness this issue's own Description names as one of
`stop_lead_ms`'s two entangled jobs, but does NOT fix the closure gate at the
current 45.0 ms default — it *shifts* the failure, reinforcing this issue's
own "no single value exists" finding with fresh, post-fix numbers:

**Before (40ms cycle, odometry stale, `stop_lead_ms=45`)**: closure gate
failed on TOUR_2 (turns 6/12/14 missing by +4.84/+3.77/+4.51deg); TOUR_1 and
TOUR_2/realistic passed. `test_managed_angle_preset`/`test_managed_seg_0_cdeg_turn`
(isolated ±90/180/270/360deg turns) all passed cleanly (±90deg cases measured
88-89.3deg, i.e. slight UNDERSHOOT). `test_tour_2_runs_to_completion` (own
5deg tolerance) already failed independently in this same run (leg 6 by
+6.06deg) — this one is NOT attributable to the odometry fix; it was already
at/past its own noise floor beforehand (ticket 001's own report called it
"borderline-flaky").

**After (40ms cycle, odometry fresh, `stop_lead_ms=45` UNCHANGED)**: the
SAME test now fails on TOUR_1 instead (turns 8/12 at +4.20/+4.39deg) while
TOUR_2/ideal, which used to be the worst offender, now passes (worst
2.42deg) — a genuine, direct improvement on the exact defect this issue's
own Description names ("no single value exists" playing out again, just
relocated). More significantly, `test_managed_angle_preset[±90]` and
`test_managed_seg_0_cdeg_turn[±90]` — previously clean passes — now FAIL,
both measuring ~+3.7 to +3.8deg OVERSHOOT against a tightened ±3.0deg band
for the 90deg case (`_managed_angle_band()`). This is a genuinely new,
broader-blast-radius regression than the closure gate alone: fixing the
staleness flips isolated 90deg turns from slight undershoot to overshoot
that is now just outside their own tolerance, at the unchanged 45ms lead.

**Sweep (0-120ms, `_make_loop(stop_lead_ms=...)` against TOUR_1+TOUR_2 x
ideal+realistic, the closure gate's own exact path) — no retune applied,
data only**: worst |error| decreases from 9.31deg (lead=0) down through
45ms (4.43deg, still failing) to a narrow window at 62.0-62.5ms
(worst=2.375deg — the ONLY sampled values under the 2.5deg shaped-band
tolerance), then rises again past 63ms (3.48deg) and keeps climbing past
70-120ms (4.3 to 8.98deg, the sign of most turns flips from overshoot to
undershoot somewhere around 65-70ms). Full-mm-resolution neighbors of the
one passing window: 60.5/61.0/61.5ms all measure 3.0-3.1deg (fail);
63.0/63.5ms measure 3.48deg (fail) — i.e. the ONLY passing region found is
about 1ms wide with ~0.13deg (5%) of headroom, not the broad flat plateau
this issue's own earlier 45.0ms pick was chosen from ("a BROAD, flat
plateau... spans lead=30-54ms" — see `test_tour_closure_gate.py`'s own
`_STOP_LEAD_MS` comment). This does NOT meet the bar for a responsible
re-baseline (this would be shipped retune #4 in as many weeks per this
issue's own Description, on an even less stable point than #3), and the
isolated-90deg-turn regression above was never even in the swept metric —
no attempt was made to find a value clearing BOTH gates simultaneously,
since the closure-gate-only search already had no safe margin.

**Disposition**: `stop_lead_ms` left UNCHANGED at 45.0ms (source + all
three robot JSONs) — no silent retune, per this sprint's own mandate
(`sprint.md` Out of Scope: "stop_lead_ms may be re-baselined... ONLY if
the closure gate demands it, must be data-derived (sweep)... recorded...
without deleting the field"). Given the search above found no value with
real margin across even the closure gate alone, retuning would trade one
fragile point for another, not fix the underlying problem. This
strengthens, not weakens, the case for THIS issue's own fix (delete
`stop_lead_ms`, land on `remaining≈0 AND ω_cmd≈0` instead of a tuned
guess) — the fresh-odometry data point above is a fourth data point (after
90/60/45) on the same "no stable single value" curve, now measured against
the CORRECTED (fresh, not stale) odometry basis this issue's own
Description already assumed as ideal.

Full sweep table and the closure-gate/button-acceptance before/after
transcripts are recorded in
`clasi/sprints/118-loop-schedule-truth-firmware-loop-reorder-sim-cadence-parity/tickets/002-stop-decision-consumes-this-cycle-odometry-relocate-movequeue-tick-into-the-pace-block.md`.
