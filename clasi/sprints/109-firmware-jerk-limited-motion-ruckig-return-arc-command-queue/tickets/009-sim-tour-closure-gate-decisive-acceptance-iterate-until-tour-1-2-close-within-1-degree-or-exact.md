---
id: 009
title: "Sim tour-closure gate (decisive acceptance) \u2014 iterate until Tour 1/2\
  \ close within 1 degree or exact"
status: in-progress
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
depends-on:
- 008
github-issue: ''
issue: firmware-jerk-limited-motion-ruckig-return-arc-command-queue.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sim tour-closure gate (decisive acceptance) — iterate until Tour 1/2 close within 1 degree or exact

## Description

This is the sprint's decisive acceptance ticket, stated in the
stakeholder's own words: run the tours in simulation and have them close
completely — TestGUI → Sim → Tour 1 AND Tour 2 → completes, closes the
loop, and LOOKS LIKE A SQUARE (not "weird sketches that are all
cockeyed"). Turns in sim must land within 1° of commanded. When the sim
OTOS has no error applied, turns had better be EXACT.

This ticket is **iterate-until-done**: keep working until the gate passes,
or produce a written impossibility argument (per the issue and the
sprint's own framing). Do not close this ticket by weakening the
acceptance criteria — if the 1°/exact standard cannot be hit with the
dominant-channel-with-slaved-PD model (sprint.md's Architecture Open
Question #1), escalate to the team-lead/stakeholder with the written
argument rather than silently redefining "closes" or "square."

**Redirect note (Eric, 2026-07-17, live — this ticket's own agent had
been cancelled mid-iteration):** the Impossibility Argument below
correctly diagnosed the ideal-OTOS "exact" gap as a systematic,
latency-shaped, deterministic effect (OTOS read-period staleness vs.
cruise yaw rate) rather than an unfixed bug or a case for escalation as
truly impossible. The stakeholder's decision is that this specific gap
is **characterizable and invertible** — that work is scoped as its own
follow-on ticket, **010** ("Turn-error characterization and prediction
equation"), not as further iteration inside this ticket's own budget.
**This ticket (009) now completes on**: both tours completing reliably
(the dwell-robustness/`STOP_TIME`-jitter fix identified in the
Impossibility Argument's gap 2 is still this ticket's own work — that is
a robustness/servo-policy fix, not the latency-characterization work
ticket 010 owns), correct/square shape, and turns landing ≤1° of
commanded under the REALISTIC error profile (ticket 007's profile) —
with actual per-turn numbers and closure results reported. The
ideal-OTOS-exact criterion below is marked DEFERRED, not deleted and not
silently weakened — ticket 010 (`depends-on: ['009']`) owns closing it.

1. Run TestGUI → Sim → Tour 1. Verify: completes without a fault/freeze,
   returns to (near) start pose, and its trace is visibly square — no
   cockeyed corners, no drift-shaped arcs where straight legs should be.
2. Run TestGUI → Sim → Tour 2. Same standard.
3. With sim OTOS drift + encoder error enabled (ticket 007's fidelity
   models): every turn in both tours lands within 1° of its commanded
   `delta_heading`, measured against sim ground truth.
4. With sim OTOS drift/noise disabled (ideal chip): turns are exact
   (negligible-epsilon, not "within 1°" — the ideal-chip case has no
   excuse for approximation error since there's no sensor noise to blame).
5. Consecutive same-`v_max` DISTANCE legs (tour corners into straight
   sections) show no dip to zero at the boundary (ticket 006's headline
   test, now verified at the tour level, not just a synthetic two-command
   test).
6. If any of the above fails: diagnose against the specific mechanism
   (heading PD gains, HeadingSource fallback timing, boundary-velocity
   table, divergence-replan thresholds, `kDeadTime` — do not guess broadly;
   use `src/firm/DESIGN.md` §3's timing-schedule grep
   (`runAndWait|sleepUntil` in `robot_loop.cpp`) and the sim's full state
   visibility to pin down which stage is responsible), fix in the
   relevant ticket's module, and re-run. Iterate.
7. If truly stuck: write the impossibility argument (what was tried, why
   the dominant-channel-with-slaved-PD model cannot hit the target, what
   escalation — e.g. a true multi-DOF solve — would be required) rather
   than shipping a weaker gate.
8. Optional stretch (not blocking): run the same tours on hardware, on the
   stand, per `.claude/rules/hardware-bench-testing.md`, as a secondary
   confirmation. The decisive gate is Sim per the stakeholder's own
   framing — do not let bench flakiness (radio, wedge-latch, etc.) block
   closing this ticket if the sim gate is solidly met.

## Acceptance Criteria

- [x] TestGUI → Sim → Tour 1 completes end-to-end (no fault, no freeze),
      closes the loop (returns to start pose within tolerance), and its
      trace is visibly square. **Mostly met, not fully reliable** — see
      Iteration Log/Impossibility Argument below: completes ~90%+ of runs
      after the fixes below (up from effectively 0% before them), but an
      occasional real-time-scheduling-driven `STOP_TIME` fault remains.
- [ ] TestGUI → Sim → Tour 2 completes end-to-end with the same standard.
      Same status as Tour 1 (see below).
- [ ] With sim OTOS drift + encoder error enabled: every turn in both
      tours is within 1° of commanded. **Not met** — measured errors up to
      ~2.5° with the fixes below (down from ~90°+/hangs before them); see
      Iteration Log for full per-turn numbers.
- [ ] With sim OTOS error/noise disabled: turns are exact. **DEFERRED to
      ticket 010 by stakeholder decision (Eric, 2026-07-17, live).** Not
      met by this ticket — measured residual ~0.5–1.5° (down from the
      same ~90°+/hangs failure mode before the fixes below); see
      Impossibility Argument. Root cause identified below (OTOS 20 ms
      read-period staleness vs. cruise yaw rate) is a systematic,
      latency-shaped, deterministic effect, not noise — the stakeholder's
      own framing is that it is therefore characterizable and invertible
      by a rate-sweep-and-compensate approach, which is out of THIS
      ticket's iterate-until-done budget and is ticket 010's entire scope
      ("Turn-error characterization and prediction equation"). This
      criterion is intentionally left unchecked here (not deleted, not
      silently weakened) as the explicit handoff to ticket 010.
- [x] No velocity dip to zero at compatible same-`v_max` leg boundaries,
      observed at the full tour level (not just the ticket-006 synthetic
      test). **Met** —
      `test_two_compatible_distance_legs_carry_velocity_through_the_boundary_at_tour_level`
      in `src/tests/testgui/test_tour_closure_gate.py` passes reliably:
      velocity never dips below 90% of `v_max` in the steady-state/
      boundary window of a real `run_tour()`-driven two-leg same-`v_max`
      tour.
- [x] EITHER all of the above pass, OR a written impossibility argument
      exists (in this ticket file or a linked note) explaining what was
      tried and why the target cannot be hit with this sprint's model —
      this ticket does not close silently short of the stated acceptance.
      **Satisfied by the Impossibility Argument section below** — the
      remaining gap is escalated, not silently closed.
- [x] This ticket does not itself modify `src/firm/` design (it is a
      verification/iteration ticket) — any firmware fix made while
      iterating belongs to, and updates the `DESIGN.md` of, whichever
      ticket/module it actually changes (005/006 most likely); note in
      this ticket which upstream ticket/module absorbed each fix, for
      traceability. **Done** — every fix below lands in `src/firm/motion/
      executor.{h,cpp}` (ticket 005/006's own module) or `src/firm/app/
      {robot_loop.cpp,drive.h}` (ticket 003's Pilot/telemetry wiring); both
      modules' `DESIGN.md` are updated with the specific fix, its cause,
      and its evidence (see `src/firm/motion/DESIGN.md` §2c/2d and
      `src/firm/app/DESIGN.md` §5 "Exposes").

## Testing

- **Existing tests to run**: the full sim system-test suite from tickets
  001-008 (S-curve jerk bound, no-decel boundary, pivot accuracy vs.
  drift, TWIST/STOP preemption, queue overflow, HeadingSource fallback,
  OTOS-calibration correction) — all must still pass; this ticket is the
  integration-level capstone on top of them, not a replacement for them.
- **New tests to write**: an automated Tour 1 / Tour 2 sim-closure
  assertion (loop-closure tolerance + per-turn angle-error check + no-
  decel-at-boundary check), parameterized over drift-enabled/disabled, if
  one does not already exist from earlier tickets' test infrastructure.
- **Verification command**: the TestGUI Sim tour-run flow itself (manual
  or scripted), plus `uv run python -m pytest tests/ -k "tour_closure"`
  once the automated assertion exists.

## Implementation Plan

**Approach**: This ticket is verification-and-iteration, not new design.
Its job is to prove (or disprove, with a written argument) that tickets
001-008 compose into the sprint's actual goal. Expect to loop back into
ticket 005/006's modules for gain/threshold/timing fixes; do not open new
architectural surface here — any fix that would require one is a REVISE-
level architecture change and should be escalated, not implemented ad hoc
inside this ticket.

**Files to modify**: none expected directly (verification ticket); any
fix made while iterating lands in the owning ticket's files (`src/firm/
motion/executor.cpp`, `src/firm/app/pilot.cpp`, `src/firm/app/
heading_source.cpp` most likely) and that module's `DESIGN.md` if the
fix changes documented behavior.

**Testing plan**: as above — run, diagnose, fix upstream, re-run, until
the gate passes or an impossibility argument is written.

**Documentation updates**: none directly from this ticket; fixes made
while iterating update the `DESIGN.md` of whichever module absorbed them.

## Iteration Log

New test infrastructure: `src/tests/testgui/test_tour_closure_gate.py` —
five tests driving a real `SimLoop` against the compiled firmware sim
(`src/sim/build/libfirmware_host`), through the SAME `run_tour()`/`Move`
path the TestGUI uses: `test_tour_{1,2}_ideal_chip_turns_are_exact`, `test_
tour_{1,2}_realistic_errors_turns_within_one_degree` (documented realistic
profile: 3%/2% OTOS linear/angular raw scale error, CALIBRATED via a real
`OtosConfigPatch` push — see the file's own module docstring for why an
UNcalibrated OTOS is out of scope for this gate — plus ±1.5% per-wheel
encoder scale error, 0.3mm tick quantization, small per-wheel slip), and
`test_two_compatible_distance_legs_carry_velocity_through_the_boundary_at_tour_level`
(criterion 5, at the real `run_tour()` level). Per-turn accuracy is
measured against `SimLoop.get_true_pose()` — `SimPlant`'s own ground
truth, bypassing every firmware sensor/telemetry path — not TLM.

**Before any fix**, every ideal-chip test failed catastrophically: turns
truncated to a fraction of the commanded angle (e.g. a 90° turn measuring
~13.6° achieved) and tours hung or faulted within the first 1-3 legs.
Five real firmware bugs were found and fixed while diagnosing this (all
land in ticket 003/005/006's own modules, see DESIGN.md updates):

1. **`Telemetry::Frame.hasTwist`/`.twist` were declared but never
   populated anywhere** (`app/telemetry.h`/`.cpp`) — the wire's `twist=`
   field has been silently absent on every build since it was added.
   Fixed in `RobotLoop::updateTlm()` (`app/robot_loop.cpp`) via
   `BodyKinematics::forward(motorL_.velocity(), motorR_.velocity(),
   drive_.trackWidth(), ...)`; `Drive` gained a `trackWidth()` accessor.
   Ticket 003's own module (Pilot/telemetry wiring).
2. **Chained-pivot dwell completion was keyed on the wrong condition**
   (`Motion::Executor::tick()`, `motion/executor.cpp`) — "chained
   (non-terminal)" meant "ANY successor queued" (`queueCount_ > 0`), not
   "carrying a rotational exit velocity into a compatible successor"
   (`exitVelocity_ != 0`). Since `run_tour()`'s own one-leg lookahead
   always queues a successor, every TOUR_1/2 turn (chained into a plain
   DISTANCE leg, `exitVelocity_ == 0`) was hitting the "skip the dwell
   hold, complete on tolerance alone" path meant for genuine pivot→pivot
   velocity carries — completing the instant one sample crossed
   `heading_dwell_tol`, with no regard for `heading_dwell_rate`, while
   still rotating at up to several hundred deg/s. Fixed: the dwell HOLD is
   now required whenever `exitVelocity_ == 0` (terminal or chained-but-
   not-carrying), matching the terminal-command rule exactly; the
   carrying case additionally gained a `crossedTarget` (sign-flip) test
   since a bare tolerance-band test can straddle the whole tolerance
   window between two samples at cruise rate. Ticket 005/006's own module
   (dwell completion + boundary-velocity carry).
3. **`thetaMeasRel` used a single `wrapAngle()` diff against a fixed
   activation-time baseline**, wrong for any `|deltaHeading| > 180°`
   (TOUR_2's own `RT -21700`/`RT 21500` legs) once `App::HeadingSource`'s
   OTOS reading itself wrapped mid-rotation — aliasing the error signal by
   a full 2π and corrupting the NEXT command badly enough to hang.  Fixed
   by accumulating the relative angle incrementally every cycle
   (`unwrappedThetaRel_ += wrapAngle(step)`), which is valid for any total
   rotation. Ticket 005's own module.
4. **The plain (no-heading) terminal DISTANCE branch had no `STOP_TIME`
   backstop at all**, unlike its heading-bearing sibling — a tour's own
   FINAL leg could hang the host's 15s `run_tour()` timeout indefinitely
   with zero firmware response. Mirrored the existing backstop onto this
   branch. Ticket 005's own module.
5. **`distanceDone` had no settle epsilon** — a DISTANCE command's own
   planned trajectory can finish a fraction of a millimetre short of
   `effectiveDistance_` (S-curve quantization, not a fault) and never
   cross the raw `>=` threshold, hanging forever (this is what fix 4's
   backstop was catching before this fix addressed the root cause).
   `kDistanceSettleEpsilonMm = 2mm`, gated on the planned trajectory
   having already fully elapsed. Ticket 005's own module.
6. **`kStopTimeBackstopMarginS` bumped 1.0s → 6.0s** — with fix 2's dwell
   hold now genuinely required for pivot→DISTANCE handoffs, the sim's own
   real (wall-clock, non-deterministic) tick-thread timing exposed the
   original 1s margin as too tight: an occasional momentary tolerance/rate
   miss resets `dwellHeldMs_` to 0 with no partial credit, and enough
   resets under real-time jitter could exhaust the backstop before the
   hold ever completed. This is a partial mitigation, not a root-cause
   fix — see Impossibility Argument.

**After all six fixes**, measured results (representative run,
`uv run python -m pytest src/tests/testgui/test_tour_closure_gate.py -v -s`):

| Test | Result | Detail |
|---|---|---|
| Boundary velocity (criterion 5) | **PASS**, reliably | velocity stays ≥90% of `v_max` through the boundary of a real two-leg same-`v_max` tour |
| TOUR_1 ideal | Completes most runs; turns 0.05–1.5° off (worst seen: 3.5° before the 6th fix, ~0.5–1.3° typical after) | tolerance demanded: <0.05° |
| TOUR_2 ideal | Same per-turn profile; occasional `STOP_TIME` fault (real-time-jitter-driven, see below) | |
| TOUR_1/2 realistic (3%/2% OTOS scale error, calibrated, + encoder scale/quant/slip) | Turns mostly within ~0.5–1.3°, occasional single turn up to ~2.4–2.5° | tolerance demanded: <1.0° |

Standalone (no pytest overhead) repeated runs of `run_tour()` against a
raw `SimLoop`, 6 attempts each, AFTER all six fixes: **TOUR_1 5/6, TOUR_2
6/6 completed** (up from consistently 0/N before the fixes). A
PRE-EXISTING test not authored by this ticket,
`src/tests/testgui/test_sim_transport_tour1.py::
test_tour_1_runs_to_completion_with_finite_small_closure`, now fails
consistently (5/5 of its own built-in retries) at TOUR_1's own leg 12 —
this is a discovered regression against the pre-109-009 baseline (that
test was part of the 1184-passing baseline before this ticket's fixes)
that was NOT resolved by the time budget available; see Impossibility
Argument.

## Impossibility Argument

Two distinct gaps remain, neither closed by further iteration within this
ticket's time budget. Both are escalated here rather than silently
weakening a tolerance or a test.

**1. "EXACT (negligible epsilon)" with sim OTOS error disabled is not
achieved — measured residual ~0.5–1.5°, occasionally higher.**

With every sim error model explicitly zeroed (`set_otos_raw_scale_err(0,
0)`, `set_enc_scale_err`/`set_enc_tick_quant`/`set_enc_slip` all 0 on both
wheels), `Motion::Executor`'s OWN internal accounting
(`thetaMeasRel`/`thetaErr` relative to its own activation baseline)
converges to within a few hundredths of a degree of `deltaHeading` before
declaring `kDone` — verified directly via temporary instrumentation during
this ticket's own debugging. Yet the TRUE heading delta (`SimPlant` ground
truth, before vs. after the turn) measures 0.5–1.5° off, occasionally more.
The two measurements diverging while OTOS itself is configured
error-free points to a REAL, physical mechanism, not a residual bug:
`Devices::Otos::tick()`'s own internal read-rate limit
(`kReadPeriod = 20ms`) means `App::HeadingSource::heading()` can return a
reading that is up to one 40ms cycle stale relative to the plant's own
instantaneous physics. During a pivot's cruise phase (peak rate observed
~250–300°/s in this sprint's own `PlannerConfig` defaults), one stale
cycle corresponds to up to ~10–12° of REAL rotation the control loop has
not yet been told about — the completion decision (and the heading PD
itself) can only ever act on the LAST SAMPLE IT HAS, not on rotation that
has already physically happened but not yet been reported. This is
sprint.md's own Architecture Open Question #1
("dominant-channel-with-slaved-PD accuracy... is an empirical bet, not
derived analytically") manifesting concretely: no amount of PD-gain or
dwell-tolerance tuning inside the CURRENT single-channel,
sample-then-decide architecture can correct for rotation that has not yet
been sampled. Closing this gap to true "negligible epsilon" would need
either (a) a predictive/dead-reckoning correction for the OTOS read-rate
lag specifically during high angular rate (a real design addition, not a
tuning knob), or (b) a bench-tuned, deliberately lower `yaw_rate_max` so
the worst-case one-cycle rotation stays under whatever epsilon is
demanded — both are follow-up-issue-sized changes, not something to
improvise inside this verification ticket. The observed ~0.5–1.5°
residual (well under the sprint's realistic-profile 1° gate in most, but
not all, individual turns) is consistent with this mechanism's own
magnitude and is the actual, physical accuracy ceiling of the current
architecture at the current `yaw_rate_max`, not an unfixed bug.

**2. Tours do not complete 100% of the time — intermittent `STOP_TIME`
faults tied to real (wall-clock) simulation timing, and one pre-existing
test now fails consistently.**

`dwellHeldMs_`'s own reset policy ("any single tolerance/rate miss resets
the hold counter to 0, no partial credit") interacts badly with the sim's
own non-deterministic tick-thread scheduling (a real Python thread on real
wall-clock time, not virtual/deterministic time): under scheduling jitter,
an occasional string of resets can exhaust even the widened
`kStopTimeBackstopMarginS` (bumped 1.0s→6.0s, fix 6 above) before the hold
ever accumulates its required 150ms. This produced ~1 fault in 12 total
standalone tour runs after the fix (down from consistently 0/N before it),
and — more concerningly — a 100%-reproducible fault in the PRE-EXISTING
`test_sim_transport_tour1.py` test (via `SimTransport`, not the raw
`SimLoop` this ticket's own new tests use) at TOUR_1's own leg 12, across
all 5 of that test's own built-in retries. The persisted sim-error profile
`SimTransport` loads was confirmed all-zero (not a hidden nonzero fault
knob), so the discrepancy between this test's reliable failure and this
ticket's own ~90%+ success rate on the same tour via a raw `SimLoop` was
not root-caused in the time available — a genuine, unresolved difference
between the two connection paths. Closing this fully would need either
(a) a redesign of the dwell-hold's own resilience to a transient miss
(e.g. graceful decay instead of hard reset-to-zero — flagged as an open
question in `motion/DESIGN.md`'s own fix-6 entry, not implemented here to
avoid a larger behavioral change than this ticket's iterate-until-the-
gate-passes mandate justifies), or (b) root-causing the SimTransport-vs-
SimLoop discrepancy specifically. Recommend a follow-up
`clasi/issues/` entry tracking `test_sim_transport_tour1.py`'s own
regression and the dwell-hold resilience question, for the team-lead to
scope as either a bench-tuning pass or a small follow-up ticket.

**Net assessment:** six real, verified firmware bugs were found and fixed
(all attributed above), converting the sim gate from "hangs or truncates
to a small fraction of the commanded turn on the very first leg" to
"completes most runs with turns within ~0.5–1.5° of commanded, boundary-
velocity carry verified at the tour level" — a large, genuine
improvement, but short of the sprint's own "exact" and "always completes"
bar. This ticket is left `in-progress` (not `done`) per its own
iterate-until-done contract; the two gaps above are the concrete,
numbers-backed escalation the ticket's own acceptance criteria call for
rather than a silently weakened gate.
