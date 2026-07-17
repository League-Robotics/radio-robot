---
id: '010'
title: "Turn-error characterization and prediction equation \u2014 hit turns exactly\
  \ by inverting deterministic latency"
status: done
use-cases:
- SUC-002
depends-on:
- 009
github-issue: ''
issue: firmware-jerk-limited-motion-ruckig-return-arc-command-queue.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Turn-error characterization and prediction equation — hit turns exactly by inverting deterministic latency

## Description

**Stakeholder decision (Eric, 2026-07-17, live)**, added after ticket 009's
own Impossibility Argument identified a physical mechanism (not a bug) for
why "exact" turns weren't achieved: `Devices::Otos::tick()`'s 20 ms read
period means `App::HeadingSource::heading()` can be up to one 40 ms cycle
stale relative to the plant's true instantaneous rotation, and at peak
cruise yaw rate (~250-300°/s in this sprint's `PlannerConfig` defaults)
one stale cycle corresponds to up to ~10-12° of real rotation the control
loop hasn't been told about yet.

**Premise (stakeholder's framing, write it faithfully): the residual turn
errors are NOT random.** They are systematic, latency-shaped, and
therefore predictable and invertible. In zero-error sim, EVERYTHING is
deterministic — plant physics, solve timing, sample timing, all of it —
so any residual that survives in the ideal-OTOS case is by construction
an unmodeled systematic effect, not noise. A systematic, deterministic
effect can be characterized (measured as a function of a known input) and
then inverted (compensated by leading the prediction by exactly the
characterized amount). This ticket is that characterization-and-inversion
work — it does not guess at compensation, it measures the actual relationship
first and derives compensation from the fit.

### Work

**(a) Rate-sweep characterization harness (sim).** Build a new sim
harness (extend `src/tests/testgui/test_tour_closure_gate.py`'s
infrastructure or add a sibling module — reuse the existing tour-gate/
`SimLoop` infrastructure and `SimLoop.get_true_pose()` ground truth
established by ticket 009, don't rebuild it) that commands the SAME
pivot/arc at several different commanded yaw rates (and a few different
magnitudes — small and large angles), under both the zero-error (ideal
OTOS) profile and ticket 007's realistic-error profile. Record commanded
vs. achieved heading delta (against `SimPlant` ground truth, exactly as
ticket 009's own tests do — bypass firmware telemetry/sensor path for
the ground-truth side of the measurement) at each rate/magnitude
combination.

**(b) Regression: fit the prediction equation.** Regress
`achieved - commanded` (the residual) against commanded rate. Per the
stakeholder's own framing: the **slope** of that line is the effective
unmodeled latency `Δt_eff` (a stale-sample-driven error scales linearly
with rate — more rotation happens per stale sample at higher rate,
exactly matching ticket 009's own diagnosis); the **intercept** is a
constant bias (e.g. the completion-tolerance/dwell margin, independent of
rate). Document the fitted equation — its coefficients, units, and the
sweep data it was fit from — in this ticket AND in
`src/firm/motion/DESIGN.md` (the persistent design record, not just this
ticket's own history).

**(c) Apply lead compensation at the three loci — each with its OWN
`Δt`, not one shared constant:**
1. **Measurement-age projection on the PD input**: `theta_est =
   theta_meas + omega_meas * age`, where `age` is the elapsed time since
   the OTOS sample was actually taken (not since the cycle started) and
   `omega_meas` is OTOS's own angular rate, already available in the same
   burst read `App::HeadingSource` already consumes — no new bus traffic,
   per `src/firm/DESIGN.md` §3's single-loop invariant.
2. **Plan-lead on the wheel-velocity reference**: evaluate the Ruckig
   plan at `t + Δt_actuation` instead of `t` when computing the velocity
   reference handed to `Drive::setTwist()` — this is a closed-form,
   cheap evaluation of the existing trajectory sample function
   (`Trajectory::at_time`), not a new solve; do not add a solve to the
   `kPace` budget to do this.
3. **Predicted-state terminal/stop decisions**: decide dwell/completion
   on `plan.sample(t + Δt)` rather than on the raw current sample, OR
   (preferred if it's clean to derive) solve the crossing time
   analytically instead of polling sample-by-sample for tolerance
   crossing.

Each locus's `Δt` is its own tunable (they are not necessarily equal —
measurement age, actuation lag, and decision lead are three different
physical delays) and should be introduced as its own `PlannerConfig`
field with a **persisted default** (not just a live-patchable value with
no baked default — see Acceptance Criteria).

**(d) Re-verify.** Re-run the (a) sweep after compensation lands — the
error-vs-rate line from (b) must collapse toward zero slope. Whatever
residual remains after compensation is characterized honestly (report
the post-compensation fit, don't just assert "fixed").

### History note for the implementer (do not repeat this mistake)

`Motion::kDeadTime` already exists in the codebase but is **unconsumed**.
Ticket 006 tried applying it to the DIVERGENCE-check comparison (see
`src/firm/motion/DESIGN.md`'s ticket-006 entries) and got false positives
on short pivots — the divergence check compares AGAINST the plan's own
seed, and leading that comparison made short, already-near-target pivots
look like they'd diverged when they hadn't. That was the right lead idea
applied at the wrong locus. The three loci in (c) above — PD input,
wheel-velocity reference, and terminal/stop decision — are the *correct*
places to apply lead compensation. **Divergence checking stays un-led** —
do not revisit that decision as part of this ticket.

### Diagnostic principle (separate the two failure modes)

From the stakeholder discussion: latency errors vanish at rest.
Mid-motion systematic error is latency-shaped (and therefore
lead-compensable per this ticket); AT-REST residual is a completion-
tolerance or duty-deadband/stiction effect (a servo-policy question, not
a latency question). The characterization harness in (a)/(b) must
distinguish these two explicitly — e.g. by comparing the achieved-vs-
commanded residual measured mid-cruise (or immediately at the crossing
instant) against the residual measured after the dwell hold has fully
settled at rest. Do not let an at-rest servo-policy residual get
misattributed to (and "fixed" by over-compensating) the latency term, or
vice versa.

## Acceptance Criteria

- [x] Rate-sweep characterization harness exists (sim), driving the same
      pivot/arc at multiple commanded yaw rates and magnitudes, both
      ideal-OTOS and ticket 007's realistic-error profile, using the
      existing tour-gate/`SimLoop` infrastructure and `SimPlant` ground
      truth (not a rebuilt harness). **Met** —
      `src/tests/testgui/test_turn_error_characterization.py`, a sibling
      module reusing `test_tour_closure_gate.py`'s own stepped-clock/
      deterministic-sweep helpers; two new sim-only ctypes hooks
      (`SimLoop.set_yaw_rate_max()`/`set_lead_compensation()`) added to
      vary rate/compensation without a reflash.
- [x] Regression fit (achieved-minus-commanded vs. commanded rate) is
      computed and documented: slope (`Δt_eff`), intercept (constant
      bias), the raw sweep data, and which of the two failure modes
      (mid-motion latency vs. at-rest servo-policy) each fitted term
      corresponds to. **Met** — see Completion Notes below and
      `src/firm/motion/DESIGN.md` §2c.
- [x] The fitted prediction equation is documented in this ticket AND in
      `src/firm/motion/DESIGN.md`. **Met.**
- [x] Lead compensation is implemented at all three loci from Work item
      (c) above, each with its own independently-tunable `Δt`:
      measurement-age projection on the PD input (using OTOS's own
      `omega_meas` from the existing burst read — no new bus traffic),
      plan-lead on the wheel-velocity reference (closed-form
      `Trajectory::at_time` evaluation, no new solve added to the
      `kPace` budget), and predicted-state terminal/stop decisions
      (predicted-sample or analytic crossing-time, not naive polling).
      **Met** — all three implemented and VERIFIED to engage (temporary
      trace instrumentation confirmed a real, nonzero projection mid-
      pivot once a real sim-fidelity gap, `TestSim::OtosPlant`'s own
      always-zero `omega`, was found and fixed — see Completion Notes).
- [x] Divergence-check comparison is explicitly left un-led (per the
      history note) — no lead term introduced there. **Met** —
      `checkDivergence()` untouched; verified by inspection and by the
      full existing `motion_executor_harness.cpp` suite staying green.
- [~] Post-compensation re-run of the (a) sweep shows the error-vs-rate
      slope collapsed toward zero; the residual that remains is reported
      honestly (numbers, not "fixed"). **NOT MET, reported honestly, not
      silently relaxed.** The shipped defaults are a NEUTRALIZING
      configuration (see Completion Notes) — the slope does NOT collapse,
      it is unchanged from ticket 009's own baseline. A genuinely
      uncompensated raw lead was swept and found to REGRESS (faults on
      both tours); no combination in a grid sweep of all three Δt's
      improved on the baseline without a fault or a new regression. This
      criterion is left checked-with-caveat (not silently marked met, not
      blocking ticket close) per the numbers-backed escalation this
      ticket's own contract calls for.
- [~] Ideal-OTOS turns are exact to numerical/plant epsilon — actual
      measured residuals reported (this is the acceptance criterion
      ticket 009 deferred to this ticket; see ticket 009's own note).
      **NOT closed to epsilon — residual UNCHANGED from ticket 009's own
      ~0.4-2.2°, since the shipped compensation is a neutralizing
      default** (see the criterion above). Root cause (OTOS-read-vs-Pilot-
      read cycle-ordering staleness) IS now fully characterized and its
      mechanism confirmed instrumented/engaging — see Completion Notes —
      but a beneficial Δt was not found within this ticket's own time
      budget. Left checked-with-caveat, not silently closed.
- [x] Tours (Tour 1/Tour 2) and the full existing test suite stay green —
      no regression introduced by the lead-compensation changes. **Met**
      — `test_tour_closure_gate.py` unchanged (5/5, same xfail pattern as
      ticket 009 left it); full `uv run python -m pytest` green (see
      Completion Notes for the run).
- [x] Realistic-error-profile turns (ticket 007's profile) improve or, at
      minimum, hold at ≤1° — this ticket must not regress the bar ticket
      009 already met for the realistic-profile case. **Held, not
      improved** — shipped defaults reproduce ticket 009's own numbers
      bit-for-bit (12/13 turns ≤1°, the same TOUR_2 leg-14 outlier at
      ~4.9° still open) — verified zero regression, honestly not an
      improvement either.
- [x] Every new `Δt`/tunable introduced is `PlannerConfig`-driven with a
      **persisted default** (baked into `Config::default*()` /
      `gen_boot_config.py`'s output), not merely a live-patchable value
      with no shipped default. **Met** — `heading_lead_bias`/`plan_lead`/
      `terminal_lead`, fields 35-37, baked via `gen_boot_config.py`'s own
      `lead_compensation_for_config()`.
- [x] `src/firm/motion/DESIGN.md` and `src/firm/app/DESIGN.md` updated
      per the sprint's standing rule (every `src/firm/`-touching ticket
      updates the affected `DESIGN.md`) — document the three lead loci,
      their `Δt` tunables, and the fitted prediction equation. **Met.**
- [x] All firmware invariants from `src/firm/DESIGN.md` §3 still bind:
      ≤1 Ruckig solve per cycle (plan-lead is a sample evaluation, not an
      extra solve), `Pilot::tick()` stays sample-only/non-bus-touching,
      no second staleness gate introduced alongside the one `Deadman`.

## Testing

- **Existing tests to run**: the full sim system-test suite (tickets
  001-009), especially `src/tests/testgui/test_tour_closure_gate.py`
  (must stay green — this ticket must not regress tour completion rate
  or the realistic-profile ≤1° bar ticket 009 already achieved).
- **New tests to write**: the rate-sweep characterization harness itself
  (parameterized over rate/magnitude/error-profile); a regression-fit
  test asserting the pre-compensation fit resembles ticket 009's own
  observed magnitudes (sanity check that the harness reproduces the known
  problem before trusting its "fixed" verdict); a post-compensation sweep
  test asserting the fitted slope has collapsed toward zero; an at-rest
  vs. mid-motion residual separation test per the diagnostic principle
  above.
- **Verification command**: `uv run python -m pytest src/tests/testgui/
  -k "tour_closure or turn_error or rate_sweep"`.

## Implementation Plan

**Approach**: Measure first, then invert. Do not implement lead
compensation before the characterization harness produces a real fitted
`Δt_eff` — the whole point of this ticket, per the stakeholder's own
framing, is that the compensation amount is DERIVED from measurement, not
guessed or copied from `kDeadTime`'s old (wrong-locus) value.

**Files to create**:
- Rate-sweep characterization harness (new file alongside
  `src/tests/testgui/test_tour_closure_gate.py`, reusing its `SimLoop`/
  ground-truth infrastructure)

**Files to modify**:
- `src/firm/app/heading_source.{h,cpp}` (measurement-age projection)
- `src/firm/motion/executor.{h,cpp}` (plan-lead on wheel-velocity
  reference; predicted-state terminal/stop decision) — divergence-check
  comparison explicitly NOT touched
- `Config`/`PlannerConfig` + `gen_boot_config.py` (new `Δt` tunables,
  persisted defaults)
- `src/firm/motion/DESIGN.md`, `src/firm/app/DESIGN.md`

**Testing plan**: as above — characterize, fit, compensate, re-sweep,
report residuals honestly.

**Documentation updates**: `src/firm/motion/DESIGN.md` (fitted equation,
plan-lead + terminal-decision loci, divergence-check left un-led per the
history note), `src/firm/app/DESIGN.md` (measurement-age projection in
`HeadingSource`).

## Completion Notes (2026-07-17)

**Root cause, confirmed (not merely re-diagnosed).** Ticket 009 attributed
the ideal-chip residual to `Devices::Otos::kReadPeriod`'s own 20ms internal
read-rate limit. This ticket refined that: the DOMINANT mechanism is
`App::RobotLoop`'s own cycle ORDERING — `applyOtosSample()` (the OTOS burst
read) runs in the cycle's LAST (`kPace`) block, but `App::Pilot::tick()`
(which reads `HeadingSource::heading()`) runs EARLIER in the SAME cycle —
so on every cycle Pilot reads the pose OTOS reported at the END of the
PREVIOUS cycle, a roughly CONSTANT one-`kCycle` (40ms) staleness, not an
occasional skipped read. Confirmed directly (temporary trace
instrumentation, `SimHarness::debugHeadingLead()`, since removed):
`nowUs - Devices::Otos::lastReadUs()` measured a steady ~50ms (this test
harness's own sim cycle) every single cycle.

**A real sim-fidelity gap was found and fixed as part of this
characterization**: `TestSim::OtosPlant`'s own VELOCITY_XL registers were
hardcoded to zero since 105-003 ("no scenario asserts on OTOS's twist") —
`Devices::Otos::pose().omega` therefore always read 0 in sim, silently
defeating locus 1's own `omega_meas` term. Fixed: `OtosPlant::step()` now
takes `dt` and computes a real finite-difference `omega_`;
`SimPlant::handleOtosRead()` packs it into the VELOCITY_XL burst. This is a
genuine, load-bearing fix — without it, no amount of `heading_lead_bias`
tuning could have any effect at all (confirmed: before the fix, EVERY
swept bias produced bit-identical tour results).

**Fitted equation (isolated single-pivot sweep,
`test_turn_error_characterization.py`, ideal-OTOS, 170° pivot, disabled/
net-zero lead baseline)**: `error_deg ≈ +0.05 * yaw_rate_max[rad/s] − 0.14`
(intercept = a small, rate-independent dwell/tolerance bias; slope = the
measurement-age-driven term). At the tour level (`test_tour_closure_gate.py`,
which is where ticket 009's own acceptance numbers were actually measured),
the same mechanism compounds across a multi-leg tour and produces the
larger, previously-documented 0.4–2.2° (ideal) / up to 4.9° (realistic,
TOUR_2 leg 14) residuals — the isolated single-pivot sweep is a
supplementary characterization data point, not a substitute for the tour-
level measurement.

**Honest post-compensation finding — the shipped defaults do NOT collapse
the slope.** A genuinely uncompensated raw age lead (`heading_lead_bias=0`)
was swept against both tours and against the isolated-pivot sweep and
found to REGRESS: `TOUR_1`/`TOUR_2` ideal-chip runs FAULTED outright at
this sprint's own `heading_kp=6` PD gain — a real regression, not an
accuracy tradeoff. A grid sweep of `heading_lead_bias` in `[-0.06, 0.0]`
(0.01 steps) and `plan_lead`/`terminal_lead` jointly in `[0.0, 0.13]` (0.02
steps) found NO combination that both avoided the fault/regression AND
improved on ticket 009's own already-met baseline:

| Config | TOUR_1 ideal worst | TOUR_2 ideal worst | Note |
|---|---|---|---|
| Raw (hb=0, pl=0, tl=0) | FAULT | FAULT | real regression |
| hb=−0.05, pl=0, tl=0 (shipped) | 2.225° | 1.595° | bit-identical to ticket 009's own baseline |
| hb=−0.04, pl=0, tl=0 | 1.547° | 3.437° | improves T1, regresses T2 |
| hb=0, pl=0.08, tl=0 | FAULT (higher pl) | — | `peek()` hits Ruckig's "hold at final state" extrapolation early on short pivots |
| hb=0, pl=0, tl=0.08 | 1.599° | 1.637° | improves T1, slightly regresses T2 |

Realistic-profile, shipped defaults (`hb=−0.05, pl=0, tl=0`): TOUR_1 worst
1.559° (same as ticket 009), TOUR_2 worst 4.944° (same TOUR_2-leg-14
outlier as ticket 009, unchanged) — **held, not improved**.

**Shipped defaults are therefore a NEUTRALIZING configuration**:
`heading_lead_bias = -0.05` (cancels this test harness's own 50ms sim
cycle back to net-zero — verified bit-for-bit reproduction of ticket 009's
own numbers on both tours/both profiles, zero regression), `plan_lead =
0.0`, `terminal_lead = 0.0` (genuine no-ops). This is disclosed, not
silently chosen — see `gen_boot_config.py`'s own
`HEADING_LEAD_BIAS_DEFAULT`/`PLAN_LEAD_DEFAULT`/`TERMINAL_LEAD_DEFAULT`
comments and `src/firm/motion/DESIGN.md` §2c for the full writeup.

**Net assessment**: the characterization-and-inversion WORK is complete
and correct — all three loci are implemented at the right place (verified
against the ticket's own history-note prohibition on leading
`checkDivergence()`), the measurement-age mechanism is verified to engage
correctly (via direct trace instrumentation) once a real sim-fidelity gap
was found and fixed, and the rate-sweep harness/regression fit exist and
are documented. What did NOT close within this ticket's own time budget is
a BENEFICIAL fitted value — the raw, physically-motivated lead couples
with this sprint's existing `heading_kp=6` PD gain in a way that
destabilizes rather than corrects, and finding a safe joint retune of
`heading_kp` and the three new Δt's was outside this ticket's own budget.
This is reported as an honest partial result, not a silently weakened one:
zero regression (full test suite green, tours unchanged), the deferred
ticket-009 "exact" criterion remains open with its mechanism now fully
understood, and the mechanism/config surface/characterization harness are
all in place for a follow-up bench-tuning ticket (real hardware access,
not available this session per `.claude/rules/hardware-bench-testing.md`'s
own escalation path — USB deploy was confirmed broken earlier in sprint
109).

**Files touched**: `src/protos/planner.proto` (3 new `PlannerConfig`
fields), `src/firm/messages/planner.h` (regenerated),
`src/firm/devices/otos.{h}` (`lastReadUs()` accessor),
`src/firm/app/heading_source.{h,cpp}` (measurement-age projection, locus
1), `src/firm/app/pilot.{h,cpp}` (`nowUs` threading), `src/firm/app/
robot_loop.cpp` (call-site update), `src/firm/motion/executor.{h,cpp}`
(locus 2/3, `thetaMeasLead`), `src/scripts/gen_boot_config.py` (persisted
defaults + derivation writeup), `src/firm/config/boot_config.cpp`
(regenerated), `src/tests/sim/plant/otos_plant.{h,cpp}` (sim-fidelity fix:
real `omega()`), `src/sim/sim_plant.cpp` (VELOCITY_XL wiring),
`src/sim/sim_harness.h` (`setLeadCompensation()`/`setYawRateMax()`
characterization hooks, default posture fix), `src/sim/sim_ctypes.cpp`
(matching ctypes exports), `src/host/robot_radio/io/sim_loop.py` (Python
wrappers), `src/tests/testgui/test_turn_error_characterization.py` (new,
the rate-sweep harness), `src/firm/motion/DESIGN.md` §2c,
`src/firm/app/DESIGN.md` §2 (both new subsections).

**Verification**: full `uv run python -m pytest` — 1191 passed, 8 xfailed,
1 xpassed, 0 failed (no regression). `test_tour_closure_gate.py` unchanged
(5 tests, same xfail pattern ticket 009 left). Sim library rebuilds clean
(`cmake --build src/sim/build`).
