---
id: '118'
title: 'Loop schedule truth: firmware loop reorder + sim cadence parity'
status: planning-docs
branch: sprint/118-loop-schedule-truth-firmware-loop-reorder-sim-cadence-parity
worktree: false
use-cases:
- SUC-063
- SUC-064
- SUC-065
- SUC-066
issues:
- restore-the-interleaved-request-settle-tick-loop-schedule.md
- stop-decision-must-see-this-cycles-odometry.md
- sim-cycle-must-match-firmware-period.md
- land-at-zero-completion-delete-stop-lead.md
- turn-error-characterization-postcompensation-tests-need-rewrite-after-lead-deletion.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 118: Loop schedule truth: firmware loop reorder + sim cadence parity

## Goals

Restore the timing schedule `RobotLoop::cycle()` regressed to (commit
`5f5a2ba7`, then compounded by the `c75f528e` "112-005" hoist experiment),
make the MOVE stop decision consume the SAME cycle's odometry instead of
last cycle's, and make the sim's per-step virtual-time advance equal the
firmware's own control period — so that every sim-measured millisecond
constant transfers to hardware without a translation factor, and the
~20° turn-completion overshoot the 2026-07-22 turn-execution review
root-caused to these three defects (F2/F3, D2/D4) is eliminated at the
source rather than compensated for by a re-tuned lead constant.

## Problem

`docs/code_review/2026-07-22-turn-execution-review.md` traces "why turn
90° doesn't land on 90°" to three self-inflicted scheduling defects, not
a control-law defect:

1. **The loop schedule is not what it's documented to be.** `5f5a2ba7`
   collapsed each motor's `requestSample()`/`tick()` adjacent, pushed
   `comms_.pump` into a settle block placed after both collects, and
   zeroed `kSettle`/`kClear` (4→0) while halving `kCycle` (40→20) to make
   the (now-wrong) schedule fit. The vendor 4ms encoder settle still
   happens — but as a *blocking* sleep inside `motorL_.tick()`/
   `motorR_.tick()`, tripping the I2C clearance safety-net fault bit every
   cycle and hiding real settle time outside the advertised pace budget.
   `c75f528e` then hoisted `drive_.tick()` above the motor ticks (a
   live-experiment cycle-order change, tracked only in project memory, not
   in an issue) on top of the regression.
2. **The stop decision reads stale odometry by construction.**
   `MoveQueue::tick()` (the completion decision) runs before
   `odom_.integrate()` in the same cycle, so every stop decision is made
   against odometry integrated at the END of the PREVIOUS cycle — a full
   cycle of heading staleness the review measured at 5.7° at cruise (2
   rad/s, 50ms sim cycle). The 45ms anticipation lead in `MoveQueue`
   exists in large part to cancel this one ordering choice.
3. **The sim runs a different robot than the one that ships.** Sim steps
   50ms of virtual time per `SimHarness::step()` call — chosen only to
   dodge `NezhaMotor`'s 40ms write-rate throttle at the firmware's
   (regressed) 20ms cycle — vs. firmware's real cycle period. Every
   sim-tuned millisecond constant and every "N cycles of latency" finding
   in the review is measured on a plant with a materially different
   control period than what ships.

## Solution

Three issues, landed in dependency order because the first two edit the
exact same function (`RobotLoop::cycle()`) and the third can only be
correctly re-baselined once the real cycle period is settled:

1. **`restore-the-interleaved-request-settle-tick-loop-schedule.md`** —
   restore the last-known-good schedule (commit `39c084c1`) with today's
   richer block bodies: `kCycle=40`, `kSettle=4`, `kClear=4`
   (stakeholder-confirmed), per-port interleave (select L → collect L →
   select R → collect R), `drive_.tick()` back inside the R-settle block
   (retiring the 112-005 hoist experiment), `Telemetry::kPrimaryPeriod`
   coupled back to `kCycle` (40).
2. **`stop-decision-must-see-this-cycles-odometry.md`** — within that
   restored schedule, relocate `moveQueue_.tick()` (+ its completion
   ack/fault staging) from the R-settle block into the trailing pace
   block, AFTER `applyOtosSample()` → `odom_.integrate()` →
   `stateEstimator_.update()` — the stop decision then reads odometry
   integrated in the SAME cycle it acts on.
3. **`sim-cycle-must-match-firmware-period.md`** — set
   `SimHarness::kCycleDtUs = 40000` so sim step period equals firmware
   `kCycle`; widen `NezhaMotor`'s write-rate-throttle margin so an
   on-schedule write at exactly the throttle interval never loses to
   timing jitter; fix the five independently-verified hardcoded 50ms/0.05s
   assumptions scattered across C++ harnesses and Python sim/test code;
   re-baseline every cadence-sensitive gate (closure gate, button
   acceptance, estimator tracking) at the new period.

Folded into ticket 1 (small, belongs with the loop work, no separate
issue file): re-point the two dangling xfail citations of the deleted
`clasi/issues/cycle-order-reorder-experiment-ab-before-hardware.md` (in
`test_tour_closure_gate.py` and `src/tests/sim/unit/test_app_robot_loop.py`)
at this sprint's own `restore-the-interleaved-...` issue, its live
successor.

4. **`land-at-zero-completion-delete-stop-lead.md`** (pulled forward from
   sprint 119, see Decision Record below) — declare MOVE completion when
   `remaining ≤ ε AND |ω_cmd| ≤ ε_ω` in `MoveQueue::tick()`, keep the
   `StopCondition` threshold/timeout as the always-armed backstop, and
   DELETE `stop_lead_ms` + the anticipation block (schema, all three
   robot JSONs, `gen_boot_config.py`, `config_sync_allowlist.json`) rather
   than re-tuning it. `StateEstimator`/`bodyAt()` is QUARANTINED (kept,
   consumer removed), not deleted.
   `turn-error-characterization-postcompensation-tests-need-rewrite-after-lead-deletion.md`
   folds into the same ticket (its own disposition note: the postcompensation
   tests characterize an approach this change retires).

## Decision Record: R6 applied (2026-07-23, mid-execution amendment)

Ticket 002 landed the odometry-freshness fix, and its closure-gate run
went RED at the unchanged `stop_lead_ms=45` — not because the fix was
wrong, but because fresh same-cycle odometry removed exactly the
staleness `stop_lead_ms` had been partly compensating for, exposing the
lead as an overcorrection once that staleness was gone (full data: a
0-120ms sweep against the closure gate's own path found no value with
real margin — see ticket 002's report and the dated addendum in
`land-at-zero-completion-delete-stop-lead.md`). Per the turn-execution
review's own R6 rule — "a change that adds or retunes a numeric constant
must name the physical quantity it models and derive it from named
constants; if adding a stage forces retuning an existing constant, the
default action is to delete the constant, not retune it" — and this
project's sprint-end-must-be-testable convention, the team-lead decision
is: **002's fresh data invalidated the stale-tuned lead; it is deleted
rather than retuned; land-at-zero (`land-at-zero-completion-delete-stop-lead.md`
+ `turn-error-characterization-postcompensation-tests-need-rewrite-after-lead-deletion.md`)
is pulled forward from sprint 119 into 118, as ticket 004, so this sprint
ends on a green, testable closure gate instead of handing a known-red
gate to a not-yet-detailed sprint 119.** Ticket 003 (sim cadence parity)
is resequenced to depend on 004 and run last, so its own gate re-baseline
reflects the FINAL regime (40ms cycle + land-at-zero) in one pass. Sprint
119's scope shrinks accordingly (see its own sprint.md) to the four
remaining issues: silent-off config-boundary kill, leg hand-off contract,
config-attic deletion, and narrative/doc relocation.

## Success Criteria

- Firmware and sim build green.
- Full `uv run python -m pytest` suite green.
- Sim tour-closure gate and button-acceptance suite green at the 40ms
  period WITH `stop_lead_ms` DELETED (amended — see Decision Record),
  per-leg bands unchanged or tightened (never silently widened).
- `grep 'runAndWait\|sleepUntil' src/firm/app/robot_loop.cpp` still the
  complete list of the firmware's waits (existing invariant, re-verified
  post-reorder).
- No surviving hardcoded 0.05s/50ms cycle assumption anywhere in the tree
  (grep gate, per the sim-cycle issue's own acceptance criteria).
- No `stop_lead` string survives anywhere in `src/` or `data/` (grep
  gate, per the land-at-zero issue's own acceptance criteria).
- **Bench gate is explicitly DEFERRED** to the phase-B bench session that
  immediately follows this sprint (stakeholder mandate, overnight
  sim-only run) — see Scope/Migration Concerns below. The sim/pytest bar
  above is this sprint's actual closing bar.

## Scope

### In Scope

- `src/firm/app/robot_loop.cpp` — schedule constants and `cycle()`'s call
  order (interleave restore + `moveQueue_.tick()` relocation).
- `src/firm/app/telemetry.h` — `kPrimaryPeriod` coupling to `kCycle`.
- `src/sim/sim_harness.h` — `kCycleDtUs`.
- `src/firm/devices/nezha_motor.cpp` — write-rate-throttle jitter margin.
- Five verified hardcoded-cadence call sites: `app_robot_loop_harness.cpp`
  (×2), `turn_prediction_capture.py`, `test_tour_closure_gate.py`,
  `sim_loop.py`.
- Cadence-sensitive harness/test re-baselining: `app_robot_loop_harness`,
  `app_telemetry_harness`, `straight_twist_harness`,
  `state_estimator_tracking_harness`, `devices_motor_harness`,
  `plant_harness`, the sim tour-closure gate, button-acceptance suite.
- Doc updates: `src/firm/app/DESIGN.md` §4, `src/sim/DESIGN.md` (cadence
  mismatch note becomes a resolved-parity note), `docs/design/design.md`
  cadence line.
- Two dangling xfail citation re-points (see Solution above).
- **(Amended, ticket 004)** `App::MoveQueue`'s completion predicate
  (`move_queue.{h,cpp}`) — land-at-zero gate, deletion of `stopLead_` +
  the anticipation block; `App::StateEstimator` — QUARANTINE (its
  `MoveQueue` consumer removed; module/`update()`/tests kept); the
  `EstimatorConfigPatch` wire arm's `stop_lead_ms` field, the
  `estimator_kwargs()` push, the pydantic estimator schema, all three
  `data/robots/*.json` (`stop_lead_ms` + `_estimator_note` blocks),
  `gen_boot_config.py`'s bake, and the `config_sync_allowlist.json` entry
  if present; `test_turn_error_characterization.py`'s postcompensation
  tests.

### Out of Scope

- **(Amended)** Anticipation-lead removal / land-at-zero completion
  semantics is NO LONGER out of scope — see Decision Record above; it is
  ticket 004 of this sprint. What remains out of scope from sprint 119:
  the silent-off shaping/anticipation config boundary
  (`kill-the-silent-off-shaping-config-boundary.md`), the leg hand-off
  contract, the (non-`stop_lead_ms`) config-attic deletion, and the
  doc-relocation sweep — all still sprint 119, now a 4-ticket sprint (see
  its own sprint.md).
- Hardware bench verification — deferred to phase-B (see Migration
  Concerns).

## Test Strategy

Sim-only this sprint, per the overnight mandate. Each ticket runs the
firmware/sim build and the full `uv run python -m pytest` suite; ticket 1
additionally runs `app_robot_loop_harness`/`app_telemetry_harness`, ticket
2 re-runs the same two plus the ordering assertion the stop-decision
issue calls for, ticket 3 re-runs every cadence-sensitive gate listed
above and adds the grep gate for surviving hardcoded cycle assumptions.
The sim tour-closure gate and button-acceptance suite are the system-level
check that per-leg accuracy bands hold (or improve) at the new 40ms
period — both must be green before the sprint is considered done. Bench
verification (the hardware checklist each of these issues' own "Bench
gate" sections call for) is out of scope for this sprint's own closing
bar; it is scheduled for the phase-B bench session immediately following.

## Architecture

**Sizing: Substantial** — this sprint touches 3+ modules with independent
ownership (`App::RobotLoop`+`App::Telemetry` in `src/firm/app`,
`Sim::SimHarness` in `src/sim`, `Devices::NezhaMotor` in
`src/firm/devices`, plus cadence-coupled surface in the host Python layer
and the test/harness tree). It is sized substantial by module count per
the sizing rubric, but see Step 4 below for why no component diagram is
included — the sprint 020 precedent applies: this is a resequencing and
constant-synchronization change across relationships that already exist,
not a new composition.

**Amendment (2026-07-23, mid-execution — see Decision Record above):**
ticket 004 (land-at-zero, pulled forward from sprint 119) adds a fourth
responsibility, touches `App::MoveQueue`'s completion predicate and
`App::StateEstimator`'s consumer count, and deletes a config-schema field
across `data/robots/*.json`/the pydantic model/`gen_boot_config.py` — a
genuine data-model change. Sizing stays Substantial (it already was); the
"no diagram" call is revisited below (Step 4) rather than reversed, since
the amendment removes a dependency edge rather than adding a new
composition.

### Step 1 — Understand the problem

Covered above (Problem/Solution). The defect is scheduling, not control
law: three constants and one call-order choice, all inside code that
already exists and already depends on itself in exactly this shape.

### Step 2 — Identify responsibilities

Three responsibility groups, coupled in the order they must land:

- **Loop schedule fidelity** — `RobotLoop::cycle()`'s constants and call
  order must match its own documented interleaved
  request→settle→collect design (`App::RobotLoop`'s own DESIGN.md §3
  invariant: "the timing schedule is exactly `robot_loop.cpp`'s
  `runAndWait` calls"). Changes independently of the other two groups in
  principle, but shares the exact same function body with the next group,
  so they must land in the same sprint without one silently reverting the
  other.
- **Stop-decision data freshness** — `MoveQueue::tick()`'s call position
  relative to `Odometry::integrate()`/`StateEstimator::update()` within
  that same function. Depends on the first group's restored schedule
  existing to relocate the call within.
- **Sim/firmware cadence parity** — `SimHarness::kCycleDtUs` and
  `NezhaMotor`'s write-throttle margin. Depends on the first group having
  settled `kCycle`'s final value (40, not 20) before the sim can adopt a
  matching step size; independent of the second group module-wise
  (`Sim`/`Devices` vs. `App`), but re-baselining its cadence-sensitive
  gates is only meaningful once both loop-schedule changes are in.
- **Completion semantics (land-at-zero, amendment)** — `MoveQueue::tick()`'s
  completion PREDICATE (as opposed to the second group's completion
  DATA-FRESHNESS, already landed): declare done on
  `remaining≈0 AND ω_cmd≈0` instead of a predicted-heading lead. Depends
  on the second group (freshness) being in place — `remaining` must be
  computed from this-cycle odometry for the predicate to be meaningful —
  and, having landed, changes what the third group's gate re-baseline
  re-baselines AGAINST (the final regime, not an intermediate one).
  Config-schema deletion (`stop_lead_ms` across JSON/pydantic/
  `gen_boot_config.py`) rides this group, not a separate one.

### Step 3 — Subsystems and modules

- **`App::RobotLoop`** (`src/firm/app/robot_loop.{h,cpp}`) — purpose:
  sequences one control-loop iteration's bus transactions, command
  dispatch, and pacing. Boundary: everything inside `cycle()`'s own call
  order; does not own device-leaf internals (motor PID, OTOS burst-read
  logic) or MoveQueue's own stop-condition math, only WHEN each is called.
  Use cases: SUC-064 (schedule), SUC-063 (stop-decision freshness).
- **`App::Telemetry`** (`src/firm/app/telemetry.h`) — purpose: emits
  outbound frames at a period coupled to the loop's own pace. Boundary:
  `kPrimaryPeriod`'s value only changes here; no behavior change beyond
  the constant. Use case: SUC-064.
- **`Sim::SimHarness`** (`src/sim/sim_harness.h`) — purpose: composes the
  real firmware graph against a simulated I2C bus and steps virtual time.
  Boundary: `kCycleDtUs`'s value and the `step()` cadence it drives; does
  not touch firmware logic. Use case: SUC-065.
- **`Devices::NezhaMotor`** (`src/firm/devices/nezha_motor.cpp`) —
  purpose: drives one motor's velocity PID and vendor duty writes.
  Boundary: only the write-rate-throttle jitter margin changes; PID/duty
  logic untouched. Use case: SUC-065.
- **Test/harness surface** (`app_robot_loop_harness.cpp`,
  `app_telemetry_harness.cpp`, `plant_harness.cpp`,
  `straight_twist_harness.cpp`, `state_estimator_tracking_harness.cpp`,
  `devices_motor_harness.cpp`, `test_tour_closure_gate.py`,
  `test_app_robot_loop.py`, `sim_loop.py`, `turn_prediction_capture.py`)
  — not a module for cohesion purposes; each file's own cadence-sensitive
  expectations are re-baselined to the new constants as a consequence of
  the three modules above changing, not an independent design decision.
- **(Amended) `App::MoveQueue`'s completion predicate** (part of
  `App::RobotLoop`'s own subsystem, `src/firm/app/move_queue.{h,cpp}`) —
  purpose: decide when an active `Move` is done. Boundary: the predicate
  itself (threshold/timeout backstop vs. land-at-zero gate); does not own
  the velocity shaper's taper math (`Motion::VelocityShaper`, unchanged)
  or `StopCondition`'s own comparison (unchanged, still the backstop).
  Use case: SUC-066.
- **(Amended) `App::StateEstimator`** (`src/firm/app/state_estimator.{h,cpp}`)
  — purpose unchanged (predict-to-now peer estimates); boundary change:
  loses its one firmware production consumer (`MoveQueue`'s anticipation
  block) and is explicitly QUARANTINED — module, `update()`, and tests
  remain for the planned fake-OTOS/fusion bench work, but nothing in the
  production call graph reads `bodyAt()` after this sprint. Use case:
  SUC-066 (the removal is part of the same predicate change).
- **(Amended) Config/schema layer** (`data/robots/*.json`, the pydantic
  estimator-config model, `gen_boot_config.py`, `config_sync_allowlist.json`)
  — purpose: persisted/generated robot configuration. Boundary: only the
  `stop_lead_ms` field and its `_estimator_note` archaeology are removed;
  every other field is untouched. This is the sprint's one genuine
  data-model change (a schema field deletion, not just a constant value
  change). Use case: SUC-066.

### Step 4 — Diagrams

**No component/module diagram.** This sprint recomposes nothing: it
resequences calls within one existing function
(`RobotLoop::cycle()`) and synchronizes three already-existing cadence
constants (`kCycle`, `kPrimaryPeriod`, `kCycleDtUs`) across modules that
already depended on each other in exactly this call shape before this
sprint. No new module, no new edge, no dependency-direction change (same
justification pattern as sprint 020's architecture doc: a diagram would
show the identical graph before and after).

**(Amended) Dependency graph — one edge REMOVED, stated in prose rather
than diagrammed.** Ticket 004 removes the `App::MoveQueue` →
`App::StateEstimator::bodyAt()` production dependency (the anticipation
block was the one call site); `StateEstimator` is quarantined, not
deleted, so the module itself and its `update()` producer side survive
with zero consumers in the production graph. A one-edge removal from an
already-small, already-documented dependency set (§5 of
`src/firm/app/DESIGN.md` already narrates every consumer of `bodyAt()`)
is fully captured by that prose; a graph diagram would show one edge
disappearing from an otherwise-unchanged five-node graph, which does not
clarify anything a sentence doesn't already say — the sprint 020
"nothing new is being composed" escape applies symmetrically to a single
subtraction as it does to zero change.

**(Amended) ERD — no diagram, but this IS a real data-model change.**
`stop_lead_ms` is deleted from the pydantic estimator-config model and
all three `data/robots/*.json` files (plus `gen_boot_config.py`'s bake
and the `config_sync_allowlist.json` entry). This is a single scalar
field's removal from an existing schema, not a new entity or
relationship — one field, one meaning, deleted everywhere it's declared
in the same commit (ticket 004's own delete-list discipline). A full ERD
would be disproportionate to a one-field deletion with no surviving
relationship to depict; the delete list in ticket 004 and this section's
own enumeration serve the same purpose an ERD would for a change this
small.

### Step 5 — What Changed / Why / Impact / Migration Concerns

**What Changed:**
- `robot_loop.cpp`: `kSettle 0→4`, `kClear 0→4`, `kCycle 20→40`;
  `cycle()`'s call order restored to the interleaved
  select→settle→collect shape with `drive_.tick()` back inside the
  R-settle block; `moveQueue_.tick()` (+ completion ack/fault staging)
  moved from the R-settle block into the trailing pace block, after
  `applyOtosSample()`/`odom_.integrate()`/`stateEstimator_.update()`.
- `telemetry.h`: `kPrimaryPeriod 20→40`.
- `sim_harness.h`: `kCycleDtUs 50000→40000`.
- `nezha_motor.cpp`: `kMinWriteIntervalUs` gets a jitter margin so an
  on-schedule 40ms write never loses to a 39.x ms real cycle.
- Five hardcoded 0.05s/50ms cadence assumptions corrected to 0.04s/40ms
  (or derived from a shared constant, implementer's choice — see Open
  Questions).
- `src/firm/app/DESIGN.md` §4, `src/sim/DESIGN.md` (its own "does not
  match" Open Question becomes a resolved-parity note), and
  `docs/design/design.md`'s cadence line updated to 40ms/~25Hz.
- **(Amended, ticket 004)** `move_queue.{h,cpp}`: land-at-zero completion
  predicate added to `MoveQueue::tick()`; `stopLead_` member/ctor param
  and the anticipation block deleted. `stop_lead_ms` deleted from the
  `EstimatorConfigPatch` wire arm, `estimator_kwargs()`, the pydantic
  estimator schema, all three `data/robots/*.json` (+ `_estimator_note`
  blocks), `gen_boot_config.py`, and `config_sync_allowlist.json`.
  `test_turn_error_characterization.py`'s postcompensation tests
  rewritten/removed per that module's own disposition note.

**Why:** Per Problem above — restores the schedule to what its own design
doc already claims it is, removes a full cycle of avoidable stop-decision
staleness, and makes sim-measured timing transferable to hardware.

**Impact on Existing Components:** `App::MoveQueue`'s completion latency
changes shape (decision now same-cycle-fresh; decision-to-duty write is
still ~1 cycle, now 40ms not 20ms — the review's own §7-R1 calls this
acceptable once the land-at-zero taper lands, which it now does THIS
sprint via ticket 004, not 119). `App::Telemetry`'s frame rate halves
(50Hz nominal → 25Hz) — a real, visible change to anyone polling
telemetry at the old assumed rate; `tlm-rate-15-19hz-vs-50hz-nominal-serial.md`
(existing, unrelated issue) will need its own nominal re-measured, noted
there per the sim-cycle issue's own acceptance criteria, not fixed here.
`Sim::SimHarness` step semantics: one `step()` call now advances exactly
one firmware cycle's worth of virtual time at the SAME period firmware
runs at (previously 2.5× coarser) — any caller reasoning about
"N sim_step() calls ≈ N firmware cycles" becomes literally true instead
of approximately true modulo a translation factor. **(Amended)**
`App::StateEstimator`'s production role changes from "consumed by
MoveQueue's anticipation block" to "no production consumer, quarantined
for future fusion work" — any code or doc that assumed `bodyAt()` feeds
completion timing is now wrong and must be corrected (the design overlay
edit for this amendment does this for `src/firm/app/DESIGN.md`).

**Migration Concerns:** **(Amended)** This sprint now includes a real
config-schema migration: `stop_lead_ms` is deleted from the pydantic
estimator model, all three `data/robots/*.json` files, and the
`EstimatorConfigPatch` wire arm's field list, in the same commit as the
`gen_boot_config.py`/`config_sync_allowlist.json` updates (ticket 004's
own delete-list discipline — schema and every consumer together, not
staggered). A robot JSON or a running firmware image from BEFORE this
sprint that still carries/expects `stop_lead_ms` is not wire-compatible
with the post-ticket-004 `EstimatorConfigPatch` shape; this is a
same-repo, same-deploy-cycle change (no robots are running old firmware
against new configs or vice versa in this project's workflow), so no
migration script or versioned rollout is needed — flagged here only so
phase-B doesn't reflash a JSON from before this sprint against a new
binary or vice versa. Deployment sequencing: this sprint's own bench
gate (each issue's "Bench gate (required)" section) is explicitly
deferred to the phase-B bench session that follows both 118 and 119, per
stakeholder mandate — record this deferral here rather than block sprint
close on hardware. The firmware telemetry rate halving (50Hz→25Hz
nominal) is a real behavior change any bench-side host tooling assuming
the old rate will need to tolerate when phase-B runs; flagged here so
phase-B isn't surprised by it.

### Step 6 — Design Rationale

**Decision 1: Restore `kCycle=40` (not retune to fit `kCycle=20`).**
*Context:* `5f5a2ba7` zeroed `kSettle`/`kClear` specifically to make an
8ms settle/clear budget fit inside a 20ms cycle; the vendor 4ms settle
still happens, just as a blocking sleep hidden inside `tick()`, defeating
the whole purpose of the `runAndWait` design (visible, budgeted waits).
*Alternatives considered:* (a) keep `kCycle=20`, find some other way to
fit ≥8ms of genuine settle/clear windows plus a non-trivial pace block —
rejected, there is no way to fit two 4ms vendor-mandated settle windows
plus meaningful borrowed work inside a 20ms total without either
under-settling (the current, broken state) or starving the pace block;
(b) restore `kCycle=40` with the full `kSettle=4`/`kClear=4` budget,
matching the last-known-good `39c084c1` skeleton — chosen,
stakeholder-confirmed. *Consequence:* the firmware's real (not aspirational)
control period becomes 40ms/~25Hz; every downstream cadence constant
(telemetry, sim) must follow or a new mismatch replaces the old one.

**Decision 2: Relocate `moveQueue_.tick()` rather than re-derive
`stop_lead_ms`.** *Context:* the review's own R1/R2 frame this as two
separable fixes — remove the avoidable staleness (R1, this sprint) vs.
replace the tuned lead with a derived one or remove the need for it (R2,
originally planned for 119, pulled into this sprint — see Decision 4).
*Alternatives considered:* (a) leave `moveQueue_.tick()` where it is and
re-tune `stop_lead_ms` to compensate for the now-different 40ms cycle's
staleness — rejected, this is exactly the retune-instead-of-remove
pattern the review's R6 condemns, and it would be the FIFTH
`stop_lead_ms` retune in the same number of weeks; (b) move
`moveQueue_.tick()` into the pace block after integration, so the
decision itself is no longer stale — chosen, per the issue's own explicit
"Sequencing: implement AFTER the loop reorder" and "all three relocated
pieces are pure compute" analysis. *Consequence (revised at Decision 4):*
this decision was ORIGINALLY paired with "leave `stop_lead_ms` at 45ms
for this sprint, 119 deletes it later" — that pairing did not survive
contact with real data (Decision 4 below); decision-to-duty latency (the
OTHER cycle of latency, judged unavoidable by the review) is unchanged at
~1 cycle, now measured in 40ms cycles, regardless of Decision 4's outcome.

**Decision 4 (added 2026-07-23, mid-execution amendment): delete
`stop_lead_ms` this sprint rather than defer to 119.** *Context:* Decision
2 above was made with the expectation that `stop_lead_ms=45` would hold
through this sprint's closure gate at the new 40ms cycle, with any
needed adjustment recorded (not a deletion) per the original Out-of-Scope
framing. Ticket 002's actual closure-gate run falsified that expectation:
fresh same-cycle odometry made the unchanged 45ms lead OVERcompensate
(TOUR_1 worst 4.39°, ±90° presets ~93.7°, previously-clean tests now
failing). A 0-120ms sweep against the closure gate's own exact path
(recorded in ticket 002's report and the dated addendum in
`land-at-zero-completion-delete-stop-lead.md`) found only a ~1ms-wide
passing window at ~62ms with no real margin — confirming, with fresh
post-fix data, the same "no single value exists" finding the original
issue's Description already argued from the value's four-retune history.
*Alternatives considered:* (a) record a same-sprint re-baseline (e.g.
62ms) per the ORIGINAL Out-of-Scope allowance — rejected: the sweep found
no value with real margin, so any recorded value would be shipping a
fifth fragile retune, precisely what R6 says not to do when a stage
change forces a retune; (b) leave the closure gate red and hand the fix
to sprint 119 — rejected under the project's sprint-end-must-be-testable
convention (a sprint does not close on a known-red system-level gate);
(c) pull `land-at-zero-completion-delete-stop-lead.md` forward into 118
as ticket 004, deleting `stop_lead_ms` and landing the taper-to-zero
predicate instead of any tuned value — chosen. *Consequence:* ticket 003
(cadence-gate re-baseline) is resequenced to depend on ticket 004 and run
last, so it re-baselines the FINAL regime once instead of the pre-118
regime now and the post-119 regime later; sprint 119 loses its
`land-at-zero-completion-delete-stop-lead.md` and
`turn-error-characterization-postcompensation-tests-need-rewrite-after-lead-deletion.md`
issues (already delivered here) and shrinks to four tickets.

**Decision 3: Sim follows firmware's cadence, not the reverse.**
*Context:* the review's D4 established the 50ms sim value was never a
deliberate simulation-fidelity choice — it existed solely to dodge
`NezhaMotor`'s write-throttle at the firmware's old (regressed) `kCycle=20`.
*Alternatives considered:* (a) leave sim at 50ms, treat every sim-tuned
value as sim-only (status quo, rejected — this is precisely "the sim is
deterministic about a different robot," the review's own words); (b) sync
`kCycleDtUs` to firmware's `kCycle` (now 40) and fix the throttle-margin
interaction that made 50ms necessary in the first place — chosen.
*Consequence:* the throttle's exact-40ms boundary case (hardware timing
jitter making a 39.x ms cycle miss the `<` comparison) needs its own
guard (part of ticket 3), since sim's exact virtual steps can't surface
that hazard — verified by code-review reasoning this sprint, confirmed on
hardware in phase-B.

### Step 7 — Open Questions

- **Derive-vs-hardcode the five cadence constants (sim-cycle issue's own
  "prefer deriving from one exported constant" recommendation).** The
  issue's acceptance criteria only requires "no surviving hardcoded
  0.05/50ms cycle assumption" (a grep gate), not a specific mechanism.
  Ticket 3's implementer may choose the minimal fix (five literal edits
  to 0.04/40000) or the fuller derivation (a shared constant reachable
  from both C++ and Python, e.g. a ctypes export). Not blocking — either
  satisfies the acceptance criteria; the fuller derivation is preferred
  if it fits the ticket's scope without materially growing it.
- **RESOLVED (2026-07-23): whether `stop_lead_ms` needs a same-sprint
  re-baseline.** This question was open when the sprint was first
  planned. Ticket 002's actual run answered it: the closure gate went
  red at the unchanged 45ms value, and a 0-120ms sweep found no value
  with real margin. Resolution is DELETION, not a re-baseline — see
  Decision 4 above and ticket 004.
- **`tlm-rate-15-19hz-vs-50hz-nominal-serial.md`'s nominal.** This
  existing, unrelated issue currently assumes a 50Hz nominal; once this
  sprint ships, the nominal is 25Hz. Not fixed here (out of scope, no
  issue in this sprint owns it) — flagged so a future session doesn't
  read stale numbers in that issue without realizing why they moved.

## Design Overlay

Design-docs opt-in is enabled. Per the flat-overlay-slot precedent
established in sprints 116/117 (`seed_sprint_design_overlay` writes each
seeded file to `design/<canonical_path.name>`, and every co-located
subsystem doc is literally named `DESIGN.md`, so only ONE subsystem-level
`DESIGN.md` can occupy a sprint's overlay directory at a time without a
silent collision), this sprint touches two subsystem `DESIGN.md` files
(`src/firm/app/DESIGN.md`, `src/sim/DESIGN.md`) but can only overlay one.

**Overlaid** (seeded pristine via `seed_sprint_design_overlay(sprint_id="118",
doc_names=["design.md", "src/firm/app/DESIGN.md"])`, edited in place to
describe the post-118 schedule, diffed, and committed on `master` before
`acquire_execution_lock` branches the sprint off it):
- `docs/design/design.md` (system doc) — cadence line (`kCycle = 20 ms`
  → `40 ms`).
- `src/firm/app/DESIGN.md` (co-located) — §4 cadence prose
  (`~50Hz/20ms` → `~25Hz/40ms`), the `cycle()` call-order description in
  §2 (moveQueue_.tick() moves from the R-settle description to the
  trailing pace-block description), retiring the 112-005
  `drive_.tick()`-hoist note. **(Amended, ticket 004)** the same overlay
  file's §1 "118 (loop schedule truth) — landed" note (added for the
  original scope) is updated in place to describe land-at-zero completion
  semantics (taper-to-zero + threshold/timeout backstop) instead of the
  "anticipation lead, `stop_lead_ms` deleted later" framing it was
  originally written with — since `stop_lead_ms` deletion now happens in
  THIS sprint, not 119. `StateEstimator`'s consumer-count-to-zero
  (quarantine) is also reflected here. Owner: ticket 004 (own acceptance
  criterion), same overlay slot as the rest of §1/§2/§4 above — no new
  slot conflict since it's the same file already in the overlay.

**Not overlaid — edited directly on the canonical doc during execution,
by the ticket that owns the change** (same convention as 116/117's
`src/firm/messages/DESIGN.md`/`src/host/robot_radio/DESIGN.md` precedent):
- `src/sim/DESIGN.md` — its own §3/§6 already flag the `kCycleDtUs`
  mismatch as an open question ("does not match the firmware's own
  kCycle... whether to shrink kCycleDtUs to 20ms to match is an open
  call, not decided here"); ticket 3 resolves that Open Question (moving
  it out of §8, since it becomes a resolved-parity statement, not an open
  one) and corrects §2/§6's "not every sim_step() call, currently 50ms"
  and "2.5× step-size difference" language. Owner: ticket 3 (own
  acceptance criterion).

At sprint close, `overlay.apply()` copies the two overlaid files above
onto their canonical targets; `src/sim/DESIGN.md` is already at its
canonical location by then and needs no apply step.

## Use Cases

Sized to the change: originally three sprint-level use cases, one per
issue, tracing the schedule/data-freshness/cadence-parity correctness
properties this sprint restores; **amended (2026-07-23) to four** with
SUC-066 (land-at-zero completion, ticket 004, pulled forward from
sprint 119 — see Decision Record). None of these are new user-visible
behavior — they are internal correctness properties that make the
existing MOVE-completion use cases (SUC-051 chain-advance, the
sim-tour-closure system tests) actually hold at the accuracy the
turn-execution review measured they should.

### SUC-063: Stop decision consumes this-cycle odometry
Parent: SUC-053 (MoveQueue's unconditional per-cycle tick, sprint 116)

- **Actor**: `App::RobotLoop` (internal — no host-visible actor; this is
  a firmware-internal correctness property observed via closure-gate
  accuracy).
- **Preconditions**: An active MOVE with an Angle or Distance stop
  condition is in progress; `Odometry::integrate()` has fresh
  same-cycle encoder samples.
- **Main Flow**:
  1. Both motors' request/collect complete for the cycle (fresh encoder
     samples cached).
  2. `applyOtosSample()` → `Odometry::integrate()` →
     `StateEstimator::update()` run in the trailing pace block, staging
     this cycle's pose/twist.
  3. `MoveQueue::tick()` runs immediately after, in the SAME pace block,
     reading the odometry/estimator state just staged in step 2 — not
     the previous cycle's.
  4. The stop-condition comparison (threshold or timeout) evaluates
     against that same-cycle data; on completion, the queue
     chain-advances the next pending Move or calls `Drive::stop()`.
- **Postconditions**: The stop decision's heading/distance staleness
  relative to physical truth is bounded by within-cycle computation order
  only (≤ a few ms), not a full cycle's worth of motion (was 5.7° at
  cruise on the old 50ms sim cycle; now bounded by same-cycle freshness
  regardless of cycle length).
- **Acceptance Criteria**:
  - [ ] `app_robot_loop_harness`'s ordering test asserts
        `moveQueue_.tick()` reads odometry/estimator state updated in the
        SAME cycle (not the previous one).
  - [ ] Interleave schedule invariants preserved: per-port
        select→settle→collect, no bus traffic in any settle window, I2C
        clearance safety-net fault bit (bit 6) clear during normal
        operation.
  - [ ] Sim tour-closure gate passes at current-or-better per-leg bands.

### SUC-064: Interleaved per-port request→settle→collect loop schedule
Parent: none (restores an invariant `App::RobotLoop`'s own DESIGN.md §3
already asserts — "the timing schedule is exactly `robot_loop.cpp`'s
`runAndWait` calls" — that commit `5f5a2ba7` silently broke)

- **Actor**: `App::RobotLoop` (internal — the firmware maintainer reading
  `robot_loop.cpp`/its DESIGN.md is the use case's real "actor": the code
  must match its own documented contract).
- **Preconditions**: Firmware boots into steady-state `cycle()`.
- **Main Flow**:
  1. `motorL_.requestSample()` (0x46 select port 1).
  2. `runAndWait(kSettle=4, {comms_.pump})` — genuine borrowed-work
     window, no bus traffic in the body.
  3. `motorL_.tick()` — collect L, velocity PID, duty write.
  4. `runAndWait(kClear=4, {updateTlm; tlm_.emit})` — post-duty clearance
     window.
  5. `motorR_.requestSample()` (0x46 select port 2).
  6. `runAndWait(kSettle=4, {processMessage; drive_.tick()})` — R settle
     window; `drive_.tick()` is pure compute, legally borrowed here.
  7. `motorR_.tick()` — collect R, velocity PID, duty write.
  8. `runAndWait(kPace, {applyOtosSample; odom_.integrate; StateEstimator::update;
     moveQueue_.tick; updateLineColor})` — the trailing pace block (SUC-063
     above is this step's own ordering detail).
- **Postconditions**: The 0x46 single-latched-select invariant holds (no
  motor ever reads the other's selected port); the I2C clearance
  safety-net fault bit stays clear during normal driving; the real
  measured cycle period is ~40ms/~25Hz, matching what the schedule's own
  constants claim (not a fiction hidden by a blocking sleep).
- **Acceptance Criteria**:
  - [ ] `robot_loop.cpp`'s `cycle()` matches the 8-step order above
        exactly; `grep 'runAndWait\|sleepUntil' robot_loop.cpp` remains
        the complete list of the firmware's waits.
  - [ ] `kSettle=4`, `kClear=4`, `kCycle=40`; `Telemetry::kPrimaryPeriod=40`.
  - [ ] The two dangling xfail citations of the deleted
        `cycle-order-reorder-experiment-ab-before-hardware.md` (in
        `test_tour_closure_gate.py` and
        `src/tests/sim/unit/test_app_robot_loop.py`) re-point at this
        sprint's `restore-the-interleaved-...` issue.
  - [ ] `app_robot_loop_harness`/`app_telemetry_harness` pass with the
        restored order and constants.
  - [ ] Bench verification (I2C fault bit clear while driving, measured
        ~40ms/~25Hz cycle period) is DEFERRED to phase-B — not required
        for this use case's sim-level acceptance.

### SUC-065: Sim control period matches firmware period
Parent: none (closes an Open Question `src/sim/DESIGN.md` §8 already
names: "whether to shrink `kCycleDtUs` to 20ms to match [is] an open
call, not decided here")

- **Actor**: A test or bench-script author driving the sim (internal
  developer-facing use case, not a robot-operator-facing one).
- **Preconditions**: `SimHarness::kCycle` (firmware) has settled at its
  new value (40, from SUC-064) before this use case's own constant
  change is meaningful.
- **Main Flow**:
  1. `SimHarness::step()` advances `kCycleDtUs` of virtual time, then
     calls `robotLoop_.cycle()` once — `kCycleDtUs` now equals firmware's
     own `kCycle` (40ms), not a 2.5× translation factor away from it.
  2. `NezhaMotor`'s write-rate throttle at exactly 40ms no longer drops an
     on-schedule write to jitter (margin added).
  3. Every previously-hardcoded 50ms/0.05s cadence assumption in the
     C++/Python test and sim-support tree is corrected to 40ms/0.04s (or
     derived from one shared constant).
- **Postconditions**: "N `sim_step()` calls ≈ N firmware cycles" is
  literally true, not approximately true modulo a translation factor; a
  sim-derived timing constant transfers to hardware without adjustment.
- **Acceptance Criteria**:
  - [ ] `SimHarness::kCycleDtUs == 40000`; an assert/test enforces sim
        step period == firmware `kCycle`, not a specific hardcoded number
        on each side independently.
  - [ ] No surviving hardcoded 0.05/50ms cycle assumption anywhere in the
        tree (grep gate) — the five verified sites at minimum:
        `app_robot_loop_harness.cpp` (×2 — own `kCycleDtUs` and
        `plant.tick(0.05f)`), `turn_prediction_capture.py`'s `_CYCLE_S`,
        `test_tour_closure_gate.py`'s `clock.now_s += 0.05`,
        `sim_loop.py`'s `_CYCLE_DURATION_S`.
  - [ ] Full sim suite + closure gate + button acceptance green at 40ms,
        bands unchanged or tightened (never widened without stakeholder
        sign-off).
  - [ ] `src/sim/DESIGN.md`'s own kCycleDtUs-mismatch Open Question
        updated to a resolved-parity statement.
  - [ ] Bench verification (measured TLM period ≈40ms, no duty-write
        drops while driving) is DEFERRED to phase-B — not required for
        this use case's sim-level acceptance.

### SUC-066: Land-at-zero MOVE completion; stop_lead_ms deleted
Parent: SUC-063 (this use case supersedes SUC-063's remaining tail —
SUC-063 removed the odometry staleness `stop_lead_ms` was partly
compensating for; this one removes the need for `stop_lead_ms` at all).
Added 2026-07-23, pulled forward from sprint 119 — see the sprint's
Decision Record.

- **Actor**: `App::MoveQueue` (internal — no host-visible actor; observed
  via closure-gate and isolated-turn accuracy, same as SUC-063/064).
- **Preconditions**: An active MOVE with an Angle or Distance stop
  condition and non-zero `ShaperLimits` (shaping enabled) is in progress;
  `remaining` is computed from this-cycle odometry (SUC-063, already
  landed).
- **Main Flow**:
  1. Each cycle, `VelocityShaper` computes `ω_cmd = √(2·α_decel·(remaining
     − jerkMargin))` — the taper already designed to bring the robot to
     rest AT the target.
  2. `MoveQueue::tick()` evaluates `remaining ≤ ε AND |ω_cmd| ≤ ε_ω`
     (`ε_ω` set just above the ~15mm/s deadband-equivalent floor) as an
     ADDITIONAL completion path alongside the existing `StopCondition`
     threshold/timeout backstop (always evaluated, unchanged).
  3. On land-at-zero completion, `Drive::stop()` stages exact zero
     (bypassing the deadband boost, engaging the rest gate) — no
     predicted-heading lead is computed; there is no tail to predict.
  4. With shaping OFF (all-zero `ShaperLimits`), `shapeAndStage()`
     early-returns, `ω_cmd` never bleeds, and the threshold/timeout
     backstop is the ONLY completion path — byte-identical to
     pre-this-ticket behavior in that regime.
- **Postconditions**: Turn/distance completion is an emergent property of
  the shaper's own taper, not a tuned time-lead guess; `stop_lead_ms`
  does not exist anywhere in `src/` or `data/`; `StateEstimator::bodyAt()`
  has no firmware production consumer (quarantined, module/tests kept).
- **Acceptance Criteria**:
  - [ ] Land-at-zero predicate lives in `MoveQueue::tick()`, not a new
        `StopCondition` Kind.
  - [ ] Shaping-off regime unchanged (threshold/timeout only).
  - [ ] TWIST Angle/Distance only; TIME/WHEELS byte-identical
        (regression-tested).
  - [ ] `ε_ω` set above the deadband-equivalent floor; ~1.7° worst-case
        coast budgeted in the acceptance band.
  - [ ] Full delete list executed in one commit (member/ctor param,
        anticipation block, wire arm field, pydantic model, all three
        robot JSONs + `_estimator_note` blocks, `gen_boot_config.py`,
        `config_sync_allowlist.json` entry).
  - [ ] `StateEstimator`/`bodyAt()` quarantined, not deleted.
  - [ ] `test_turn_error_characterization.py` disposition resolved (not a
        bare xfail flip).
  - [ ] No `stop_lead` string survives in `src/` or `data/` (grep gate).
  - [ ] Sim tour-closure gate green at current bands with `stop_lead_ms`
        deleted (TOUR_1/TOUR_2, ideal/realistic); the two preset tests
        that regressed in ticket 002's addendum pass within their
        existing bands; isolated 90° within ±2° sim-deterministic.
  - [ ] Bench verification DEFERRED to phase-B — not required for this
        use case's sim-level acceptance.

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning document is complete (sprint.md, including its
      Architecture and Use Cases sections)
- [ ] Architecture review passed (or skipped, for changes with no
      architectural impact)
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On | Issue(s) |
|---|-------|------------|----------|
| 001 | Restore the interleaved request-settle-tick loop schedule | — | restore-the-interleaved-request-settle-tick-loop-schedule.md |
| 002 | Stop decision consumes this-cycle odometry (relocate `MoveQueue::tick` into the pace block) | 001 | stop-decision-must-see-this-cycles-odometry.md |
| 004 | Land at zero: complete on remaining≈0 AND ω_cmd≈0; delete stop_lead_ms | 002 | land-at-zero-completion-delete-stop-lead.md, turn-error-characterization-postcompensation-tests-need-rewrite-after-lead-deletion.md |
| 003 | Sim control period parity: `kCycleDtUs=40000`, throttle jitter margin, sweep hardcoded 0.05s cadence assumptions | 001, 002, 004 | sim-cycle-must-match-firmware-period.md |

**Amended (2026-07-23, mid-execution):** ticket 004 (land-at-zero,
pulled forward from sprint 119 — see Decision Record above) inserted
between 002 and 003, and ticket 003's dependency updated to include 004.
Tickets execute serially in the EXECUTION order 001 → 002 → 004 → 003
(not numeric order) — 001 and 002 edit the exact same function
(`RobotLoop::cycle()`) and must not race each other; 004 depends on 002's
odometry freshness and must land before 003's gate re-baseline is
meaningful (003 now re-baselines the FINAL regime — 40ms cycle +
land-at-zero — in one pass instead of two).
