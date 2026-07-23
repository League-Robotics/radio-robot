---
id: '118'
title: 'Loop schedule truth: firmware loop reorder + sim cadence parity'
status: planning-docs
branch: sprint/118-loop-schedule-truth-firmware-loop-reorder-sim-cadence-parity
worktree: false
use-cases: ['SUC-063', 'SUC-064', 'SUC-065']
issues:
- restore-the-interleaved-request-settle-tick-loop-schedule.md
- stop-decision-must-see-this-cycles-odometry.md
- sim-cycle-must-match-firmware-period.md
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

## Success Criteria

- Firmware and sim build green.
- Full `uv run python -m pytest` suite green.
- Sim tour-closure gate and button-acceptance suite green at the 40ms
  period, per-leg bands unchanged or tightened (never silently widened).
- `grep 'runAndWait\|sleepUntil' src/firm/app/robot_loop.cpp` still the
  complete list of the firmware's waits (existing invariant, re-verified
  post-reorder).
- No surviving hardcoded 0.05s/50ms cycle assumption anywhere in the tree
  (grep gate, per the sim-cycle issue's own acceptance criteria).
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

### Out of Scope

- Anticipation-lead removal / land-at-zero completion semantics — this
  sprint deliberately leaves `stop_lead_ms`/anticipation AS-IS (45ms);
  deleting it is sprint 119's job (`land-at-zero-completion-delete-stop-lead.md`),
  which explicitly sequences itself after this sprint's odometry-freshness
  fix. If the new 40ms cycle shifts closure-gate numbers enough to demand
  it, `stop_lead_ms` may be re-baselined (not deleted) this sprint, and
  the reason recorded in the owning ticket.
- The silent-off shaping/anticipation config boundary
  (`kill-the-silent-off-shaping-config-boundary.md`), the leg hand-off
  contract, the config-attic deletion, and the doc-relocation sweep — all
  sprint 119.
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

### Step 4 — Diagrams

**No component/module diagram.** This sprint recomposes nothing: it
resequences calls within one existing function
(`RobotLoop::cycle()`) and synchronizes three already-existing cadence
constants (`kCycle`, `kPrimaryPeriod`, `kCycleDtUs`) across modules that
already depended on each other in exactly this call shape before this
sprint. No new module, no new edge, no dependency-direction change (same
justification pattern as sprint 020's architecture doc: a diagram would
show the identical graph before and after). No ERD — no data-model
change. No dependency graph — no module dependency changes, only the
cadence at which existing dependencies fire.

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

**Why:** Per Problem above — restores the schedule to what its own design
doc already claims it is, removes a full cycle of avoidable stop-decision
staleness, and makes sim-measured timing transferable to hardware.

**Impact on Existing Components:** `App::MoveQueue`'s completion latency
changes shape (decision now same-cycle-fresh; decision-to-duty write is
still ~1 cycle, now 40ms not 20ms — the review's own §7-R1 calls this
acceptable once 119's land-at-zero taper lands). `App::Telemetry`'s frame
rate halves (50Hz nominal → 25Hz) — a real, visible change to anyone
polling telemetry at the old assumed rate; `tlm-rate-15-19hz-vs-50hz-nominal-serial.md`
(existing, unrelated issue) will need its own nominal re-measured, noted
there per the sim-cycle issue's own acceptance criteria, not fixed here.
`Sim::SimHarness` step semantics: one `step()` call now advances exactly
one firmware cycle's worth of virtual time at the SAME period firmware
runs at (previously 2.5× coarser) — any caller reasoning about
"N sim_step() calls ≈ N firmware cycles" becomes literally true instead
of approximately true modulo a translation factor.

**Migration Concerns:** No data/schema migration (numeric timing
constants only, no wire-format or persisted-config change). Deployment
sequencing: this sprint's own bench gate (each issue's "Bench gate
(required)" section) is explicitly deferred to the phase-B bench session
that follows both 118 and 119, per stakeholder mandate — record this
deferral here rather than block sprint close on hardware. The firmware
telemetry rate halving (50Hz→25Hz nominal) is a real behavior change any
bench-side host tooling assuming the old rate will need to tolerate when
phase-B runs; flagged here so phase-B isn't surprised by it.

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
119's job). *Alternatives considered:* (a) leave `moveQueue_.tick()` where
it is and re-tune `stop_lead_ms` to compensate for the now-different
40ms cycle's staleness — rejected, this is exactly the retune-instead-of-
remove pattern the review's R6 condemns, and it would be the FIFTH
`stop_lead_ms` retune in the same number of weeks; (b) move
`moveQueue_.tick()` into the pace block after integration, so the
decision itself is no longer stale, leaving `stop_lead_ms` at its current
45ms value for THIS sprint (119 deletes it) — chosen, per the issue's own
explicit "Sequencing: implement AFTER the loop reorder" and "all three
relocated pieces are pure compute" analysis. *Consequence:* decision-to-
duty latency (the OTHER cycle of latency, judged unavoidable by the
review) is unchanged at ~1 cycle, now measured in 40ms cycles.

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
- **Whether `stop_lead_ms` needs a same-sprint re-baseline.** Out of
  Scope states it stays at 45ms unless the 40ms cycle shift demands
  otherwise for the closure gate to hold. This is a real possibility (the
  review's own timeline data assumed a 50ms sim cycle) — ticket 3's
  closure-gate re-run will surface whether this is needed; if so, the
  ticket records the new value and why, without deleting the field
  (119's job).
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
  `drive_.tick()`-hoist note.

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

Sized to the change: three sprint-level use cases, one per issue, tracing
the schedule/data-freshness/cadence-parity correctness properties this
sprint restores. None of these are new user-visible behavior — they are
internal correctness properties that make the existing MOVE-completion
use cases (SUC-051 chain-advance, the sim-tour-closure system tests)
actually hold at the accuracy the turn-execution review measured they
should.

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
| 003 | Sim control period parity: `kCycleDtUs=40000`, throttle jitter margin, sweep hardcoded 0.05s cadence assumptions | 001, 002 | sim-cycle-must-match-firmware-period.md |

Tickets execute serially in the order listed (`worktree: false`) — 001
and 002 edit the exact same function (`RobotLoop::cycle()`) and must not
race each other; 003 re-baselines gates that are only meaningful once
both loop-schedule changes are in.
