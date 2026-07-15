---
id: '003'
title: Sim decay-window generalization (SimApi duty-changed scripting)
status: done
use-cases:
- SUC-026
depends-on: []
github-issue: ''
issue:
- host-planner-design-lessons-from-drive-v2-review.md
- heading-loop-output-clamp-and-velocity-resonance.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sim decay-window generalization (SimApi duty-changed scripting)

## Description

`clasi/issues/sim-api-multi-write-decay-window.md`:
`TestSim::SimApi::scriptCycleBusResponses()` (`tests/sim/support/
sim_api.cpp`, 105-004) provisions exactly ONE post-command duty write per
injected command, at a single hand-derived `pendingEventCycle_` index. This
is correct for every scenario built through sprint 105 (commanded `|v_x|`
always far above the plant's achievable ceiling, so the PID output stays
saturated at ±1.0 forever once set — "one write, then never again"). A
scenario that lets the PID settle toward a REACHABLE target — exactly what
this sprint's own profile deceleration ramps and `injectStop()` calls do —
drives the PID output back out of saturation as the error shrinks, issuing
SEVERAL more duty writes as the quantized output counts down. None of these
are provisioned by the single-transition script, desyncing the shared
`I2CBus` script FIFO past roughly 4 cycles (verified in 105-006: `connRight`
flipping false, `velLeft` freezing at a wrong value, a false
`kFaultWedgeLatch` trip).

This ticket generalizes the scripting helper to detect an actual
`appliedDuty()` CHANGE per leaf, per cycle, rather than assuming exactly one
transition at one precomputed index — per the issue's own "Direction"
section — so a later sim scenario (ticket 006) can observe a full,
multi-cycle closed-loop settle to (approximately) zero without FIFO
desync artifacts. No dependency on tickets 001/002/004 — pure
test-infrastructure work, independently buildable.

## Acceptance Criteria

- [x] A new (or existing) sim scenario steps well past the old ~4-cycle safe
      window through a full profile-style deceleration and asserts
      convergence to (approximately) zero velocity with NO bus-error/
      fault-bit artifacts (`connRight`/`connLeft` staying true throughout,
      no false `kFaultWedgeLatch`, `velLeft`/`velRight` tracking the
      expected decay curve rather than freezing).
- [x] 105-006's existing scripted-twist STOP-phase scenario (which currently
      documents and stays within the old ~4-cycle bound) is left passing
      unchanged, OR is updated to assert full convergence now that the bound
      is lifted — implementer's call, explicitly documented in Completion
      Notes either way.
- [x] `SimApi`'s public surface (`step`/`injectCommand`/`injectTwist`/
      `injectStop`/`drainTelemetry`/the timing diagnostic) is unchanged —
      this is an internal scripting-helper change only, confirmed by
      inspection (no signature change in `sim_api.h`).
- [x] `tests/sim/plant/`, `tests/sim/support/`, and `tests/sim/system/` test
      suites stay green.
- [x] Full project test suite green (`uv run python -m pytest`).

## Testing

- **Existing tests to run**: full `tests/sim/` suite (`uv run python -m
  pytest tests/sim/`), especially every 105-001..006 scenario that exercises
  `scriptCycleBusResponses()` (boot, twist-ramp, stop, deadman-expiry, fault
  injection).
- **New tests to write**: a decay-to-zero scenario (new, or an extension of
  105-006's own STOP-phase scenario) that steps past the old 4-cycle bound
  and asserts exact/near-zero convergence with no fault artifacts.
- **Verification command**: `uv run python -m pytest tests/sim/ -v`.

## Implementation Plan

**Approach**: In `tests/sim/support/sim_api.cpp`'s
`scriptCycleBusResponses()`, replace the single `pendingEventCycle_`-index
comparison with a per-leaf, per-cycle check of whether `appliedDuty()`
differs from the value scripted for the PREVIOUS cycle — scripting a fresh
write only when it actually changed, driven dynamically each cycle instead
of assumed at one hand-derived index. Decide (and document) whether the
existing `notePendingActuationChange()` entry point (used by the
deadman-expiry scenario, which provokes a change with no fresh command)
still needs to exist alongside the new dynamic detection, or is superseded
by it.

**Files to modify**:
- `tests/sim/support/sim_api.{h,cpp}` — `scriptCycleBusResponses()` and its
  private per-leaf "last scripted duty" tracking state.

**Files to create**: a new decay-to-zero sim scenario under
`tests/sim/system/` (or an extension of 105-006's existing scripted-twist
harness), per the first Acceptance Criterion.

**Testing plan**: every existing 105-001..006 sim scenario must keep
passing unchanged (this ticket generalizes the scripting mechanism, it does
not change any scenario's own expected behavior); the new decay-to-zero
scenario is the proof the extended window actually works.

**Documentation updates**: `clasi/issues/sim-api-multi-write-decay-
window.md`'s status updated to resolved once merged, referencing this
ticket.

## Completion Notes

**Mechanism**: `SimApi::scriptCycleBusResponses()`'s single hand-derived
`pendingEventCycle_` index is replaced by `SimApi::DutyPredictor` (a private
nested class, one instance per leaf: `predictorLeft_`/`predictorRight_`),
which runs a minimal, deliberately-scoped replica of the pieces of
`NezhaMotor::tick()` (+ `Devices::MotorVelocityPid::compute()` +
`Devices::MotorArmor::armoredWrite()`) that decide whether a duty write
actually reaches the bus this cycle, under this harness's own fixed gains
(pure-P, `kp=0.01`). `tickPredict(position, cycle)` is called once per leaf,
per cycle, BEFORE `robotLoop_.cycle()` runs, using the exact position value
about to be scripted onto the bus from `WheelPlant::position()`; it returns
whether a SECOND (duty) write should be scripted this cycle, in addition to
the steady-state request write.

Target staging (`injectTwist()`/`injectStop()`/`notePendingActuationChange()`,
all now routed through a new private `stageActuationChange(atCycle, vL, vR)`)
carries the REAL staged wheel-velocity target (via `BodyKinematics::inverse()`
for `injectTwist()`, always `(0,0)` for `injectStop()`/`notePendingActuationChange()`
since every caller of the latter today is an autonomous `App::Drive::stop()`),
applied at the same R(this cycle)/L(next cycle) asymmetric offset the old
mechanism already derived from `App::Drive::tick()`'s own call-order (it runs
between `motorL_.tick()` and `motorR_.tick()` within one `cycle()` call).
`notePendingActuationChange(int atCycle)`'s public signature is UNCHANGED
(AC #3) — it is retained, not superseded, because something still has to
tell the dynamic predictor a target changed with no injected command to
point at (the deadman-expiry scenario).

**Two bugs found and fixed during verification** (both via a throwaway
stderr trace comparing predicted vs. real per-cycle firmware state — removed
before this commit, not left in the tree):

1. An early version gated the encoder-freshness/velocity tracking on the
   same `active_` (mode-transition) flag as the PID/write dispatch. This is
   wrong: `NezhaMotor::tick()`'s steps 1-3 (including the freshness anchor)
   run UNCONDITIONALLY every tick() call, regardless of `mode_` — only step
   4's dispatch switch is mode-gated. Gating the anchor too pushed the
   predictor's own freshness anchor one cycle late for the leaf whose
   activation lags (L), desyncing every `freshElapsed` computed after that
   leaf's first real velocity change. Fixed by running the freshness update
   unconditionally and gating only the PID/write dispatch.
2. The predictor initially replicated only `writeRawDuty()`'s own
   write-on-change/slew/stop-exemption logic, missing
   `Devices::MotorArmor::armoredWrite()`'s reversal-dwell + output-deadband
   gate that sits BETWEEN the PID's raw output and `writeRawDuty()`. A duty
   SIGN FLIP (exactly what STOP-after-forward-ramp produces) does not reach
   `writeRawDuty()` as computed — it is forced to zero for a
   `kDefaultReversalDwell`=100ms (2-cycle) dwell window first. Missing this
   layer caused the predictor to script a write on a cycle where the real
   firmware's write-on-change gate actually suppressed one (a repeated
   forced-zero), an extra scripted entry that desynced the shared write FIFO
   a cycle later — the exact `connLeft` flipping false / stuck `encLeft`
   signature the original issue described. Fixed by porting
   `armoredWrite()`'s dwell/deadband state machine into `DutyPredictor`
   (`dwelling_`/`dwellDeadline_`/`lastRequestedDuty_`), using a cycle-derived
   `nowMs` (`cycle * (kCycleDtUs/1000)`) since only relative now-vs-deadline
   comparisons matter, not the absolute clock value.

**Strengthened assertion**: `scripted_twist_demo_harness.cpp`'s STOP phase
(105-006) is UPDATED (not left unchanged) — `kStopCycles` raised from 4 to
12 (3x the old bound), and the assertion changed from "velocity dropped
>=25% from peak within the safe window" to full convergence: both
`velLeft`/`velRight`'s last observed sample must be within `kConvergedVelocity`
(5mm/s) of exactly zero. Verified against the real compiled harness (not
just reasoned about) at multiple window sizes, including stepping the
scenario out to 60+ cycles to directly observe the SAME scenario's converged
residual eventually and correctly trip a REAL (not false) `kFaultWedgeLatch`
once `Devices::MotorArmor`'s own `kWedgeThreshold` (10 consecutive identical
tenths-of-mm encoder reads — the same "boundary-latch" flavor
`.clasi/knowledge/encoder-wedge-boundary-latch.md` documents for real
hardware) accumulates from the converged residual's own encoder quantization
stall. 12 cycles lands comfortably before that trip point while proving
genuine multi-write convergence, not a coincidental pass.

**Issue resolution**: `clasi/issues/sim-api-multi-write-decay-window.md` is
fully resolved by this ticket (the "Direction" section's own proposed fix —
detect `appliedDuty()`/duty-write changes dynamically per cycle — is exactly
what `DutyPredictor` implements) and is archived to done alongside this
ticket.

**Test totals**: `uv run python -m pytest tests/sim/` — 347 passed. Full
project suite `uv run python -m pytest` — 569 passed. No regressions in any
existing 105-001..006 scenario (fault-knob scenarios in particular stay
correct because every one of them keeps its commanded target far enough
above the plant's ceiling that the PID stays saturated regardless of the
measured value — `DutyPredictor` reaches the same write/no-write answer
whether or not it models the fault knobs' effect on the SCRIPTED read, which
it deliberately does not, documented inline in `sim_api.h`).

**Note on `source/app/robot_loop.{cpp,h}`**: these files show as modified in
the working tree (an unrelated extract-method refactor, apparently from a
concurrent session on this shared checkout) but were NOT touched by this
ticket's own work and are NOT included in this ticket's commit.
