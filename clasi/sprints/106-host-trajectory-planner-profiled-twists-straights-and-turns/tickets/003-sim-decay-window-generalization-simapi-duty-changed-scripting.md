---
id: '003'
title: Sim decay-window generalization (SimApi duty-changed scripting)
status: open
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

- [ ] A new (or existing) sim scenario steps well past the old ~4-cycle safe
      window through a full profile-style deceleration and asserts
      convergence to (approximately) zero velocity with NO bus-error/
      fault-bit artifacts (`connRight`/`connLeft` staying true throughout,
      no false `kFaultWedgeLatch`, `velLeft`/`velRight` tracking the
      expected decay curve rather than freezing).
- [ ] 105-006's existing scripted-twist STOP-phase scenario (which currently
      documents and stays within the old ~4-cycle bound) is left passing
      unchanged, OR is updated to assert full convergence now that the bound
      is lifted — implementer's call, explicitly documented in Completion
      Notes either way.
- [ ] `SimApi`'s public surface (`step`/`injectCommand`/`injectTwist`/
      `injectStop`/`drainTelemetry`/the timing diagnostic) is unchanged —
      this is an internal scripting-helper change only, confirmed by
      inspection (no signature change in `sim_api.h`).
- [ ] `tests/sim/plant/`, `tests/sim/support/`, and `tests/sim/system/` test
      suites stay green.
- [ ] Full project test suite green (`uv run python -m pytest`).

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
