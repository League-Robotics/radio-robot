---
id: '074'
title: 'OTOS fusion recovery and health visibility: fusion-gate re-admission, bench-OTOS-sim,
  wire-surfaced OTOS health'
status: done
branch: sprint/074-otos-fusion-recovery-and-health-visibility-fusion-gate-re-admission-bench-otos-sim-wire-surfaced-otos-health
use-cases: []
issues:
- otos-not-used-frozen-pose-ekf-rejects-everything.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 074: OTOS fusion recovery and health visibility: fusion-gate re-admission, bench-OTOS-sim, wire-surfaced OTOS health

## Goals

Make the OTOS fusion-gate failure mode described in
`otos-not-used-frozen-pose-ekf-rejects-everything.md` recoverable and
diagnosable without hardware: close the actual code-review-confirmed gaps
in the live fusion path, give host-sim real bench-OTOS parity with
firmware, and surface OTOS health on the wire so this class of failure is
never silent again.

## Problem

TLM `otos=` froze for an entire session (on the stand and while driving
500mm on the playfield) while `ekf_rej` climbed on almost every tick,
leaving the EKF running encoder-only with no cross-check. Bench mode
(`DBG OTOS BENCH`), meant to let this be exercised without hardware, shows
the same frozen signature instead of simulating motion. Code review (this
sprint's investigation, documented in full in `architecture-update.md`)
found the issue's own named suspects do not hold up:

- `Robot::otosCorrect()` -- the function the issue points at -- has had
  zero live callers since the ordered-tick cutover (sprint 060). The live
  path is `Drive::tickUpdate()` STEP 5.
- The CR-06 warn-persistence gate (sprint 065) already re-admits fusion
  after a clean-tick run; it is not latched forever, and this is already
  regression-tested.

The actual defects: (1) `Drive::_otos` is a reference bound once at boot
and never re-seated, so `DBG OTOS BENCH` has no effect on the live
fusion/telemetry path; (2) `SimHardware` has no bench-otos object to swap
to at all, so bench mode is a no-op in sim/TestGUI-sim too; (3) the CR-06
gate only watches the OTOS chip's self-reported STATUS byte, not whether
the pose value is actually changing, so a readable-but-stuck reading with a
clean STATUS byte sails through and gets fused every tick -- the only
explanation that accounts for both a frozen `otos=` and a climbing
`ekf_rej` at once.

## Solution

Four changes, all inside three existing modules:

1. Give `SimHardware` real bench-OTOS parity with `NezhaHAL`/`MecanumHAL`
   (a real `BenchOtosSensor` member, a real HAL-level pointer swap,
   reachable via a new `Hardware::benchOtosPtr()` used uniformly by
   `DebugCommands` across all builds).
2. Fix `Drive` to read the active odometer live via `Hardware::otos()`
   every tick, instead of a construction-time-bound reference -- the same
   indirection `Robot::otosCorrect()` already (uselessly) implements.
3. Extend the existing CR-06 fusion gate with a value-staleness check
   (readable + status-clean + pose not changing while encoders show
   motion), reusing the existing block/re-admit state machine unchanged.
4. Add one additive, always-visible TLM field (`otos_health=`) reporting
   the raw STATUS byte and the fusion-blocked state every frame, and
   document what `otos=` itself reflects.

Full design, diagrams, and rationale in `architecture-update.md`.

## Success Criteria

- A sim test toggles `DBG OTOS BENCH 1` mid-session and shows the live
  fusion/telemetry path switch to the bench sensor's simulated motion.
- A sim test drives the fusion gate into a stuck-value block and back to
  healthy, and confirms `ekf_rej` stops climbing while blocked and fusion
  resumes on re-admission.
- TLM gains `otos_health=<status>,<blocked>`, golden-TLM regenerated, host
  `parse_tlm` updated.
- `otos=` semantics are documented and a regression test confirms no
  stale-cache-masks-a-read-failure defect exists.
- Full existing suite (~2672 baseline) still passes; no existing OTOS-gate
  or golden-TLM test needs behavior changes beyond the golden capture
  regeneration itself.

## Scope

### In Scope

- `SimHardware` bench-OTOS HAL parity + `Hardware::benchOtosPtr()`.
- `Drive`'s live-OTOS indirection fix.
- CR-06 gate value-staleness hardening (reusing existing state machine).
- `otos_health=` wire field + golden-TLM regeneration + host parser update.
- Documentation of `otos=` semantics + a read-failure regression test.
- Marking `Robot::otosCorrect()` as documented dead code (comment only).

### Out of Scope

- The physical hardware's actual root cause (I2C fault vs. REG_OFFSET vs.
  sensor stall) -- requires HIL, explicitly deferred (see Open Questions in
  `architecture-update.md`).
- Deleting `Robot::otosCorrect()` and its `EVT otos lost` emission --
  flagged as a stakeholder decision for a future cleanup sprint.
- Retiring the now-redundant standalone `SimHandle::benchOtos` ctypes test
  hooks -- left in place, flagged as a future cleanup candidate.
- Any host-side alerting/UI built on top of the new health field.

## Test Strategy

Sim-only (no hardware this sprint). New/updated sim regression tests for:
bench-OTOS-sim tracking commanded motion after a `DBG OTOS BENCH` toggle;
the live-indirection fix (fails pre-fix, passes post-fix, built on the
bench-parity substrate); the stuck-value gate (blocks + re-admits, reusing
the existing STATUS-bit test's shape); a read-failure regression for
`otos=`'s freshness gate. Golden-TLM capture regenerated and its existing
test re-run against the new capture. Full baseline suite re-run to confirm
no regression outside the touched files.

## Architecture Notes

See `architecture-update.md` for the full 7-step design, three Mermaid
diagrams, five Design Rationale decisions, and the Architecture Self-Review
verdict (APPROVE WITH CHANGES). Key constraint carried into ticketing:
`Drive`'s constructor signature changes (`IOdometer&` -> `Hardware&`); the
sole production call site is `Robot.cpp`, confirmed by grep during
planning.

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | SimHardware bench-OTOS parity | — |
| 002 | Drive live-OTOS indirection | 001 |
| 003 | Fusion-gate stuck-value hardening | 002 |
| 004 | OTOS health on the wire | 003 |

Tickets execute serially in the order listed.
