---
id: 092
title: 'Motion & OTOS hardware fixes: bounded stop-decel seed correction, hardware
  pose-fusion investigation, and OTOS SparkFun library port'
status: done
branch: sprint/092-motion-otos-hardware-fixes-bounded-stop-decel-seed-correction-hardware-pose-fusion-investigation-and-otos-sparkfun-library-port
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
issues:
- d-t-terminal-reverse-persists-decel-reseed-from-plan-velocity.md
- poseestimator-fused-pose-frozen-on-hardware.md
- otos-lever-arm-necessity-and-library-port.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 092: Motion & OTOS hardware fixes: bounded stop-decel seed correction, hardware pose-fusion investigation, and OTOS SparkFun library port

## Goals

Close out the three hardware-touching pool issues left over from sprint
089's own bench pass: (1) finish the D/T terminal-reverse-motion fix that
089-007 found only partially resolved the confirmed bug, (2) root-cause and
fix `PoseEstimator`'s fused pose being frozen on real hardware (blocking
`TURN`/`G` completion), and (3) faithfully port the upstream SparkFun OTOS
library and re-test whether the chip's `REG_OFFSET` register removes the
need for host-side lever-arm compensation. This is the hardest, most
hardware-dependent sprint of the current batch.

## Problem

Sprint 089 migrated `D`/`T`/`R`/`S`/`TURN`/`RT` onto a Ruckig-backed jerk-
limited planner specifically to eliminate a confirmed terminal reverse-
spin/overshoot bug. The bench pass (089-007) showed the reverse motion
reduced (from ~16-23mm to 11-23mm) but not eliminated: the stop-triggered
decel re-solve seeds from the plan's own believed velocity
(`Motion::JerkTrajectory::sample()`), while the real, loosely-tracking PID
runs the wheel faster than the plan believes — so the decel is seeded too
low and the PID brakes the still-fast wheel into reverse creep. Separately,
that same bench pass found `Subsystems::PoseEstimator`'s fused pose frozen
on hardware across 1.3+ m of real encoder travel (a pre-existing defect,
unrelated to the Ruckig migration, that blocks `TURN` and `G` from
completing on the stand). Independently, a 2026-07-07 stakeholder design
review of `source/hal/lever_arm.h` raised doubt that the OTOS chip's own
`REG_OFFSET` mounting-offset register is actually unwritable (the prior
claim), suspecting instead that this project's hand-rolled OTOS driver is
incomplete relative to the upstream SparkFun reference implementation.

## Solution

1. **D/T bounded stop-decel seed correction** — a narrowly-scoped, bounded
   correction toward the measured wheel velocity, applied ONLY at the
   stop-triggered decel-arm handoff (`armDistanceStopDecel()`/
   `armVelocityStopDecel()`/`armRotationalStopDecel()`), never at the
   routine per-tick sample or the goal-start solve — so it does not reopen
   089 Decision 8's general "never seed from measured state" contract (the
   contract that avoids the 087-009 limit-cycle bug), only extends it with
   a third, tightly-guarded, one-shot exception at exactly the handoff where
   089-007 found the residual defect.
2. **PoseEstimator hardware pose-fusion fix** — code investigation
   comparing the sim path (which works) against the real hardware path
   (which does not), landing the most plausible fix, with sim regression
   coverage as the acceptance gate (sim cannot reproduce the defect itself).
3. **OTOS SparkFun library port** — port the upstream driver near line-by-
   line into `Hal::OtosOdometer` (register map, scaling, `setOffset`/
   `getOffset`, signal-process config, IMU calibration), then bench-re-test
   `REG_OFFSET`; finalize `source/hal/lever_arm.h` into exactly one end
   state (deleted if the chip honors the register, folded into
   `OtosOdometer` otherwise — folding is the default when the bench cannot
   run).

## Success Criteria

- `uv run python -m pytest tests/sim` is green throughout, including new
  coverage proving (a) the bounded stop-decel correction eliminates
  reverse-creep under an injected real-plant-faster-than-plan divergence
  without reopening the 087-009 limit-cycle signature, (b) `PoseEstimator`'s
  existing sim coverage is unregressed, and (c) the ported OTOS driver's new
  surface is unit-tested.
- Each ticket's bench step is attempted and its outcome (pass, reduced-but-
  not-eliminated, or unreachable) is recorded honestly; no ticket blocks
  sprint close solely because a bench step could not be completed — descope
  to a fresh `clasi/issues/` follow-on instead.
- `source/hal/lever_arm.h` ends the sprint in exactly one of its two valid
  end states (deleted or folded into `OtosOdometer`), never standalone.

## Scope

### In Scope

- The bounded stop-decel seed-correction mechanism for `D`/`T`/`TURN`/`RT`
  (linear and rotational `Motion::JerkTrajectory` channels).
- `PoseEstimator` hardware fused-pose root-cause investigation and fix.
- The OTOS driver's SparkFun library port and its unit test coverage.
- The `REG_OFFSET` bench re-test and `lever_arm.h`'s final disposition.

### Out of Scope

- `relay-round-trip-bench-verification.md` and
  `watchdog-motors-gate-radio-bench-verification.md` (both pool issues) —
  both need the relay dongle, which is unplugged this sprint. Left in the
  pool for a sprint where the relay is available.
- Any general re-tuning of the velocity PID (issue 1's option (b)) or a
  stakeholder-accepted terminal-tolerance decision (option (c)) — this
  sprint designs and ships option (a), the bounded correction, as primary;
  see Architecture Notes for the flag-for-stakeholder-decision path if (a)
  cannot be proven safe in sim.
- `GOTO_GOAL` (`G`)'s own motion-generation mechanism — untouched, per
  089 Decision 5.

## Test Strategy

Sim is the blocking gate for every ticket in this sprint (`uv run python -m
pytest tests/sim`). Bench verification (robot on the stand, direct USB
serial — the relay dongle is unplugged) is secondary and best-effort per
ticket, with an explicit descope-to-follow-on-issue path if a bench step
cannot be completed or the robot wedges/latches — mirroring the honest
descope pattern sprint 089 ticket 007 already used. The D/T fix specifically
requires a sim test that INJECTS the real-plant-faster-than-plan divergence
condition (via synthetic Planner-tier observations, mirroring 089-006's
approach) since the sim's idealized plant cannot naturally reproduce it.

## Architecture Notes

See `architecture-update.md` for the full design. Key constraint: the D/T
fix must not reopen sprint 089's own documented 087-009 limit-cycle bug
class (measured-velocity feedback closing a loop through the plant's own
delayed response) — the bounded correction is scoped as a single, one-shot,
capped nudge at the decel-arm instant, never a persistent per-tick feedback
path. If sim testing cannot prove this is safe, the ticket flags it for a
stakeholder decision rather than shipping a blind control change.

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan (autonomous auto-approve mode
      — see `architecture-update.md`'s gate notes; no live human review this
      pass)

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | D/T/TURN/RT bounded stop-decel seed correction | — |
| 002 | PoseEstimator hardware fused-pose investigation and fix | — |
| 003 | OTOS driver: faithful SparkFun library port | — |
| 004 | OTOS REG_OFFSET bench re-test and lever-arm disposition | 003 |

Tickets execute serially in the order listed. 001 and 002 are mutually
independent (different subsystems) and could execute in either order;
001 is listed first only because its root cause is already fully traced
(Grounding), making it the most immediately actionable. 003 must precede
004 (004 needs `setOffset()`/`getOffset()` to exist before it can
bench-re-test `REG_OFFSET`).
