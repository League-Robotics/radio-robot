---
id: '006'
title: 'Cross-boundary carry: boundary-velocity table + divergence replan triggers'
status: done
use-cases:
- SUC-003
depends-on:
- '005'
github-issue: ''
issue: firmware-jerk-limited-motion-ruckig-return-arc-command-queue.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Cross-boundary carry: boundary-velocity table + divergence replan triggers

## Description

This ticket adds the "no decel between same-vmax commands" requirement —
the reason a fixed-depth queue exists at all. Without it, every multi-leg
tour decelerates to a stop at each leg boundary even when two consecutive
legs are compatible (same direction, same/similar `v_max`). This ticket
also lands the full replan-trigger table from the issue (enqueue-adjacent,
replace, divergence, handoff, STOP) — ticket 003 only implemented the
`replace` trigger for TIMED mode; this ticket completes the set for
DISTANCE-mode chains.

1. One-command-lookahead boundary velocity: hand Ruckig's position
   interface a nonzero `target_velocity` (already verified supported at
   c63ec6c, per the issue) computed by:
   ```
   exitSpeed(active, next):
     none, sign reversal, or pivot on either side -> 0
     else ve = min(vmaxEff(active), vmaxEff(next))
     if next is DISTANCE: ve = min(ve, reachableEntrySpeed(|next.distance|))
     reachableEntrySpeed(d) = -k + sqrt(k^2 + 2*aDecel*d), k = aDecel^2/(2*jerk)
       // jerk==0: sqrt(2*aDecel*d)
   ```
   Pivot→pivot chains carry rotational velocity through the same code
   path (rotational domain) — only rest-terminated pivots get the
   servoed dwell landing (ticket 005 already restricts dwell to the
   final pivot in a chain; this ticket is what actually produces the
   non-dwelling handoff for intermediate pivots at nonzero velocity).
2. Replan triggers, each re-solving the affected channel seeded from the
   channel's OWN last sample — never measured sensors (the 087-009
   limit-cycle contract is inviolable, per the issue):
   - (a) enqueue adjacent to active, exit speed changes > 1 mm/s →
     `retarget(remaining)` with new target velocity.
   - (b) replace — tail: as (a); active: in-place `solveToVelocity`
     (TIMED) or full re-activate from moving state.
   - (c) divergence — thresholds verbatim: 5 mm retarget / 40 mm reanchor
     linear, 0.3 rad reanchor rotational, 60 ms min interval; reanchor is
     the one sanctioned measured-state seed, accel forced 0.
   - (d) handoff — activate next, velocity-continuous by construction.
   - (e) STOP — flush queue (`FLUSHED` events) + `solveToVelocity(0)`
     both channels.
3. State machine completion: `RAMP_TO_REST` (empty queue at speed) accepts
   a mid-decel enqueue with moving-state replan, per the issue's state
   diagram (`IDLE -> RUNNING -> (handoff self-loop) -> RAMP_TO_REST ->
   IDLE`).

## Acceptance Criteria

- [x] `exitSpeed(active, next)` implemented exactly per the formula above,
      including the `reachableEntrySpeed` jerk==0 sentinel branch.
      (`Executor::computeExitVelocity()`/`reachableEntrySpeed()`,
      `executor.cpp`.)
- [x] Two same-`v_max`, same-direction, non-pivot DISTANCE commands
      execute with velocity never dipping below `v_max * (1 - epsilon)`
      at the shared boundary (sim system test — this is the sprint's
      headline "no decel between same-vmax commands" requirement).
      `src/tests/sim/system/boundary_velocity_harness.cpp` Scenario 1
      (`min velocity >= vMax*0.9` within 2 cycles of the boundary),
      `test_boundary_velocity.py`.
- [x] Sign reversal, pivot-adjacent, or "no successor" cases correctly
      force `exitSpeed = 0` (decelerate to rest at the boundary).
      Scenarios 2/3 in `boundary_velocity_harness.cpp`.
- [x] Pivot→pivot chains carry rotational velocity through the same
      boundary-velocity code path; only the final pivot in a chain
      dwells (ticket 005's dwell restriction is now actually exercised
      by a real non-dwelling handoff). Scenario 4 in
      `boundary_velocity_harness.cpp`.
- [x] All five replan triggers (a-e) implemented; each re-solve seeds from
      the channel's own last sample, never from measured sensors, except
      trigger (c)'s reanchor (the one sanctioned exception, with accel
      forced to 0). (a)/(b)-tail: `maybeRetargetActiveForSuccessorChange()`;
      (b)-active: existing 109-003/109-005 `activate(cmd, retarget=true)`
      path; (c): `checkDivergence()`/`plan()`'s reanchor/retarget dispatch;
      (d): `activateNextOrIdle()`'s velocity-continuous seed; (e): STOP is
      UNCHANGED by deliberate choice — ticket 003 already established
      (and this ticket's own acceptance criteria are silent on changing
      it) that the wire STOP path stays an immediate `Drive::stop()` for
      safety, with `Pilot::flush()` alongside it; see
      `src/firm/motion/DESIGN.md` §2d's own "STOP (trigger (e)) is
      unchanged" note.
- [x] Divergence thresholds match the issue verbatim (5 mm retarget /
      40 mm reanchor linear, 0.3 rad reanchor rotational, 60 ms min
      interval between reanchors). One addition beyond the verbatim
      table, NOT a deviation from it: the 5mm linear retarget tier also
      requires 3 consecutive ticks past threshold before acting (an anti-
      transient guard added after catching a real accuracy regression
      from reacting to ordinary velocity-PID ramp-lag — see
      `src/firm/motion/DESIGN.md` §2d's own "Two bugs this ticket caught").
- [x] `RAMP_TO_REST` accepts a mid-decel enqueue with a moving-state
      replan (does not require returning to full rest first).
      `enqueue()`'s own `(state_==kIdle || state_==kRampToRest) &&
      queueCount_==0` condition.
- [x] `src/firm/motion/DESIGN.md` updated with the boundary-velocity table
      and replan-trigger table (this is exactly the kind of module-level
      design detail the doc should carry, per its role as the persistent
      design record for this subsystem). New §2d; `app/DESIGN.md`'s own
      `kDeadTime` Open-Questions entry updated to match the actual (non-
      live) outcome.
- [x] Bench: two-command no-decel run on the stand (per `.claude/rules/
      hardware-bench-testing.md`) — visually and via encoder/OTOS trace,
      confirms no dip to zero between two compatible legs. **Deferred,
      per explicit dispatch instruction this session** ("Bench: deferred
      (USB deploy broken; no attempts needed this ticket)") — consistent
      with ticket 005's own already-documented finding that
      `mbdeploy deploy --build` fails with an ambiguous-device-registry
      error this session (ticket 005's own Bench note). No new
      `mbdeploy probe` attempt was made this ticket since 005 already
      exhausted the one-attempt escalation path this session. The
      sim-level equivalent (`boundary_velocity_harness.cpp` Scenario 1 +
      `test_heading_source.py`'s own ideal-plant accuracy gate, both
      green) is the acceptance evidence available this session; a real
      bench run remains outstanding until USB deploy is fixed.

## Testing

- **Existing tests to run**: ticket 005's arc/pivot/dwell tests (must
  remain passing); ticket 003's TIMED-mode replace-trigger test (still
  valid, now alongside the DISTANCE-mode triggers).
- **New tests to write**: two same-`v_max` DISTANCE commands, no inter-
  command decel (velocity never dips below `v_max * (1 - epsilon)`);
  sign-reversal / pivot-adjacent forces `exitSpeed = 0`; divergence-
  trigger unit tests at each threshold boundary; `RAMP_TO_REST` mid-decel
  enqueue test; boundary-velocity table unit tests (`reachableEntrySpeed`
  including the jerk==0 branch).
- **Verification command**: `uv run python -m pytest src/tests/sim/
  system/ -k "boundary or divergence or handoff"`.

## Implementation Plan

**Approach**: This ticket is almost entirely inside `Motion::Executor` —
no new modules, no wire changes. It's the highest-risk ticket for subtle
bugs (five interacting replan triggers) so budget for the divergence-
threshold and reanchor-seeding tests to be written first (they're the
easiest to get wrong per the issue's explicit "seeded from the channel's
OWN last sample — never measured sensors" warning).

**Files to modify**:
- `src/firm/motion/executor.{h,cpp}` (boundary-velocity table, all five
  replan triggers, RAMP_TO_REST mid-decel handling)
- `src/firm/motion/DESIGN.md` (boundary-velocity + replan-trigger tables)

**Testing plan**: as above — the no-decel sim test is this sprint's
headline regression test and should be treated as load-bearing for
ticket 009's tour-closure gate (a tour with visible stop-start stutter at
every corner is not "closes and looks like a square").

**Documentation updates**: `src/firm/motion/DESIGN.md`.
