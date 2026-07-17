---
id: '009'
title: "Sim tour-closure gate (decisive acceptance) — iterate until Tour 1/2\
  \ close within 1 degree or exact"
status: open
use-cases: [SUC-001, SUC-002, SUC-003, SUC-004, SUC-005]
depends-on: ['008']
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

- [ ] TestGUI → Sim → Tour 1 completes end-to-end (no fault, no freeze),
      closes the loop (returns to start pose within tolerance), and its
      trace is visibly square.
- [ ] TestGUI → Sim → Tour 2 completes end-to-end with the same standard.
- [ ] With sim OTOS drift + encoder error enabled: every turn in both
      tours is within 1° of commanded.
- [ ] With sim OTOS error/noise disabled: turns are exact.
- [ ] No velocity dip to zero at compatible same-`v_max` leg boundaries,
      observed at the full tour level (not just the ticket-006 synthetic
      test).
- [ ] EITHER all of the above pass, OR a written impossibility argument
      exists (in this ticket file or a linked note) explaining what was
      tried and why the target cannot be hit with this sprint's model —
      this ticket does not close silently short of the stated acceptance.
- [ ] This ticket does not itself modify `src/firm/` design (it is a
      verification/iteration ticket) — any firmware fix made while
      iterating belongs to, and updates the `DESIGN.md` of, whichever
      ticket/module it actually changes (005/006 most likely); note in
      this ticket which upstream ticket/module absorbed each fix, for
      traceability.

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
