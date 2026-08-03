---
sprint: '131'
title: 'Observable over proxy: takeover vs estop, honest stop and pose, one period owner'
status: closed
---

# Sprint 131 Close Report — Observable over proxy

## Goal

Close six A-priority defects from the 2026-08-02 post-130 review and
midpoint review, all sharing one root mechanism named in the post-130
post-mortem's root cause #1: checking a proxy for a physical fact
instead of the fact itself. Six tickets, executed serially on
`sprint/131-observable-over-proxy-takeover-vs-estop-honest-stop-and-pose-one-period-owner`
(HEAD `33a2b031`).

## Tickets (6/6 done)

| # | Title | Commit | Issue | Result |
|---|-------|--------|-------|--------|
| 001 | `takeover()`/`estop()` split + sign-aware bias on reversal | `841a018b` | A-move-takeover-wipes-the-controllers-learned-state | resolved |
| 002 | Commanded-zero through Stage B + Stage B freshness gates + genuine `NezhaMotor` freshness | `0cfa5f1f` | A-commanded-zero-leaks-through-stage-b | resolved |
| 003 | Speed floor: ratio-preserving scale, not common-mode-only (REVISED) | `29578345` withdrawn -> `47908f39` | A-speed-floor-snaps-the-planner-differential | resolved |
| 004 | `positionEpoch` consumers: `Odometry` + planner pose channels re-anchor across rebaseline | `0edefd06` | A-position-rebaseline-destroys-the-pose | resolved |
| 005 | One control period: absolute end-of-cycle deadline pacing | `335f4c09` | A-nominal-50ms-vs-delivered-54ms | resolved |
| 006 | Release `decelLatched` on re-measured rise + large-angle/chained/transient `planner_tests` | `7176605a` | A-tour2-146-degree-turn-still-undershoots-after-130-010 | **NOT resolved** (`completes_issue: false`) |

Five of six linked issues resolved. `A-tour2-146-degree-turn-still-undershoots-after-130-010.md` remains open, deliberately — see Findings below. Full delta narrative is in `sprint.md`'s `## Sprint Changes` section and the two Architecture `### Revision` subsections it points to.

## Verification

Sim-tier throughout (`.claude/rules/hardware-bench-testing.md`'s bench gate is undischarged — see Bench Debt below). Team-lead measured, at close:

| suite | result |
|---|---|
| `planner_tests` (standalone, Python-free) | 8/8 passed |
| `src/tests/sim` | 462 passed, 1 xfailed, 2 xpassed, 0 failed (460 at sprint start) |
| `src/tests/unit` | 611 passed, 3 failed |
| `src/tests/testgui` | 595 passed, 4 failed |

The 7 failures are inherited, not caused by this sprint: measured at base `22a3c368` before any ticket landed and unchanged at close. The 3 `test_gen_boot_config_planner.py` failures are provably pre-existing by dependency (this sprint touched nothing under `src/scripts/`, `data/robots/`, or `src/host/`); the 4 testgui failures were measured directly at base. Tracked in `clasi/issues/A-seven-untriaged-failing-tests-poison-every-no-regressions-claim.md`.

The turn-accuracy observable (`test_tour_closure_gate.py`, TOUR_1, single-step harness), measured at every commit:

| commit | turn 2 | turn 4 |
|---|---|---|
| base `22a3c368` | -2.68 deg | -13.19 deg |
| 001 `841a018b` | -2.567 deg | -13.080 deg |
| 002 `0cfa5f1f` | -2.567 deg | -13.080 deg |
| 003 first attempt `29578345` | **-10.38 deg** | **-20.07 deg** |
| 003 revised `47908f39` | -2.567 deg | -13.080 deg |
| 004, 005, 006 | -2.567 deg | -13.080 deg |

## Findings — the two things this sprint actually learned

**1. Ticket 003 shipped a measured regression and was withdrawn.** Its
original plan (floor the common mode, let the differential pass
through) was faithful to its issue and wrong: `Planner::planWheels()`
emits a pure differential for every Angle Move, so the common mode is
exactly zero for the entire turn and a common-mode-only floor never
engages. Cost: ~7-8 degrees of added undershoot per turn, caught by the
team-lead's own boundary verification, not by the ticket's original
acceptance criteria. Replaced by a ratio-preserving scale that is
bit-identical to the pre-sprint behavior for a symmetric pivot and a
strict improvement for an asymmetric arc. See `sprint.md`'s Architecture
"Revision — Ticket 003's speed-floor semantics."

**2. Ticket 006 disconfirmed its own premise — the sprint's most
valuable output.** Post-130 review's F10 proposed `decelLatched`'s
one-way trap as the cause of TOUR_2's 146-degree-turn undershoot.
Ticket 006 instrumented every `planWheels()` call across 2,351 ticks
(both tours, both error profiles) and found the triggering condition
**never fires** — every turn completes via the ordinary
profile-complete branch, never the stall backstop the hypothesis
requires; per-turn numbers are bit-identical before and after the fix.
The latch fix shipped anyway as a verified, negative-control-tested
latent-safety improvement (it only ever widens what the profiler may
command, never restricts), but it is not the cause of the undershoot.
What ticket 006 found instead: TOUR_1's leg 3 carries 6.45 deg of true
heading drift under the ideal profile, uncaptured by 130-010's
cumulative-baseline carry across a completed Distance leg — a plausible
but not fully sufficient explanation (doesn't fully account for
turn 4's -13.08 deg; a second contributor likely exists). The
146-degree turn's own error sign also flipped during this sprint
(-10.12 deg -> +5.19/+14.07 deg), suggesting a direction-dependent
mechanism. See `sprint.md`'s Architecture "Revision — Ticket 006's
`decelLatched` premise disconfirmed."

Two of ticket 006's acceptance criteria are left deliberately unchecked
(not marked done against a false premise or a stale baseline): the
TOUR_2-lands-in-band criterion (contingent on the disconfirmed
hypothesis), and the non-regression criterion's literal "<= 2.72 deg"
figure, which was 130-010's own stale number — ticket 006's actual
ticket-start baseline was -24.699 deg, reproduced bit-for-bit, so
ground-truth non-regression holds even though the stated figure
doesn't.

## Bench debt (declared, not faked)

`tovez` has been wedged (dead-I2C signature) since 2026-08-01; nothing
in this sprint is hardware-verified. Ticket 005's negative control is
worth recording as a method: the sim steps at exactly `kCycle` and
structurally cannot exhibit the 54 ms delivered period, so the
implementer injected sleep jitter and measured accumulation instead,
reproducing 54.568 ms against hardware's measured 54.000 ms, then
confirmed the new pacing test fails against the old code. Outstanding
bench work: ticket 003's floor-value re-fit (needs the
loaded-actuation-floor measurement) and ticket 005's delivered-period
re-measurement (`src/tests/bench/planner_square_tour.py`).

## Follow-up (not filed as fresh issues by this report — for the team-lead)

- Re-open investigation into
  `A-tour2-146-degree-turn-still-undershoots-after-130-010.md` (or split
  a fresh issue) targeting the Distance-leg cumulative-baseline carry
  not folding in the leg's own true heading drift —
  `straight_leg_cruise_headings` in `test_tour_closure_gate.py` is the
  existing measurement tool. The 146-degree turn's direction-dependent
  sign flip is a live clue worth chasing alongside it.
- The next physical bench session (once `tovez` is reachable) should
  re-run the full standing gate plus the two bench items above.
