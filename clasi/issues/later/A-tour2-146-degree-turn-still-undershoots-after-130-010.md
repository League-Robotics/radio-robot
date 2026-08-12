---
status: pending
priority: high
filed: 2026-08-11
filed_by: "programmer (sprint 136 ticket 002, full-suite baseline triage)"
related:
- chain-advance-completion-margin-narrow-pocket.md
- B-planner-residuals-and-separability.md
tickets:
- 136-002
---

# Turn-accuracy / tour-completion family still fails the shaped-band gate (146-degree residual re-opened)

## Description

This issue is a **refiling**, not a new discovery. The path
`clasi/issues/later/A-tour2-146-degree-turn-still-undershoots-after-130-010.md`
is cited by name in sprint 136's own `sprint.md` (Problem section) and ticket
002 (both written at sprint-planning time, 2026-08-11) as an existing tracked
issue -- but the file does not exist anywhere in the live `clasi/issues/`
tree. It only survives as a point-in-time snapshot at
`clasi/sprints/done/131-observable-over-proxy-takeover-vs-estop-honest-stop-and-pose-one-period-owner/issues/A-tour2-146-degree-turn-still-undershoots-after-130-010.md`,
archived when sprint 131 closed. Nobody re-filed a live successor, even
though that sprint's own ticket 006 (which targeted this issue) recommended
exactly that in its Completion Notes ("recommended follow-up... not filed as
a formal issue by me -- I do not have issue-authoring scope as the executing
programmer"). This refiles it, verified against current measurements, so
sprint 136 ticket 002's `xfail` marks have a real target.

## Full history (verified by reading the archived files directly, not assumed)

1. **Sprint 130 ticket 010** fixed the general turn-shaping undershoot
   (worst per-turn error -20.8 deg -> 2.72 deg) but left TOUR_2's 146-degree
   turn missing by -10.12 deg, not scaling with angle -- suggesting a second,
   additive mechanism. Filed as the original `A-tour2-146-degree...` issue.

2. **Sprint 131 ticket 006** targeted the leading hypothesis (`decelLatched`
   as a one-way trap that forbids re-measurement recovery). The fix was
   implemented and is real -- `planner_tests` gained scenarios proving the
   mechanism exists and is now fixed -- but **the ticket's own Completion
   Notes report the fix did NOT close the observed `test_tour_closure_gate.py`
   failures**: direct instrumentation of every `planWheels()` call across
   2351 ticks (both tours, both error profiles) found the `decelLatched`
   release condition's own trigger never fires organically. The actual
   mechanism the ticket identified instead: `Planner::tick()`'s
   cumulative-baseline ledger (`carryHeading_ = active_.baselineHeading` on
   a completed Distance leg) does not fold in how much that leg's OWN
   heading actually drifted during execution before the ledger is read by
   the FOLLOWING turn -- the mirror image of the already-known,
   already-accepted "axis-drop coast at chain boundaries" defect
   (119-002, tracked separately at
   [[chain-advance-completion-margin-narrow-pocket]]). This does not fully
   close the arithmetic on every turn checked, so a second contributing
   factor may also exist. Ticket 006's own frontmatter records
   `completes_issue: false`.

3. **Sprint 121 ticket 003** (`land-at-zero-at-orthogonal-chain-boundaries.md`,
   now closed/archived at
   `clasi/sprints/done/121-.../issues/done/land-at-zero-at-orthogonal-chain-boundaries.md`)
   re-measured the shaped-band tolerances against a new orthogonal-boundary
   land-at-zero split and left them unchanged (worst case stayed the same
   order of magnitude). `test_tour_closure_gate.py`'s own in-file comments
   still cite this issue's path as "the live investigation" -- it is not
   live; it is done. That stale cross-reference is corrected by this
   refiling (the `xfail` reason strings this ticket touches now point here
   instead).

4. **Sprint 135** (`sprint-135-pre-existing-test-failures-need-triage.md`,
   this sprint's own source issue) re-measured the same family and found the
   shaped-band gate still failing (~14 degrees observed that sprint), plus a
   NEW-looking precondition failure in `test_gui_button_acceptance.py`
   (`tour_btn_tour_1 must be enabled before this test`) that it explicitly
   left unattributed pending investigation.

5. **Sprint 136 ticket 002** (this ticket) re-measured fresh on
   2026-08-11 at commit range `fc2869a2`..`e5339c27`-and-later. Current
   failures, confirmed by direct pytest run:
   - `test_tour_closure_gate.py::test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band`
     -- the real (non-`xfail`) gate, still fails.
   - `test_gui_button_acceptance.py::test_tour_1_runs_to_completion` and
     `::test_tour_2_runs_to_completion` -- still fail.
   - `test_gui_button_acceptance.py::test_stop_mid_tour_halts_motion_and_flushes_the_queue`
     -- fails on the precondition sprint 135 flagged
     (`tour_btn_tour_1 must be enabled before this test`). Read directly
     (`_run_tour()`'s helper, `test_gui_button_acceptance.py`): the wait
     loop only exits early when `stop_tour_btn` disables (the tour worker
     thread finished, however it finished); if a tour run in an EARLIER
     test in the same module hits `TOUR_TIMEOUT_S` with the worker still
     active, the button state leaks into the next test in the same pytest
     session. This test runs immediately after `test_tour_2_runs_to_completion`
     in file order, which is the same family's own failure -- the most
     likely explanation, though not independently isolated by re-ordering
     the tests (that would cost more than this ticket's triage budget
     warrants). Filed here as the SAME family rather than a separate issue
     because the plausible mechanism is downstream of items 1-2 above, not
     an independent GUI defect.

## Why this is Out of Scope for sprint 136 (per sprint.md)

Sprint 136 is a baseline-and-layering sprint; the turn-accuracy/
tour-completion family is real, previously-flagged, unresolved firmware
behavior that is out of proportion to fix inside it. Ticket 002 formally
accepts it here with a strict, issue-referenced `xfail` rather than
attempting a fix -- see `sprint.md`'s own Out of Scope section.

## What to do (future sprint)

1. Continue sprint 131 ticket 006's own recommended follow-up: investigate
   whether `Planner::tick()`'s Distance-leg completion case
   (`carryHeading_ = active_.baselineHeading`) should fold in the leg's own
   measured heading drift before handing the baseline to the next turn.
   `test_tour_closure_gate.py`'s own `straight_leg_cruise_headings`
   instrumentation is the existing tool to measure it.
2. Re-attempt the arithmetic 131-006 could not fully close (leg 3's 6.45 deg
   max drift did not by itself account for turn 4's -13.08 deg) -- there may
   be a second, uninvestigated contributor.
3. Independently isolate the `test_stop_mid_tour_halts_motion_and_flushes_the_queue`
   precondition failure (item 5 above) by running it alone / re-ordered, to
   confirm or refute that it is downstream of the tour-completion family
   rather than a distinct GUI state-leak defect.
4. Coordinate with [[chain-advance-completion-margin-narrow-pocket]], which
   covers the adjacent (but distinct) turn-into-straight chain-boundary
   residual -- both concern the same family of boundary-carried error, from
   opposite sides of the boundary.

## Verification

- `test_tour_closure_gate.py::test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band`
  passes without widening `_TURN_TOLERANCE_SHAPED_DEG`.
- `test_gui_button_acceptance.py`'s two tour-completion tests and the
  stop-mid-tour test all pass on repeated runs (tour outcomes use a
  background tick thread and are not bit-reproducible; confirm across
  multiple runs, not once).
- The `xfail` marks this issue backs (`test_tour_closure_gate.py`,
  `test_gui_button_acceptance.py`) are removed once the above hold.

## Related

- `clasi/sprints/done/131-.../tickets/done/006-release-decellatched-on-re-measured-rise-large-angle-chained-transient-planner-tests.md`
  -- the fullest existing investigation, including the Completion Notes this
  issue's History section quotes from directly.
- `clasi/sprints/done/121-.../issues/done/land-at-zero-at-orthogonal-chain-boundaries.md`
  -- closed; re-measured these same tolerances once, did not close them.
- `clasi/issues/later/chain-advance-completion-margin-narrow-pocket.md` --
  adjacent, complementary, still-open issue on the same boundary-residual
  family.
- `clasi/sprints/136-.../issues/sprint-135-pre-existing-test-failures-need-triage.md`
  -- this sprint's own source issue, Group 2.
