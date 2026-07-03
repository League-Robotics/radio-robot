---
id: "004"
title: "Regression sweep + Tour-1 xfail removal"
status: open
use-cases:
- SUC-001
- SUC-002
depends-on:
- "001"
- "002"
- "003"
github-issue: ""
issue: sim-turn-undershoot.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Regression sweep + Tour-1 xfail removal

## Description

Final verification pass for sprint 073. Tickets 001–003 fix three
independent, composing defects (stale coast-anticipation constant, an
unscrubbed clean-sim plant, a TestGUI default that doesn't reconcile
calibration) — this ticket is the acceptance vehicle proving they land
the combined `RT` accuracy bar the sprint brief requires, and finalizes
the deliberately-updated-test bookkeeping.

1. Adds a new angle-sweep regression test,
   `tests/simulation/system/test_073_rt_angle_sweep.py`, parametrized over
   `[4500, 9000, 18000, 30000]` centidegrees (45°/90°/180°/300°),
   constructing a fresh, ZERO-configuration `Sim()`, issuing `RT <cdeg>`,
   and asserting `abs(true_heading_deg - commanded_deg) < ~1.0` via
   `sim.get_true_pose()`. This is the sprint's headline acceptance test —
   it exercises Tickets 001 and 002 TOGETHER (Ticket 001 alone would still
   show the ~+8.7% slip-driven gap at large angles; Ticket 002 alone would
   still show the constant ~3.3° coast gap at all angles).
2. Updates `tests/simulation/system/test_069_rt_90deg_body_scrub.py`'s
   `test_rt_90deg_identity_no_scrub` (which passes `body_rot_scrub=None`
   specifically to test "the DEFAULT is a no-op") — after Ticket 002, the
   DEFAULT is no longer a no-op by design (it is seeded from
   `cfg.rotationalSlip`). Rewrite the test to either (a) explicitly pass
   `body_rot_scrub=1.0` to test the SETTER's neutral value (preserving the
   original intent: `rotSlip=1.0` + no scrub = identity), or (b) assert the
   NEW default behavior directly, explicitly resetting scrub to `1.0`
   first if testing the identity claim. `test_rt_90deg_with_body_scrub_
   matching_rot_slip` and `test_rt_scrub_cancellation_matches_identity_
   not_uncorrected_baseline` already pass explicit values for every scrub
   field on every call (confirmed by architecture-update.md Step 1) —
   unaffected.
3. Runs `tests/testgui/test_tour1_geometry.py::test_tour1_traces_the_tour_at_zero_error`
   (opt-in GUI tier: `uv run --group gui python -m pytest
   tests/testgui/test_tour1_geometry.py -v`) once Tickets 001–003 have
   landed. If it passes, removes the `xfail(strict=True)` marker (required
   — `strict=True` turns an unexpected pass into a failure) and updates
   the module docstring's "Currently XFAIL" section. If it does NOT fully
   pass, documents the residual gap precisely (e.g. `TURN`'s own
   closed-loop tolerance, an unrelated geometry constant drift) rather
   than force-removing the marker.
4. Confirms the full suite (`uv run python -m pytest`) is green at 2655 +
   this sprint's net new/updated tests, 0 failures, before AND after this
   ticket's own changes — mirrors 072 Ticket 004's own methodology.
5. Updates `sprint.md` with the confirmed before/after baseline and the
   exact before/after numbers for each deliberately-updated test
   (`test_rt_slip.py`'s coast constant, `test_069_rt_90deg_body_scrub.py`'s
   rewritten identity test, the three TestGUI default-profile tests, and
   the Tour-1 xfail outcome).

No wire-protocol grammar change. No new `RobotConfig`/`SIMSET` field. This
ticket depends on 001, 002, AND 003 — it needs the complete fix to
validate the combined ≤~1° acceptance bar and to safely evaluate the
Tour-1 xfail.

See `architecture-update.md` Step 4c (ticket dependency graph), Step 5
"Ticket 004", Step 7 Open Question 5 (xfail removal is contingent, not
guaranteed); `usecases.md` SUC-001's and SUC-002's acceptance criteria
(the combined sweep).

## Acceptance Criteria

- [ ] New `tests/simulation/system/test_073_rt_angle_sweep.py` exists,
      parametrized over 45°/90°/180°/300° (`[4500, 9000, 18000, 30000]`
      centidegrees), constructs a fresh zero-configuration `Sim()` per
      case, issues `RT <cdeg>`, and asserts the true-pose heading is
      within ~1° of commanded for every angle in the sweep.
- [ ] `tests/simulation/system/test_069_rt_90deg_body_scrub.py::test_rt_90deg_identity_no_scrub`
      is rewritten to account for Ticket 002's new non-neutral default
      (either explicit `body_rot_scrub=1.0` to preserve the original
      "setter neutral value" intent, or an explicit assertion against the
      new seeded default) — passes, and its docstring documents the
      before/after reasoning.
- [ ] `test_rt_90deg_with_body_scrub_matching_rot_slip` and
      `test_rt_scrub_cancellation_matches_identity_not_uncorrected_baseline`
      are confirmed passing UNMODIFIED (both already pass explicit values
      for every scrub field).
- [ ] `tests/testgui/test_tour1_geometry.py::test_tour1_traces_the_tour_at_zero_error`
      is run (opt-in GUI tier) after Tickets 001–003 land. If it passes,
      the `xfail(strict=True)` marker is removed and the module docstring
      updated. If it does not fully pass, the residual gap is documented
      precisely in this ticket's implementation notes and the marker
      stays — this is an acceptable, explicitly-permitted outcome, not a
      ticket failure.
- [ ] Full suite (`uv run python -m pytest`) run twice consecutively after
      a `--clean` sim rebuild, confirmed green at the expected final count
      (2655 + net sprint delta), 0 failures.
- [ ] `sprint.md`'s Success Criteria/Test Strategy sections are updated
      with: the confirmed before/after baseline (test counts), and a list
      of every deliberately-updated test with its before/after values
      (`test_rt_slip.py`'s `coast_mm` constant → formula-derived value;
      `test_069_rt_90deg_body_scrub.py`'s identity-test rewrite;
      `test_sim_prefs.py`/`test_transport.py`/
      `test_070_004_sim_errors_from_cal.py`'s `0.26`/`1.0` →
      `0.0`/calibration-resolved defaults; the Tour-1 xfail outcome,
      whichever way it resolved).
- [ ] No pre-existing, unrelated test regresses — any full-suite failure
      other than the sprint's own named, deliberately-updated tests is
      investigated and resolved (or reported as a blocking finding) before
      this ticket is marked done.

## Testing

- **Existing tests to run**: the full suite, twice consecutively (project
  convention for a milestone/closure-level ticket, per 072 Ticket 004
  precedent); `test_069_rt_90deg_body_scrub.py`'s other two tests in
  isolation to confirm unaffected; `tests/testgui/test_tour1_geometry.py`
  under the opt-in GUI tier.
- **New tests to write**: `test_073_rt_angle_sweep.py` (the headline
  sweep test, 4 parametrized cases).
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: This ticket runs last, after Tickets 001–003 have landed.
Start with a full-suite run to get a concrete before-this-ticket count.
Write `test_073_rt_angle_sweep.py` first — it is the actual proof that
Tickets 001+002 combine correctly; if it fails, that is a signal to
revisit Ticket 001's formula (possibly needing the documented empirical
correction factor per Ticket 001's own Design Rationale fallback) before
proceeding further in this ticket. Once the sweep passes, rewrite
`test_069_rt_90deg_body_scrub.py`'s identity test, confirm the other two
tests in that file are genuinely unaffected by running them in isolation.
Run the Tour-1 GUI test under its opt-in marker; remove the xfail only if
it genuinely passes, documenting the outcome either way. Re-run the full
suite twice consecutively after a `--clean` sim rebuild to confirm the
final green count. Update `sprint.md` with the confirmed numbers and the
before/after table for every deliberately-updated test across all four
tickets.

**Files to create/modify**:
- New `tests/simulation/system/test_073_rt_angle_sweep.py`.
- `tests/simulation/system/test_069_rt_90deg_body_scrub.py` —
  `test_rt_90deg_identity_no_scrub` rewritten.
- `tests/testgui/test_tour1_geometry.py` — `xfail(strict=True)` marker
  removed and docstring updated, ONLY if the test genuinely passes;
  otherwise document the residual gap in this ticket's implementation
  notes and leave the file unmodified.
- `clasi/sprints/073-sim-turn-accuracy-coast-anticipation-from-ramp-dynamics-and-slip-bookkeeping-reconciliation/sprint.md`
  — Success Criteria/Test Strategy sections updated with confirmed
  before/after baseline and the deliberately-updated-test summary.

**Testing plan**: full-suite run before this ticket's changes (baseline),
targeted runs of the new sweep test and the rewritten identity test, the
opt-in Tour-1 GUI test, then two consecutive full-suite runs after a
`--clean` sim rebuild to confirm the final count is stable and green.

**Documentation updates**: `sprint.md`'s Success Criteria/Test Strategy
sections with the confirmed final numbers, the deliberately-updated-test
before/after table, and the Tour-1 xfail outcome (removed, or left in
place with the documented residual gap).
