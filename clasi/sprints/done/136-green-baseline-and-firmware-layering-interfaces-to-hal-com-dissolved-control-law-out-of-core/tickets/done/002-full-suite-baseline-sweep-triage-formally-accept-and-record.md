---
id: '002'
title: Full-suite baseline sweep -- triage, formally accept, and record
status: done
use-cases:
- SUC-001
depends-on:
- '001'
github-issue: ''
issue: sprint-135-pre-existing-test-failures-need-triage.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Full-suite baseline sweep -- triage, formally accept, and record

## Description

Ticket 001 fixed the two dominant, root-caused failure clusters (roughly
45-50 of the 53 failures measured on clean HEAD at sprint-planning time:
14 unit + 39 sim). This ticket is the actual baseline-establishment work:
run all three suites fresh, triage whatever remains, and leave the tree in
a state where "no new regressions" is a claim Half B can trust — this is
the whole point of the hard gate ahead of ticket 003.

This ticket's job is triage and formal acceptance, not fixing new
firmware behavior. In particular, the turn-accuracy/tour-completion family
flagged during planning (`test_tour_closure_gate.py`,
`test_gui_button_acceptance.py`'s two tour tests,
`test_otos_calibration_convergence.py`, `test_camera_combo.py`) is real,
previously-flagged, unresolved firmware behavior
(`clasi/issues/later/A-tour2-146-degree-turn-still-undershoots-after-130-
010.md`) that is explicitly Out of Scope for this sprint (see sprint.md) —
formally accept it with a strict, issue-referenced `xfail`, do not attempt
a fix. If triage surfaces something genuinely new (not already covered by
a tracked issue), file a fresh issue via the `issue` skill before marking
it `xfail` — never an unattributed `xfail`.

This ticket also implements issue A's two process fixes: making every
non-strict `xfail` in the three suites strict (so a future xpass fails
loudly instead of hiding — the 2 xpassed measured in `src/tests/sim` at
planning time are the clearest known instance), and recording the
resulting baseline somewhere a future sprint reads at start.

## Acceptance Criteria

- [ ] `uv run python -m pytest src/tests/unit src/tests/sim
      src/tests/testgui` run fresh (post-ticket-001), full output
      captured and reviewed line by line — not sampled.
- [ ] Every failure remaining after ticket 001 is triaged: fixed in this
      ticket if cheap and directly root-caused during triage, otherwise
      formally accepted.
- [ ] Every formally-accepted failure carries `@pytest.mark.xfail(reason=
      "...", strict=True)` (or the file's existing xfail idiom, made
      strict) naming a tracked issue by filename. A failure with no
      tracked issue gets one filed (via the `issue` skill) before being
      marked — no unattributed `xfail`.
- [ ] Every pre-existing non-strict `xfail` across the three suites
      (including the 2 xpassed measured in `src/tests/sim` at planning
      time) becomes `strict=True`.
- [ ] The turn-accuracy/tour-completion family is formally accepted
      (strict `xfail`, referencing
      `clasi/issues/later/A-tour2-146-degree-turn-still-undershoots-
      after-130-010.md` or the closest-matching tracked issue after
      direct verification of which issue actually names each specific
      test) — not fixed.
- [ ] The resulting baseline (failure count — ideally 0 non-xfailed —
      measurement date, commit hash) is recorded somewhere a sprint reads
      at start: `src/tests/CLAUDE.md` if it exists, created there if not,
      per issue A's proposed fix item 3.
- [ ] `clasi/issues/later/A-seven-untriaged-failing-tests-poison-every-
      no-regressions-claim.md` is resolved per its own Verification
      section (triage done, strict xfails in place, baseline recorded —
      its criteria require triage and tracking, not that every defect be
      fixed) and moved to done.
- [ ] A second, independent `uv run python -m pytest` run across all
      three suites, done AFTER the xfail marks land, confirms: either
      zero failures, or a failure set every entry of which is a
      `strict=True` xfail referencing a tracked issue — confirmed by
      re-running, not assumed from the first pass.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/unit
  src/tests/sim src/tests/testgui`, run at least twice — once to
  establish the post-ticket-001 failure set, once after xfail marks land
  to confirm they match real failures (not silently papering over
  something that actually passes, which would be its own defect).
- **New tests to write**: none — this ticket's surface is `xfail` marks
  and the baseline record, not new test code.
- **Verification command**: `uv run python -m pytest src/tests/unit
  src/tests/sim src/tests/testgui`
