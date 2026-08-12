---
status: done
priority: high
tickets:
- '136-002'
---

# Seven tests fail on a clean tree, none are triaged, and they poison every "no regressions" claim

## Resolution (2026-08-11, sprint 136 ticket 002)

Resolved per this issue's own Verification section, though the underlying
tree has moved on substantially since this issue was filed (2026-08-03,
commit `22a3c368`) — the original seven are not the same seven that needed
triaging at resolution time (sprint 132's telemetry-emit rebuild and
several turn-shaping tickets landed in between), but the issue's own three
concerns are all closed:

1. **Every failure on the tree is now either fixed or carries a strict,
   issue-referenced `xfail`.** `uv run python -m pytest src/tests/unit
   src/tests/sim src/tests/testgui` -- `975 passed / 0 failed`, `493 passed
   / 0 failed / 4 xfailed`, `592 passed / 0 failed / 3 skipped / 19
   xfailed`, confirmed by an independent second run after every `xfail`
   mark landed (this ticket's own acceptance criterion, not assumed from
   the first pass).
2. **No non-strict `xfail` remains** anywhere in the three suites --
   confirmed by repo-wide grep, not sampled.
3. **The baseline is recorded** in `src/tests/CLAUDE.md` (its own new
   "Baseline" section), with the counts above, the tracked issues backing
   every `xfail`, and a pointer to this ticket's own git history for the
   exact measurement commit.

Item 4 (boundary checks must cover all three suites) is a process
practice, not something a single ticket can close by itself, but the
finding is what motivated this very ticket running all three suites, twice
each, rather than sampling one.

The original "seven" specifically: items 1-3
(`test_gen_boot_config_planner.py`) and item 4
(`test_flags_bit16_shaping_disabled_asserts_when_push_stripped`) were
already resolved by earlier sprints (132's telemetry rebuild and 130-009's
literal reshape both landed and green well before this ticket started —
neither appears in this ticket's own fresh measurement). Items 5-7 (the
turn-accuracy/tour-completion family) are exactly the ones sprint 131
ticket 006 investigated and did NOT fully close (see its own Completion
Notes) -- formally accepted here with a strict `xfail` referencing the
refiled `clasi/issues/later/A-tour2-146-degree-turn-still-undershoots-
after-130-010.md`, since sprint 136 is a baseline-and-layering sprint, not
the sprint to fix them. The "2 xpassed" this issue names (sprint-131-era
measurement) is stale by the time of this resolution -- sprint 136's own
planning-time measurement found 1 xpassed in `src/tests/sim`, which this
ticket identified by name (`test_angle_stop_lands_close_to_target_with_
tovez_nocal_calibration`), confirmed genuinely passing (5/5 repeated
runs), and un-`xfail`ed rather than marking strict (the correct resolution
for a capability that now genuinely works, not a silent xpass to paper
over).

## Description

Measured 2026-08-03 during sprint 131's boundary verification, at commit
`22a3c368` (clean master, before sprint 131 began) and confirmed unchanged at
`47908f39`:

| suite | result |
|---|---|
| `src/tests/sim` | 460 passed, 1 xfailed, 2 xpassed, **0 failed** |
| `src/tests/unit` | 611 passed, **3 failed** |
| `src/tests/testgui` | 595 passed, 3 skipped, 8 xfailed, 2 xpassed, **4 failed** |

The seven:

1-3. `src/tests/unit/test_gen_boot_config_planner.py`
   - `test_planner_config_for_config_reads_tovez_json`
   - `test_planner_config_for_config_raises_with_no_robot_config`
   - `test_generate_emits_default_planner_limits_byte_identical_to_pre_ticket_literals`

4. `src/tests/testgui/test_sim_loop.py::test_flags_bit16_shaping_disabled_asserts_when_push_stripped`
5. `src/tests/testgui/test_gui_button_acceptance.py::test_tour_1_runs_to_completion`
6. `src/tests/testgui/test_gui_button_acceptance.py::test_tour_2_runs_to_completion`
7. `src/tests/testgui/test_tour_closure_gate.py::test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band`

Plus **2 xpassed** in `src/tests/sim` — tests marked `xfail` non-strict that now
pass, so nothing reports the change.

## Why this is A rather than housekeeping

**This exact problem has now cost two consecutive sprints.**

Sprint 130's post-mortem records that four tickets measured "no regressions"
against a baseline of "2 known failures" which turned out to be tests that had
**never compiled** — a missing `-I src` flag nobody had read the error for. Its
root-cause section names the mechanism: *"'Known pre-existing failure' is a
claim, not a fact."*

Sprint 131 then hit the same class from the other direction. The team-lead's
boundary check ran only `src/tests/sim` (463 of ~1,689 tests) and reported
"460 passed, 0 failed" at two ticket boundaries. Both statements were true and
neither was sufficient: a genuine ~7° turn-accuracy regression introduced by
ticket 003 lived in `src/tests/testgui`, which was not being run. It surfaced
only because a subagent complained about something else. Hours then went into
bisecting and re-characterizing what "green" even means.

A baseline nobody has triaged is not a baseline. Every sprint that starts
against this tree either re-derives it by hand or unknowingly reports a
regression as inherited.

## What is probably true about each (NOT yet verified — that is the work)

- **1-3** are almost certainly sprint-130 `PlannerLimits` 34→18 reshape residue:
  one is literally named `..._byte_identical_to_pre_ticket_literals`, pinned to
  literals that 130-009 deliberately changed. Proven pre-existing by dependency
  (sprint 131 touched nothing under `src/scripts/`, `data/robots/`, or
  `src/host/`), but nobody has decided whether the test or the generator is
  wrong.
- **4** was *predicted* by the sprint-130 knowledge doc: *"a test asserting
  'shaping is off unless pushed' now fails because the unified boot path bakes
  real shaper defaults. That one is probably the test encoding a
  pre-unification assumption — confirm before editing either side."* Nobody
  confirmed, and no issue was ever filed.
- **5-7** are turn-undershoot, the `decelLatched` territory of
  [[A-tour2-146-degree-turn-still-undershoots-after-130-010]] (sprint 131
  ticket 006). Note 5 and 6 use the background tick thread and are not
  bit-reproducible; 7 is the single-step harness and is. Sprint 131 ticket 006
  is expected to move 7 and possibly 5/6 — but that must be *measured*, not
  assumed.

## What to do

1. Triage each of the seven: is the test wrong, or the code? Decide and record.
   A test that encodes a superseded assumption gets deleted or rewritten with a
   note saying why; a test exposing a real defect gets an issue.
2. Make surviving `xfail`s **strict**, so an xpass fails loudly instead of
   silently. The 2 xpassed above are invisible today.
3. Record the resulting green baseline somewhere a sprint reads at start —
   `src/tests/CLAUDE.md` is the natural home — with the date and commit it was
   measured at.
4. Establish that the boundary check covers **all three suites**. Sprint 131's
   ~26% coverage was an accident of habit, not a decision.

## Verification

- `uv run python -m pytest src/tests/sim src/tests/unit src/tests/testgui`
  produces either zero failures, or a failure set every entry of which is named
  in a tracked issue with a stated reason.
- No non-strict `xfail` remains.
- The recorded baseline matches a fresh run on a clean checkout.

## Related

- `docs/post-mortem/2026-08-02-sprint-130-wheels-solid.md` — root cause 1 and
  the errors-of-analysis section.
- `docs/knowledge/2026-08-02-sprint-130-residuals-and-what-went-wrong.md` —
  "'Known pre-existing failure' is a claim, not a fact."
- [[B-app-robot-loop-harness-never-compiled]] — the corpse harness and the
  non-strict-xfail problem, of which this is the broader instance.
- Sprint 131's boundary measurements are the evidence above.
