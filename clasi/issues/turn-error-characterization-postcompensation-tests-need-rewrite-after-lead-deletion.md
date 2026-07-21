---
title: test_turn_error_characterization.py's postcompensation tests need rewrite/deletion
  once lead-compensation fields are removed
filed: 2026-07-19
filed_by: programmer (111-002, baseline sim-suite triage)
status: pending
related:
- motion-control-terminal-blips-reconciled-fix-plan.md
sprint: '119'
---

# test_turn_error_characterization.py postcompensation tests need attention

## What's going on

`src/tests/testgui/test_turn_error_characterization.py::test_postcompensation_ideal_matches_shipped_defaults`
(parametrized `[30.0]`/`[170.0]`) is currently `xfail(strict=False)`,
quarantined by 111-002 with a citation to
`clasi/issues/motion-control-terminal-blips-reconciled-fix-plan.md`.

The test's own `_DISABLED = (-0.05, 0.0, 0.0)` constant was an exact,
hand-duplicated snapshot of `gen_boot_config.py`'s shipped
`HEADING_LEAD_BIAS_DEFAULT`/`PLAN_LEAD_DEFAULT`/`TERMINAL_LEAD_DEFAULT` at
the time ticket 109-010 wrote it (both leads were `0.0`), so its assertion
("shipped config produces the same result as `_DISABLED`") held by
construction. Commit `740bff35` later re-tuned `PLAN_LEAD_DEFAULT` from
`0.0` to `0.20` (a real, documented, bench-motivated change -- "eliminates
the terminal PD reversal entirely") without updating this test, so the
assertion now fails: shipped lead compensation measurably costs
~0.25-0.6deg of ideal-chip pivot accuracy relative to a true zero
baseline.

## Why this wasn't just fixed in 111-002

Re-pointing `_DISABLED` at the live `PLAN_LEAD_DEFAULT` would make the
assertion tautological (shipped vs. shipped). A genuinely-independent zero
baseline would need re-deriving what this test *should* assert now --
but the reconciled fix plan's own step 3 already **deletes** the entire
lead-sampling mechanism (`plan_lead`/`terminal_lead`/`heading_lead_bias`)
this test module exists to characterize, explicitly superseding
`later/turn-lead-compensation-gain-cotuning.md` (109-010's own follow-up
issue this test module supports). Rewriting the test's numeric
expectations to match a value (`plan_lead=0.20`) that will not survive
the sprint would be wasted work.

## What needs to happen

When a future sprint-111 ticket executes the reconciled fix plan's step 3
("Delete the lead-sampling machinery") and step 7 ("drop the now-unused
lead fields -- `plan_lead`, `terminal_lead`, `heading_lead_bias`,
`min_speed`"), it must also revisit
`src/tests/testgui/test_turn_error_characterization.py` as a whole:

- The `test_postcompensation_*` tests (Work item (d) in the module's own
  docstring) characterize lead-compensation gain-tuning, an approach the
  reconciled plan has abandoned in favor of closing the loop with real
  feedback. Once the lead fields are gone, this whole module's premise
  (measure staleness, invert it with a lead) no longer applies to the
  production code path.
- Likely disposition: delete the module (or the parts of it that assume
  lead-compensation exists), replacing it with acceptance coverage for
  whatever the reconciled plan's steps 4-5 (acceleration feedforward +
  bounded position-feedback trim) actually ship, per that plan's own
  merged acceptance criteria (§6).
- Do not just flip the `xfail` back to a plain assertion -- the whole
  approach this test validates is being replaced, not merely re-tuned.

This is a fresh, explicit tracking entry so this cleanup is not silently
forgotten once ticket 004 (which covers the OTHER 18 dead `PlannerConfig`
fields, a different list -- see the reconciled plan's §1.4) closes and
looks like it already handled "the dead fields" work.
