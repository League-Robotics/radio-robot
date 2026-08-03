---
id: '018'
title: "Full-system verification (sim baseline, ARM build, parity harnesses, read-back/bake-push/no-silent-no-op\
  \ properties, 16→2 demo)"
status: open
use-cases:
- SUC-001
- SUC-007
depends-on:
- '017'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Full-system verification (sim baseline, ARM build, parity harnesses, read-back/bake-push/no-silent-no-op properties, 16→2 demo)

## Description

The first of the two acceptance-concentrated closing tickets. This is
where "no regressions" becomes a real, enforceable claim against the
**corrected** pre-sprint baseline, measured on `master` immediately
pre-sprint (2026-08-03) against the FULL DEFAULT test collection
(`testpaths = ["src/tests/sim", "src/tests/unit", "src/tests/testgui"]`
— plain `uv run python -m pytest`, not the `src/tests/sim`-only slice):

```
7 failed, 1757 passed, 3 skipped, 9 xfailed, 4 xpassed
```

This corrects TWO figures earlier versions of this ticket/`sprint.md`
carried in error: the original "484 passed / 2 known failures" (stale —
22 tests were removed by the `src/tests/dev/` reorganization since that
figure was recorded), and a later correction that reported "462 passed /
0 failed" against the `src/tests/sim` slice ONLY, with an accompanying
claim that
[[A-seven-untriaged-failing-tests-poison-every-no-regressions-claim]] is
stale — that claim was **wrong**; the issue is exactly accurate. The
seven known, accepted failures are:

- 3× `test_gen_boot_config_planner.py` (a stale `headingHoldGain`
  expectation, `2.0f` vs. the current `0.0`)
- 2× `test_gui_button_acceptance.py` (tour 1/2)
- 1× `test_sim_loop.py::test_flags_bit16_shaping_disabled_asserts_when_push_stripped`
- 1× `test_tour_closure_gate.py` (a 90° commanded turn achieving 76.92°)

**"No regressions" means no EIGHTH failure appears — it does not mean
these seven become zero.** This ticket is not a mandate to fix them;
fixing any of them is out of scope unless a specific ticket's own work
happens to touch the exact code path (none currently does).

Assemble and prove every Success Criteria property from `sprint.md` in
one place.

## Acceptance Criteria

- [ ] Full default-collection suite run: plain `uv run python -m pytest`
      (NOT the `src/tests/sim`-only slice) shows **no more than 7
      failed**, and any failure beyond the 7 named above (in the
      Description) is investigated and explained, not silently accepted
      as "close enough." A failure COUNT match (7) is not sufficient on
      its own if the failing TESTS differ from the named seven — check
      identity, not just count.
- [ ] The passed/skipped/xfailed counts are compared against the
      baseline (1757 passed, 3 skipped, 9 xfailed) and any drop in
      `passed` or unexplained change in `skipped`/`xfailed` is
      investigated, not silently accepted.
- [ ] The 4 `xpassed` tests from the baseline are triaged: for each,
      either its `xfail` marker is removed (if genuinely fixed) or a
      documented reason is recorded for why it stays marked — not
      silently re-inherited. An xpass is a quiet lie about what the
      suite proves.
- [ ] ARM build succeeds (full toolchain, not just `HOST_BUILD`).
- [ ] The `kEncodeScratchCap` `static_assert` (ticket 015) is confirmed
      present and was demonstrated to fire.
- [ ] `composition_root_parity_harness.cpp` passes, with the sim's seven
      enumerated `BootOverrides` divergences (`boot_wiring.h:76-101`)
      each individually confirmed still preserved OR explicitly retired
      with a stated reason — not silently dropped.
- [ ] The generated parity guard (ticket 003) passes against the FINAL,
      post-reshape generated pair.
- [ ] **Read-back equals the file**: build from `tovez.json` (reshaped),
      aggregate `get_config()` over every `ConfigTarget`, diff against
      `tovez.json` — clean.
- [ ] **Bake/push parity**: build from `tovez.json`, push `tovez.json`'s
      values to a robot baked from `togov.json`, confirm identical
      resulting behavior to a robot baked from `tovez.json` directly.
- [ ] **No silent no-ops**: push to a boot-only target (`GEOMETRY` or
      `PLANNER`) and assert `ERR_NOT_LIVE`, not `OK`. Separately, the
      trap-1 regression test (ticket 015) is re-confirmed passing here
      as part of this ticket's own full-system pass.
- [ ] **16 touchpoints → 2, demonstrated**: add a throwaway config field
      to `robot_config.proto` and a robot JSON; show these are the ONLY
      two files touched (a diff, not a claim).
- [ ] All findings (pass/fail per bullet above) are recorded in this
      ticket's completion notes — this ticket does not "pass" partially;
      every bullet above is either checked or the ticket is not done.

## Testing

- **Existing tests to run**: this ticket IS the test run — see
  Acceptance Criteria.
- **New tests to write**: the 16→2 demonstration, the read-back-equals-file
  diff, and the bake/push parity comparison are themselves new,
  disposable verification scripts/tests if none already exist from
  earlier tickets.
- **Verification command**: `uv run python -m pytest` (full default
  collection — `src/tests/sim` + `src/tests/unit` + `src/tests/testgui`,
  NOT the `src/tests/sim`-only slice) plus the ARM build command plus
  each specific harness/demonstration named above.

## Implementation Plan

**Approach**: Work through the Acceptance Criteria list top to bottom;
this ticket is verification-only, not new feature work — if a bullet
fails, the fix belongs in whichever earlier ticket owns that property
(reopen it) rather than patched in here, unless the fix is trivial and
clearly scoped to verification tooling itself.

**Files to modify**: none expected beyond test/harness files, unless a
genuine bug is found in earlier work, in which case the fix lands in the
owning file and is noted in completion notes.

**Testing plan**: as above.

**Documentation updates**: `sprint.md`'s own Success Criteria section can
be checked off / annotated with this ticket's findings once complete (a
team-lead/sprint-closure concern, not blocking this ticket's own
completion).
