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
**corrected** pre-sprint baseline: **462 passed, 1 xfailed, 2 xpassed, 0
failed** (`uv run python -m pytest src/tests/sim -q`, measured on
`master` immediately pre-sprint, 2026-08-03). This corrects the stale
"484 passed / 2 known failures" figure this document previously carried
— 22 tests were removed by the `src/tests/dev/` reorganization since
that figure was recorded, not a coverage loss caused by this sprint; the
tree is genuinely green today, and
[[A-seven-untriaged-failing-tests-poison-every-no-regressions-claim]]'s
premise of seven failing tests on a clean tree is also stale — do not
carry either stale premise forward.

Assemble and prove every Success Criteria property from `sprint.md` in
one place.

## Acceptance Criteria

- [ ] Full sim suite run: `uv run python -m pytest src/tests/sim -q` at
      or above **462 passed, 0 failed**. Any count DROP is investigated
      and explained, not silently accepted.
- [ ] The 2 `xpassed` tests from the baseline are triaged: either their
      `xfail` marker is removed (if genuinely fixed) or a documented
      reason is recorded for why they stay marked — not silently
      re-inherited. An xpass is a quiet lie about what the suite proves.
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
- **Verification command**: `uv run python -m pytest src/tests/sim -q`
  plus the ARM build command plus each specific harness/demonstration
  named above.

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
