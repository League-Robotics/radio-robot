---
id: '004'
title: Delete check_config_sync.py + allowlist + CI job
status: done
use-cases:
- SUC-001
depends-on:
- '003'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Delete check_config_sync.py + allowlist + CI job

## Description

Delete `src/scripts/check_config_sync.py` and
`src/scripts/config_sync_allowlist.json` outright (Design Rationale
Decision 5 — the generated parity guard from ticket 003 is a strictly
stronger, mechanical guarantee; the old lint's hand-curated
`PATCH_TO_PYDANTIC` map and 58-entry allowlist have nothing left to
compare once its premise — hand-maintained definitions that can drift —
no longer holds). Remove the "Config registry sync lint" CI job from
`.github/workflows/build.yml`, confirmed present at lines 8-19: the job
named `Config registry sync lint (check_config_sync.py)` that runs
`python src/scripts/check_config_sync.py`.

## Acceptance Criteria

- [x] `src/scripts/check_config_sync.py` does not exist on disk.
- [x] `src/scripts/config_sync_allowlist.json` does not exist on disk.
- [x] `.github/workflows/build.yml` no longer contains the
      `config-registry-sync` job (removed, not renamed).
- [x] A repo-wide grep for `check_config_sync` and `config_sync_allowlist`
      outside git history returns nothing.
- [x] CI (or the local equivalent) has no dangling reference that would
      fail on a missing script.

## Implementation Notes

- Deleted `src/scripts/check_config_sync.py`,
  `src/scripts/config_sync_allowlist.json`, and
  `src/tests/unit/test_check_config_sync.py` (the test that existed
  solely to test the lint — not listed by path in the ticket body but
  matched "Any test that exists solely to test the lint").
- Removed the `config-registry-sync` job from
  `.github/workflows/build.yml` (was lines 8-19). Confirmed no other
  job's `needs:` referenced it (none of `build.yml`'s jobs use `needs:`
  at all) and no other workflow file under `.github/workflows/`
  mentioned the job or the script.
- Repo-wide grep for `check_config_sync`/`config_sync_allowlist`
  restricted to `*.py`/`*.yml` (the ticket's own verification command)
  now returns only two explanatory, past-tense mentions that name what
  replaced the deleted lint: `src/tests/unit/test_config_parity_capi.py`
  and `src/scripts/gen_messages.py` (both already say "deleted by/next,
  ticket 004"). Left as-is — these are exactly the kind of forward
  pointer the ticket's own description asks for ("a future reader...
  needs to see what took over"), not dangling references.
- Found and fixed three additional *stale* (not merely historical)
  references outside the ticket's named file list, since they asserted
  the lint still exists rather than explaining its replacement:
  - `build.py` had a comment claiming `check_config_sync.py` is "a
    separate CI lint... nothing to condition for it here" — removed
    (nothing left to condition for).
  - `src/README.md`'s `scripts/` table row listed `check_config_sync.py
    (CI lint)` as part of the directory's current contents — removed
    the clause, reworded "Codegen + lint tooling" to "Codegen tooling".
  - `src/scripts/DESIGN.md` described the script as `live, CI-only` in
    its orientation table, a Constraints bullet, and an Interfaces
    bullet — all three removed; replaced the orientation-table lead-in
    with a short note naming the replacement
    (`config_parity_capi.{h,cpp}` + `test_config_parity_capi.py`,
    132-003) so the doc doesn't just silently lose the history.
- Left every reference inside `clasi/sprints/done/**`,
  `clasi/issues/done/**`, `docs/architecture/architecture-update-*.md`,
  and this sprint's own `sprint.md`/`issues/the-configuration-object.md`
  untouched — those are historical planning/architecture records, out
  of scope for a code-deletion ticket to rewrite.
- Left `src/protos/drivetrain.proto`'s one comment (citing "the
  `check_config_sync.py` allowlist precedent" as design rationale for
  why `ekf_r_fix_xy`/`ekf_r_fix_theta` are wire-tunable rather than
  JSON-baked) and `src/firm/config/config_parity_capi.h`'s doc comment
  (naming what it replaces) untouched for the same reason as the two
  `.py` files above — explanatory, not dangling.

## Testing

- **Existing tests to run**: n/a — deletion only.
- **New tests to write**: none — this ticket removes a check; ticket 003
  already added its replacement.
- **Verification command**: `grep -rn "check_config_sync\|config_sync_allowlist"
  --include="*.py" --include="*.yml" .` returning nothing outside `.git/`.

## Implementation Plan

**Approach**: Delete both files; edit `build.yml` to remove the job
block.

**Files to delete**: `src/scripts/check_config_sync.py`,
`src/scripts/config_sync_allowlist.json`.

**Files to modify**: `.github/workflows/build.yml` (remove lines 8-19's
job).

**Testing plan**: as above.

**Documentation updates**: none.
