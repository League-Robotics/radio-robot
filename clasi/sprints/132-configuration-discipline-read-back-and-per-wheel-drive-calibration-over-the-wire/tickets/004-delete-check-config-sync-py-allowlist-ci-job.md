---
id: '004'
title: Delete check_config_sync.py + allowlist + CI job
status: open
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

- [ ] `src/scripts/check_config_sync.py` does not exist on disk.
- [ ] `src/scripts/config_sync_allowlist.json` does not exist on disk.
- [ ] `.github/workflows/build.yml` no longer contains the
      `config-registry-sync` job (removed, not renamed).
- [ ] A repo-wide grep for `check_config_sync` and `config_sync_allowlist`
      outside git history returns nothing.
- [ ] CI (or the local equivalent) has no dangling reference that would
      fail on a missing script.

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
