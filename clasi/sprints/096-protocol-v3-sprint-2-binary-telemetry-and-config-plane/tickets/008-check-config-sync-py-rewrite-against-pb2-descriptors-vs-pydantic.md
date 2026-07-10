---
id: '008'
title: check_config_sync.py rewrite against pb2 descriptors vs pydantic
status: open
use-cases: [SUC-007]
depends-on: ['001']
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# check_config_sync.py rewrite against pb2 descriptors vs pydantic

## Description

Full rewrite of `scripts/check_config_sync.py` — its CURRENT
implementation targets `source/types/Config.h` and `source/robot/
ConfigRegistry.cpp`, neither of which exists anywhere in the current
`source/` tree (both are `source_old`-era artifacts from before the
greenfield rebuild; confirmed by direct search during architecture
planning). This is not a "retool" of live logic — there is none to retool
— see architecture-update.md Decision 6. Depends on ticket 001 (the pb2
descriptors for the curated Patch messages must exist).

**Approach**:
1. Replace the old Set-A (struct fields)/Set-B (registered keys)/Set-C
   (firmware usage) comparison entirely. New comparison: diff
   `host/robot_radio/config/robot_config.py`'s pydantic model fields
   against the generated `pb2` descriptors for `DrivetrainConfigPatch`/
   `MotorConfigPatch`/`PlannerConfigPatch` (ticket 001's curated wire
   surface) — the fields a binary client can actually set/get, NOT the
   full internal `DrivetrainConfig`/`MotorConfig`/`PlannerConfig`
   messages (most of which have no wire-config verb at all and are
   correctly out of this comparison's scope).
2. Report: a pydantic field with no corresponding Patch descriptor field
   (invisible to the binary config plane); a Patch descriptor field with
   no corresponding pydantic field (host can't represent something the
   wire allows); any type/bound mismatch between the two, where checkable.
3. Preserve `scripts/config_sync_allowlist.json`'s role as an escape
   hatch for known-intentional exceptions (same file, new category names
   reflecting the new comparison — document the mapping from the old
   Set-A/B/C language to the new pydantic-vs-pb2-descriptor comparison in
   the script's own module docstring, so a future reader isn't confused
   by the history).
4. Update the script's own module docstring to describe the NEW
   comparison model — do not leave stale references to `Config.h`/
   `ConfigRegistry.cpp`.

**Files to modify**: `scripts/check_config_sync.py`,
`scripts/config_sync_allowlist.json` (format/category names updated).

## Acceptance Criteria

- [ ] `python scripts/check_config_sync.py` exits 0 against the current
      tree (or reports genuine, fixable drift — never a crash from a
      missing input file, which is the CURRENT broken behavior).
- [ ] A field present in the pydantic model but absent from the curated
      pb2 Patch descriptors (or vice versa) is reported, not silently
      ignored.
- [ ] The allowlist mechanism (`scripts/config_sync_allowlist.json`) is
      preserved in spirit (an escape hatch for known-intentional
      exceptions) with its new category names documented in the script's
      own module docstring.
- [ ] The script's module docstring no longer references
      `source/types/Config.h` or `source/robot/ConfigRegistry.cpp`.

## Testing

- **Existing tests to run**: none directly test this script today (it was
  non-functional); confirm no other script/CI step depends on its old
  Set-A/B/C output format.
- **New tests to write**: a test (or a manual verification step, if the
  script has no existing test harness) confirming the script exits 0 on
  the current tree post-rewrite, and correctly flags an intentionally
  introduced pydantic/pb2 mismatch in a scratch scenario.
- **Verification command**: `python scripts/check_config_sync.py`
