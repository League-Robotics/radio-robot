---
status: pending
filed: 2026-07-23
filed_by: team-lead (119-004 out-of-scope findings)
related: []
sprint: '126'
---

# Stale Ruckig CMake comment + dead DEV-command-family docs in src/tests

Two staleness findings 119-004 surfaced but correctly left out of its scope:

1. `CMakeLists.txt:183-184` comment claims Ruckig is "restored ... load-bearing
   again" (sprint 109); sprint 115-002 re-deleted it permanently. Fix the
   comment (and check whether any Ruckig build machinery it describes is also
   dead weight).

2. `src/tests/DESIGN.md`, `src/tests/CLAUDE.md`, and several
   `src/tests/bench/*.py` scripts describe a `DEV` command family
   (docs/protocol-v2.md §16) that no longer exists anywhere in `src/firm/`
   (grep-verified 2026-07-23). Materially larger than a comment fix: the bench
   scripts that depend on DEV verbs are dead against current firmware and the
   phase-B bench session will discover which ones matter. Refresh or archive
   them; update the two docs.
