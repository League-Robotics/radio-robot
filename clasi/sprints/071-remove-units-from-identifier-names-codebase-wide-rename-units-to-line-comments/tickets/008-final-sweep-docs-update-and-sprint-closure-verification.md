---
id: '008'
title: Final sweep, docs update, and sprint closure verification
status: open
use-cases: [SUC-001, SUC-002, SUC-003, SUC-004, SUC-005, SUC-006, SUC-007]
depends-on: ['005', '006', '007']
github-issue: ''
issue: remove-units-from-identifier-names.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Final sweep, docs update, and sprint closure verification

## Description

Close out the sprint: confirm zero remaining unit-suffixed identifiers in
`source/`, zero remaining `FIXME` markers related to this issue,
byte-identical wire output, and a green suite; update the prose
documentation files that quote specific C++ field names renamed by
tickets 002-007. This ticket is the sprint's own acceptance-criteria
closure, mirroring the issue's own acceptance criteria list line-for-line
(`architecture-update.md` Step 3).

This ticket depends on 005, 006, and 007 (every rename ticket must have
landed before the final whole-tree grep can certify the issue's
acceptance criterion: "No identifier in `source/`... embeds a unit
suffix... except documented exclusions").

Scope:
- `docs/protocol-v2.md`, `docs/architecture.md`, `docs/overview.md`,
  `docs/kinematics-model.md`: prose/table mentions of renamed C++ field
  names updated.
- Final `grep -rniE "\b[a-z_][a-z0-9_]*(mm|mms|deg|dps|us|pct|hz)\b"
  source/` (case-insensitive, word-boundary) and `grep -rn "FIXME"
  source/` both return **zero** results (excluding this sprint's own
  planning-doc prose, which lives outside `source/`).
- Full `uv run python -m pytest` run: confirm 2620 (or the then-current
  ticket-adjusted count) passed, 0 failed.
- Confirm no code change is otherwise needed — this is expected to be a
  documentation-only ticket unless the final sweep's grep surfaces a
  residual identifier tickets 002-007's own acceptance criteria didn't
  catch, in which case fix it here and note it.

If the final grep does surface a residual, treat fixing it as within this
ticket's scope (it is exactly the closure check this ticket exists to
run) — but if the residual is large or structurally surprising (e.g. an
entire file family tickets 002-007 missed), stop and flag it rather than
silently absorbing a second sweep's worth of work into a "final sweep"
ticket; report back rather than scope-creeping.

See `architecture-update.md` Step 5 ("008 — Final sweep, docs, closure"),
Step 7 Open Questions (esp. #1, sprint 072 recommendation), the
Architecture Self-Review's "Verdict: APPROVE"; `usecases.md` SUC-001
through SUC-007 (all).

## Acceptance Criteria

- [ ] `grep -rniE "\b[a-z_][a-z0-9_]*(mm|mms|deg|dps|us|pct|hz)\b"
      source/` returns zero results (word-boundary, case-insensitive;
      excludes wire-key string literals per the Wire-Compatibility
      Exclusion Table, which are not identifiers).
- [ ] `grep -rn "FIXME" source/` returns zero results.
- [ ] `docs/protocol-v2.md`, `docs/architecture.md`, `docs/overview.md`,
      `docs/kinematics-model.md` updated wherever they quote a C++
      field/identifier name renamed by tickets 002-007.
- [ ] Every `SET`/`GET`/`SIMSET`/`SIMGET`/`STREAM`/`TLM`/`SNAP` wire byte
      is identical before and after the full sprint (spot-check against
      `tests/_infra/golden_tlm_capture.json`, which requires no
      regeneration).
- [ ] Full test suite green (`uv run python -m pytest`): 2620 passed (or
      the then-current ticket-adjusted count from 002/005/006/007's own
      test updates), 0 failed.
- [ ] No `data/robots/*.json`, `host/robot_radio/config/robot_config.py`,
      or other `host/robot_radio/` file was modified across the whole
      sprint (Decision 1 and Decision 6 scope boundary — confirm via
      `git diff --stat` against the sprint's base commit).
- [ ] Sprint-level confirmation that this issue
      (`remove-units-from-identifier-names.md`) is only **partially**
      closed by sprint 071 (the `source/` C++ half) — the host-Python half
      remains open and is recommended for a follow-up sprint (072), per
      `architecture-update.md` Decision 1 and Open Question 1. This ticket
      does not mark the parent issue fully resolved.

## Testing

- **Existing tests to run**: full suite (`uv run python -m pytest`) as
  the final closure gate; a manual `SET`/`GET`/`SIMSET`/`SIMGET` smoke
  round-trip against a running sim instance is recommended but not
  required if the automated suite already covers the affected keys.
- **New tests to write**: none — this ticket verifies, it does not add
  new behavior.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Run the two closure greps first (unit-suffix, FIXME) across
`source/`. If either returns a result, fix it (small residual) or stop
and report (large/structural residual) before touching docs. Then sweep
the four named prose docs for stale identifier mentions, update them, and
run the full suite one final time as the sprint-closure gate.

**Files to modify**:
- `docs/protocol-v2.md`
- `docs/architecture.md`
- `docs/overview.md`
- `docs/kinematics-model.md`
- (contingently) any `source/` file if the final grep surfaces a small
  residual missed by 002-007

**Testing plan**: full suite run as the closure gate; no isolated test
tier needed since this ticket touches no runtime code path under normal
(no-residual) conditions.

**Documentation updates**: this ticket *is* the documentation-update pass
for the sprint's prose docs.
