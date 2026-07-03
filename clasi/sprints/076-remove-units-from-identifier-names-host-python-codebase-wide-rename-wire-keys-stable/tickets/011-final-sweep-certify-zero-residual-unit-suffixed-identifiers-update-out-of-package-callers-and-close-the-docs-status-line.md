---
id: '011'
title: 'Final sweep: certify zero residual unit-suffixed identifiers, update out-of-package
  callers, and close the docs status line'
status: open
use-cases:
- SUC-008
- SUC-009
depends-on:
- '006'
- '009'
- '010'
github-issue: ''
issue: remove-units-from-identifier-names-host-python.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Final sweep: certify zero residual unit-suffixed identifiers, update out-of-package callers, and close the docs status line

## Description

This is the sprint's closing ticket. It exists because the parent issue's
acceptance criteria are phrased as a whole-codebase invariant that only a
final, whole-tree grep can certify, and because five locations
(`tests/bench/`, `tests/field/`, `tests/_infra/calibrate/`,
`tests/_infra/tools/`, top-level `host/*.py`) have **zero automated test
coverage** and would otherwise silently accumulate stale keyword-argument
call sites across the whole sprint (`architecture-update.md` Step 5 "Why").

Scope:
- `host/calibrate_linear.py`, `host/calibrate_angular.py`,
  `host/calibrate_verify.py` (5 occurrences) — top-level scripts.
- `testkit/*` (2 occurrences).
- `tests/bench/*.py` (214 occurrences, 22 files) — **no automated
  protection**; every keyword-argument call site of every function/method
  renamed by tickets 001–010, and this tree's own local unit-suffixed
  identifiers (per the issue's "host-side tools/scripts" framing), updated
  here.
- `tests/field/*.py` (22 occurrences, 4 files) — same treatment.
- `tests/_infra/calibrate/*.py` (115 occurrences, 10 files) — same
  treatment.
- `tests/_infra/tools/*.py` (10 occurrences, 2 files) — same treatment.
- `config/robot_config.py` — **confirm-only, zero renames expected**: this
  pass's own full read found 13 pydantic `BaseModel` classes with zero
  `Field(alias=...)` anywhere, meaning every JSON key **is** the bare
  Python attribute name (Decision 4). This ticket's job is to re-confirm
  that exclusion still holds on the checked-out code, not to edit the
  file.
- `docs/coding-standards.md` — update the Python-convention section's
  status line from "not yet applied to any `host/` file ... sprint 072 is
  the sprint that will apply it" to state that **sprint 076** applied it
  (the work slipped five sprints per the roadmap; the convention text
  itself is unchanged).
- A final repo-wide certification grep (see Acceptance Criteria).

**Explicitly out of scope** (do not touch, per `architecture-update.md`'s
Decisions 1 and 3): `tests/old/` (36 files, 145 occurrences — deprecated,
`norecursedirs`-excluded, zero coverage); `tests/simulation/`'s and
`tests/_infra/sim/`'s own internal mock-class identifiers that mirror
*firmware* C++ naming (e.g. `t0Ms`, `encLMm`, `sTimeoutMs` in
`test_motion_command.py`'s `MotionBaseline`/`HardwareState` classes) — only
their `robot_radio` call sites are updated, as a mechanical consequence of
tickets 001–010's renames, and only if not already caught by an earlier
ticket incidentally touching the same file.

## Hard Contract (applies to this and every sprint 076 ticket)

- **Pure rename — no behavioral change.**
- **Every renamed declaration carries a `# [unit]` comment** (for the
  top-level scripts and `testkit/*`; the bench/field/infra trees are
  primarily caller-side keyword-argument updates, but any of their own
  locally-declared unit-suffixed identifiers get the same treatment).
- **Wire keys/tokens and pydantic attributes are STABLE.** `git diff` must
  show **zero changes** to `data/robots/*.json`, `robot_config.schema.json`,
  and `config/robot_config.py`.
- **Full suite green throughout**:
  - `uv run python -m pytest -q` = **2682 passed, 0 failed**.
  - `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/ -q`
    = **579 passed, 2 xfailed**.
- **No automated protection for bench/field/infra trees** — a stakeholder
  with hardware access should spot-run at least one bench script (e.g.
  `tests/bench/smoke_ritual.py`) post-sprint as a manual gate; flag this to
  the team-lead when reporting this ticket complete.
- **Ignore environmental `data/robots` drift** — if `git status` shows a
  pre-existing, unrelated modification to `data/robots/*.json` from before
  this sprint started, do not touch it and do not attribute it to this
  ticket; this ticket's own acceptance criterion is that *this ticket's
  own diff* introduces zero changes there.

## Acceptance Criteria

- [ ] `host/calibrate_linear.py`, `host/calibrate_angular.py`,
      `host/calibrate_verify.py` (5 occurrences) renamed, converging on
      every prior ticket's decided names for shared identifiers.
- [ ] `testkit/*` (2 occurrences) renamed.
- [ ] `tests/bench/*.py` (214 occurrences, 22 files): every keyword-argument
      call site of every renamed function/method updated; every local
      unit-suffixed identifier renamed with a `# [unit]` comment.
- [ ] `tests/field/*.py` (22 occurrences, 4 files): same treatment.
- [ ] `tests/_infra/calibrate/*.py` (115 occurrences, 10 files): same
      treatment.
- [ ] `tests/_infra/tools/*.py` (10 occurrences, 2 files): same treatment.
- [ ] `config/robot_config.py`: `git diff` shows **zero changes** — this
      ticket only confirms the wholesale pydantic exclusion still holds
      (13 classes, no `Field(alias=...)`), it does not edit this file.
- [ ] `data/robots/*.json` and `robot_config.schema.json`: `git diff` shows
      zero changes attributable to this ticket.
- [ ] `docs/coding-standards.md`'s Python-convention section states the
      convention has been applied by **sprint 076** (not the originally
      forward-referenced "072"); no other text in that section changes.
- [ ] Final repo-wide grep:
      `grep -rniE "\b[a-z_][a-z0-9_]*_(mm|mms|deg|dps|ms|us|pct|hz)\b" host/ tests/`
      — excluding `tests/old/`, `tests/simulation/`, `tests/_infra/sim/`,
      and this sprint's own planning documents' historical prose — returns
      **zero** results outside documented exclusions (wire-key strings,
      `config/robot_config.py`'s pydantic fields, `sim_prefs.py`'s SIMSET
      mapping-table values).
- [ ] `uv run python -m pytest -q` = 2682 passed, 0 failed.
- [ ] `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/ -q`
      = 579 passed, 2 xfailed.
- [ ] The issue `remove-units-from-identifier-names-host-python.md`'s own
      acceptance criteria (no unit-suffixed identifier outside documented
      exclusions; every renamed declaration carries the `# [unit]`
      comment; pure rename; wire compatibility preserved) are satisfied
      repo-wide, not just within this ticket's own file set.
- [ ] Hard Contract above holds.

## Testing

- **Existing tests to run**: full default suite plus the testgui tier (see
  Verification commands). `tests/bench/`, `tests/field/`,
  `tests/_infra/calibrate/`, `tests/_infra/tools/` have no automated
  pytest collection (`--collect-only` returns "no tests collected" for
  `tests/bench` and `tests/field`, confirmed in `architecture-update.md`
  Step 1) — verification there is the repo-wide grep plus, ideally, a
  stakeholder manual bench-script run.
- **New tests to write**: none required — pure rename.
- **Verification commands**:
  - `uv run python -m pytest -q` (confirm 2682 passed, 0 failed).
  - `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/ -q`
    (confirm 579 passed, 2 xfailed).
  - `grep -rniE "\b[a-z_][a-z0-9_]*_(mm|mms|deg|dps|ms|us|pct|hz)\b" host/ tests/`
    (excluding `tests/old/`, `tests/simulation/`, `tests/_infra/sim/`) —
    zero results outside documented exclusions.
  - `git diff --stat -- data/robots/ host/robot_radio/config/robot_config.py`
    — empty output.

## Implementation Plan

**Approach**: Sweep the no-coverage trees first (bench/field/infra/
top-level scripts/testkit), since they carry the highest silent-breakage
risk, then run the certification grep, then close the docs status line,
then do the final full-suite runs.

1. `host/calibrate_linear.py`, `calibrate_angular.py`, `calibrate_verify.py`
   — rename the 5 occurrences, converging on prior tickets' names.
2. `testkit/*` — rename the 2 occurrences.
3. `tests/bench/*.py` (22 files) — for each file, update every renamed
   function/method's keyword-argument call sites and any local
   unit-suffixed identifier; add `# [unit]` comments to local
   declarations.
4. `tests/field/*.py` (4 files) — same treatment.
5. `tests/_infra/calibrate/*.py` (10 files) — same treatment.
6. `tests/_infra/tools/*.py` (2 files) — same treatment.
7. Read `config/robot_config.py` in full one more time; confirm the
   13-class pydantic exclusion still holds and `git diff` shows zero
   changes to this file.
8. Run the certification grep across `host/` and `tests/` (excluding
   `tests/old/`, `tests/simulation/`, `tests/_infra/sim/`); resolve any
   unexpected hit by tracing it back to the ticket that should have caught
   it, or by renaming it here if it's genuinely a missed occurrence in
   this ticket's own scope.
9. Update `docs/coding-standards.md`'s status line to reference sprint 076.
10. Run the full default suite, then the testgui tier, confirming both
    baselines.
11. Note in the ticket's completion notes that a stakeholder should
    spot-run at least one bench script (e.g. `tests/bench/smoke_ritual.py`)
    post-sprint as a manual gate beyond this ticket's grep certification.

**Files to create/modify**:
- `host/calibrate_linear.py`, `host/calibrate_angular.py`,
  `host/calibrate_verify.py`
- `host/robot_radio/testkit/*` (2 occurrences)
- `tests/bench/*.py` (22 files)
- `tests/field/*.py` (4 files)
- `tests/_infra/calibrate/*.py` (10 files)
- `tests/_infra/tools/*.py` (2 files)
- `host/robot_radio/config/robot_config.py` — confirm-only, no edit
  expected.
- `docs/coding-standards.md` — one status-line edit.

**Testing plan**: Run `uv run python -m pytest -q` (2682 passed, 0
failed), `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/ -q`
(579 passed, 2 xfailed), and the repo-wide certification grep. Recommend a
stakeholder-run bench script (e.g. `tests/bench/smoke_ritual.py`) as a
post-sprint manual gate, since bench/field/infra trees have no automated
coverage.

**Documentation updates**: `docs/coding-standards.md`'s Python-convention
status line, updated to state sprint 076 (not 072) applied the convention
to `host/`.
