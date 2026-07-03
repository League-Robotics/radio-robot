---
id: '006'
title: 'Navigation modules: rename unit-suffixed identifiers in go-to and path-approach
  math'
status: open
use-cases:
- SUC-005
depends-on:
- '002'
github-issue: ''
issue: remove-units-from-identifier-names-host-python.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Navigation modules: rename unit-suffixed identifiers in go-to and path-approach math

## Description

`host/robot_radio/nav/navigator.py`, `_approach_utils.py`, `camera_goto.py`,
and `nav_params.py` compute navigation/path-following commands (go-to,
PurePursuit-adjacent approach math). `nav/camera_goto.py` has **zero**
`robot_radio` imports (pure math, duck-typed robot argument) — independent
of every other subsystem's rename order; `navigator.py` depends on
`controllers/pid.py` (zero hits) and its own `nav/` siblings only
(`architecture-update.md` Step 2).

**Dependency note**: filed with `depends-on: [002]` rather than as a
second root ticket. `architecture-update.md`'s Step 4a dependency-graph
diagram omits an explicit `T002 → T006` edge, but Step 5's own "Why"
section states plainly that "Tickets 004/005/006 are mutually independent
(each depends only on 002 ...)" — and `usecases.md` SUC-005's Main Flow
confirms navigation "issues drive commands through the (already-renamed,
SUC-001) protocol layer using renamed locals." This ticket resolves that
minor diagram/prose inconsistency conservatively (depend on 002) so that
any renamed protocol-layer keyword argument this subsystem calls with is
guaranteed already converged (Decision 2) before this ticket starts — a
ticketing-detail decision within the sprint-planner's dependency-ordering
authority, not a reopening of the architecture review.

Renames (per Step 5): `nav/navigator.py` (`tolerance_mm`/`speed_mms` →
bare names with `# [unit]`); `nav/_approach_utils.py` (`r_mm` → `radius
# [mm]`); `nav/camera_goto.py` (`target_mm` → `target  # [mm]`);
`nav/nav_params.py`.

`nav/pose.py`, `nav/pose_align.py`, and `controllers/pid.py` are already
clean (zero unit-suffix hits, Step 1) — no edit expected.

Total scope: 163 rename-eligible occurrences (Step 3).

## Hard Contract (applies to this and every sprint 076 ticket)

- **Pure rename — no behavioral change.** Navigation trajectories and
  arrival decisions must be numerically identical to pre-076 for the same
  inputs.
- **Every renamed declaration carries a `# [unit]` comment.**
- **No wire-key surface in this layer** — navigation issues drive commands
  through the already-renamed `robot/protocol.py` (ticket 002); nothing in
  `nav/` itself builds a wire string directly.
- **Full suite green throughout**: `uv run python -m pytest -q` remains
  **2682 passed, 0 failed**.
- **Cross-cutting kwargs**: any call into `robot/protocol.py` or
  `robot/`-layer methods using a ticket-002/003-renamed keyword argument
  must already use the converged name; fix any stale one found here.
- **Ignore environmental `data/robots` drift.**

## Acceptance Criteria

- [ ] `nav/navigator.py`: `tolerance_mm`/`speed_mms` → bare names with
      `# [unit]`.
- [ ] `nav/_approach_utils.py`: `r_mm` → `radius` with `# [mm]`.
- [ ] `nav/camera_goto.py`: `target_mm` → `target` with `# [mm]`.
- [ ] `nav/nav_params.py`: all unit-suffixed identifiers renamed with
      `# [unit]` comments.
- [ ] `nav/pose.py`, `nav/pose_align.py`, `controllers/pid.py` are
      confirmed to remain clean (zero unit-suffixed identifiers) — no edit
      expected.
- [ ] `nav/camera_goto.py`'s duck-typed `robot` argument handling is
      unaffected — this file has zero `robot_radio` imports and its rename
      is fully self-contained.
- [ ] Navigation-related unit/system tests pass with unchanged numeric
      assertions (per `usecases.md` SUC-005).
- [ ] Hard Contract above holds.

## Testing

- **Existing tests to run**: navigation-related unit tests in
  `tests/simulation/unit/` (grep for `Navigator`/`camera_goto`/
  `_approach_utils` imports to enumerate the exact files).
- **New tests to write**: none required — pure rename.
- **Verification command**: `uv run python -m pytest -q` (confirm 2682
  passed, 0 failed).

## Implementation Plan

**Approach**: Rename file-by-file; confirm `camera_goto.py`'s
zero-import independence means it can be renamed in any order relative to
the rest of this ticket's files.

1. `nav/navigator.py` — rename `tolerance_mm`/`speed_mms` and any other
   unit-suffixed identifier.
2. `nav/_approach_utils.py` — rename `r_mm` → `radius`.
3. `nav/camera_goto.py` — rename `target_mm` → `target`; verify no
   `robot_radio` import is introduced by the rename itself.
4. `nav/nav_params.py` — rename remaining unit-suffixed identifiers.
5. Confirm `nav/pose.py`, `nav/pose_align.py`, `controllers/pid.py` need no
   edit.
6. Grep this file set for every renamed identifier's old name and for any
   protocol-layer keyword call site to confirm convergence on ticket
   002/003's decided names.
7. Run navigation-related unit tests, then the full suite.

**Files to create/modify**:
- `host/robot_radio/nav/navigator.py`
- `host/robot_radio/nav/_approach_utils.py`
- `host/robot_radio/nav/camera_goto.py`
- `host/robot_radio/nav/nav_params.py`
- `host/robot_radio/nav/pose.py`, `nav/pose_align.py`,
  `host/robot_radio/controllers/pid.py` — reviewed only, no edit expected.

**Testing plan**: Run navigation-related unit tests individually, then
`uv run python -m pytest -q` and confirm the 2682 baseline holds.

**Documentation updates**: None in this ticket.
