---
id: 008
title: Documentation sweep + reorganization-proposal verification and closure
status: open
use-cases: ["SUC-002", "SUC-004"]
depends-on: ["007"]
github-issue: ''
issue:
- firmware-layering-cleanup-interfaces-to-hal-impls-to-platform-microbit.md
- proposal-platform-hardware-hal-core-reorganization.md
completes_issue:
  proposal-platform-hardware-hal-core-reorganization.md: false
# Left off default (true) deliberately for the proposal issue only: this
# ticket's own Acceptance Criteria allow the outcome "leave the issue
# open/re-scoped to steps 7-8" as a valid judgment call, not just "move to
# done" -- auto-archival on ticket completion would pre-empt that call.
# The layering-cleanup issue (this ticket's other linked issue) keeps the
# default: it archives once every ticket referencing it (003-009) is done.
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Documentation sweep + reorganization-proposal verification and closure

## Description

Phase 6 of the layering-cleanup issue, combined with verifying and closing
out `proposal-platform-hardware-hal-core-reorganization.md`'s remaining
open item — both are "catch the docs up to the tree" work.

**Part A — documentation sweep.** Update every doc that describes the
pre-ticket-003-through-007 tree: `CLAUDE.md`'s "Architecture — layered
firmware" block (`com/` removed from the layer list, `control/` added,
the control-law ownership sentence repointed from `core/differential_
drive.cpp` to `control/differential_drive.cpp` and `Core::` to
`Control::`), `docs/design/design.md` §2's subsystem table, every touched
subsystem's own co-located `DESIGN.md` (`hal/`, `platform/`, `hardware/`,
`core/`, `kinematics/`, `control/` — the last one created in ticket 006,
confirm it's complete rather than recreating it), plus two specific stale
cross-references named during planning: `.claude/rules/hardware-bench-
testing.md`'s claim that firmware applies `radiochan::kDefault` (already
false — the actual source is `Config::kRadioChannel`, baked from the
robot JSON's `connection.radio_channel`, per `main.cpp`'s composition —
confirm the current line number post-ticket-005's main.cpp shrinkage
before citing it), and `src/protos/robot_config.proto`'s stale
`src/motion` doc-comment paths (should read `src/firm/motion`). Also
sweep `data/robots/*.json` note strings for any pre-reorg path reference.

**Part B — proposal verification and closure.**
`proposal-platform-hardware-hal-core-reorganization.md`'s own Sequencing
section claims steps 1-6 landed via commits `8a86f5bd` and `1c7b70d3`
(2026-08-09): `platform/`, `hardware/{generic,nezha,hiwonder,planetx}/`,
`hal/`, `app/`→`core/`, `Drive`→`DifferentialDrive`, motion moved in,
kinematics extracted. **Do not just re-assert this claim — independently
verify it** (grep/read the actual current tree and cite the evidence),
the same standard sprint.md's own Architecture section held itself to
during planning. Step 6 (`FakeOtos` relocation) is resolved by this
sprint's ticket 003 deletion — confirm that deletion actually landed and
cite it. Update the issue so it states steps 7-8 (transport generalization
+ `WifiTransport`; `Hal::Wheel` + Robot/RobotState formalization) as the
only remaining open work, each explicitly owned by a named future sprint
or issue — not implicitly left dangling as "this sprint's leftover."
Then either move the issue to done (if the update makes it fully
resolved) or leave it open/re-scoped to exactly steps 7-8 (judgment call
— document which and why in Completion Notes).

## Acceptance Criteria

- [ ] `CLAUDE.md`'s architecture block reflects: `com/` removed;
      `control/` added between `hal/`/`kinematics/` and `core/`; the
      control-law sentence and its file-path citation repointed to
      `Control::DifferentialDrive` / `control/differential_drive.cpp`.
- [ ] `docs/design/design.md` §2's subsystem table: `com/` row removed
      (with a pointer to where its responsibilities went); new `control/`
      row added; `hal/`/`platform/`/`hardware/`/`core/`/`kinematics/`
      rows updated to match the moved/renamed files.
- [ ] Every touched subsystem's own `DESIGN.md` updated to match its
      current contents (`hal/`, `platform/`, `hardware/`, `core/`,
      `kinematics/`); `control/DESIGN.md` confirmed complete (created in
      ticket 006).
- [ ] `.claude/rules/hardware-bench-testing.md`'s `radiochan::kDefault`
      claim corrected to name `Config::kRadioChannel` and its actual
      source, with a current (post-cleanup) line-number citation.
- [ ] `src/protos/robot_config.proto`'s stale `src/motion` doc-comment
      paths updated to `src/firm/motion`.
- [ ] `data/robots/*.json` note strings referencing pre-reorg paths (if
      any found by grep) updated.
- [ ] `proposal-platform-hardware-hal-core-reorganization.md`'s steps 1-6
      independently re-verified against the current tree — the specific
      commands/greps run are cited as evidence in Completion Notes, not
      just asserted.
- [ ] The issue's step 6 marked resolved, citing ticket 003's deletion.
- [ ] The issue updated to name steps 7-8 as the only remaining open
      work, each attributed to a named future sprint/issue.
- [ ] The issue moved to done, or explicitly left open/re-scoped with a
      documented reason.
- [ ] `just build-sim` and ARM build both still clean (docs-only ticket,
      confirm no regression).

## Testing

- **Existing tests to run**: `just build-sim`, `uv run python -m pytest
  src/tests/sim/unit/test_layer_isolation.py` (confirm no regression from
  a docs-only ticket).
- **New tests to write**: none — documentation and issue-tracking only.
- **Verification command**: the specific greps/checks named in the
  acceptance criteria above, each cited with its actual output in
  Completion Notes.
