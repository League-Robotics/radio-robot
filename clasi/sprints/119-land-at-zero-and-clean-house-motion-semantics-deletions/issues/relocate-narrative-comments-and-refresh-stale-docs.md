---
status: in-progress
filed: 2026-07-22
filed_by: "team-lead (turn-execution review \xA76-5/\xA76-6/D6, measured 2026-07-22)"
related:
- specify-and-assert-the-leg-handoff-contract.md
- restore-the-interleaved-request-settle-tick-loop-schedule.md
sprint: '119'
tickets:
- 119-004
---

# Relocate narrative comments to DESIGN.md/git; refresh stale doc references

## Description

The contract belongs in the header; the history belongs in DESIGN.md and
git. Measured narrative surface (2026-07-22):

- `src/firm/app/move_queue.h`: 374 lines, **265 comment (71%)**, 82 code;
  53-line file-header essay.
- `src/sim/sim_harness.h`: 595 lines, **374 comment (63%)**; 84-line header
  essay.
- `src/host/robot_radio/planner/tour.py`: 104-line module docstring + a
  mostly-prose constants block (`:229-288`).
- `test_tour_closure_gate.py`: `_XFAIL_REASON_IDEAL` = **116 lines**
  (`:549-664`), `_XFAIL_REASON_REALISTIC` = 65 lines (`:666-730`), inline
  reason 38 lines (`:822-859`), `_STOP_LEAD_MS` block ~51 lines
  (`:113-167`).

Mechanical relocation, zero behavior risk: keep the contract (what/invariants
/units), move sprint archaeology into the owning `DESIGN.md` (or delete where
git already tells the story), and shrink xfail reasons to one sentence + a
live issue link.

## Stale references to fix (verified locations)

1. `.claude/rules/hardware-bench-testing.md` — asserts protocol v2 is
   current (`:37-38`), links `source/commands/CommandProcessor.cpp` (`:40`,
   dead path) and `docs/protocol-v2.md §13` (`:45`); STALE smoke table
   (`:42+`). Rewrite the deploy/verify flow against docs/protocol-v4.md +
   MOVE-era bench scripts. (NOT CLAUDE.md — the review misattributed;
   CLAUDE.md's only rot is a generic `source/` mention at `:13` → `src/`.)
2. `.claude/rules/coding-standards.md:170` — `source/commands/
   SystemCommands.cpp` dead path (the HALT POS wire-string example);
   re-point to its live equivalent or mark as historical example.
3. TestGUI panel label "Managed — Ruckig"
   (`testgui/__main__.py:925`; stale comments `:735,:758`) — Ruckig was
   never shipped and was explicitly rejected; rename to "Managed" (or
   "Managed — MOVE").
4. Dangling xfail citations of the deleted
   `clasi/issues/cycle-order-reorder-experiment-ab-before-hardware.md` in
   `test_tour_closure_gate.py` and `src/tests/sim/unit/test_app_robot_loop.py`
   — re-point at `restore-the-interleaved-request-settle-tick-loop-schedule.md`
   (its live successor).
5. `docs/specification.md:72,333` + `docs/architecture.md:11,46,108,154,157`
   — still describe the deleted ASCII CommandProcessor / `source/app/`
   architecture; refresh or mark superseded-by pointers to docs/design/.
6. **No rule/doc anywhere points at docs/protocol-v4.md as current** — every
   protocol pointer is v2-era. Fix the pointers in .claude/rules/ and doc
   entry points.
7. `src/host/robot_radio/DESIGN.md:60` claims the planner package is
   "Dormant, by stakeholder decision" and import-broken — contradicted by
   live `run_tour()` callers across testgui/tests. Correct it to the actual
   status (tour path live; which parts are genuinely dormant, say so
   precisely).

## Scope guard

- Stop-lead-specific prose (`_STOP_LEAD_MS` block, `_estimator_note`s) is
  deleted by `land-at-zero-completion-delete-stop-lead.md` — this issue
  relocates/trims only what survives that deletion; sequence after it.
- Design-docs opt-in is enabled: DESIGN.md changes ride the sprint's design
  overlay and must pass `clasi design validate` at close.

## Acceptance

- move_queue.h / sim_harness.h / tour.py headers reduced to contract-only
  (target: comment ratio < 40% in the headers; no sprint numbers outside
  DESIGN.md/git references).
- Every listed stale reference fixed; grep gate for `source/commands/`,
  `protocol-v2.md` (outside docs/protocol-v2.md itself and archives),
  `Ruckig` (outside historical DESIGN.md notes), and the deleted issue
  filename.
- Full suite green; no behavior diffs (comment/docs/label changes only,
  except the GUI label string).
