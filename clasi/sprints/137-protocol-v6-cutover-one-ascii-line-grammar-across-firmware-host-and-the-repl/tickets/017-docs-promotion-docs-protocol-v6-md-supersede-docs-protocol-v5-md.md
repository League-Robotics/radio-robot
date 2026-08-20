---
id: '017'
title: Docs promotion (docs/protocol-v6.md) + supersede docs/protocol-v5.md
status: open
use-cases: [SUC-008]
depends-on: ['016']
github-issue: ''
issue:
- protocol-v6-spec.md
- protocol-and-implementation-simplification-review.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Docs promotion (docs/protocol-v6.md) + supersede docs/protocol-v5.md

## Description

**Scope is `docs/` ONLY.** Promote this sprint's linked issue
(`protocol-v6-spec.md`) to `docs/protocol-v6.md`, and mark
`docs/protocol-v5.md` as superseded — the sprint's final ticket, run only
after ticket 016's hardware verification passes (Design Rationale
Decision 4: "shipped" reads as hardware-proven, not merely code-complete).

Do **not** touch `.claude/rules/hardware-bench-testing.md`'s
protocol-v5-specific bench-flow table (its `config()`/`otos_config()`
ack-ring specifics go stale once ticket 013 lands) — that update is
explicitly flagged to the stakeholder/team-lead in `sprint.md`'s Step 7 open
questions as a process-doc decision outside this sprint's scope, and the
team-lead handles it at sprint close, not this ticket. This ticket is
dispatched to a programmer agent since `docs/` is outside sprint-planner's
and team-lead's write scope (`sprint.md` Scope note).

## Acceptance Criteria

- [ ] `docs/protocol-v6.md` exists, containing the promoted spec content
      (updated to reflect anything that changed during implementation vs.
      the original proposal).
- [ ] `docs/protocol-v5.md` is marked superseded, pointing to
      `docs/protocol-v6.md`.
- [ ] This ticket touches only files under `docs/` — no changes to
      `.claude/rules/hardware-bench-testing.md` or any other rules file
      (explicitly out of scope here; flagged to team-lead separately).
- [ ] This ticket does not start until ticket 016 (hardware verification)
      has passed.
- [ ] The sprint's closing report is expected to carry forward this
      ticket's own flagged open items from `sprint.md`'s Step 7 (the REPL
      host-side reinterpretation, the `hardware-bench-testing.md`
      fast-follow, the `src/protos/*.proto` retain-vs-delete boundary) as
      stakeholder-facing notes — this ticket itself does not resolve them,
      only surfaces that they remain open.

## Testing

- **Existing tests to run**: N/A (docs-only ticket).
- **New tests to write**: N/A.
- **Verification command**: N/A — manual review that `docs/protocol-v6.md`
  accurately reflects the shipped, hardware-verified v6 protocol and
  `docs/protocol-v5.md`'s supersession note is present.
