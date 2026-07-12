---
id: '006'
title: Sprint 098 closure validation
status: open
use-cases: [SUC-005]
depends-on: ['003']
github-issue: ''
issue:
- heading-loop-cascade-control-turns-terminate-on-target.md
- real-robot-motion-calibration-undershoot.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 098 closure validation

## Description

Sprint-level closure validation. Re-confirm the whole sprint's deliverable
— whichever combination of tickets 001-005 actually shipped — builds
clean, passes the full sim suite, and holds on real hardware, before the
sprint is closed. This is the mandatory final gate per
`.claude/rules/hardware-bench-testing.md` and this sprint's own acceptance
criteria (`heading-loop-cascade-control-turns-terminate-on-target.md`'s
"Acceptance" section, items 1-4).

Depends on 003 (always). If tickets 004 and/or 005 were executed, their
own hardware acceptance criteria must have passed first — this ticket does
not re-litigate them, it re-confirms the SYSTEM as a whole on the final
commit.

## Acceptance Criteria

- [ ] `just build-clean` succeeds on the final sprint commit.
- [ ] Full `uv run python -m pytest` is green — record the final pass count
      and confirm no regression from the pre-sprint baseline (615+ in
      `tests/sim/`, per the issue's own acceptance criterion 1).
- [ ] If tickets 004 and/or 005 were executed: re-run the SAME
      `turn_sweep.py --relay --both` grid (or a representative subset if a
      full grid re-run is not practical) ticket 003 used, and confirm no
      regression from ticket 003's own recorded baseline — this is the
      final hardware acceptance pass reflecting whatever landed.
- [ ] If tickets 004 and/or 005 were SKIPPED/deferred: confirm no
      firmware-affecting change has landed on the branch since ticket
      003's own hardware pass (a `git log`/diff check against ticket 003's
      commit) — if none, ticket 003's own results stand as current and
      this ticket does NOT need to re-run the playfield grid; state this
      explicitly rather than silently skipping the check.
- [ ] A final stand (no-wedge) check is performed regardless of whether
      004/005 ran — spin the wheels both directions one more time, confirm
      no commanded terminal reversal beyond the motor armor's window, on
      the EXACT commit being closed.
- [ ] The USB-reflash-then-playfield-relay two-location dependency is
      documented explicitly in this ticket's completion notes (which steps
      happened at which location).
- [ ] Any deferred/skipped optional ticket (004 and/or 005) is noted
      explicitly in this ticket's completion notes with its reason for
      deferral, for the team-lead's sprint-close review — per
      `.claude/rules/mcp-required.md`'s process discipline, this is not
      silently dropped.
- [ ] `data/robots/tovez.json`'s final `heading_kp`/`heading_kd` values (as
      landed by ticket 003's iteration, and by ticket 005 if any
      further live-tuned value was adopted as the new boot default) are
      confirmed consistent between the JSON and whatever was last
      bench-verified — no stale/untested value ships as the boot default.

## Testing

- **Existing tests to run**: full `uv run python -m pytest`;
  `tests/bench/turn_sweep.py --relay --both` (full or representative
  subset per the AC above).
- **New tests to write**: none — this ticket is a validation/closure pass
  over already-existing tests and instruments, not a source of new
  coverage.
- **Verification command**: `uv run python -m pytest`;
  `uv run python tests/bench/turn_sweep.py --relay --both`.

## Implementation Plan

**Approach**: A validation ticket, not a code-writing ticket — no source
changes expected unless the closure checks above surface a gap (in which
case, fix it here; this ticket is the sprint's own safety net for exactly
that).

**Files to modify**: none expected;
`clasi/sprints/098-heading-loop-cascade-control-turns-terminate-on-target/
sprint.md`'s own completion notes are the natural place to log this
ticket's final verdict for the team-lead.

**Files to create**: none.

**Testing plan**: as above.

**Documentation updates**: none beyond this ticket's own completion notes.
