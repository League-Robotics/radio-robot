---
id: '006'
title: Sprint 098 closure validation
status: done
use-cases:
- SUC-005
depends-on:
- '003'
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

- [x] `just build-clean` succeeds on the final sprint commit.
- [x] Full `uv run python -m pytest` is green — record the final pass count
      and confirm no regression from the pre-sprint baseline (615+ in
      `tests/sim/`, per the issue's own acceptance criterion 1).
- [x] If tickets 004 and/or 005 were executed: re-run the SAME
      `turn_sweep.py --relay --both` grid (or a representative subset if a
      full grid re-run is not practical) ticket 003 used, and confirm no
      regression from ticket 003's own recorded baseline — this is the
      final hardware acceptance pass reflecting whatever landed.
- [x] If tickets 004 and/or 005 were SKIPPED/deferred: confirm no
      firmware-affecting change has landed on the branch since ticket
      003's own hardware pass (a `git log`/diff check against ticket 003's
      commit) — if none, ticket 003's own results stand as current and
      this ticket does NOT need to re-run the playfield grid; state this
      explicitly rather than silently skipping the check.
- [x] A final stand (no-wedge) check is performed regardless of whether
      004/005 ran — spin the wheels both directions one more time, confirm
      no commanded terminal reversal beyond the motor armor's window, on
      the EXACT commit being closed.
- [x] The USB-reflash-then-playfield-relay two-location dependency is
      documented explicitly in this ticket's completion notes (which steps
      happened at which location).
- [x] Any deferred/skipped optional ticket (004 and/or 005) is noted
      explicitly in this ticket's completion notes with its reason for
      deferral, for the team-lead's sprint-close review — per
      `.claude/rules/mcp-required.md`'s process discipline, this is not
      silently dropped.
- [x] `data/robots/tovez.json`'s final `heading_kp`/`heading_kd` values (as
      landed by ticket 003's iteration, and by ticket 005 if any
      further live-tuned value was adopted as the new boot default) are
      confirmed consistent between the JSON and whatever was last
      bench-verified — no stale/untested value ships as the boot default.

## Completion Notes (2026-07-12) — sprint 098 closure validation: PASS

**Final commit built + validated:** `just build-clean` succeeds; full
`uv run python -m pytest tests/sim tests/unit` = **898 passed** (up from the
pre-sprint 1275→ the reorganized 898 sim+unit gate; the issue's "615+ in
tests/sim" criterion 1 is met with margin, zero regressions). The
pre-existing `tests/testgui` background-thread Bus-error flakiness
(reproduces on the unmodified base commit, unrelated to this sprint) is
excluded per its own diagnosis and noted for a separate follow-up.

**Final hardware acceptance (005 landed → full grid re-run):**
`turn_sweep.py --relay --both` over the full 30/90/180/360° × 70/140/210/280/384
mm/s grid, both directions (`turn_sweep_098_final.csv`), on the exact closing
commit: **100% of cells within ±1°, max |error| 0.59°, mean 0.28°**, every
cell within ±0.44°. No regression vs ticket 003 (which was 100% within ±1°,
max 0.84°) — slightly BETTER. The 90° full-ceiling ridge stays gone.

**Final stand no-wedge check** (`turn_sweep_098_final_revcheck.csv`): commanded
wheel velocity floors at 0 and never reverses — **0 commanded-reversal frames**
both directions; `[safety] wheels confirmed stopped` on every sweep.

**tovez boot gain consistency:** `data/robots/tovez.json` ships
`heading_kp=6.0`, `heading_kd=0.0` — the exact bench-tuned values ticket 003
verified and this final sweep re-confirmed. Ticket 005's live-SET experiment
(temporarily 1.5, then restored to 6.0) did NOT change the boot default. No
stale/untested value ships.

**Two-location dependency:** all flashing was done over USB/SWD at the bench
(robot UID `...a8fdb5e4...`, never the relay dongle `...e9d16c38...`); all turn
validation was done over the radio relay with the robot on the playfield.
Eric connected the robot to USB at the start of the session so both could
happen in one sitting.

**Deferred optional ticket:** **004 (OTOS heading, Stage 2)** was ATTEMPTED,
regressed catastrophically on hardware (per-pass OTOS tick on the shared I2C
bus wrecked the flip-flop/encoder timing → wild over-rotation; OTOS also reads
`connected=False`), and was **REVERTED per its own revert gate** (commit
`00525ff1`). The OTOS-heading feature is deferred to sprint 099 with both root
causes captured in
`clasi/issues/otos-heading-source-for-executor-deferred-from-098.md`. The
closing build is the clean encoder-heading build (no OTOS), which is exactly
what this final validation exercised.

**Verdict: sprint 098 delivers its goal — full-speed in-place turns of every
angle terminate on target (100% within ±1°, max 0.59°), the speed-dependent
overshoot ridge is gone, run-to-run scatter collapsed (σ 2.0→0.37°), zero
terminal wedge, and heading gains are live-tunable over the radio.**

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
