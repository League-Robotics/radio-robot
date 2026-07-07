---
id: "009"
title: "Control retuning and HITL bench acceptance gate"
status: open
use-cases: [SUC-001, SUC-002, SUC-003, SUC-004, SUC-005, SUC-006]
depends-on: ["008"]
github-issue: ""
issue:
- plan-file-a-design-issue-blackboard-architecture-state-objects-command-queues.md
- preserve-serial-silence-safety-watchdog-in-greenfield-loop.md
# completes_issue: Controls whether linked issues are archived when this ticket
# is moved to done. Default: true (archive when all referencing tickets are done).
# Set to false (scalar) to suppress archival for ALL linked issues on this ticket.
# Set to a mapping {filename.md: false} to suppress archival per issue filename.
# Use false for tickets that partially address a multi-sprint umbrella issue.
completes_issue: true
# exception: Written by a lower agent when it cannot proceed (see architecture §exception-protocol).
# exception:
#   thrown_by: "programmer"          # "programmer" | "sprint-planner"
#   thrown_at: "2026-05-07T14:23:00Z"
#   attempted: |
#     Description of what was attempted before giving up.
#   conflict: "architecture-update.md §3 — reason the agent is blocked"
#   surface: "internal"              # "user-visible" | "internal"
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Control retuning and HITL bench acceptance gate

## Description

Close out sprint 087. This ticket directly depends on ticket 008 and
therefore, transitively, on every prior ticket (001-008) being done. Five
things happen here:

1. **Verify/adjust control tuning** for the added one-tick synchronous-update
   latency (Decision 6, Open Question 1 in `architecture-update.md`) against
   the sim and the hardware bench — adjust `Drivetrain`/`Planner` gains or
   thresholds **only if** a real regression is observed, not pre-emptively.
2. **Run the full existing regression suite** (`tests/sim/unit/`,
   `tests/sim/system/`) to confirm the whole rearchitecture is
   behavior-preserving at the wire level.
3. **Execute the hardware-bench gate** per
   `.claude/rules/hardware-bench-testing.md` **and** this sprint's own
   radio-specific requirement (Decision 9): deploy to the robot on the
   stand, confirm every sensor responds, confirm wheels drive and encoders
   increment in both directions, and round-trip at least one command over
   the **radio relay** specifically (not only serial — a serial-only pass
   cannot catch a missing slack-loop yield).
4. **Confirm the serial-silence safety watchdog** neutralizes on the stand
   under comms silence, exercised over the radio path specifically — this
   is the linked watchdog issue's own Bench/HITL acceptance criterion, and
   this ticket is what finally closes that issue out (ticket 007 delivered
   the sim-side/same-pass correctness; `completes_issue` was set `false`
   for that issue on ticket 007 for exactly this reason).
5. **Confirm sim/hardware parity** — the same command sequence produces
   equivalent behavior in `tests/_infra/sim` and on the real robot, within
   the tolerances the existing bench scripts already use.

## Acceptance Criteria

- [ ] Full `tests/sim/unit/` and `tests/sim/system/` suites pass.
- [ ] Determinism/order-independence (SUC-001) is re-confirmed against the
      **full rebuilt loop** (not just the isolated primitives from ticket
      007) — re-ordering the mandatory-tick call sequence produces
      bit-identical `x[k+1]` for a fixed `x[k]` and fixed inputs.
- [ ] Control tuning is verified against the sim first; any regression
      attributable to the one-tick latency is documented with before/after
      values and the specific gain/threshold changed, in this ticket's
      completion notes.
- [ ] Hardware-bench gate (`.claude/rules/hardware-bench-testing.md` items
      1-3) passes on the stand: sensors alive (encoders, OTOS, line, color,
      digital/analog), wheels drive both directions with encoders
      incrementing proportionally, round-trip over the actual transport.
- [ ] The round-trip-over-transport check is performed specifically over
      the **radio relay** (not serial-only), confirming Decision 9's yield
      is present and effective in the shipped firmware.
- [ ] Comms-silence safety watchdog: on the stand, with no statement
      arriving for longer than the configured window over the **radio**
      path, the wheels neutralize and `EVT dev_watchdog` is observed;
      feeding a statement re-arms it. (Closes the linked watchdog issue's
      Bench/HITL acceptance criterion.)
- [ ] Sim/hardware parity: a representative command sequence (e.g. a short
      drive + turn + `SI` + `ZERO enc` sequence) produces equivalent
      encoder/pose behavior in sim and on the bench, within existing
      tolerance bands.
- [ ] Sprint 087's full acceptance bar (every SUC-001 through SUC-006
      acceptance criterion in `usecases.md`) is confirmed met end-to-end,
      not just per-ticket.

## Implementation Plan

**Approach.** This ticket is verification-and-tuning, not new production
architecture — it may complete with zero source changes beyond tuning
constants, which is an expected, successful outcome (per sprint 085's own
precedent for verification-first tickets), **not** a sign of misscoping.

**Files to modify:** possibly `source/subsystems/drivetrain.{h,cpp}` or
`planner.{h,cpp}`'s gain/threshold constants only, if retuning proves
necessary; no structural changes expected.

**Testing plan:**
- Run the full existing automated suite.
- Execute the hardware-bench gate live (radio + serial + watchdog +
  sensors + drive), per `.claude/rules/hardware-bench-testing.md`'s
  "Standing verification gate" — this sprint is explicitly one of the
  firmware sprints that gate applies to (it touches the HAL, motor
  control, sensing, and the command protocol's internal transport).
- Confirm sim/bench parity with a scripted comparison run.
- **Verification command**: `uv run pytest tests/sim` then the live bench sequence per `.claude/rules/hardware-bench-testing.md`.

**Documentation updates:** Consider refreshing
`.claude/rules/hardware-bench-testing.md`'s stale pre-v2 quick-smoke table
(flagged stale in the file itself) against the post-rearchitecture command
surface — optional stretch, not required for this ticket's acceptance
unless the team-lead requests it.
